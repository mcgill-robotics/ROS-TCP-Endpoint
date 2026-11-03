#  Copyright 2020 Unity Technologies
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.

"""
Connection scenarios for the endpoint, driven by a fake Unity client.

This is not collected by colcon test. Run it by hand with ROS 2 sourced,
from the repository root:

    python3 test/integration/scenarios.py

or inside docker with test/integration/run_in_docker.sh.

It starts the endpoint as a subprocess, plays Unity with FakeUnity, and plays
the rest of the ROS graph with a normal rclpy node. Each scenario prints PASS
or FAIL. The exit code is the number of failed scenarios.
"""

import os
import signal
import subprocess
import sys
import threading
import time
import traceback

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.serialization import deserialize_message, serialize_message
from std_msgs.msg import String
from std_srvs.srv import Trigger

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from fake_unity import FakeUnity  # noqa: E402

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
PORT = int(os.environ.get("ENDPOINT_TEST_PORT", "10011"))
CLIENT_TIMEOUT = 3.0
LOG_PATH = os.environ.get("ENDPOINT_TEST_LOG", "/tmp/endpoint_test.log")


class Endpoint:
    """The endpoint under test, run as a separate process like ros2 run would."""

    def __init__(self):
        self.log = open(LOG_PATH, "a")
        env = dict(os.environ)
        env["PYTHONPATH"] = REPO_ROOT + os.pathsep + env.get("PYTHONPATH", "")
        env["PYTHONUNBUFFERED"] = "1"
        self.proc = subprocess.Popen(
            [
                sys.executable,
                "-m",
                "ros_tcp_endpoint.default_server_endpoint",
                "--ros-args",
                "-p",
                "ROS_TCP_PORT:={}".format(PORT),
                "-p",
                "TCP_CLIENT_TIMEOUT:={}".format(CLIENT_TIMEOUT),
            ],
            env=env,
            stdout=self.log,
            stderr=subprocess.STDOUT,
        )

    def connect(self, timeout=10.0):
        deadline = time.time() + timeout
        while True:
            try:
                return FakeUnity(port=PORT)
            except OSError:
                if time.time() > deadline or self.proc.poll() is not None:
                    raise
                time.sleep(0.2)

    def stop(self, sig=signal.SIGINT, timeout=10.0):
        """Returns True if the process exited within timeout."""
        if self.proc.poll() is None:
            self.proc.send_signal(sig)
        try:
            self.proc.wait(timeout)
            return True
        except subprocess.TimeoutExpired:
            self.proc.kill()
            self.proc.wait()
            return False


def wait_until(predicate, timeout):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if predicate():
            return True
        time.sleep(0.05)
    return predicate()


class RosSide(Node):
    def __init__(self):
        super().__init__("endpoint_scenarios")
        self.received = []
        self.create_subscription(String, "/from_unity", lambda m: self.received.append(m.data), 10)
        self.pub = self.create_publisher(String, "/to_unity", 10)
        self.create_service(Trigger, "/ros_trigger", self.on_trigger)
        self.unity_srv_client = self.create_client(Trigger, "/unity_trigger")

    def on_trigger(self, request, response):
        response.success = True
        response.message = "hello from ros"
        return response

    def count_nodes(self, name):
        return sum(1 for n in self.get_node_names() if n == name)


def register_all(unity):
    unity.syscommand(
        "__publish", topic="/from_unity", message_name="std_msgs/String", queue_size=10, latch=False
    )
    unity.syscommand("__subscribe", topic="/to_unity", message_name="std_msgs/String")
    unity.syscommand("__ros_service", topic="/ros_trigger", message_name="std_srvs/Trigger")
    unity.syscommand("__unity_service", topic="/unity_trigger", message_name="std_srvs/Trigger")


def publish_from_unity(unity, text):
    unity.send_raw("/from_unity", serialize_message(String(data=text)))


def subscription_reaches(ros, unity, text, timeout=5.0):
    # Publish repeatedly: discovery between the new subscription and our publisher takes a moment.
    deadline = time.time() + timeout
    while time.time() < deadline:
        ros.pub.publish(String(data=text))
        payload = unity.wait_for("/to_unity", timeout=0.3)
        if payload is not None and deserialize_message(payload, String).data == text:
            return True
    return False


# ---------------------------------------------------------------- scenarios


def s_handshake(ep, ros, ctx):
    unity = ep.connect()
    ctx["unity"] = unity
    payload = unity.wait_for("__handshake", timeout=5)
    return payload is not None, "handshake received" if payload else "no handshake"


def s_unity_publish(ep, ros, ctx):
    unity = ctx["unity"]
    register_all(unity)
    time.sleep(1.0)
    for i in range(5):
        publish_from_unity(unity, "msg{}".format(i))
        time.sleep(0.05)
    ok = wait_until(lambda: len(ros.received) >= 5, 5)
    alive = not unity.closed_by_peer.is_set()
    return ok and alive, "ROS received {} of 5, connection alive={}".format(
        len(ros.received), alive
    )


def s_ros_to_unity(ep, ros, ctx):
    ok = subscription_reaches(ros, ctx["unity"], "to unity 1")
    return ok, "subscription forwarded" if ok else "nothing reached Unity"


def s_ros_service_from_unity(ep, ros, ctx):
    unity = ctx["unity"]
    unity.syscommand("__request", srv_id=101)
    unity.send_raw("/ros_trigger", serialize_message(Trigger.Request()))
    header = unity.wait_for("__response", timeout=5)
    body = unity.wait_for("/ros_trigger", timeout=5)
    if header is None or body is None:
        return False, "no service response (header={}, body={})".format(header, body)
    resp = deserialize_message(body, Trigger.Response)
    return resp.message == "hello from ros", "response: {!r}".format(resp.message)


def s_unity_service_from_ros(ep, ros, ctx):
    unity = ctx["unity"]
    if not ros.unity_srv_client.wait_for_service(timeout_sec=5):
        return False, "unity service never appeared"
    future = ros.unity_srv_client.call_async(Trigger.Request())
    header = unity.wait_for("__request", timeout=5)
    if header is None:
        return False, "Unity never got the request"
    import json

    srv_id = json.loads(header.decode())["srv_id"]
    reply = Trigger.Response(success=True, message="hello from unity")
    unity.syscommand("__response", srv_id=srv_id)
    unity.send_raw("/unity_trigger", serialize_message(reply))
    ok = wait_until(future.done, 5)
    if not ok:
        return False, "ROS caller never got a response"
    return future.result().message == "hello from unity", future.result().message


def s_unknown_and_remove_syscommands(ep, ros, ctx):
    unity = ctx["unity"]
    unity.syscommand("__not_a_real_command")
    unity.syscommand("__remove_subscriber", topic="/does_not_exist")
    time.sleep(0.5)
    before = len(ros.received)
    publish_from_unity(unity, "after weird syscommands")
    ok = wait_until(lambda: len(ros.received) > before, 3)
    alive = not unity.closed_by_peer.is_set()
    return ok and alive, "connection alive={}, publish ok={}".format(alive, ok)


def s_endpoint_heartbeat(ep, ros, ctx):
    unity = ctx["unity"]
    start = unity.keepalives_received
    ok = wait_until(lambda: unity.keepalives_received > start, 4)
    return ok, "endpoint sent {} keepalives".format(unity.keepalives_received - start)


def s_unity_disconnects_mid_unity_service(ep, ros, ctx):
    # Unity gets a request for its service, then dies without answering.
    unity = ctx["unity"]
    future = ros.unity_srv_client.call_async(Trigger.Request())
    header = unity.wait_for("__request", timeout=5)
    if header is None:
        return False, "Unity never got the request"
    unity.close()
    ok = wait_until(future.done, 8)
    if not ok:
        return False, "ROS caller still blocked 8 s after Unity disconnected"
    # The endpoint must still forward subscriptions to the next Unity session.
    unity2 = ep.connect()
    ctx["unity"] = unity2
    register_all(unity2)
    fwd = subscription_reaches(ros, unity2, "after abandoned service", timeout=6)
    return fwd, "caller released, forwarding to new session={}".format(fwd)


def s_reconnect_no_duplicates(ep, ros, ctx):
    unity = ctx["unity"]
    unity.close()
    for _ in range(3):
        u = ep.connect()
        register_all(u)
        time.sleep(0.5)
        u.close()
    unity = ep.connect()
    ctx["unity"] = unity
    register_all(unity)
    time.sleep(1.5)
    counts = {
        name: ros.count_nodes(name)
        for name in ("from_unity_RosPublisher", "to_unity_RosSubscriber", "ros_trigger_RosService")
    }
    dupes = {k: v for k, v in counts.items() if v != 1}
    fwd = subscription_reaches(ros, unity, "after reconnects")
    before = len(ros.received)
    publish_from_unity(unity, "after reconnects")
    pub = wait_until(lambda: len(ros.received) > before, 3)
    return (not dupes) and fwd and pub, "node counts={} sub ok={} pub ok={}".format(
        counts, fwd, pub
    )


def s_silent_client_dropped(ep, ros, ctx):
    # A client that connects and then goes quiet (Unity frozen, or the editor
    # leaked the socket on domain reload) must be dropped by the endpoint.
    silent = ep.connect()
    silent.wait_for("__handshake", timeout=5)
    ok = silent.closed_by_peer.wait(CLIENT_TIMEOUT + 4)
    silent.close()
    return ok, "endpoint closed the silent connection" if ok else "silent connection kept open"


def s_half_open_then_new_session(ep, ros, ctx):
    # Old session stays open but silent, new session connects: new one must work at once.
    old = ctx["unity"]
    new = ep.connect()
    ctx["unity"] = new
    register_all(new)
    fwd = subscription_reaches(ros, new, "to the new session", timeout=6)
    old_dropped = old.closed_by_peer.wait(CLIENT_TIMEOUT + 4)
    still_fwd = subscription_reaches(ros, new, "still the new session", timeout=4)
    return fwd and still_fwd and old_dropped, "new fwd={}, old dropped={}, new still ok={}".format(
        fwd, old_dropped, still_fwd
    )


def keep_alive_loop(ctx, stop):
    # Real Unity sends a keepalive every second when idle.
    while not stop.is_set():
        u = ctx.get("unity")
        if u is not None:
            try:
                u.keepalive()
            except OSError:
                pass
        stop.wait(1.0)


def main():
    rclpy.init()
    ros = RosSide()
    executor = MultiThreadedExecutor()
    executor.add_node(ros)
    threading.Thread(target=executor.spin, daemon=True).start()

    open(LOG_PATH, "w").close()
    ep = Endpoint()
    ctx = {}
    stop_keepalive = threading.Event()
    results = []

    def run(name, fn, endpoint):
        try:
            ok, detail = fn(endpoint, ros, ctx)
        except Exception:
            ok, detail = False, traceback.format_exc()
        results.append((name, ok))
        print("{} {}: {}".format("PASS" if ok else "FAIL", name, detail), flush=True)

    run("handshake", s_handshake, ep)
    threading.Thread(target=keep_alive_loop, args=(ctx, stop_keepalive), daemon=True).start()
    run("unity publishes to ROS", s_unity_publish, ep)
    run("ROS topic forwarded to Unity", s_ros_to_unity, ep)
    run("Unity calls a ROS service", s_ros_service_from_unity, ep)
    run("ROS calls a Unity service", s_unity_service_from_ros, ep)
    run("unknown and remove_* sys commands", s_unknown_and_remove_syscommands, ep)
    run("endpoint sends keepalives", s_endpoint_heartbeat, ep)
    run("Unity dies during a Unity service call", s_unity_disconnects_mid_unity_service, ep)
    run("reconnect without duplicate nodes", s_reconnect_no_duplicates, ep)
    run("silent client is dropped", s_silent_client_dropped, ep)

    # The keepalive loop follows ctx["unity"], so once the scenario switches to a new
    # session the old one stops sending anything and looks half-open to the endpoint.
    run("half-open old session, new session", s_half_open_then_new_session, ep)

    # Ctrl+C while a client is connected.
    unity = ctx["unity"]
    exited = ep.stop(signal.SIGINT, timeout=10)
    eof = unity.closed_by_peer.wait(2) if exited else False
    results.append(("ctrl+c", exited))
    print("{} ctrl+c with a client connected: exited={}, client saw close={}, rc={}".format(
        "PASS" if exited else "FAIL", exited, eof, ep.proc.returncode), flush=True)
    stop_keepalive.set()

    # Restart on the same port straight away (old sockets in TIME_WAIT).
    ep2 = Endpoint()
    try:
        u = ep2.connect(timeout=10)
        ok = u.wait_for("__handshake", timeout=5) is not None
        u.close()
    except Exception as e:
        ok = False
        print(e)
    results.append(("restart", ok))
    print("{} endpoint restart on same port: handshake={}".format("PASS" if ok else "FAIL", ok))
    ep2.stop(signal.SIGINT, timeout=10)

    failed = [n for n, ok in results if not ok]
    print("\n{} passed, {} failed".format(len(results) - len(failed), len(failed)))
    sys.stdout.flush()
    # Skip interpreter teardown: tearing down a spinning executor thread can abort the process.
    os._exit(len(failed))


if __name__ == "__main__":
    main()
