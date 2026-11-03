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

import os
import rclpy
import socket
import json
import sys
import threading
import time
import importlib

from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import MultiThreadedExecutor
from rclpy.serialization import deserialize_message

from .tcp_sender import UnityTcpSender
from .client import ClientThread
from .subscriber import RosSubscriber
from .publisher import RosPublisher
from .service import RosService
from .unity_service import UnityService


class TcpServer(Node):
    """
    Initializes ROS node and TCP server.
    """
    def __init__(self, node_name, buffer_size=4194304, connections=10, tcp_ip=None, tcp_port=None):
        """
        Initializes ROS node and class variables.

        Args:
            node_name:               ROS node name for executing code
            buffer_size:             SO_RCVBUF/SO_SNDBUF requested for Unity connections, in bytes.
                                     Linux caps it at net.core.rmem_max / wmem_max.
            connections:             Max number of queued connections. See Python Socket documentation
        """
        super().__init__(node_name)

        self.declare_parameter("ROS_IP", "0.0.0.0")
        self.declare_parameter("ROS_TCP_PORT", 10000)
        # Seconds without any data from Unity before the connection is considered dead.
        # Unity sends a keepalive every KeepaliveTime (1 s by default) when idle. 0 disables.
        # dynamic_typing so that both 10 and 10.0 are accepted from a launch file.
        self.declare_parameter(
            "TCP_CLIENT_TIMEOUT", 10.0, ParameterDescriptor(dynamic_typing=True)
        )
        self.client_timeout = float(self.get_parameter("TCP_CLIENT_TIMEOUT").value)

        if tcp_ip:
            self.loginfo("Using ROS_IP override from constructor: {}".format(tcp_ip))
            self.tcp_ip = tcp_ip
        else:
            self.tcp_ip = self.get_parameter("ROS_IP").get_parameter_value().string_value

        if tcp_port:
            self.loginfo("Using ROS_TCP_PORT override from constructor: {}".format(tcp_port))
            self.tcp_port = tcp_port
        else:
            self.tcp_port = self.get_parameter("ROS_TCP_PORT").get_parameter_value().integer_value

        self.unity_tcp_sender = UnityTcpSender(self)

        self.node_name = node_name
        self.publishers_table = {}
        self.subscribers_table = {}
        self.ros_services_table = {}
        self.unity_services_table = {}
        self.buffer_size = buffer_size
        self.connections = connections
        self.syscommands = SysCommands(self)
        self.pending_srv_id = None
        self.pending_srv_is_request = False

        self.shutting_down = False
        self.client_threads = set()
        self.client_threads_lock = threading.Lock()

    def start(self, publishers=None, subscribers=None):
        if publishers is not None:
            self.publishers_table = publishers
        if subscribers is not None:
            self.subscribers_table = subscribers
        server_thread = threading.Thread(target=self.listen_loop)
        # Exit the server thread when the main thread terminates
        server_thread.daemon = True
        server_thread.start()

    def listen_loop(self):
        """
            Creates and binds sockets using TCP variables then listens for incoming connections.
            For each new connection a client thread will be created to handle communication.

            This thread must never die: if it does, the endpoint keeps running but Unity can
            never connect again. So bind is retried, and accept errors are logged and skipped.
        """
        self.loginfo("Starting server on {}:{}".format(self.tcp_ip, self.tcp_port))
        tcp_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # Lets a restarted endpoint bind while old connections are still in TIME_WAIT.
        tcp_server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        # Set the buffer sizes on the listening socket so accepted sockets inherit them
        # before the handshake (the TCP window scale is fixed during the handshake).
        tcp_server.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, self.buffer_size)
        tcp_server.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, self.buffer_size)

        while not self.shutting_down:
            try:
                tcp_server.bind((self.tcp_ip, self.tcp_port))
                break
            except OSError as e:
                # e.g. another endpoint (or a zombie one) still holds the port
                self.logerr(
                    "Can't bind {}:{} ({}), retrying in 2 s".format(self.tcp_ip, self.tcp_port, e)
                )
                time.sleep(2)

        tcp_server.listen(self.connections)
        # Wake up regularly so shutdown() doesn't depend on close() interrupting accept().
        tcp_server.settimeout(1.0)

        while not self.shutting_down:
            try:
                (conn, (ip, port)) = tcp_server.accept()
            except socket.timeout:
                continue
            except OSError as e:
                if self.shutting_down:
                    break
                self.logerr("ros_tcp_endpoint.TcpServer: accept failed: {}".format(e))
                time.sleep(0.1)
                continue

            try:
                self.configure_client_socket(conn)
                client = ClientThread(conn, self, ip, port)
                with self.client_threads_lock:
                    self.client_threads.add(client)
                client.start()
            except Exception as e:
                # e.g. the client reset the connection right after connecting
                self.logerr("ros_tcp_endpoint.TcpServer: dropping new connection: {}".format(e))
                conn.close()

        tcp_server.close()

    def configure_client_socket(self, conn):
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

        # Detect a dead peer (Unity machine gone, cable pulled, VM paused) even if nothing
        # is being sent. Unity's own keepalives cover the case where the Unity process is
        # alive but its connection thread is not (see client_timeout below).
        conn.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
        if hasattr(socket, "TCP_KEEPIDLE"):  # Linux
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPIDLE, 5)
        elif hasattr(socket, "TCP_KEEPALIVE"):  # macOS
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPALIVE, 5)
        if hasattr(socket, "TCP_KEEPINTVL"):
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPINTVL, 2)
        if hasattr(socket, "TCP_KEEPCNT"):
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPCNT, 3)

        # Read timeout: if Unity sends nothing (not even its 1 s keepalive) for this long,
        # recv raises socket.timeout and the ClientThread cleans up. Without it, a socket
        # leaked by the Unity editor (e.g. exiting play mode) kept a ClientThread alive forever.
        # Note this also applies to sendall() in the sender thread.
        if self.client_timeout > 0:
            conn.settimeout(self.client_timeout)
        else:
            conn.settimeout(None)

    def client_finished(self, client):
        with self.client_threads_lock:
            self.client_threads.discard(client)

    def shutdown(self):
        """
            Stop accepting connections and close every Unity connection, so Unity sees the
            endpoint go away right now instead of after a timeout.
        """
        self.shutting_down = True
        with self.client_threads_lock:
            clients = list(self.client_threads)
        for client in clients:
            client.close()

    def send_unity_error(self, error):
        self.unity_tcp_sender.send_unity_error(error)

    def send_unity_message(self, topic, message):
        self.unity_tcp_sender.send_unity_message(topic, message)

    def send_unity_service(self, topic, service_class, request):
        return self.unity_tcp_sender.send_unity_service_request(topic, service_class, request)

    def send_unity_service_response(self, srv_id, data):
        self.unity_tcp_sender.send_unity_service_response(srv_id, data)

    def handle_syscommand(self, topic, data):
        # getattr without a default raised AttributeError for unknown commands, which
        # killed the client thread. Names starting with "_" are not commands.
        name = topic[2:]
        function = None if name.startswith("_") else getattr(self.syscommands, name, None)
        if function is None:
            self.send_unity_error("Don't understand SysCommand.'{}'".format(topic))
            self.logwarn("Ignoring unknown SysCommand '{}'".format(topic))
        else:
            message_json = data.decode("utf-8")[:-1]
            params = json.loads(message_json)
            function(**params)

    def loginfo(self, text):
        self.get_logger().info(text)

    def logwarn(self, text):
        self.get_logger().warning(text)

    def logerr(self, text):
        self.get_logger().error(text)

    def setup_executor(self):
        """
            Since rclpy.spin() is a blocking call the server needed a way
            to spin all of the relevant nodes at the same time.

            MultiThreadedExecutor allows us to set the number of threads
            needed as well as the nodes that need to be spun.
        """
        num_threads = (
            len(self.publishers_table.keys())
            + len(self.subscribers_table.keys())
            + len(self.ros_services_table.keys())
            + len(self.unity_services_table.keys())
            + 1
        )
        # The tables are normally empty here (Unity registers topics later), so the sum
        # above is 1. With a single thread, one Unity service callback waiting for Unity
        # blocked every subscription and every ROS service response in the endpoint.
        num_threads = max(num_threads, os.cpu_count() or 4, 4)
        executor = MultiThreadedExecutor(num_threads)

        executor.add_node(self)

        for ros_node in self.publishers_table.values():
            executor.add_node(ros_node)
        for ros_node in self.subscribers_table.values():
            executor.add_node(ros_node)
        for ros_node in self.ros_services_table.values():
            executor.add_node(ros_node)
        for ros_node in self.unity_services_table.values():
            executor.add_node(ros_node)

        self.executor = executor
        executor.spin()

    def unregister_node(self, old_node):
        if old_node is not None:
            # Take the node out of the executor before destroying it, so the executor
            # never waits on handles that are being destroyed.
            if self.executor is not None:
                self.executor.remove_node(old_node)
            old_node.unregister()

    def destroy_nodes(self):
        """
            Clean up all of the nodes
        """
        for ros_node in self.publishers_table.values():
            ros_node.destroy_node()
        for ros_node in self.subscribers_table.values():
            ros_node.destroy_node()
        for ros_node in self.ros_services_table.values():
            ros_node.destroy_node()
        for ros_node in self.unity_services_table.values():
            ros_node.destroy_node()

        self.destroy_node()


class SysCommands:
    def __init__(self, tcp_server):
        self.tcp_server = tcp_server

    def subscribe(self, topic, message_name):
        if topic == "":
            self.tcp_server.send_unity_error(
                "Can't subscribe to a blank topic name! SysCommand.subscribe({}, {})".format(
                    topic, message_name
                )
            )
            return

        message_class = self.resolve_message_name(message_name)
        if message_class is None:
            self.tcp_server.send_unity_error(
                "SysCommand.subscribe - Unknown message class '{}'".format(message_name)
            )
            return

        old_node = self.tcp_server.subscribers_table.get(topic)
        if old_node is not None:
            if old_node.msg == message_class:
                # Unity re-sends every registration when it reconnects. Keep the existing
                # node instead of destroying and re-creating it every time.
                self.tcp_server.loginfo("RegisterSubscriber({}) already registered".format(topic))
                return
            self.tcp_server.unregister_node(old_node)

        new_subscriber = RosSubscriber(topic, message_class, self.tcp_server)
        self.tcp_server.subscribers_table[topic] = new_subscriber
        if self.tcp_server.executor is not None:
            self.tcp_server.executor.add_node(new_subscriber)

        self.tcp_server.loginfo("RegisterSubscriber({}, {}) OK".format(topic, message_class))

    def publish(self, topic, message_name, queue_size=10, latch=False):
        if topic == "":
            self.tcp_server.send_unity_error(
                "Can't publish to a blank topic name! SysCommand.publish({}, {})".format(
                    topic, message_name
                )
            )
            return

        message_class = self.resolve_message_name(message_name)
        if message_class is None:
            self.tcp_server.send_unity_error(
                "SysCommand.publish - Unknown message class '{}'".format(message_name)
            )
            return

        old_node = self.tcp_server.publishers_table.get(topic)
        if old_node is not None:
            if (
                old_node.message_class == message_class
                and old_node.queue_size == queue_size
                and old_node.latch == latch
            ):
                self.tcp_server.loginfo("RegisterPublisher({}) already registered".format(topic))
                return
            self.tcp_server.unregister_node(old_node)

        new_publisher = RosPublisher(topic, message_class, queue_size=queue_size, latch=latch)

        self.tcp_server.publishers_table[topic] = new_publisher
        if self.tcp_server.executor is not None:
            self.tcp_server.executor.add_node(new_publisher)

        self.tcp_server.loginfo("RegisterPublisher({}, {}) OK".format(topic, message_class))

    def ros_service(self, topic, message_name):
        if topic == "":
            self.tcp_server.send_unity_error(
                "RegisterRosService({}, {}) - Can't register a blank topic name!".format(
                    topic, message_name
                )
            )
            return
        message_class = self.resolve_message_name(message_name, "srv")
        if message_class is None:
            self.tcp_server.send_unity_error(
                "RegisterRosService({}, {}) - Unknown service class '{}'".format(
                    topic, message_name, message_name
                )
            )
            return

        old_node = self.tcp_server.ros_services_table.get(topic)
        if old_node is not None:
            if old_node.service_class == message_class:
                self.tcp_server.loginfo("RegisterRosService({}) already registered".format(topic))
                return
            self.tcp_server.unregister_node(old_node)

        new_service = RosService(topic, message_class)

        self.tcp_server.ros_services_table[topic] = new_service
        if self.tcp_server.executor is not None:
            self.tcp_server.executor.add_node(new_service)

        self.tcp_server.loginfo("RegisterRosService({}, {}) OK".format(topic, message_class))

    def unity_service(self, topic, message_name):
        if topic == "":
            self.tcp_server.send_unity_error(
                "RegisterUnityService({}, {}) - Can't register a blank topic name!".format(
                    topic, message_name
                )
            )
            return

        message_class = self.resolve_message_name(message_name, "srv")
        if message_class is None:
            self.tcp_server.send_unity_error(
                "RegisterUnityService({}, {}) - Unknown service class '{}'".format(
                    topic, message_name, message_name
                )
            )
            return

        old_node = self.tcp_server.unity_services_table.get(topic)
        if old_node is not None:
            if old_node.service_class == message_class:
                self.tcp_server.loginfo(
                    "RegisterUnityService({}) already registered".format(topic)
                )
                return
            self.tcp_server.unregister_node(old_node)

        new_service = UnityService(str(topic), message_class, self.tcp_server)

        self.tcp_server.unity_services_table[topic] = new_service
        if self.tcp_server.executor is not None:
            self.tcp_server.executor.add_node(new_service)

        self.tcp_server.loginfo("RegisterUnityService({}, {}) OK".format(topic, message_class))

    # Unity sends these from Unsubscribe() etc. They had no handler, so the first
    # Unsubscribe() call raised AttributeError and dropped the connection.
    def remove_subscriber(self, topic):
        self._remove(self.tcp_server.subscribers_table, topic, "Subscriber")

    def remove_publisher(self, topic):
        self._remove(self.tcp_server.publishers_table, topic, "Publisher")

    def remove_ros_service(self, topic):
        self._remove(self.tcp_server.ros_services_table, topic, "RosService")

    def remove_unity_service(self, topic):
        self._remove(self.tcp_server.unity_services_table, topic, "UnityService")

    def _remove(self, table, topic, kind):
        old_node = table.pop(topic, None)
        if old_node is not None:
            self.tcp_server.unregister_node(old_node)
            self.tcp_server.loginfo("Remove{}({}) OK".format(kind, topic))

    def response(self, srv_id):  # the next message is a service response
        self.tcp_server.pending_srv_id = srv_id
        self.tcp_server.pending_srv_is_request = False

    def request(self, srv_id):  # the next message is a service request
        self.tcp_server.pending_srv_id = srv_id
        self.tcp_server.pending_srv_is_request = True

    def topic_list(self):
        self.tcp_server.unity_tcp_sender.send_topic_list()

    def resolve_message_name(self, name, extension="msg"):
        try:
            names = name.split("/")
            module_name = names[0]
            class_name = names[1]
            importlib.import_module(module_name + "." + extension)
            module = sys.modules[module_name]
            if module is None:
                self.tcp_server.logerr("Failed to resolve module {}".format(module_name))
            module = getattr(module, extension)
            if module is None:
                self.tcp_server.logerr(
                    "Failed to resolve module {}.{}".format(module_name, extension)
                )
            module = getattr(module, class_name)
            if module is None:
                self.tcp_server.logerr(
                    "Failed to resolve module {}.{}.{}".format(module_name, extension, class_name)
                )
            return module
        except (IndexError, KeyError, AttributeError, ImportError) as e:
            self.tcp_server.logerr("Failed to resolve message name: {}".format(e))
            return None
