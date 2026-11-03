# ROS TCP Endpoint (McGill Robotics fork)

This is a fork of Unity's [ROS-TCP-Endpoint](https://github.com/Unity-Technologies/ROS-TCP-Endpoint), ROS 2 branch, release 0.7.0. It runs on the ROS 2 side of the McGill Robotics AUV stack (Humble, also checked on Jazzy) and talks to our Unity 6 simulator through the matching connector fork, [kaidalisohaib/ROS-TCP-Connector](https://github.com/kaidalisohaib/ROS-TCP-Connector) (branch `main-ros2-optimized`).

Upstream is no longer maintained. We forked it to cut some per-message overhead, and because the bridge often got stuck when the sim or ROS was not closed properly, or when the two were started and stopped in a different order. The wire protocol is unchanged, so this endpoint still works with the upstream connector and the other way round.

## What is different from upstream

Performance changes (summer 2026):

- `TCP_NODELAY` on Unity sockets.
- Socket buffers (`SO_RCVBUF`/`SO_SNDBUF`) set from `buffer_size`, which now defaults to 4 MB. The kernel caps this at `net.core.rmem_max`/`wmem_max`, so with default sysctls you actually get about 416 KB.
- Subscriptions are created with `raw=True`. Messages going to Unity are forwarded as the serialized bytes, without being deserialized and serialized again.
- ROS service calls from Unity wait on a `threading.Event` instead of busy-waiting on the future.

The summer commit also changed `recvall` to return a `bytearray`. That broke every publish and service call from Unity (rclpy only takes `bytes`), and has been reverted.

Connection fixes (`connection-robustness` branch):

- Dead Unity clients are detected. Each Unity socket has a read timeout (`TCP_CLIENT_TIMEOUT`, default 10 s) and TCP keepalive. Unity sends a keepalive every second when idle, so silence means the Unity side is gone. A socket the editor never closed used to keep its thread alive forever.
- The endpoint sends keepalives to Unity: one after the handshake, then one after every second of silence. Upstream and fork connectors both ignore them. The fork connector uses them to notice a dead endpoint.
- The listen thread can't die anymore. Bind is retried, and accept or socket setup errors are logged and skipped.
- Ctrl+C works with Unity connected. Client threads are daemons, all sockets are closed (Unity sees the disconnect right away and starts reconnecting), nodes are destroyed and rclpy is shut down.
- Reconnecting doesn't churn nodes. If Unity re-registers a topic with the same type and settings, the existing node is kept.
- Unity service calls can't hang or crash the endpoint. They used to do one or the other when Unity was not connected, disconnected mid-call, or answered very fast. The executor also has at least 4 threads now instead of 1, so one waiting Unity service doesn't block every subscription.
- ROS service calls from Unity give up on shutdown or when that Unity connection closes. Unity always gets an answer, an empty response if the call failed. Responses for a closed connection are dropped instead of being sent to the next Unity session.
- Unknown sys commands, `__remove_*` commands (sent by `Unsubscribe()`, never implemented upstream) and bad messages no longer close the connection.

## Installing

Clone the branch into your workspace and build:

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws/src
git clone -b main-ros2-optimized https://github.com/kaidalisohaib/ROS-TCP-Endpoint.git ros-tcp-endpoint
cd ~/ros2_ws
colcon build --packages-select ros_tcp_endpoint
source install/setup.bash
ros2 launch ros_tcp_endpoint endpoint.py
```

Use `-b connection-robustness` until that branch is merged into `main-ros2-optimized`.

Or as a git submodule, pinned to a commit:

```bash
git submodule add -b main-ros2-optimized https://github.com/kaidalisohaib/ROS-TCP-Endpoint.git ros2_ws/src/ros-tcp-endpoint
cd ros2_ws/src/ros-tcp-endpoint && git checkout <commit> && cd -
git add .gitmodules ros2_ws/src/ros-tcp-endpoint
```

Parameters (all optional):

- `ROS_IP` (default `0.0.0.0`) and `ROS_TCP_PORT` (default `10000`), same as upstream.
- `TCP_CLIENT_TIMEOUT` (seconds, default `10.0`). How long a Unity connection may stay silent before it is closed. `0` turns it off. Raise it if you pause the editor with the upstream connector, or if you stop Unity at breakpoints.

```bash
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_TCP_PORT:=10000 -p TCP_CLIENT_TIMEOUT:=10.0
```

## Testing

`test/integration/scenarios.py` starts the endpoint, plays Unity with a small fake client (`fake_unity.py`), and checks publish, subscribe, both service directions, reconnects, half-open and silent clients, Ctrl+C and restart. Without ROS installed locally, run it in docker:

```bash
test/integration/run_in_docker.sh                                      # ros:humble-ros-base
test/integration/run_in_docker.sh mcgillrobotics/auv_2026:latest-amd64  # our image (Jazzy)
```

## Known limitations

- Publishers, subscribers and services stay registered after Unity disconnects, same as upstream. A subscription from an earlier Unity session keeps forwarding to the current one, which ignores topics it doesn't know.
- Only one Unity client gets subscription data: the most recent connection. Older connections can still publish.
- The send queue to Unity is unbounded, so a Unity side that reads too slowly makes it grow.
- A ROS service call from Unity whose server dies keeps its thread until Unity disconnects. There is no per-call timeout.
- A ROS caller of a Unity service gets an empty response if Unity isn't connected or disconnects mid-call. It can't tell that apart from a real empty response, but the endpoint logs an error.

---

*Original upstream README below.*

# ROS TCP Endpoint

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

## Introduction

[ROS](https://www.ros.org/) package used to create an endpoint to accept ROS messages sent from a Unity scene using the [ROS TCP Connector](https://github.com/Unity-Technologies/ROS-TCP-Connector) scripts.

Instructions and examples on how to use this ROS package can be found on the [Unity Robotics Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub/blob/master/tutorials/ros_unity_integration/README.md) repository.

## Community and Feedback

The Unity Robotics projects are open-source and we encourage and welcome contributions.
If you wish to contribute, be sure to review our [contribution guidelines](CONTRIBUTING.md)
and [code of conduct](CODE_OF_CONDUCT.md).

## Support
For questions or discussions about Unity Robotics package installations or how to best set up and integrate your robotics projects, please create a new thread on the [Unity Robotics forum](https://forum.unity.com/forums/robotics.623/) and make sure to include as much detail as possible.

For feature requests, bugs, or other issues, please file a [GitHub issue](https://github.com/Unity-Technologies/ROS-TCP-Endpoint/issues) using the provided templates and the Robotics team will investigate as soon as possible.

For any other questions or feedback, connect directly with the
Robotics team at [unity-robotics@unity3d.com](mailto:unity-robotics@unity3d.com).

## License
[Apache License 2.0](LICENSE)