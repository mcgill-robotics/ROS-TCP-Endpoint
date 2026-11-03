# Changelog

All notable changes to this repository will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/en/1.0.0/) and this project adheres to [Semantic Versioning](http://semver.org/spec/v2.0.0.html).

## Unreleased

Changes in the McGill Robotics fork. See the top of README.md for the reasons.

### Upgrade Notes

New ROS parameter `TCP_CLIENT_TIMEOUT` (seconds, default 10, 0 disables). A Unity connection that sends nothing for that long is closed. The Unity connector sends a keepalive every second when idle (and the fork keeps sending it while the editor is paused). Raise this if you use the upstream connector and pause the editor, or if you stop Unity at breakpoints.

Use together with the connector fork (kaidalisohaib/ROS-TCP-Connector, branch `main-ros2-optimized`). Either side still works with the upstream version of the other.

### Known Issues

- Registered publishers, subscribers and services are kept when Unity disconnects, same as upstream. A subscription registered by an earlier Unity session keeps forwarding to the current one, which ignores topics it doesn't know.
- The per-connection send queue is unbounded. A Unity side that reads too slowly makes it grow.

### Added

Added Sonarqube scanner

- `TCP_CLIENT_TIMEOUT` parameter (read timeout on Unity sockets), also in `launch/endpoint.py`.
- TCP keepalive on Unity sockets (5 s idle, 2 s interval, 3 probes on Linux).
- The endpoint sends an 8-byte keepalive (empty destination, same format as Unity's) right after the handshake and after every second with nothing to send. Unity ignores it.
- Handlers for `__remove_subscriber`, `__remove_publisher`, `__remove_ros_service` and `__remove_unity_service`, which Unity sends from `Unsubscribe()` and friends.
- `test/integration/`: a fake Unity client and a scenario script that runs against a live endpoint (`run_in_docker.sh` uses `ros:humble-ros-base`).

### Changed

- `TCP_NODELAY` on accepted sockets.
- `buffer_size` now defaults to 4 MB (4194304) and is applied as `SO_RCVBUF`/`SO_SNDBUF`, on the listening socket so accepted sockets inherit it. Linux caps it at `net.core.rmem_max`/`wmem_max` and doubles the result, so with the default 212992 you get 425984 bytes, not 4 MB. It is not used for recv chunk sizes.
- Subscriptions use `raw=True`. The callback gets the serialized bytes and the endpoint forwards them without deserializing and re-serializing. The bytes are still copied once to add the destination and length header.
- Re-registering a topic with the same type (and the same queue size and latch for publishers) keeps the existing node. Unity re-sends every registration on reconnect, and before, each reconnect destroyed and re-created every node.
- The executor has at least max(cpu_count, 4) threads. It was sized from the empty tables at startup, which gave 1 thread.
- A ROS service call from Unity that fails now gets an empty response, so the Unity caller doesn't wait forever.
- A Unity service that gets no answer (Unity not connected or disconnected) returns an empty response to the ROS caller and logs an error.

### Removed

- The summer change that made `recvall` return a `bytearray`. See Fixed.

### Fixed

- Unity -> ROS publishing and Unity -> ROS service calls. `recvall` returned a `bytearray`, which `Publisher.publish()` and `deserialize_message()` reject with TypeError. The first publish killed the connection.
- `service.py`: replaced the busy-wait on the service future with a `threading.Event`. The wait now wakes every 0.5 s and gives up on `rclpy.shutdown()` or when the calling Unity connection closes. Concurrent calls to the same service no longer share `self.future`.
- A ROS service response is dropped if the Unity connection that asked for it is gone, instead of going to the next Unity session.
- A Unity service call while Unity was not connected returned None, which made rclpy raise TypeError in `executor.spin()` and stopped the whole endpoint.
- `ThreadPauser` lost the wakeup when Unity answered before the executor thread started waiting, which blocked that thread forever. It uses an Event now.
- Unity service requests pending on a connection are released when that connection closes.
- Unknown sys commands, unknown service ids and any other error while handling one message no longer end the connection. They are logged and reported to Unity.
- A stale `__request`/`__response` header from a dead connection could swallow the next session's first message.
- A Unity socket that was never closed (editor leaving play mode, frozen Unity) kept a ClientThread blocked forever. The read timeout and TCP keepalive now close it.
- The listen thread could die on a bind or accept error, leaving an endpoint that never accepted again. Bind is retried and accept errors are logged.
- Ctrl+C hung while a Unity client was connected (non-daemon ClientThread), and skipped `destroy_nodes()`. The endpoint now closes all sockets and shuts down cleanly.
- Nodes are removed from the executor before being destroyed.

## [0.7.0] - 2022-02-01

### Added

Added Sonarqube scanner

Send information during hand shaking for ros and package version checks

Send service response as one queue item


## [0.6.0] - 2021-09-30

Add the [Close Stale Issues](https://github.com/marketplace/actions/close-stale-issues) action

### Upgrade Notes

### Known Issues

### Added

Support for queue_size and latch for publishers. (https://github.com/Unity-Technologies/ROS-TCP-Endpoint/issues/82)

### Changed

### Deprecated

### Removed

### Fixed

## [0.5.0] - 2021-07-15

### Upgrade Notes

Upgrade the ROS communication to support ROS2 with Unity

### Known Issues

### Added

### Changed

### Deprecated

### Removed

### Fixed

## [0.4.0] - 2021-05-27

Note: the logs only reflects the changes from version 0.3.0

### Upgrade Notes

RosConnection 2.0: maintain a single constant connection from Unity to the Endpoint. This is more efficient than opening one connection per message, and it eliminates a whole bunch of user issues caused by ROS being unable to connect to Unity due to firewalls, proxies, etc.

### Known Issues

### Added

Add a link to the Robotics forum, and add a config.yml to add a link in the Github Issues page

Add linter, unit tests, and test coverage reporting

### Changed

Improving the performance of the read_message in client.py, This is done by receiving the entire message all at once instead of reading 1024 byte chunks and stitching them together as you go.

### Deprecated

### Removed

Remove outdated handshake references

### Fixed
