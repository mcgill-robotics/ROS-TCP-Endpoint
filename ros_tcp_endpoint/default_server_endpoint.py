#!/usr/bin/env python

import rclpy

from ros_tcp_endpoint import TcpServer

try:
    from rclpy.executors import ExternalShutdownException
except ImportError:  # older than Galactic

    class ExternalShutdownException(Exception):
        pass


def main(args=None):
    rclpy.init(args=args)
    tcp_server = TcpServer("UnityEndpoint")

    tcp_server.start()

    try:
        tcp_server.setup_executor()
    except (KeyboardInterrupt, ExternalShutdownException):
        # Ctrl+C. Before, this skipped the cleanup below and printed a traceback.
        pass
    finally:
        # Close the listening socket and every Unity connection first, so Unity sees the
        # endpoint go away immediately and reconnects to the next one.
        tcp_server.shutdown()
        tcp_server.destroy_nodes()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
