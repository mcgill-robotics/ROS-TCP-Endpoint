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

import rclpy
import re
import threading

from rclpy.serialization import deserialize_message

from .communication import RosSender


class RosService(RosSender):
    """
    Class to send messages to a ROS service.
    """

    def __init__(self, service, service_class):
        """
        Args:
            service:        The service name in ROS
            service_class:  The service class in catkin workspace
        """
        strippedService = re.sub("[^A-Za-z0-9_]+", "", service)
        node_name = f"{strippedService}_RosService"
        RosSender.__init__(self, node_name)

        self.service_topic = service
        self.cli = self.create_client(service_class, service)
        self.req = service_class.Request()
        self.service_class = service_class

    def send(self, data, should_abort=None):
        """
        Takes in serialized message data from source outside of the ROS network,
        deserializes it into it's class, calls the service with the message, and returns
        the service's response.

        Args:
            data: The already serialized message_class data coming from outside of ROS
            should_abort: Optional callable. While waiting for the response it is polled,
                and the call is abandoned when it returns True (e.g. Unity disconnected).

        Returns:
            service response, or None if the call failed or was abandoned
        """
        message_type = type(self.req)
        message = deserialize_message(data, message_type)

        if not self.cli.service_is_ready():
            self.get_logger().error(
                "Ignoring service call to {} - service is not ready.".format(self.service_topic)
            )
            return None

        # Local variable, not self.future: two Unity calls to the same service can be in
        # flight at once, each on its own thread.
        future = self.cli.call_async(message)

        # The response is delivered by the executor (another thread), which sets the event.
        # Wake up regularly so that this thread does not outlive rclpy.shutdown() or the
        # Unity connection that asked for the call.
        done_event = threading.Event()
        future.add_done_callback(lambda _: done_event.set())
        while not done_event.wait(0.5):
            if not rclpy.ok() or (should_abort is not None and should_abort()):
                self.cli.remove_pending_request(future)
                self.get_logger().warning(
                    "Abandoned call to {} (shutdown or Unity disconnected)".format(
                        self.service_topic
                    )
                )
                return None

        try:
            return future.result()
        except Exception as e:
            self.get_logger().info(f"Service call failed {e}")
            return None

    def unregister(self):
        """

        Returns:

        """
        self.destroy_client(self.cli)
        self.destroy_node()
