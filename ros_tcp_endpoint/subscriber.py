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
import socket
import re
import threading

from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy, QoSProfile
from rclpy.event_handler import SubscriptionEventCallbacks

from .communication import RosReceiver


class RosSubscriber(RosReceiver):
    """
    Class to send messages outside of ROS network with automatic QoS adaptation.
    """

    def __init__(self, topic, message_class, tcp_server, queue_size=10):
        """
        Args:
            topic:         Topic name to publish messages to
            message_class: The message class in catkin workspace
            queue_size:    Max number of entries to maintain in an outgoing queue
        """
        strippedTopic = re.sub("[^A-Za-z0-9_]+", "", topic)
        self.node_name = f"{strippedTopic}_RosSubscriber"
        RosReceiver.__init__(self, self.node_name)
        self.topic = topic
        self.msg = message_class
        self.tcp_server = tcp_server
        self.queue_size = queue_size
        self._lock = threading.Lock()
        self._is_best_effort = False

        # Pre-check: If an active publisher on the topic is already BEST_EFFORT, start as BEST_EFFORT
        reliability = QoSReliabilityPolicy.RELIABLE
        try:
            publishers_info = self.get_publishers_info_by_topic(self.topic)
            if any(p.qos_profile.reliability == QoSReliabilityPolicy.BEST_EFFORT for p in publishers_info):
                reliability = QoSReliabilityPolicy.BEST_EFFORT
                self._is_best_effort = True
                self.get_logger().info(
                    f"Discovered BEST_EFFORT publisher on {self.topic}; subscribing with BEST_EFFORT QoS."
                )
        except Exception as e:
            self.get_logger().debug(f"Could not query publisher info for {self.topic}: {e}")

        self.subscription = self._create_subscription_with_reliability(reliability)

    def _on_incompatible_qos(self, event):
        """
        DDS callback triggered when an incompatible publisher is discovered.
        Dynamically upgrades the subscription to BEST_EFFORT if a publisher offered BEST_EFFORT.
        """
        with self._lock:
            if not self._is_best_effort:
                self.get_logger().warn(
                    f"Incompatible QoS detected on topic '{self.topic}' (publisher offered BEST_EFFORT). "
                    f"Dynamically switching subscription to BEST_EFFORT."
                )
                self._is_best_effort = True
                try:
                    if self.subscription is not None:
                        self.destroy_subscription(self.subscription)
                    self.subscription = self._create_subscription_with_reliability(QoSReliabilityPolicy.BEST_EFFORT)
                except Exception as e:
                    self.get_logger().error(f"Failed to recreate subscription for {self.topic} as BEST_EFFORT: {e}")

    def _create_subscription_with_reliability(self, reliability):
        qos_profile = QoSProfile(
            depth=self.queue_size,
            reliability=reliability
        )
        callbacks = SubscriptionEventCallbacks(incompatible_qos=self._on_incompatible_qos)
        return self.create_subscription(
            self.msg,
            self.topic,
            self.send,
            qos_profile,
            event_callbacks=callbacks,
            raw=True
        )

    def send(self, data):
        """
        Connect to TCP endpoint on client and pass along message
        Args:
            data: message data to send outside of ROS network

        Returns:
            self.msg: The deserialize message
        """
        self.tcp_server.send_unity_message(self.topic, data)
        return self.msg

    def unregister(self):
        """
        Clean up subscription and destroy node.
        """
        with self._lock:
            if self.subscription is not None:
                self.destroy_subscription(self.subscription)
                self.subscription = None
            self.destroy_node()
