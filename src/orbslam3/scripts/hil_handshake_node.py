#!/usr/bin/env python3
"""Drive the ORB-SLAM3 wrapper's config handshake for LIVE camera operation.

Upstream ros2_orb_slam3 is built for offline dataset playback: mono_driver_node.py
reads JPEGs off disk, performs a config handshake with the C++ node, then streams
frames itself. In HIL there is no folder of images -- frames arrive live on
/ovcam/image_raw -- but the C++ node still refuses to process anything until the
handshake completes:

    void MonocularMode::Img_callback(...)
    {
        if (!initialized_ || pAgent == nullptr) { return; }   // <-- silently drops every frame

The initialized_ flag is set only by experimentSetting_callback, which fires on a String
published to <prefix>/experiment_settings naming the settings YAML. So all this
node has to do is send that String, wait for the ack, and get out of the way --
the C++ node's own image subscription (remapped to the live camera in the stack
config) then does the rest.

Deliberately does NOT stream images: doing so would add a Python hop to the frame
path and make ORB-SLAM3's measured latency incomparable with OV2SLAM and
ORB-SLAM2, which both consume the camera topic directly.
"""

import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class HilHandshake(Node):
    def __init__(self):
        super().__init__('hil_handshake_node')
        self.declare_parameter('settings_name', 'HIL_SIM')
        self.declare_parameter('topic_prefix', '/mono_py_driver')
        self.declare_parameter('timeout_sec', 120.0)
        self.declare_parameter('resend_period_sec', 1.0)

        self.settings = str(self.get_parameter('settings_name').value)
        prefix = str(self.get_parameter('topic_prefix').value)
        self.timeout = float(self.get_parameter('timeout_sec').value)
        period = float(self.get_parameter('resend_period_sec').value)

        self.acked = False
        self.elapsed = 0.0

        self.pub = self.create_publisher(String, prefix + '/experiment_settings', 1)
        self.create_subscription(String, prefix + '/exp_settings_ack', self._ack, 10)
        # Resend periodically rather than once: the C++ node may still be loading
        # its ~150 MB vocabulary when we come up, and a single publish on a fresh
        # topic can be missed entirely before its subscription is matched.
        self.timer = self.create_timer(period, self._tick)
        self.get_logger().info(
            "handshake: sending settings '%s' to %s/experiment_settings"
            % (self.settings, prefix))

    def _ack(self, msg):
        if not self.acked:
            self.acked = True
            self.get_logger().info('handshake ACKed (%s) -- ORB-SLAM3 will now '
                                   'process live frames' % msg.data.strip())

    def _tick(self):
        if self.acked:
            self.timer.cancel()
            # Stay alive: exiting would tear down the node, and rclpy shutdown in
            # a composed launch can take the whole process with it.
            self.timer = self.create_timer(3600.0, lambda: None)
            return
        self.elapsed += float(self.get_parameter('resend_period_sec').value)
        if self.elapsed > self.timeout:
            self.get_logger().error('no ack after %.0f s -- ORB-SLAM3 never '
                                    'initialised' % self.timeout)
            raise SystemExit(1)
        m = String()
        m.data = self.settings
        self.pub.publish(m)


def main():
    rclpy.init(args=sys.argv)
    node = HilHandshake()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
