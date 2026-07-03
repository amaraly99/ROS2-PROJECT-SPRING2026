# slam_readiness.py -- debounced ORB-SLAM2 tracking-state check.
#
# ORB-SLAM2 only (see monocular-slam-node.cpp): tracking_state values are
# -1=SYSTEM_NOT_READY, 0=NO_IMAGES_YET, 1=NOT_INITIALIZED, 2=OK, 3=LOST.
# OV2SLAM never publishes this topic at all -- this gate is scoped to
# ORB-SLAM2 for that reason (see run_stack_hil.sh's init_gate guard).

from std_msgs.msg import Int32

TRACKING_OK = 2   # ORB_SLAM2::Tracking::OK


class SlamReadiness:
    def __init__(self, node, debounce):
        self.debounce = debounce
        self._consec_ok = 0
        node.create_subscription(Int32, '/slam/tracking_state', self._on_state, 10)

    def _on_state(self, msg):
        self._consec_ok = self._consec_ok + 1 if msg.data == TRACKING_OK else 0

    def ready(self):
        return self._consec_ok >= self.debounce
