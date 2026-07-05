# slam_readiness.py -- debounced tracking-state check. Backend-agnostic: only
# depends on the /slam/tracking_state contract (std_msgs/Int32, 2=ready), not
# on which SLAM backend publishes it.
#
# ORB-SLAM2 (monocular-slam-node.cpp) republishes its own tracking_state enum
# verbatim: -1=SYSTEM_NOT_READY, 0=NO_IMAGES_YET, 1=NOT_INITIALIZED, 2=OK, 3=LOST.
# OV2SLAM (ov2slam_node.cpp) has no equivalent enum, so its wrapper republishes
# its bvision_init_ flag as 1=not-ready / 2=ready instead -- coarser (no
# LOST/RELOCALIZING), but sufficient since only the ready value is checked below.

from std_msgs.msg import Int32

TRACKING_OK = 2   # ORB_SLAM2::Tracking::OK / OV2SLAM bvision_init_==true


class SlamReadiness:
    def __init__(self, node, debounce):
        self.debounce = debounce
        self._consec_ok = 0
        node.create_subscription(Int32, '/slam/tracking_state', self._on_state, 10)


    def _on_state(self, msg):
        self._consec_ok = self._consec_ok + 1 if msg.data == TRACKING_OK else 0

    def ready(self):
        return self._consec_ok >= self.debounce
