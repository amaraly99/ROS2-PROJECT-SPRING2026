# pose_tracker.py -- ground-truth position, anchored once.
#
# Every "return to center" call in motion.py anchors back to the ONE start pose
# recorded here, not to wherever the drone happens to be relative to its last
# move. That's what keeps ramp-lag drift from compounding cycle over cycle --
# leg 5's return always targets the same fixed point leg 1's did.

from std_msgs.msg import Float64MultiArray


class PoseTracker:
    def __init__(self, node):
        self.x = self.y = self.z = 0.0
        self.start = None   # (x, y, z), set once on the first message
        node.create_subscription(Float64MultiArray, '/sim/drone_pose', self._on_pose, 10)

    def _on_pose(self, msg):
        self.x, self.y, self.z = msg.data[0], msg.data[1], msg.data[2]
        if self.start is None:
            self.start = (self.x, self.y, self.z)

    def has_pose(self):
        return self.start is not None

    def delta_from_start(self, axis):
        idx = {'y': 1, 'z': 2}[axis]
        current = {'y': self.y, 'z': self.z}[axis]
        return self.start[idx] - current
