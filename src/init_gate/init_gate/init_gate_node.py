# init_gate_node.py -- thin ROS2 wiring only. Which warmup CYCLE runs is now
# SWAPPABLE: init_gate.module (ROS param) picks a profile in init_gate/profiles/,
# and init_gate.params.* (ROS params) override that profile's scalar DEFAULTS.
# Both are supplied by run_stack_hil.sh via --ros-args -p ... from the config.
# Policy is in cycle.py, SLAM readiness in slam_readiness.py, actuation in motion.py.

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from init_gate.slam_readiness import SlamReadiness
from init_gate.motion import MotionCommander
from init_gate.cycle import run_cycle
from init_gate.profiles import load as load_profile

# Named OUTBOUND unit directions for init_gate.params.legs (body frame; matches
# servo_core convention: vx forward(+), vy LEFT(+), vz UP(+)). Add a name here to
# make it selectable from config -- no other file needs to change.
NAMED_DIRECTIONS = {
    "left":     (0.0, +1.0, 0.0),
    "right":    (0.0, -1.0, 0.0),
    "up":       (0.0,  0.0, +1.0),
    "up_slow":  (0.0,  0.0, +0.25),  # gentle climb -- same leg_speed/duration as
                                     # other legs, but a smaller unit vector so the
                                     # effective vertical rate is a quarter of whatever
                                     # leg_speed is configured (all legs share one
                                     # scalar speed today; this is how we get 'slow'
                                     # for just one leg without a bigger param change).
    "down":     (0.0,  0.0, -1.0),
    "forward":  (+1.0, 0.0, 0.0),
    "backward": (-1.0, 0.0, 0.0),
}


class InitGateNode(Node):
    def __init__(self):
        super().__init__("init_gate_node")

        # module picks the profile; the 4 scalars default to <0 = "use profile default".
        self.declare_parameter("module", "default")
        self.declare_parameter("legs", "")  # comma-joined names, e.g. "left,right,up"
        self.declare_parameter("leg_speed", -1.0)
        self.declare_parameter("leg_duration_sec", -1.0)
        self.declare_parameter("timeout_sec", -1.0)
        self.declare_parameter("ready_debounce", -1)

        module = self.get_parameter("module").value
        self.profile = load_profile(module)
        d = self.profile.DEFAULTS

        # Config-level leg override (init_gate.params.legs). Named OUTBOUND directions
        # only -- cycle.py auto-mirrors each for the return leg (see cycle.py), so
        # legs: [up] already includes its own "come back down". Unknown names are
        # logged and skipped; if that empties the list, fall back to profile.LEGS
        # entirely -- the gate must never run with zero legs.
        legs_override = self.get_parameter("legs").value
        self.legs = self.profile.LEGS
        if legs_override:
            picked = []
            for name in (n.strip() for n in legs_override.split(",") if n.strip()):
                vec = NAMED_DIRECTIONS.get(name)
                if vec is None:
                    self.get_logger().warn(
                        f"[init_gate] unknown leg name {name!r} in init_gate.params.legs "
                        f"(known: {sorted(NAMED_DIRECTIONS)}) -- skipped")
                    continue
                picked.append((name,) + vec)
            if picked:
                self.legs = picked
            else:
                self.get_logger().warn(
                    "[init_gate] init_gate.params.legs had no valid names -- "
                    f"falling back to profile {module!r}'s own legs")

        def pick(name, default, cast):
            v = self.get_parameter(name).value
            try:
                return cast(v) if v is not None and float(v) >= 0 else default
            except (TypeError, ValueError):
                return default

        self.leg_speed        = pick("leg_speed",        d["leg_speed"],        float)
        self.leg_duration_sec = pick("leg_duration_sec", d["leg_duration_sec"], float)
        self.timeout_sec      = pick("timeout_sec",      d["timeout_sec"],      float)
        self.ready_debounce   = pick("ready_debounce",   d["ready_debounce"],   int)

        pub_cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.pub_bench = self.create_publisher(String, "/bench/state", 10)
        self.readiness = SlamReadiness(self, self.ready_debounce)
        self.motion = MotionCommander(self, pub_cmd)

        self.get_logger().info(
            "[init_gate] module=%r legs=%s leg_speed=%s leg_dur=%ss timeout=%ss debounce=%s"
            % (module, [l[0] for l in self.legs], self.leg_speed,
               self.leg_duration_sec, self.timeout_sec, self.ready_debounce))

    def log_state(self, s):
        msg = String()
        msg.data = s
        self.pub_bench.publish(msg)
        self.get_logger().info(f"[init_gate] {s}")

    def run(self):
        return run_cycle(self.motion, self.readiness, self.log_state,
                         self.legs, self.leg_speed,
                         self.leg_duration_sec, self.timeout_sec)


def main():
    rclpy.init()
    node = InitGateNode()
    ok = node.run()
    rclpy.shutdown()
    exit(0 if ok else 1)


if __name__ == "__main__":
    main()
