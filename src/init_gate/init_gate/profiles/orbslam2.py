# ORBSLAM2 gate - left/right/up translation parallax (ORB-SLAM2 triangulates its
# first map from real translation; see orbslam2_eval.yaml). Explicit copy of the
# 'default' values = the motion current ORBSLAM2 runs use. Phase 4 will confirm the
# Julien-exact leg_speed (HANDOFF notes 0.6 for OV2SLAM's gate vs the 1.2 here) -
# tune it per-run via config init_gate.params.leg_speed, no code change needed.
LEGS = [
    ("left",  0.0, +1.0, 0.0),
    ("right", 0.0, -1.0, 0.0),
    ("up",    0.0,  0.0, +1.0),
]
DEFAULTS = dict(leg_speed=1.2, leg_duration_sec=1.0, timeout_sec=30.0, ready_debounce=2)
