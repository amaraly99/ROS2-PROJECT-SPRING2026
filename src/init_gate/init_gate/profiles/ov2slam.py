# OV2SLAM gate - vertical (up) parallax only. HANDOFF 2026-07-08: OV2SLAM inits on
# a vertical leg; the lateral left/right legs added ~2-3m gate-release drift that
# worsened the proportional controller's arced approach. (OV2SLAM configs currently
# DISABLE the gate; this profile is for when a gated OV2SLAM run is wanted.)
LEGS = [
    ("up", 0.0, 0.0, +1.0),
]
DEFAULTS = dict(leg_speed=1.2, leg_duration_sec=1.0, timeout_sec=30.0, ready_debounce=2)
