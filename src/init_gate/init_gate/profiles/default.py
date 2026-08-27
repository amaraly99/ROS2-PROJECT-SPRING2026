# default gate - left/right/up translation legs (generic parallax warmup).
# Preserves the pre-Phase-2 init_gate behavior EXACTLY (old params.py:
# LEG_SPEED=1.2, LEG_DURATION_SEC=1.0, TIMEOUT_SEC=30.0, READY_DEBOUNCE=2).
LEGS = [
    ("left",  0.0, +1.0, 0.0),
    ("right", 0.0, -1.0, 0.0),
    ("up",    0.0,  0.0, +1.0),
]
DEFAULTS = dict(leg_speed=1.2, leg_duration_sec=1.0, timeout_sec=30.0, ready_debounce=2)
