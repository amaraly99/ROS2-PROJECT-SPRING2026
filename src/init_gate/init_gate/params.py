# Every tunable in one place -- the only file you touch to retune the gate.

LEG_SPEED = 0.6            # m/s -- matches search_strafe_speed default (config/hil/bench_fsm.yaml)
LEG_DURATION_SEC = 1.0     # seconds per leg (out, then same again to mirror back) -> 0.6m/leg
READY_DEBOUNCE = 2          # consecutive tracking_state==OK reads, mirrors lockon_consec pattern
TIMEOUT_SEC = 30.0
