# Every tunable in one place -- the only file you touch to retune the gate.

LEG_SPEED = 1.2            # m/s -- bumped above search_strafe_speed's 0.6 default (config/hil/bench_fsm.yaml)
                           # to build gate parallax faster during warmup
LEG_DURATION_SEC = 1.0     # seconds per leg (out, then same again to mirror back) -> 0.6m/leg
READY_DEBOUNCE = 2          # consecutive tracking_state==OK reads, mirrors lockon_consec pattern
TIMEOUT_SEC = 30.0
