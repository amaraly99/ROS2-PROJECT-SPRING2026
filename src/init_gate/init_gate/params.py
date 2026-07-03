# Every tunable in one place -- the only file you touch to retune the gate.

LEG_SPEED = 0.6            # m/s -- matches search_strafe_speed default (config/hil/bench_fsm.yaml)
LEG_DURATION_SEC = 1.0      # was 3.0 (1.8m/leg) -- too far, shrunk to 0.6m/leg. Retune freely.
READY_DEBOUNCE = 2          # consecutive tracking_state==OK reads, mirrors lockon_consec pattern
TIMEOUT_SEC = 30.0
SAFETY_MIN_ALTITUDE = 0.5   # matches bench_fsm.yaml's safety_min_altitude (cheap extra clamp)
