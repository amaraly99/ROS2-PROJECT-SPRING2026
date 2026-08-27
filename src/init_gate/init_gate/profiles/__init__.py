# init_gate profiles - one warmup CYCLE per SLAM, swappable via config
# (init_gate.module). Each profile module exposes:
#   LEGS     : list of (name, ux, uy, uz) UNIT body-frame directions; cycle.py
#              scales each by leg_speed and mirrors it (out + back).
#   DEFAULTS : dict(leg_speed, leg_duration_sec, timeout_sec, ready_debounce),
#              overridable per-run from config init_gate.params.*
# Add a SLAM's gate = drop a new <name>.py here, set init_gate.module: <name>.
import importlib


def load(module_name):
    """Return the profile module for init_gate.module=<name>; fall back to
    'default' on a typo/missing file (never crash the gate)."""
    name = module_name or "default"
    try:
        return importlib.import_module("init_gate.profiles." + name)
    except ModuleNotFoundError:
        return importlib.import_module("init_gate.profiles.default")
