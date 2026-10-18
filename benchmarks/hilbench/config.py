"""config.py — the only module that reads the YAML file and the command line.

It knows the COMMON settings only. A kind declares its own settings as a list of FieldSpec
(RunKind.fields()); load() stores them in Settings.kind_cfg and never reads one of their keys.

Merge order (later wins): built-in default -> YAML file -> command line.
YAML layout: the common sections `session`, `matlab`, `network`, `pi`, plus one section named
like the kind for the kind's own fields.

CONTRACT (fixed): FieldSpec, COMMON_FIELDS, Settings. The functions below them are filled in
by the harness implementation (see CONTRACT.md).
"""
import argparse
import hashlib
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Optional, Sequence

from .util import WS_ROOT


@dataclass(frozen=True)
class FieldSpec:
    """One setting.

    name     attribute name on Settings (common) or key in Settings.kind_cfg (kind-owned)
    section  YAML section it lives in ("session", "matlab", "network", "pi", or the kind name)
    key      YAML key inside that section (defaults to name)
    default  built-in default; None + required=True means the user must give it
    options  allowed values for a text field (None = free value)
    required the value must not be None after the merge
    help     one line for --help
    cast     callable applied to a command-line string (int, float, str, parse_bool, ...)
    cli      command-line option, e.g. "--runs" (defaults to "--" + name with "_" -> "-")
    """
    name: str
    section: str
    default: Any = None
    options: Optional[Sequence[str]] = None
    required: bool = False
    help: str = ""
    cast: Callable[[str], Any] = str
    key: Optional[str] = None
    cli: Optional[str] = None

    @property
    def yaml_key(self):
        return self.key or self.name

    @property
    def cli_flag(self):
        return self.cli or "--" + self.name.replace("_", "-")


def parse_bool(text):
    """'true'/'false' (any case) -> bool. Anything else raises ValueError."""
    t = str(text).strip().lower()
    if t in ("true", "1", "yes"):
        return True
    if t in ("false", "0", "no"):
        return False
    raise ValueError(f"not a boolean: {text!r}")


# The common settings of every kind. Defaults are the values of the first benchmark file
# (Option A for the Pi), plus the approved new ones (pace guard).
COMMON_FIELDS = (
    FieldSpec("tag", "session", None, required=True, help="session name; the runner never invents one"),
    FieldSpec("runs", "session", 10, cast=int, help="runs per condition"),
    FieldSpec("cooldown_s", "session", 10.0, cast=float, help="pause after each run, s"),
    FieldSpec("pace_guard", "session", "stop", options=("stop", "warn", "off"), help="action when the heartbeat falls"),
    FieldSpec("pace_min_ratio", "session", 0.85, cast=float, help="limit = ratio x heartbeat of run 1"),
    FieldSpec("host", "matlab", "192.168.1.201", help="MATLAB supervisor host"),
    FieldSpec("port", "matlab", 55556, cast=int, help="MATLAB supervisor TCP port"),
    FieldSpec("windows_user", "matlab", "homie", help="ssh user for the Windows Wi-Fi read"),
    FieldSpec("wait_s", "matlab", 180.0, cast=float, cli="--matlab-wait", help="wait for MATLAB, then stop"),
    FieldSpec("hold_timeout_s", "matlab", 45.0, cast=float, cli="--hold-timeout", help="start_hold must reach 'held'"),
    FieldSpec("dds", "network", "fastrtps", options=("fastrtps", "cyclonedds"), help="must match MATLAB"),
    FieldSpec("ros_domain_id", "network", 0, cast=int, cli="--domain"),
    FieldSpec("wifi_min_pct", "network", 90.0, cast=float, cli="--wifi-min", help="minimum Wi-Fi % on both machines"),
    FieldSpec("check_windows_wifi", "network", True, cast=parse_bool, help="false is logged as a deviation"),
    FieldSpec("boot_timeout_s", "pi", 60.0, cast=float, cli="--boot-timeout"),
    FieldSpec("controller_cores", "pi", "none", help='none or a core list: "3", "2,3", "0-2"'),
    FieldSpec("runner_cores", "pi", "none", help='none or a core list: "0-2"'),
    FieldSpec("cpu_governor", "pi", "leave", options=("leave", "performance")),
    FieldSpec("net_buffers", "pi", "leave", options=("leave", "hil")),
)


@dataclass(frozen=True)
class Settings:
    """The frozen result of load(). Common fields are attributes; kind fields are in kind_cfg."""
    kind: str
    tag: str
    runs: int
    cooldown_s: float
    pace_guard: str
    pace_min_ratio: float
    host: str
    port: int
    windows_user: str
    wait_s: float
    hold_timeout_s: float
    dds: str
    ros_domain_id: int
    wifi_min_pct: float
    check_windows_wifi: bool
    boot_timeout_s: float
    controller_cores: str
    runner_cores: str
    cpu_governor: str
    net_buffers: str
    kind_cfg: dict = field(default_factory=dict)   # kind-owned values; core never reads a key
    dry_run: bool = False
    config_path: Optional[str] = None              # the YAML file used (None = not found)
    overrides: tuple = ()                          # names where the command line differs from the YAML
    problems: tuple = ()                           # filled only for reporting; load() exits on problems


# ── the functions (CONTRACT.md, config.py section) ──
_CORE_LIST = re.compile(r"^\d+(-\d+)?(,\d+(-\d+)?)*$")


def _is_core_spec(v):
    """True for the text 'none' or a core list: 3, 2,3, 0-2 (a range must not run backwards)."""
    if not isinstance(v, str):
        return False
    if v == "none":
        return True
    if not _CORE_LIST.match(v):
        return False
    for part in v.split(","):
        a, _, b = part.partition("-")
        if b and int(b) < int(a):
            return False
    return True


def _help(spec):
    text = spec.help or spec.name
    if spec.options:
        text += " [" + " | ".join(spec.options) + "]"
    return (text + f" (default: {spec.default})").replace("%", "%%")


def build_parser(kind_name, extra_fields):
    """One option per common field and per kind field, plus --config and --dry-run.

    No parser defaults (all None), so 'given on the command line' can be detected."""
    ap = argparse.ArgumentParser(prog=kind_name, description=f"HIL benchmark: {kind_name}")
    for spec in list(COMMON_FIELDS) + list(extra_fields):
        ap.add_argument(spec.cli_flag, dest=spec.name, type=spec.cast, choices=spec.options,
                        default=None, help=_help(spec))
    ap.add_argument("--skip-windows-wifi", action="store_true", default=None,
                    help="same as check_windows_wifi false (logged as a deviation)")
    ap.add_argument("--config", default=None, help="settings file (default config/hil/<kind>_bench.yaml)")
    ap.add_argument("--dry-run", action="store_true", help="preflight + status only; sends nothing that changes MATLAB")
    return ap


def _fix_yaml_value(spec, v, problems):
    """YAML value -> the type of the field. PyYAML reads bare off/on as False/True: map them back."""
    if v is None:
        return None
    if spec.cast is str:
        if isinstance(v, bool):
            return "on" if v else "off"
        return str(v)
    try:
        return spec.cast(v)
    except (ValueError, TypeError) as e:
        problems.append(f"{spec.section}.{spec.yaml_key}: {v!r} is not valid ({e})")
        return None


def _kind_problems(fields, values):
    """Generic checks of a kind's own fields: required and options."""
    out = []
    for spec in fields:
        v = values.get(spec.name)
        if v is None:
            if spec.required:
                out.append(f"{spec.section}.{spec.yaml_key} is required (option {spec.cli_flag})")
        elif spec.options and v not in spec.options:
            out.append(f"{spec.section}.{spec.yaml_key}: {v!r} is not one of {', '.join(map(str, spec.options))}")
    return out


def load(argv, kind_cls, config_path=None):
    import yaml
    fields = list(kind_cls.fields())
    ap = build_parser(kind_cls.name, fields)
    args = ap.parse_args(argv)

    # 1. built-in defaults
    values = {s.name: s.default for s in list(COMMON_FIELDS) + fields}

    # 2. the YAML file
    path = Path(args.config or config_path or WS_ROOT / "config" / "hil" / f"{kind_cls.name}_bench.yaml")
    cfg_found = path.is_file()
    cfg = (yaml.safe_load(path.read_text()) or {}) if cfg_found else {}
    problems, yaml_values = [], {}
    for spec in list(COMMON_FIELDS) + fields:
        section = cfg.get(spec.section) or {}
        if spec.yaml_key in section:
            v = _fix_yaml_value(spec, section[spec.yaml_key], problems)     # step 3: bool fix
            yaml_values[spec.name] = v
            values[spec.name] = v

    # 4. the command line wins; 5. overrides = where it differs from the YAML
    overrides = []
    for spec in list(COMMON_FIELDS) + fields:
        cli = getattr(args, spec.name, None)
        if spec.name == "check_windows_wifi" and args.skip_windows_wifi:
            cli = False
        if cli is None:
            continue
        if values[spec.name] not in (None, cli):      # the value from the YAML file, else the built-in default
            overrides.append(spec.name)
        values[spec.name] = cli

    # 6. no normalisation: an empty value is a problem, it never becomes "none"
    common = {s.name: values[s.name] for s in COMMON_FIELDS}
    kind_cfg = {s.name: values[s.name] for s in fields}
    s = Settings(kind=kind_cls.name, **common, kind_cfg=kind_cfg, dry_run=bool(args.dry_run),
                 config_path=str(path) if cfg_found else None, overrides=tuple(overrides))

    # 7. problems
    problems += validate(s) + _kind_problems(fields, kind_cfg) + list(kind_cls.validate(s))
    if problems:
        for p in problems:
            print(f"settings problem: {p}", file=sys.stderr)
        sys.exit(2)
    return s


def validate(settings):
    """Problems of the common fields only (the kind checks its own)."""
    out = []
    if not settings.tag or not str(settings.tag).strip():
        out.append("tag is empty: give session.tag in the settings file or --tag")
    for spec in COMMON_FIELDS:
        v = getattr(settings, spec.name)
        if v is None and spec.name != "tag":
            out.append(f"{spec.section}.{spec.yaml_key} is empty: give a value")
        elif spec.options and v is not None and v not in spec.options:
            out.append(f"{spec.section}.{spec.yaml_key}: {v!r} is not one of {', '.join(map(str, spec.options))}")
    if settings.wifi_min_pct is not None and not 0 <= settings.wifi_min_pct <= 100:
        out.append(f"network.wifi_min_pct: {settings.wifi_min_pct} is outside 0..100")
    if settings.runs is not None and settings.runs < 1:
        out.append(f"session.runs: {settings.runs} must be 1 or more")
    for name in ("controller_cores", "runner_cores"):
        v = getattr(settings, name)
        if v is not None and not _is_core_spec(v):
            out.append(f'pi.{name}: {v!r} is not the text "none" or a core list like "3", "2,3", "0-2"')
    return out


def as_snapshot(settings):
    """Effective value of every common field, the kind values, overrides, the YAML path and its sha256."""
    snap = {s.name: getattr(settings, s.name) for s in COMMON_FIELDS}
    snap.update(kind=settings.kind, kind_cfg=dict(settings.kind_cfg), overrides=list(settings.overrides),
                dry_run=settings.dry_run, config_path=settings.config_path)
    snap["config_sha256"] = (hashlib.sha256(Path(settings.config_path).read_bytes()).hexdigest()
                             if settings.config_path else None)
    return snap
