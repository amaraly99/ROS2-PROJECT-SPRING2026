"""model_check.py — the Pi is the source of truth for the Simulink models.

config/hil/models.yaml says what each model must contain. MATLAB reports what is loaded
(Supervisor.model_info). The runner compares both before run 1. No hash is used, because
every .slx save writes a new hash (SIM_PARAMS.md): the parameters are compared.
"""
import math
import re
import xml.etree.ElementTree as ET
import zipfile
from pathlib import Path

import bench_geometry as geo

from .util import WS_ROOT

REGISTRY_PATH = WS_ROOT / "config" / "hil" / "models.yaml"
DEFAULT_FX = 554.0
_ONOFF = ("right_cam", "depth_port")


def _onoff(v):
    """PyYAML reads bare off/on as False/True: map them back."""
    if isinstance(v, bool):
        return "on" if v else "off"
    return v


def load_registry(path=None):
    """The parsed models.yaml: {"models": {"mono": entry, "rgbd": entry, "stereo": {"0.11": entry, ...}}}."""
    import yaml
    data = yaml.safe_load(Path(path or REGISTRY_PATH).read_text()) or {}
    models = data.get("models") or {}
    out = {}
    for mode, entry in models.items():
        if mode == "stereo":
            out[mode] = {str(b): _fix_entry(e) for b, e in (entry or {}).items()}
        else:
            out[mode] = _fix_entry(entry)
    return {"models": out}


def _fix_entry(e):
    e = dict(e or {})
    for key in _ONOFF:
        if key in e:
            e[key] = _onoff(e[key])
    return e


def _entry(registry, camera_mode, baseline_m):
    models = (registry or load_registry())["models"]
    if camera_mode not in models:
        raise KeyError(f"camera mode {camera_mode!r} is not in models.yaml (known: {', '.join(models)})")
    if camera_mode != "stereo":
        return models[camera_mode]
    if baseline_m is None:
        raise KeyError("stereo needs a baseline in metres")
    key = f"{float(baseline_m):g}"
    if key not in models["stereo"]:
        raise KeyError(f"stereo baseline {key} is not in models.yaml (known: {', '.join(models['stereo'])})")
    return models["stereo"][key]


def _expected_from_entry(e):
    x, y, z, yaw = geo.START
    fx = float(e.get("fx", DEFAULT_FX))
    right_cam = e.get("right_cam", "off")
    return {
        "model": Path(e["file"]).stem, "file": e["file"],
        "x_ic": x, "y_ic": y, "z_ic": z, "yaw_ic": yaw,
        "fx_left": fx,
        "fx_right": fx if right_cam == "on" else None,
        "right_cam": right_cam,
        "right_y": float(e["right_y"]) if right_cam == "on" and e.get("right_y") is not None else None,
        "depth_port": e.get("depth_port", "off"),
    }


def expected(camera_mode, baseline_m=None, registry=None):
    """What the model of this camera mode must contain. Keys are the keys of model_info."""
    return _expected_from_entry(_entry(registry, camera_mode, baseline_m))


def _same(key, exp, live):
    """True when live matches exp. Numbers within 1e-6; right_cam off also accepts absent."""
    if isinstance(exp, (int, float)) and not isinstance(exp, bool):
        return isinstance(live, (int, float)) and abs(float(live) - float(exp)) <= 1e-6
    if key == "right_cam" and exp == "off":
        return live in ("off", "absent")
    return live == exp


def check_live(model_info, expected):
    """One line per mismatch between what MATLAB reports and what the Pi expects. [] = all fine."""
    out = []
    for key, exp in expected.items():
        if key == "file" or exp is None:
            continue
        live = model_info.get(key)
        if live is None:
            out.append(f"{key}: MATLAB did not report it, the Pi expects {exp}")
        elif not _same(key, exp, live):
            out.append(f"{key}: MATLAB has {live}, the Pi expects {exp}")
    return out


def _eval_ic(text):
    """'20', '2*pi', '2*pi-0.0460' -> float. Anything else -> 0.0."""
    text = (text or "").strip()
    if not re.fullmatch(r"[0-9eE+\-*/(). pi]+", text):
        return 0.0
    try:
        return float(eval(text, {"__builtins__": {}}, {"pi": math.pi}))
    except Exception:
        return 0.0


def _blocks(root):
    """Block name -> {parameter name: text}. Camera parameters sit one level deeper (InstanceData)."""
    out = {}
    for b in root.iter("Block"):
        params = {}
        for p in b.iter("P"):
            params.setdefault(p.get("Name"), p.text or "")
        out[b.get("Name")] = params
    return out


def _vec(text):
    nums = re.findall(r"-?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?", text or "")
    return [float(n) for n in nums]


def read_slx(path):
    """Read a .slx file (a zip) and return the same keys as model_info. A missing IC counts as 0.0."""
    with zipfile.ZipFile(path) as z:
        root = ET.fromstring(z.read("simulink/systems/system_root.xml"))
    blocks = _blocks(root)

    def ic(name):
        return _eval_ic(blocks.get(name, {}).get("InitialCondition")) if name in blocks else 0.0
    left = blocks.get("Simulation 3D Camera", {})
    right = blocks.get("Simulation 3D Camera Right")
    fl = _vec(left.get("FocalLength"))
    out = {
        "model": Path(path).stem,
        "x_ic": ic("x_integrator"), "y_ic": ic("y_integrator"),
        "z_ic": ic("z_integrator"), "yaw_ic": ic("yaw_integrator"),
        "fx_left": fl[0] if fl else None,
        "fx_right": None, "right_cam": "absent", "right_y": None,
        "depth_port": left.get("DepthOutportEnabled", "off"),
    }
    if right is not None:
        out["right_cam"] = "off" if right.get("Commented") == "on" else "on"
        fr, mt = _vec(right.get("FocalLength")), _vec(right.get("tmountOffset"))
        out["fx_right"] = fr[0] if fr else None
        out["right_y"] = mt[1] if len(mt) > 1 else None
    return out


def check_files(registry=None, ws_root=WS_ROOT):
    """Compare every registry entry whose .slx exists in the repository. One line per mismatch,
    each starting with the file name. [] = all fine (also when no file exists)."""
    reg = registry or load_registry()
    out = []
    for mode, entry in reg["models"].items():
        entries = entry.values() if mode == "stereo" else [entry]
        for e in entries:
            f = Path(ws_root) / e["file"]
            if not f.exists():
                continue
            exp = _expected_from_entry(e)
            # a right camera that is off may be present in the file but commented out; a stereo file
            # stores its right camera commented out too, and set_stereo(1) turns it on at load
            got = read_slx(f)
            if exp["right_cam"] == "on" and got["right_cam"] == "off":
                got["right_cam"] = "on"
            out += [f"{e['file']}: {line}" for line in check_live(got, exp)]
    return out
