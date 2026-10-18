# hilbench — implementation contract

Approved design: `benchmarks/HILBENCH_DESIGN.html` (class cards) and
`benchmarks/SLAM_RUNNER_MIGRATION_PLAN.html` (plan). This file fixes the exact names and
signatures, so that the harness, the SLAM kind and the tests can be written at the same time.
The files `config.py` (FieldSpec, COMMON_FIELDS, Settings), `trial.py` (types), `kinds/base.py`
and `factory.py` already exist and are the contract: do not change their public names.

## 0. Hard rules (all work)

1. **The core never names a kind.** No file under `benchmarks/hilbench/` outside `kinds/` and
   `factory.py` may contain the words `controller` or `slam` in code (comments, docstrings and
   string data such as default file paths are checked by `scripts/check_hilbench_layers.py`, so:
   none at all outside kinds/ and factory.py — use neutral words like "process under test").
   Exception: `config.py` field names `controller_cores` (it is the approved setting name) — the
   guard script allow-lists exactly this one identifier.
2. **Layers** (import only downwards): L0 `util`, `bench_geometry` · L1 `config` · L2 `supervisor`,
   `net`, `pi_state`, `container`, `gate`, `model_check`, `overhead` (none imports another L2) ·
   L3 `trial`, `ui`, `summary` · L4 `session` · L5 `kinds/*` (may import L0–L4 and `kinds/base`;
   never another kind, except `kinds/slam.py` may import its helper `kinds/slam_probe.py`) ·
   L6 `factory`.
   Note: `trial.py` defines the types and is imported by `kinds/base.py`; that is allowed (L3 < L5).
3. **Do not edit**: `benchmarks/ctrl_runner.py`, `benchmarks/ctrl_runner`, `run_controller_benchmark.sh`,
   `config/hil/ctrl_bench.yaml`, `benchmarks/hil_supervisor_client.py`, `benchmarks/controller_hil_bench.sh`,
   `matlab/slam_traj_probe.m`, `matlab/slam_traj_probe_rgbd.m`, anything in `scripts/hil_matrix/`.
   The controller campaign keeps running on the old runner.
4. **Never run** the bench, Docker, ssh, MATLAB, `sudo`, or anything that talks to 192.168.1.201.
   Tests are offline. A fake TCP supervisor on 127.0.0.1 is fine.
5. **No git commits, no pushes.** Amar commits.
6. Python: `~/.venvs/hilrun/bin/python` (has rich, PyYAML; no pytest). Tests use `unittest`.
   Run all tests: `cd benchmarks && ~/.venvs/hilrun/bin/python -m unittest discover -s hilbench/tests -t . -v`
   Import style inside the package: relative (`from .util import run`); `bench_geometry` is a
   top-level module next to the package (`import bench_geometry as geo`).
7. Write plain, short comments in the style of `ctrl_runner.py`.

## 1. Module signatures (harness implementer fills these; everyone else may call them)

### util.py
- `run(cmd, timeout=30, check=False) -> subprocess.CompletedProcess` (text, captured; check -> RuntimeError with command + first 300 chars of stderr)
- `now_ns() -> int`

### config.py  (types already written; implement the 4 functions)
- `build_parser(kind_name, extra_fields) -> argparse.ArgumentParser` — one option per COMMON_FIELDS and
  per extra FieldSpec (`spec.cli_flag`, `type=spec.cast`, `choices=spec.options`), plus `--config PATH`
  and `--dry-run`. No parser defaults (all `None`), so "given on the command line" is detectable.
- `load(argv, kind_cls, config_path=None) -> Settings`
  1. defaults from COMMON_FIELDS and `kind_cls.fields()`;
  2. YAML file: `--config`, else `config_path`, else `config/hil/<kind_cls.name>_bench.yaml` (repo-relative);
     common fields from their section, kind fields from section `<kind_cls.name>`; missing file ->
     `config_path=None` (the session logs a deviation);
  3. bool fix: for a text field (cast is str) whose YAML value is a bool, map `False->"off"`, `True->"on"`;
  4. command line wins; 5. `overrides` = names whose command-line value differs from the YAML value;
  6. no normalisation of "no pinning": only the text `none` or a core list (`3`, `2,3`, `0-2`) is valid;
  7. `problems = validate(s) + kind_cls.validate(s)`; if any: print each on stderr, `sys.exit(2)`;
  8. return the frozen Settings (`kind=kind_cls.name`).
- `validate(settings) -> list[str]` — common fields only: tag empty/None; `wifi_min_pct` outside 0..100;
  a text field outside its options; `controller_cores`/`runner_cores` not `none` and not a core list.
- `as_snapshot(settings) -> dict` — every common field, `kind_cfg`, `overrides`, `config_path` and the
  sha256 of the YAML file (or None).

### supervisor.py  (generic protocol only)
- `class SupervisorError(RuntimeError)`
- `class Supervisor(host, port, timeout_s=5.0)`; attribute `status_calls` (int).
- `send(cmd) -> str` (one line out, one line back; empty or `error...` -> SupervisorError)
- `parse_status(line) -> dict` (module function; numbers -> float)
- `status() -> dict` (increments status_calls)
- `wait_matlab(wait_s, on_tick=None, poll_s=2.0) -> bool` (on_tick(seconds_left))
- `hold(timeout_s, poll_s=1.0) -> dict` (send start_hold; poll status; `state=held` -> return status;
  `frozen` or timeout -> `trial.RigFailed("start_hold", ...)`)
- `resume() -> (sent_ns, ok_ns)`, `stop()`, `pub_on(hz)` (`pub_start <hz>`), `pub_off()` (`pub_stop`)
- `model_info() -> dict` — sends `model_info`; reply `model_info key=value ...` parsed like status.
  Keys: `model`, `x_ic`, `y_ic`, `z_ic`, `yaw_ic`, `fx_left`, `fx_right` (or `none`), `right_cam`
  (`on|off|absent`), `right_y` (or `none`), `depth_port` (`on|off`), `left_image`, `right_image`.

### net.py
- `pct_from_dbm(dbm)`, `wifi_pi(iface="wlan0") -> (ssid, dbm, pct)`, `wifi_windows(ssh_target) -> (ssid, pct, dbm, err)`
- `check_links(settings) -> LinkReport` (dataclass: `ok: bool`, `lines: dict[name -> (ok, text)]`, `info: dict`)
- `class WifiSampler(threading.Thread)(path, iface="wlan0")`: `start()`, `stop()`, `rates` (deque 60), `error`

### pi_state.py
- `HIL_NET_BUFFERS`, `parse_cores(spec) -> set[int]`, `governors()`, `net_buffers()`
- `pi_state() -> dict` (`cpu_temp_c`, `throttled`, `mem_total_mb`, `mem_available_mb`, `swap_used_mb`)
- `affinity_of(exe_name) -> str`, `process_memory(exe_name) -> dict | None` (`rss_mb`, `hwm_mb`)
- `class PiSettings(settings)`: `apply() -> dict`, `restore()`, `snapshot() -> dict`

### overhead.py
- `class OverheadMeter(status_counter=None)`: `sample()` (lowest MemAvailable), `result() -> dict`
  (`flight_wall_s`, `runner_cpu_pct_core`, `runner_status_calls`, `pi_cpu_total_pct`, `runner_rss_mb`,
  `runner_hwm_mb`, `pi_mem_available_min_mb`). `status_counter` is a callable returning the call count.
- `class PaceGuard(mode="stop", min_ratio=0.85)`: `record(k, hb_hz)`, `check() -> ("ok"|"degraded", text)`,
  `baseline_hz`, `history`. `hb_hz=None` is recorded and never degrades.

### ui.py
- `spark(values, lo=None, hi=None) -> str`
- `class Keys(threading.Thread)`: `pause`, `abort`, `show_log`, `restore()`, `suspend()`, `resume_cbreak()`
- `class Screen(header_text)`: `links` (dict), `banner`, `steps`, `flight` (dict|None), `flight_rows`
  (callable -> rows), `columns`, `rows`, `log_lines` (callable -> lines | None); `set_link(name, ok, text)`,
  `step(name, status, detail="")` (also calls `on_step(name, status, detail)` if set), `render()`.
- `ask_next_or_abort(live, keys, k, reason) -> "n"|"a"`

### container.py
- `class Container(name="ros2_perception_stack", ws_root=...)`: `ensure_running(screen, log_path) -> bool`,
  `exec_cmd(args, env=None, timeout=30) -> CompletedProcess` (`sudo -n docker exec ...`), `pkill(names)`
- `class BenchProcess(argv, console_log, parse_line, in_container=True, container=None, env=None)`:
  starts `argv` (inside the container via `docker exec -e K=V ... bash -lc` when `in_container`,
  else on the host), reader thread; every line -> log file, `lines` deque(400), then
  `parse_line(line, self)`. Attributes: `lines`, `run_rel`, `gate_wait`, `recording`, `error`, `extra` (dict).
  `alive()`, `stop(timeout=25, kill_names=(), signal_cmd=None)`.
  Default parse (applied before the kind's): `Run dir: (bags/\S+)` -> run_rel; `waiting for record gate`
  -> gate_wait; line starting `[bench] recording` -> recording; `[bench] ERROR` -> error.

### gate.py
- `class RecordGate(ws_root, session_name, k, n_topics)`: `host_path`, `container_path`
  (`/workspace/bags/.gate_<session>_<k>`), `arm()`, `open()`, `close()`,
  `wait_recorder(run_dir, timeout=15.0, poll_s=0.3) -> int` (counts `Subscribed to topic` in `run_dir/bag_record.log`).

### model_check.py
- Registry file `config/hil/models.yaml` (schema in section 3).
- `load_registry(path=None) -> dict`, `expected(camera_mode, baseline_m=None, registry=None) -> dict`
  (start values filled from `bench_geometry.START`, fx 554 unless the entry says otherwise),
  `check_live(model_info: dict, expected: dict) -> list[str]` (numbers within 1e-6; one line per mismatch),
  `read_slx(path) -> dict` (unzip `simulink/systems/system_root.xml`: integrator ICs, FocalLength,
  right camera `Commented` and `tmountOffset` y, `DepthOutportEnabled`; IC missing -> 0.0),
  `check_files(registry=None, ws_root=...) -> list[str]` (each entry whose file exists in the repo).

### summary.py
- `format_row(k, cond_label, res, kind) -> list[str]`, `append_runs_csv(path, row: list)`,
  `finalize(session_folder, kind, python) -> Path | None`.

### trial.py  (types written; implement `Trial`)
- `class Trial(k, n_total, cond, settings, services, kind, screen, keys)` with the attributes in its
  docstring; `run() -> TrialResult`. Exceptions: `RigFailed` -> `RIG_FAILED`, `BackendFailed` ->
  `BACKEND_FAILED`, any other exception -> `RIG_FAILED` with `"<step>: <Type>: <msg>"`. The bench is
  always stopped (kind.stop) when boot succeeded, also after a failure.
- `named(step)` context manager as in the design.

### session.py
- `class Session(settings, kind_cls)`: builds Services and the kind; `folder` =
  `reports/<kind>_<tag>_<stamp>/`; `name`; `say(msg)`, `deviation(msg)`;
  `schedule() -> list[(k, cond)]` = `[c for _ in range(runs) for c in kind.conditions()]`, numbered from 1;
  `preflight(live) -> bool`; `snapshot(applied)`; `run() -> int` (exit code).
  After each run: `PaceGuard.record(k, kind.pace_metric(res))`; `degraded` + mode stop -> banner
  "Pace fell to <ratio> of run 1. Paste hil_session_start in MATLAB, then start the next session." and stop.
  `kind.asks_on(res.outcome)` -> ask n / a. Model check: if `kind.model_key()` is not None and not a dry
  run, `model_check.check_live(supervisor.model_info(), expected(*key))` must be empty.

### Entry points (repo root and benchmarks/)
- `benchmarks/hilbench_run.py <kind> [options]` -> `factory.main(kind, argv)`.
- Controller: `benchmarks/ctrl_runner2.py` (= main("controller")) + `run_controller_benchmark2.sh`.
  (`ctrl_runner.py` is replaced only after the campaign; not now.)
- SLAM: `benchmarks/slam_runner.py` (= main("slam")) + `run_slam_benchmark.sh` (same shape as
  `run_controller_benchmark.sh`: venv python, real terminal).

## 2. YAML settings files

`config/hil/controller_bench.yaml` (new; the old `ctrl_bench.yaml` stays) and
`config/hil/slam_bench.yaml`: sections `session`, `matlab`, `network`, `pi` with the COMMON_FIELDS
keys, plus the kind section. Quote every word value (`"off"`, `"none"`).

## 3. models.yaml (Pi = source of truth for models)

```yaml
# config/hil/models.yaml — what each Simulink model must contain. Start values come from
# bench_geometry.START; focal length 554 unless an entry says otherwise.
models:
  mono:   {file: matlab/hil_closed_loop_unified.slx,         right_cam: off, depth_port: off}
  rgbd:   {file: matlab/hil_closed_loop_unified_rgbd.slx,    right_cam: off, depth_port: on}
  stereo:
    "0.11": {file: matlab/hil_closed_loop_unified_stereo_011.slx, right_cam: on, right_y: -0.11, depth_port: off}
    "0.36": {...}   # 0.42, 0.54, 0.61 likewise
```
MATLAB model name = file stem. `hil_session_start.m` reads the same file stem.

## 4. SLAM arm files: `config/hil/slam_arms/<arm>.yaml`

```yaml
name: ov2_fast
slam_type: ov2slam              # orbslam2 | orbslam3 | ov2slam | rtabmap
ready_signal: timing_csv        # timing_csv | handshake_ack
init_patience_s: 9
stack:                          # one unified stack config per camera mode this arm supports
  mono: unified_ov2slam_fast_mono
  stereo: unified_ov2slam_fast_stereo
```
`camera_modes` = the keys of `stack`. Each value names `config/hil/stack/<value>.yaml`, which must have
`init_gate.enabled: false`, `slam.startup_delay_sec: 2`, `network.dds: fastrtps`,
`network.matlab_host_ip: 192.168.1.201`.

## 5. MATLAB protocol additions (additive; old commands and the no-argument path unchanged)

- `probe_init <vx> <cycles> <lambda_m> <yaw_amp_deg>` / `probe_mission <vx> <cycles> <lambda_m> <yaw_amp_deg>`
  -> `ok` at once (sets base vars `hilprobe_req` = 'init'|'mission' and `hilprobe_args`); refuses with
  `error busy` while a probe runs, `error no_loop` when the waiting loop is not running.
- `probe_status` -> `probe_status state=idle|running|done invalid=0|1 maxtime_hit=<n> dist_to_target_m=<m|nan> end_x=.. end_y=.. end_z=..`
- `model_info` -> `model_info model=<name> x_ic=.. y_ic=.. z_ic=.. yaw_ic=.. fx_left=.. fx_right=..|none right_cam=on|off|absent right_y=..|none depth_port=on|off left_image=rgb|mono right_image=rgb|mono`
- `hil_session_start.m` is unchanged (the controller campaign uses it). SLAM uses the separate function file `hil_session_start_slam.m`:
  `hil_session_start_slam('mono'|'stereo'|'rgbd', baseline_m)`: model from `config/hil/models.yaml`
  semantics (file stem), runs set_stereo/set_rgbd as needed, then ends in the waiting loop that runs
  `slam_traj_probe_unified` (or `_rgbd_unified`) on the main thread when `hilprobe_req` is set.
