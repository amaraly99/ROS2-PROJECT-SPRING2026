# Extending the HIL stack — adding controllers & detectors

Goal: **drop in a new controller or detector by touching as few files as possible.**
This doc is the map of every touch point + the contracts a new module must honour.

The stack has three swappable slots (`detector`, `controller`, `slam`), each selected
by one field in a nested stack config (`config/hil/stack/<name>.yaml`, see
`full_ov2slam.yaml` for the commented schema). `scripts/parse_stack.py` flattens that
config to env vars; `run_stack_hil.sh` consumes them and launches
`src/sim_camera_bridge/launch/hil_simulation.launch.py`.

```
MATLAB image ─▶ sim_camera_bridge ─▶ ovcam_bridge ─▶ /ovcam/image_raw
                                                          │
        DETECTOR ◀────────────────────────────────────────┘
        (yolo | oracle | …) ──▶ /yolo/detections ──▶ CONTROLLER ──▶ /cmd_vel ──▶ MATLAB
                                                       (FSM owns the loop)
        SLAM (sidecar) ──▶ /slam/pose   (decoupled; recorded for RMSE only)
```

---

## The two hard contracts (do NOT break these)

These topic names are currently **hardcoded** in `servo_core/servo_fsm_node.cpp`
(`/yolo/detections` sub, `/cmd_vel` pub). The `topics:` blocks in the config schema are
reserved for a future remap pass — **they do not rewire anything today.** So:

| Slot | MUST publish | Type | QoS |
|---|---|---|---|
| **detector** | `/yolo/detections` | `yolo_msgs/DetectionArray` | best-effort |
| **controller** | `/cmd_vel` | `geometry_msgs/Twist` | (FSM handles it) |

`Detection.msg` = `class_name, confidence, center_x, center_y, size_width, size_height`
(pixels). `DetectionArray.msg` = `std_msgs/Header header` + `Detection[] detections`.

---

## Adding a CONTROLLER  (the easy, very-modular path)

The shared `servo_core::ServoFsmNode` owns **everything** — the SEARCH→APPROACH→REACHED
state machine, sim-heartbeat safety, clamp/ramp/floor filters, `/yolo/detections` and
`/cmd_vel` plumbing, and `/bench/state` instrumentation. A controller supplies **only**
the body-frame velocity for one APPROACHING tick. The interface is
`servo_core/include/servo_core/servo_controller.hpp`:

```cpp
class IServoController {
  virtual void init(const ServoInputs& cfg) {}        // optional: precompute fixed quantities
  virtual ServoVel computeApproach(const ServoInputs& in) = 0;  // the law (called every tick)
  virtual const char* name() const = 0;               // stamped into /bench/state
};
```
`ServoInputs` gives you normalized centring error (`ex_norm`,`ey_norm`), the smoothed
bbox (`cx,cy,bw,bh`), `bbox_ratio` (range proxy), bbox-based depth `Z`,
`target_bbox_ratio` setpoint, and full intrinsics. `ServoVel` is `{vx,vy,vz,wz}` in
Simulink body frame (vx fwd+, vy LEFT+, vz UP+, wz YAW-LEFT+). Every existing controller
(proportional/ibvs/h_vs/pbvs) is just this interface + a `main` that does:

```cpp
auto node = std::make_shared<servo_core::ServoFsmNode>("myctrl_node");
node->set_controller(std::make_unique<mypkg::MyController>(node.get()));
```

### Checklist — new controller `foo`
1. **Package** `src/foo_servo/` — copy `src/hil_servo/` as the template:
   - `include/foo_servo/foo_controller.hpp` — implement `IServoController`
     (`computeApproach`, `name()` returns `"foo"`); read gains via `node_->declare_parameter`.
   - `src/foo_controller.cpp` — the law.
   - `src/foo_servo_main.cpp` — `ServoFsmNode` + `set_controller` (4 lines, copy hil_servo).
   - `CMakeLists.txt` + `package.xml` — depend on `servo_core`, `rclcpp` (copy + rename).
2. **Gains file** `config/hil/bench_foo.yaml` — your `foo_node:` params (copy a sibling).
3. **Register in the launch map** — `hil_simulation.launch.py`, `CTRL_MAP`:
   ```python
   'foo': ('foo_servo', 'foo_servo_node', 'bench_foo.yaml'),
   ```
4. **Allow the name** — `run_stack_hil.sh`, the controller `case`:
   `ibvs|proportional|h_vs|pbvs|foo)`.
5. **Build list** — `run_stack_hil.sh`, `STACK_PKGS="… foo_servo"`.
6. **(optional) Offline controller bench** — to A/B it with `benchmarks/controller_hil_bench.sh`,
   add `foo` to its `case` guards + a `foo) NODE_EXEC=foo_servo_node` entry.
7. **Use it** — set `controller.type: foo` in a stack config (`controller.cpu:` pins it).

That's it. The FSM, safety, recording, and `/cmd_vel` come for free.

---

## Adding a DETECTOR

A detector turns frames into `/yolo/detections`. Two flavours exist as templates:
- **`oracle`** (`src/oracle_detector/`, pure Python ROS node) — projects the known target
  through the sim drone pose. No host process, no hardware. **Best template for a new
  software/synthetic detector.**
- **`yolo`** (`src/yolo_bridge/` in-container + `src/yolo_producer/yolo_producer.py` on the
  host) — the producer runs on the **host** for Hailo NPU access and writes `/yolo_shm`;
  the bridge reads the shm and publishes ROS. **Template for a hardware-backed detector.**

### Checklist — new detector `bar`
1. **Package** that publishes `/yolo/detections` (`yolo_msgs/DetectionArray`, best-effort).
   Copy `oracle_detector` (no hardware) or `yolo_bridge`+`yolo_producer` (host hardware).
2. **Register in the launch map** — `hil_simulation.launch.py`, `DET_MAP` (mirrors
   `CTRL_MAP`). One row:
   ```python
   # detector_name: (package, executable, node_name, [param_yaml,...], wants_target_class)
   'bar': ('bar_detector', 'bar_detector_node', 'bar_detector', ['bench_bar.yaml'], False),
   ```
   `[param_yaml,...]` are loaded from `config/hil/`; set `wants_target_class=True` to
   also inject `{'target_class': …}` (oracle-style). `detector_cpu` pinning is automatic.
3. **Allow the name** — `run_stack_hil.sh`, the detector `case`: `yolo|oracle|bar)`.
4. **Host process (only if needed)** — `run_stack_hil.sh` step 3 currently does
   `if [[ "$DETECTOR" == "yolo" ]]` to launch `yolo_producer` on the host (pinned via
   `DETECTOR_HOST_CPU`). If `bar` needs a host-side producer, add a branch here; a
   pure-ROS detector (oracle-style) needs nothing.
5. **Hailo guard** — the pre-flight `[[ "$DETECTOR" == "yolo" ]]` check requires
   `/dev/hailo0`. Extend it only if `bar` also needs the NPU.
6. **Build list** — `STACK_PKGS="… bar_detector"`.
7. **Use it** — `detector.type: bar` (+ `detector.cpu:`/`host_cpu:`) in a stack config.

---

## File-touch summary

| To add a… | Code | Config registration |
|---|---|---|
| **controller** | new `src/<x>_servo/` (impl `IServoController`) | `CTRL_MAP` (launch), controller `case` + `STACK_PKGS` (run_stack), `config/hil/bench_<x>.yaml` |
| **detector** | new `src/<x>_detector/` (pub `/yolo/detections`) | `DET_MAP` (launch), detector `case` + maybe host branch + `STACK_PKGS` (run_stack) |
| **SLAM** | (usually none — it's a sidecar) | whole `slam:` block in the stack config; container name MUST start `slam_`. See `handoff` notes for the ORB-SLAM live-adapter caveat. |

After any code change: `./run_stack_hil.sh build <pkg>` (colcon in Docker), then
`./run_stack_hil.sh --config <your-config>`.

## Known modularity gaps (honest state, worth closing)
- **`topics:` / `gains_file` config fields are parsed but not wired** — the FSM hardcodes
  `/yolo/detections` + `/cmd_vel`, and the launch picks gains by controller name via
  `CTRL_MAP` (not from `controller.gains_file`). Changing those config fields has no
  effect yet. Wire them through the launch when you need per-run topic/gain overrides.
