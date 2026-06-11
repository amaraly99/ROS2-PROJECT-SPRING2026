# Project Direction and Status Document
## Onboard Vision Pipeline Framework — ROS2 HIL
**As of: June 11, 2026 | Authors: Ahmed Dhaouadi, Amar Aly | Supervisor: Dr. Mohamed Hassan**

---

## 1. New Project Identity

### What This Paper Is Now

This is a **framework paper**, not a characterization paper. The core contribution is an open-source, modular, extensible ROS2-enabled hardware-in-the-loop infrastructure that supports end-to-end evaluation of onboard vision pipelines for autonomous robots. The specific hardware (RPi5 + Hailo-10H), simulator (MATLAB/Simulink/Unreal Engine), SLAM algorithms, detectors, and controllers are all **instantiations** of the framework, not the framework itself. A reviewer should be able to read this paper and substitute Jetson Orin for RPi5, or Gazebo for Unreal Engine, and understand that the framework would still apply.

### What This Paper Is Not

It is not "we characterized co-scheduling interference on RPi5 + Hailo-10H." That is evidence supporting the framework's value, not the claim.

### The Two Core Insights (Professor's Words)

**Insight 1 — Placement and system-level trade-offs:** In an isolated component benchmark, you always want to accelerate each component as much as possible. In a full end-to-end system where components share compute, moving one component to an accelerator changes the resource budget for everything else. Where you place components and how you schedule them has a first-order effect on end-to-end behavior that single-component benchmarks cannot predict.

**Insight 2 — Component behavior differs in isolation vs. end-to-end:** An algorithm that looks optimal in standalone benchmarks may not be optimal when evaluated as part of the full pipeline. The framework makes this difference measurable and explicit.

### Why the Framework Matters

The embedded robotics literature is fragmented. Papers evaluate SLAM, YOLO, or controllers in isolation. Nobody integrates all three, runs them under shared compute, and measures what actually happens end-to-end with a closed-loop controller as the outcome variable. The framework closes this gap — and it does so with full reproducibility because it is open-source and modular.

### Physical AI Framing

The professor flagged "Physical AI" (NVIDIA's terminology for end-to-end perception-to-actuation pipelines) as a useful framing anchor. Use it in the abstract and introduction as context for why end-to-end evaluation matters, not as a primary contribution claim.

---

## 2. Target Venue and Deadlines

| Item | Target |
|------|--------|
| Primary venue | IEEE Transactions on Computers |
| Backup venue | IEEE Access |
| Draft finalized | Mid-July 2026 (soft) |
| Submission | Late July 2026 (soft) |
| Current draft format | ieeeaccess class (working draft) |

---

## 3. System Configuration Matrix

### Axes

| Axis | Options | Count |
|------|---------|-------|
| SLAM algorithm | OV2SLAM Accurate Stereo, OV2SLAM Fast Stereo, ORB-SLAM2 Stereo, ORB-SLAM2 Mono, ORB-SLAM3 Stereo, RTAB-Map Stereo | 6 |
| Object detector | YOLOv8 n/s/m, YOLOv11 n/s/m, YOLO26n/s/m, Traditional OpenCV method (TBD), VLM-based detector (TBD), possibly 1 more non-YOLO | ~12 |
| Controller | Custom proportional (image-space, current), ViSP IBVS (image-plane corners/points), ViSP PBVS or hybrid (uses pose — needs SLAM topic integration) | 3 |

**Theoretical combinations: 6 × 12 × 3 = 216**

**Target tested configurations: ~25 representative configurations** selected to cover:
- Baseline (best known config): OV2SLAM Fast + YOLO26n + custom controller
- SLAM sweep (all 6) with fixed detector and controller
- Detector sweep (representative subset ~5 detectors) with fixed SLAM and controller
- Controller sweep (all 3) with fixed SLAM and detector
- Selected cross-combinations showing interesting interactions

### Sweep Strategy Note

The paper does not claim exhaustive combinatorial coverage. It claims that the framework supports plug-and-play substitution across these axes, and demonstrates this by running a representative subset. The ~25 configurations serve as evidence of modularity.

---

## 4. Component Status

### 4.1 SLAM

| Algorithm | Standalone (EuRoC) | System Integration | Notes |
|-----------|-------------------|-------------------|-------|
| OV2SLAM Accurate Stereo | DONE (10 runs/seq, green) | Not yet | Needs end-to-end HIL run |
| OV2SLAM Fast Stereo | DONE (10 runs/seq, green) | Not yet | Same |
| ORB-SLAM2 Stereo | DONE (10 runs/seq, green) | Not yet | Partial tracking failures documented |
| ORB-SLAM2 Mono | DONE (10 runs/seq, green) | Not yet | Same |
| ORB-SLAM3 Stereo | DONE (10 runs/seq, green) | Not yet | Non-deterministic under ROS2 wrapper; V2_01 all failed |
| RTAB-Map Stereo | Numbers in table (NOT green — verify if measured or estimated) | Not yet | Confirm measurement status |

**Responsible: Amar (standalone done, system integration is next step)**

### 4.2 Object Detectors

| Detector | Standalone | System Integration | Notes |
|----------|-----------|-------------------|-------|
| YOLO26n | Done (current system baseline) | Done (HIL loop operational) | Ahmed's responsibility |
| YOLOv8 n/s/m | Not started | Not started | Ahmed |
| YOLOv11 n/s/m | Not started | Not started | Ahmed |
| YOLO26s, 26m | Not started | Not started | Ahmed |
| Traditional OpenCV (TBD) | Not started | Not started | Method not yet decided |
| VLM-based (TBD) | Not started | Not started | Specific model not yet decided |

**YOLO selection justification needed in paper:** cite literature that uses YOLOv8 and YOLO26 specifically in robotics contexts. YOLOv10 is excluded by design (not widely adopted in robotics; jump went directly to v11). State this explicitly.

### 4.3 Controllers

| Controller | Status | Notes |
|-----------|--------|-------|
| Custom proportional (image-space) | Working, ~20 Hz | Does NOT use SLAM topics — professor flagged this as a gap |
| ViSP IBVS (full library) | Not working | Previous attempt required ~100 Hz detection; needs workaround for 20 Hz operation |
| ViSP PBVS or hybrid | Not started | Requires SLAM pose as input (/vo_pose) — this is the SLAM-in-control-loop controller |

**Architecture note on SLAM-in-control-loop: see Section 5.1 below.**

### 4.4 HIL Infrastructure

| Component | Status |
|-----------|--------|
| MATLAB/Simulink/Unreal Engine on Windows | Working, ~20 Hz |
| ROS2 Fast DDS link (Pi eth0 192.168.56.2, laptop 192.168.56.1) | Working |
| sim_camera_bridge C++ node (bgr8 → NV12 → SHM) | Working |
| POSIX seqlock SHM ring | Working |
| /cmd_vel closed-loop back to MATLAB | Working |
| Closed-loop baseline (Config A, 16 May 2026) | Confirmed stable |
| run_stack_hil.sh with argument passing | Exists — verify argument interface with Claude Code |
| Automated multi-configuration experiment script | Does NOT exist — needs to be built |
| Per-stage latency instrumentation | Partial — stages and measurement anchors not fully defined |

### 4.5 Pipeline Rates (Current Baseline, Config A)

| Stage | Rate | Notes |
|-------|------|-------|
| MATLAB camera publish | ~20 Hz | sim_cam_pub_count confirms |
| Camera delivery at Pi | ~20 Hz | Current state — was 17 Hz earlier, believed resolved |
| YOLO detections (/yolo/detections) | ~20 Hz | Controller fires at detection rate |
| /cmd_vel publish | ~20 Hz | Current custom controller rate |
| MATLAB telemetry | 20 Hz | /sim/drone_pose, /sim/heartbeat |

**Note: The manuscript previously recorded 17 Hz camera, 9 Hz detections, 4 Hz cmd_vel. Ahmed has confirmed the current system runs at ~20 Hz end-to-end. Verify exact current rates with Claude Code before updating the manuscript.**

---

## 5. Open Architecture Decisions (Must Be Resolved Before Experiments)

### 5.1 SLAM in the Control Law — Critical Tension

**The problem:** The current manuscript's Contribution #4 ("Indirect SLAM-to-controller link") is built on the premise that SLAM is explicitly NOT in the control loop, and that any effect of SLAM on controller quality is mediated purely through CPU contention. The professor has flagged that the custom controller should integrate SLAM topics.

**Proposed resolution:** Define three controller types that form a deliberate spectrum:
- **Controller A (custom proportional):** Pure image-space, no SLAM. The "indirect link" story applies here.
- **Controller B (ViSP IBVS):** Image-space using ViSP library features/corners. No SLAM in loop.
- **Controller C (ViSP PBVS or hybrid):** Uses /vo_pose from OV2SLAM as part of the control law. Direct SLAM-to-controller coupling.

This preserves the "indirect link" story for Controllers A and B, AND adds a new comparative story: direct vs. indirect SLAM-controller coupling as a design dimension. This is a richer result than the current single-controller framing.

**Decision needed:** Confirm this resolution with Dr. Hassan before implementation.

### 5.2 ViSP at 20 Hz — Workaround Needed

The 4-corner IBVS formulation requires fast detection rates (~100 Hz) for numerical stability of the interaction matrix. At 20 Hz, the following approaches are viable:

- **Reduced-feature IBVS:** Use 1 or 2 point features (bbox centroid, centroid + area proxy) rather than 4 corners. Simpler interaction matrix, stable at lower rates.
- **PBVS with OV2SLAM pose:** Does not depend on detection rate, uses pose estimate. Requires SLAM to be running.
- **ViSP visual servoing with prediction:** ViSP supports prediction between detections. May allow stable operation at 20 Hz.

**Action needed:** Research ViSP configuration for sub-100 Hz operation before implementation. Claude Code can inspect current ViSP attempt and identify the specific failure mode.

### 5.3 Non-YOLO Detector Selection — Not Decided

Two non-YOLO detector types are needed to demonstrate modularity. Candidates:

**Traditional OpenCV:**
- CSRT tracker (OpenCV tracking API): Initializes from first YOLO detection, then tracks without re-detection. Shows OpenCV as a lightweight fallback.
- Background subtraction (MOG2/KNN) + contour detection: Purely classical, no DNN. Good contrast against YOLO.
- Color-based detection + morphology: Appropriate for the stop-sign target (high red saturation).
- **Recommended:** Background subtraction + contour for maximum contrast with YOLO in the paper narrative.

**VLM-based:**
- GroundingDINO: Open-vocabulary, text-prompted detection. "Stop sign" as text query. Runs on CPU; will be slow on RPi5 (~1–5 Hz), which is actually interesting for the paper — shows the latency penalty of VLM-based detection in the end-to-end loop.
- Florence-2 (Microsoft): Smaller VLM, better suited to edge deployment than full DINO.
- NVIDIA DINOv2 + lightweight head: If professor specifically said "NVIDIA LLM," he may mean NVIDIA's implementation of a vision-language model.
- **Recommended:** GroundingDINO for maximum contrast with YOLO (open-vocabulary vs. class-specific) and clear narrative about latency trade-offs.

**Decision needed:** Confirm detector choices with Dr. Hassan.

### 5.4 SHM Abstraction — Verify Modularity

The pipeline's modularity claim depends on being able to swap any detector node without changing downstream nodes (controller, SLAM). The SHM interface between detector output and controller input must be a stable contract. Claude Code needs to verify: is the detection output SHM schema fixed (bbox centroid, confidence, class), and does any current code assume YOLO-specific fields?

### 5.5 Configuration Selection — Define the ~25

The ~25 representative configurations need to be explicitly listed before the automated experiment script is built, because the script's design depends on knowing exactly which (SLAM, detector, controller) triples to run. A proposed selection process:

1. Fix baseline: (OV2SLAM Fast, YOLO26n, Controller A)
2. SLAM sweep: (all 6 SLAM, YOLO26n, Controller A) → 6 configs
3. Detector sweep: (OV2SLAM Fast, {YOLO26n, YOLOv8n, YOLOv11n, OpenCV, VLM}, Controller A) → 5 configs
4. YOLO size sweep: (OV2SLAM Fast, {n/s/m} for each of YOLO8/11/26, Controller A) → 9 configs
5. Controller sweep: (OV2SLAM Fast, YOLO26n, all 3 controllers) → 3 configs
6. Cross-combinations of interest (e.g., worst SLAM + best detector, best SLAM + VLM detector) → 2–4 configs

**Total: ~25–27 configurations.** This is the list to formalize before building the automation script.

---

## 6. Outstanding Work — Prioritized

### Priority 1: Architectural decisions (this week, before any code work)
- Confirm Controller A/B/C resolution with Dr. Hassan
- Confirm non-YOLO detector selection (GroundingDINO + OpenCV method)
- Finalize the ~25 configuration list on paper
- Verify RTAB-Map standalone data (green vs. not-green in table)

### Priority 2: System integration (Amar's immediate next step)
- Run each SLAM algorithm in the full HIL loop with Config A (YOLO26n + Controller A)
- Collect per-stage latency and /cmd_vel rate per SLAM algo
- This produces the end-to-end SLAM comparison data

### Priority 3: ViSP controller implementation
- Diagnose current ViSP failure mode (Claude Code inspection)
- Implement reduced-feature IBVS or PBVS at 20 Hz
- Test in HIL loop

### Priority 4: YOLO model sweep (Ahmed)
- Integrate YOLOv8 n/s/m, YOLOv11 n/s/m into the SHM pipeline
- Confirm model swap is a single-argument change in run_stack_hil.sh
- Run standalone latency benchmarks
- Run end-to-end in HIL loop per model

### Priority 5: Non-YOLO detector integration
- Implement GroundingDINO node with SHM output interface
- Implement OpenCV-based detector node with same SHM interface
- Run standalone and end-to-end

### Priority 6: Automated experiment script
- Define exact configuration list (output of Priority 1)
- Write script that runs the full stack for a given (SLAM, detector, controller) triple, collects metrics for N runs, and saves to CSV
- Validate on 2–3 configurations before full sweep

### Priority 7: Paper rewrite
- New title (framework-first, not co-scheduling-first)
- New abstract (framework contribution, physical AI framing, two key insights)
- New contribution list (5 items restructured around framework claim)
- Rewrite Section III (architecture) to be more conceptual, less implementation-specific
- Fill Section IV (component characterization) — prose for detector and controller sections
- Fill Section V (experiments) after data is collected
- Fill Section VI (HIL validation) after data is collected
- Write conclusion once headline numbers are known

### Priority 8: Code cleanup and README (pre-submission)
- Modularize and clean up codebase (professor's explicit action item)
- Write proper README with install instructions and configuration guide
- Ensure public repo is submission-ready

---

## 7. Manuscript Restructuring Requirements

### Title Direction
Current: "Workload Placement and Co-Scheduling Interference in an Onboard Vision Pipeline..."
Needed: Something that leads with the framework and mentions Physical AI, modularity, or end-to-end evaluation. Example direction: "An Open-Source Modular ROS2 Framework for End-to-End Hardware-in-the-Loop Evaluation of Onboard Vision Pipelines"

**The final title should be decided after the evaluation section is written, per professor's advice.**

### Contribution List (Restructured)
1. **The framework itself:** Open-source, modular, ROS2-enabled HIL infrastructure supporting plug-and-play substitution of SLAM, detector, and controller components, with ~25 end-to-end configurations validated.
2. **SLAM Pareto ablation + cross-algorithm comparison:** (Existing — keep, reframe as evidence of framework enabling system-level SLAM evaluation)
3. **Detector sweep — component vs. end-to-end:** YOLO family + non-YOLO detectors, comparing standalone throughput against end-to-end pipeline latency and controller quality. (New — not yet in manuscript)
4. **Controller comparison under shared compute:** Three controller types across the same hardware configurations, showing how controller choice interacts with compute budget. (New — partially implied but not explicit)
5. **Workload placement and interference characterization:** The co-scheduling interference curve and placement sweep as an example of the system-level insights the framework enables. (Current contributions 2–4, reframed as one result among several)

### Section Structure Changes
- Section III (Architecture): Add explicit "modularity and extensibility" subsection. Remove implementation-specific details (script names, exact IPs, VirtualBox specifics) to a supplementary or appendix. Keep figures conceptual.
- Section IV (Component Characterization): Add Detector Selection subsection (currently a TODO), add Controller Design subsection (currently a TODO).
- Section V (System-Level Experiments): Restructure to cover all four experimental axes — SLAM sweep, detector sweep, controller sweep, workload placement/interference. Currently only covers placement/interference.
- Section VI (HIL Validation): Fill after experiments are complete.

---

## 8. Questions for Claude Code Inspection (Plan Mode Only — No Changes)

Send these to Claude Code with instruction: **inspect and report only, make no changes.**

### A. Pipeline architecture
1. List all ROS2 nodes currently in the stack with their topic subscriptions, publications, and approximate publish rates.
2. What is the exact data path from SHM camera write to /cmd_vel publish? List every stage with the shared memory keys or ROS2 topic names at each handoff.
3. What is the current measured /cmd_vel publish rate in the HIL loop? How is this measured (rostopic hz, timestamp logging, or other)?

### B. run_stack_hil.sh
4. What arguments does run_stack_hil.sh currently accept? Show the argument parsing section.
5. Is there an OV2SLAM core affinity command (taskset or similar) in run_stack_hil.sh? Show the relevant lines.
6. Can the SLAM algorithm, detector model, and controller be changed by passing different arguments, or do they require editing the script?

### C. SHM interface and modularity
7. What is the exact schema of the detection output SHM (field names, types, layout)? Is it YOLO-specific or detector-agnostic?
8. Does the controller node read from the detection SHM directly, or via a ROS2 topic? Show the relevant code.
9. Is there any code that assumes YOLO-specific detection metadata (class names, confidence thresholds) downstream of the detector node?

### D. Controller node
10. Show the controller node's full topic subscription list. Does it subscribe to /vo_pose or any SLAM topic? Confirm it does not.
11. What is the current FSM state machine implementation? List the states and the transition conditions.
12. What is the exact control law for vx, vy, vz, wz? Show the relevant lines.
13. What were the exact error messages or failure modes when ViSP IBVS was attempted? Is there any old ViSP code still in the repo?

### E. YOLO integration
14. How is the YOLO model loaded currently? Is the model path/name a runtime argument or hardcoded?
15. Does the YOLO node support loading different YOLO variants (v8, v11, v26) with the same node binary, or does each variant require a different node?
16. Is there a yolo_cpu_producer.py script (the SHM producer for CPU-side YOLO in Config B)? If so, show its interface.

### F. Measurement and instrumentation
17. Are there any existing timestamp logs or per-stage latency measurements in the current codebase? Show what exists.
18. Is there a logging script that captures /cmd_vel rate, YOLO detection rate, and camera rate simultaneously?
19. What is the heartbeat topic (/sim/heartbeat) used for? Is it used as a latency measurement anchor?

### G. Experiment automation
20. Is there any existing script that runs the stack, collects data, and saves results? Even a partial one?
21. How long does a single HIL run currently take from stack launch to clean shutdown?

### H. Repository structure
22. Show the top-level directory structure of ~/ROS2-PROJECT-SPRING2026.
23. Is there a Docker compose or equivalent file that defines the full container configuration?
24. Which files were recently modified (git log --oneline -20)?
