# yolo_ros — superseded

An early Python controller that subscribed to `yolo_msgs/DetectionArray` and produced simple control
output. **Superseded by [`visp_servo`](../visp_servo/)**, which does proper four-corner IBVS with a
five-state mission machine and SLAM-derived depth.

Nothing in `run_stack.sh`, `run_stack_hil.sh`, the launch files, or the benchmark harnesses
references this package. It is kept only so the early experiments in the git history remain
buildable.

If you are looking for the controller, you want `visp_servo`.
