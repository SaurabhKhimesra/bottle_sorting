# Third-party code

The `tm12_bottle_sorting` package is ours and is MIT licensed — see
[LICENSE](LICENSE). Everything else in `src/` belongs to someone else and
keeps its own terms.

| Path | Project | Licence | How it's included |
|---|---|---|---|
| `src/realsense-ros` | [Intel RealSense ROS wrapper](https://github.com/IntelRealSense/realsense-ros) | Apache-2.0 | Copied into the tree. Two launch files were edited for our two-camera rig, which is why it isn't a submodule. Intel's `LICENSE` and `NOTICE` are kept alongside it. |
| `src/tmr_ros1` | [Techman Robot ROS1 driver](https://github.com/TechmanRobotInc/tmr_ros1) | BSD-3-Clause | Submodule, pinned at `b5d92ba` (1.10.3) |
| `src/yolov5` | [Ultralytics YOLOv5](https://github.com/ultralytics/yolov5) | AGPL-3.0 | Submodule, pinned at `2236169` |

One thing worth flagging: **YOLOv5 is AGPL-3.0**. It was fine for coursework,
but anyone reusing the brand classifier in a product needs to deal with that —
either comply with the AGPL or swap the classifier for something under a
permissive licence. The rest of the pipeline doesn't depend on it; YOLOv5 is
reached by shelling out to `detect.py` from `brand_detection.py`, so it is a
replaceable piece.
