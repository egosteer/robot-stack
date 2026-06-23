# Hand-eye calibration

Put **your robot's** hand-eye calibration results in this directory. It is the default
`calibration_path` in `src/model_interface/config/model_interface.yaml`, and `model_interface`
loads from here at startup. The directory is git-ignored (calibration is specific to each
robot), so everything here except this README is local to your machine.

## Expected layout

One sub-directory per (camera, arm), each containing `calibration_results/result.npz`:

```
assets/calibration/
  <name>-head-cam-left-arm-<...>/calibration_results/result.npz
  <name>-head-cam-right-arm-<...>/calibration_results/result.npz
  <name>-chest-cam-left-arm-<...>/calibration_results/result.npz
  <name>-chest-cam-right-arm-<...>/calibration_results/result.npz
```

- The sub-directory name must contain the camera (`head` / `chest`) and the arm
  (`left` / `right`); `model_interface` matches by those substrings and, if several match,
  uses the lexicographically latest one.
- `result.npz` must contain `T_cam2base` (4×4) and `camera_matrix` (3×3).
- You only need the cameras selected by `camera_setup` (`head`, `chest`, or `both`).
