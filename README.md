<h1 align="center">EgoSteer: A Full-Stack System Towards Steerable Dexterous Manipulation from Egocentric Videos</h1>

<p align="center">
  <a href="https://arxiv.org/abs/XXXX.XXXXX"><img src="https://img.shields.io/badge/Paper-arXiv-b31b1b?style=for-the-badge&logo=arxiv&logoColor=white" alt="Paper"></a>
  <a href="https://egosteer.github.io/"><img src="https://img.shields.io/badge/Project-Page-1a73e8?style=for-the-badge&logo=googlechrome&logoColor=white" alt="Project Page"></a>
  <a href="https://huggingface.co/EgoSteer/datasets"><img src="https://img.shields.io/badge/%F0%9F%A4%97%20Data-Hugging%20Face-FFD21E?style=for-the-badge&labelColor=555555" alt="Data"></a>
  <a href="https://huggingface.co/EgoSteer/models"><img src="https://img.shields.io/badge/%F0%9F%A4%97%20Model-Hugging%20Face-FFD21E?style=for-the-badge&labelColor=555555" alt="Model"></a>
</p>

<p align="center">
  <a href="https://github.com/egosteer/egosmith"><img src="https://img.shields.io/badge/EgoSmith-Data%20Pipeline-24292e?style=for-the-badge&logo=github&logoColor=white" alt="EgoSmith"></a>
  <a href="https://github.com/egosteer/robot-stack"><img src="https://img.shields.io/badge/Robot%20Stack-this%20repo-2ea44f?style=for-the-badge&logo=github&logoColor=white" alt="Robot Stack"></a>
  <a href="https://github.com/egosteer/egosteer"><img src="https://img.shields.io/badge/EgoSteer-Model-6f42c1?style=for-the-badge&logo=github&logoColor=white" alt="EgoSteer"></a>
</p>

<p align="center">
  <img src="assets/figure/teaser.png" width="100%">
</p>

Our **full-stack system** integrates [EgoSmith](https://github.com/egosteer/egosmith), [Robot Stack](https://github.com/egosteer/robot-stack) (this repo), and [EgoSteer](https://github.com/egosteer/egosteer) to learn from large-scale egocentric human videos and facilitate data-efficient real-robot post-training, enabling steerable dexterous manipulation across over 40 tasks alongside few-shot adaptation to complex, long-horizon tasks.

This repository contains the unified **Robot Stack** for teleoperation, model inference, and human-in-the-loop correction. It runs on the RealMan embodiment out of the box, and can be easily extended to other robot embodiments.

![Robot stack overview](assets/figure/robot-stack.png)

## Deployment

The stack runs inside a Docker container (ROS 2 Humble). The robot host requires only Docker and a running X server.

**1. Clone the repository** (on the host):
```bash
git clone https://github.com/egosteer/robot-stack.git
cd robot-stack
```

**2. Configure the host**:
```bash
# enlarge the UDP buffers used by RealSense streams over DDS
sudo sh -c 'cat > /etc/sysctl.d/60-ros2-realsense.conf <<EOF
net.core.rmem_max=2147483647
net.core.wmem_max=2147483647
net.core.rmem_default=2147483647
net.core.wmem_default=2147483647
EOF'
sudo sysctl --system

# install the host-side audio player for the voice prompts
sudo apt update && sudo apt install -y mpg123
```
The in-container MuJoCo viewer renders on the host's X display, which by default rejects connections from the container's root user. The following command grants this access immediately and on every subsequent login (by appending it to `~/.xprofile`):
```bash
echo 'xhost +SI:localuser:root > /dev/null 2>&1' >> ~/.xprofile && xhost +SI:localuser:root
```

**3. Obtain the Docker image.** Pull the prebuilt image, or build it from the `Dockerfile`:
```bash
docker pull egosteerai/robot-stack:latest
# or build it locally:
docker build -t egosteerai/robot-stack:1.0.0 -t egosteerai/robot-stack:latest - < Dockerfile
```

**4. Create the container.** `create_container.sh <name>` creates a Docker container with the given name from the image, mounts the repository into it, maps the host `/dev`, and opens an interactive shell. For example, to create a container named `robot-stack`:
```bash
./create_container.sh robot-stack       # re-enter this container later with: docker exec -it robot-stack bash
```
The container's shell automatically sources `robot.bashrc` from the mounted repository. `robot.bashrc` sources ROS 2 and defines the following convenience commands used in the robot stack:
```bash
robot --teleop      # cameras + hands (absolute glove mapping) + data gloves
robot --inference   # cameras + hands (model-driven) + arms
robot --replay      # hands + arms only, no cameras (replay actuators)
interface           # model-inference client
collect             # teleoperation recording and pedal control
replay              # play back a recorded .rrd trajectory
mock                # publish mock data for testing
steamvr             # start SteamVR
cb / s              # colcon build / source install/setup.bash
```
Because this repository is mounted into the container, `robot.bashrc` can be edited on the host and the changes take effect in the next container shell, without rebuilding the image or re-creating the container. The `robot`, `interface`, `collect`, and `replay` commands each run `colcon build` before launching, so the workspace is compiled on first use and no separate build step is required.

**5. Set up the udev rules.** The two RuiYan RY-H2 dexterous hands connect to the host through a single RS485-to-USB quad-serial adapter, with the left and right hands on different interfaces of the same adapter, and the two PsiBot SynGlove-Air data gloves connect as individual USB-serial devices. The udev rules in `assets/udev_rules` map each device to a fixed name by its USB identity (independent of the USB port and stable across reboots), so the launch files always reach the correct hand or glove:

| Device | Stable name |
|---|---|
| RuiYan RY-H2 hands (left / right) | `/dev/hand_left` / `/dev/hand_right` |
| SynGlove-Air gloves (left / right) | `/dev/glove_left` / `/dev/glove_right` |

Install the rules once on the host:
```bash
cd assets/udev_rules && ./install.sh
```
`create_container.sh` maps the host `/dev` into the container, so these names are visible inside it as well.

**6. Set up SteamVR.** SteamVR is Valve's runtime for VR and tracking hardware. In this stack it is used only for teleoperation and human-in-the-loop correction, where it drives the Vive trackers and exposes each tracker's 6-DoF pose through its OpenVR interface for the `tracker` package to read. The command below prompts for a free Steam account, then installs a headless SteamVR runtime (no VR headset required) into `assets/SteamVR/`.
```bash
cd assets/SteamVR && ./setup.sh
```

## Usage

Before the first run, the hardware serial numbers must be set in their configuration files. The two tracker serials are set in `src/tracker/config/dual_tracker.yaml` as `left_serial_number` and `right_serial_number`, and can be found by `python assets/SteamVR/detect_trackers.py` while SteamVR is running. The head and chest camera serials are set in `src/camera/config/cameras.yaml`.

### Teleoperation data collection
In this mode, a human operator drives the robot, steering the arms with the Vive trackers and the hands with the data gloves, and the resulting trajectories are recorded as demonstration data.

The recording is configured in `src/teleop_collection/config/teleop_collection.yaml` through `task_name` and `audio_language`. Episodes are written to `recordings/<task_name>/`, and the voice prompts are played on the host in the selected language.

To launch teleoperation, execute the following commands in separate Docker container shells:
```bash
steamvr            # Vive trackers
robot --teleop     # cameras + hands (absolute glove mapping) + data gloves
collect            # recording and pedal control
```

The entire data-collection workflow is controlled with a foot pedal:
- **Pedal 1** starts recording, and stops it on the next press.
- **Pedal 2** enables the arm to follow the tracker, and disables it on the next press.
- **Pedal 3** discards the last recording.

### Model inference
In this mode the trained policy drives the robot autonomously from a natural-language instruction. The policy runs on a model server, which lives in [a separate repository](https://github.com/egosteer/egosteer) and consumes camera, proprioception, and language observations to return actions. This stack runs the client that streams those observations to the server and executes the actions it returns.

The client is configured via `src/model_interface/config/model_interface.yaml`, where `human_in_the_loop` is kept `false` for pure inference. The hand-eye calibration results are placed under `assets/calibration/`, the default location read by the client; see [`assets/calibration/README.md`](assets/calibration/README.md) for the expected layout.

On the host, launch the instruction terminal and open <http://localhost:8081>:
```bash
python assets/host_interaction/server.py     # English UI; --cn for Chinese, --dev for bilingual
```
Then, inside the container, launch the stack and the inference client, each in its own shell:
```bash
robot --inference   # cameras + arms + hands
interface           # inference client
```
With everything running, the operator types a natural-language instruction into the web terminal, and it is delivered to the robot whenever the model requests one. Execution is controlled with a single foot pedal. Pedal 1 starts execution, and the next press ends execution and resets the robot.

### Human-in-the-loop correction
This mode runs model inference while allowing the operator to intervene. The model controls the robot by default, and at any moment the operator can take over, steering the arms and hands exactly as in teleoperation, correct the model's behavior, and then return control to the model. Takeover relies on *relative motion mapping*. From the instant the operator takes over, the change in the operator's motion relative to that instant is applied to the robot's current pose, ensuring that the arms and hands continue smoothly from where they are rather than jumping to the operator's absolute pose. The operator is therefore advised to mirror the robot's motion throughout the episode, which makes intervention easier to carry out successfully and keeps the operation intuitive.

The configuration follows model inference, except that `human_in_the_loop` is set to `true` in `src/model_interface/config/model_interface.yaml`. The recording destination is set in the same file through `task_name`, and the episodes are written to `recordings/<task_name>/`.

Start `steamvr` first, then launch the same commands as in model inference, each in its own shell:
```bash
steamvr
robot --inference
interface
```

The entire correction workflow is controlled with a foot pedal:
- **Pedal 1** starts model inference and recording; the next press ends execution, stops the recording if it is still running, and then resets the robot.
- **Pedal 2** toggles control between the model and the operator.
- **Pedal 3** stops the current recording on the first press, then discards that just-stopped recording on the next press.

## Utilities

**Running the inference client without hardware.** The `mock_robot_data` package publishes synthetic camera, hand, and arm observations on the same topics as the real sensors, allowing the inference client to start and connect to the model server without a physical robot. The mock stream is fixed and does not respond to the actions the model returns, so this checks the observation flow and the client-to-server link rather than closed-loop robot behavior. Follow the model inference workflow with `robot --inference` replaced by `mock`:
  ```bash
  mock        # synthetic observations in place of robot --inference
  interface   # inference client
  ```

**Replaying a recording.** The `replay_rrd` package plays a recorded trajectory back through the arms and hands, without the model server or cameras, which is useful for reviewing a collected episode or confirming that the actuators reproduce it faithfully. Configure the playback in `src/replay_rrd/config/replay_rrd.yaml` by pointing `rrd_file_path` at the recording and, optionally, setting `target_hz` to the playback rate. Bring up the actuators with `robot --replay` and start playback with `replay`, each in its own shell. Playback first homes the robot, then replays the entire trajectory, and exits automatically once the recording ends:
  ```bash
  robot --replay     # arms + hands only
  replay             # plays back rrd_file_path
  ```
