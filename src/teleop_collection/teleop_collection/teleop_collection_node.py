#!/usr/bin/env python3
import os
import shlex
import signal
import subprocess
import threading
import time

import requests
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from sensor_msgs.msg import JointState


class TeleopCollectionNode(Node):
    def __init__(self):
        super().__init__('teleop_collection_node')

        self.declare_parameter('recording_service_timeout', 5.0)
        self.declare_parameter('home_settle_sec', 3.0)        # after fork, wait for nodes to come up + home, then publish human
        self.declare_parameter('glove_wait_timeout', 30.0)    # wait for glove online before latching human
        self.declare_parameter('shutdown_timeout', 5.0)       # max wait when killing arm/tracker processes
        self.declare_parameter(
            'arm_launch_command',
            ['ros2', 'launch', 'arm', 'dual_arms.launch.py', 'enable_viewer:=false'])
        self.declare_parameter(
            'tracker_launch_command',
            ['ros2', 'launch', 'tracker', 'dual_tracker.launch.py'])
        # Voice prompts (best-effort, played via host's host_interaction/server.py; silent if not running)
        self.declare_parameter('ui_service_host', 'localhost')
        self.declare_parameter('ui_service_port', 8081)
        self.declare_parameter('audio_language', 'English')  # voice prompt language: Chinese / English

        self.recording_service_timeout = self.get_parameter('recording_service_timeout').value
        self.home_settle_sec = self.get_parameter('home_settle_sec').value
        self.glove_wait_timeout = self.get_parameter('glove_wait_timeout').value
        self.shutdown_timeout = self.get_parameter('shutdown_timeout').value
        self.arm_launch_command = self._launch_command('arm_launch_command')
        self.tracker_launch_command = self._launch_command('tracker_launch_command')
        self.ui_host = self.get_parameter('ui_service_host').value
        self.ui_port = self.get_parameter('ui_service_port').value
        self.audio_language = str(self.get_parameter('audio_language').value).strip().capitalize()
        self.ui_session = requests.Session()

        self.pub_commander = self.create_publisher(String, '/commander', 10)

        self.recording_client = self.create_client(SetBool, '/toggle_recording')
        self.discard_recording_client = self.create_client(Trigger, '/discard_recording')

        # Track whether the glove is online (hand_ik asserts on glove data before switching to human)
        self._glove_seen = {'left': False, 'right': False}
        self.create_subscription(
            JointState, '/action/left_glove/joints', lambda m: self._glove_cb('left'), 10)
        self.create_subscription(
            JointState, '/action/right_glove/joints', lambda m: self._glove_cb('right'), 10)

        # State (booleans only, no state machine)
        self._lock = threading.Lock()
        self.recording_state = 'idle'   # idle / recording / stopped / discarded
        self.arm_teleop_on = False
        self._arm_busy = False          # pedal 2 toggle in progress (avoid blocking listener thread / re-triggering)

        # arm + tracker subprocesses
        self._proc_lock = threading.Lock()
        self.teleop_processes = {}      # {'arm': Popen, 'tracker': Popen}

        from pynput import keyboard
        self.kbd_listener = keyboard.Listener(on_press=self.on_key_press)
        self.kbd_listener.start()

        # Background startup: wait for glove online -> latch /commander=human (for hands)
        self._startup_thread = threading.Thread(target=self._startup_sequence, daemon=True)
        self._startup_thread.start()

        self.get_logger().info(
            "🦶 Teleop collection ready. Pedals: 1=start/stop recording  2=start/stop arm  3=discard last recording")

    # ---------- Utilities ----------
    def play_sound(self, name):
        """Play a voice prompt via host's host_interaction/server.py (best-effort, silent if not running)."""
        url = f"http://{self.ui_host}:{self.ui_port}/play/{self.audio_language}/{name}"
        threading.Thread(
            target=lambda: self._post_quiet(url), daemon=True).start()

    def _post_quiet(self, url):
        try:
            self.ui_session.post(url, timeout=0.5)
        except Exception:
            pass

    def _launch_command(self, parameter_name):
        value = self.get_parameter(parameter_name).value
        if isinstance(value, str):
            command = shlex.split(value)
        else:
            command = [str(part) for part in value]
        if not command:
            raise ValueError(f"{parameter_name} must not be empty")
        return command

    # ---------- Startup sequence ----------
    def _glove_cb(self, side):
        self._glove_seen[side] = True

    def _startup_sequence(self):
        deadline = time.monotonic() + self.glove_wait_timeout
        while rclpy.ok() and not (self._glove_seen['left'] and self._glove_seen['right']):
            if time.monotonic() > deadline:
                self.get_logger().warning(
                    "Timed out waiting for glove data, latching /commander=human anyway (make sure tele_hand is started)")
                break
            time.sleep(0.1)
        for _ in range(3):
            self.pub_commander.publish(String(data='human'))
            time.sleep(0.2)
        self.get_logger().info("✋ commander latched to human: hands follow the glove; arms wait for pedal 2")

    # ---------- Pedals ----------
    def on_key_press(self, key):
        try:
            k = key.char
        except AttributeError:
            return
        if k == '1':
            self._toggle_recording()
        elif k == '2':
            self._toggle_arm_teleop()
        elif k == '3':
            self._discard_recording()

    def _toggle_recording(self):
        with self._lock:
            state = self.recording_state
        if state == 'recording':
            if self._stop_recording():
                self.play_sound('stop_recording')
                self.get_logger().info("⏹️  Pedal 1: stop recording")
        else:
            if self._start_recording():
                self.play_sound('start_recording')
                self.get_logger().info("⏺️  Pedal 1: start recording")

    def _toggle_arm_teleop(self):
        # Run in a background thread (includes home_settle wait) so the pynput listener isn't blocked and other pedals (e.g. start recording) stay responsive
        with self._lock:
            if self._arm_busy:
                self.get_logger().info("Pedal 2: arm toggle in progress, ignoring")
                return
            self._arm_busy = True
            on = self.arm_teleop_on
        threading.Thread(target=self._arm_toggle_worker, args=(on,), daemon=True).start()

    def _arm_toggle_worker(self, on):
        try:
            if not on:
                self._start_arm_teleop()
            else:
                self._stop_arm_teleop()
        finally:
            with self._lock:
                self._arm_busy = False

    def _discard_recording(self):
        with self._lock:
            state = self.recording_state
        if state != 'stopped':
            self.get_logger().info("Pedal 3: no recording to discard, ignoring")
            return
        if self._discard():
            self.play_sound('delete')
            self.get_logger().info("🗑️  Pedal 3: discarded last recording")

    # ---------- Arm teleop: fork / kill ----------
    def _start_arm_teleop(self):
        self.play_sound('start_arm')
        self.get_logger().info("▶️  Pedal 2: starting tracker + arms (homing...)")
        commands = {'tracker': self.tracker_launch_command, 'arm': self.arm_launch_command}
        with self._proc_lock:
            for name, command in commands.items():
                proc = self.teleop_processes.get(name)
                if proc is not None and proc.poll() is None:
                    self.get_logger().info(f"{name} already running: pid={proc.pid}")
                    continue
                try:
                    proc = subprocess.Popen(command, start_new_session=True)
                    self.teleop_processes[name] = proc
                    self.get_logger().info(f"Started {name}: {' '.join(command)} (pid={proc.pid})")
                except Exception as exc:
                    self.get_logger().error(f"Failed to start {name}: {exc}")
        # Wait for nodes to come up + new arm_ik (config=home) to drive arms to home, then switch to human for relative mapping.
        time.sleep(self.home_settle_sec)
        # /commander is volatile: messages sent before the new arm_ik subscribes are dropped, so publish several times within a window.
        for _ in range(8):
            self.pub_commander.publish(String(data='human'))
            time.sleep(0.2)
        with self._lock:
            self.arm_teleop_on = True
        self.get_logger().info("▶️  Pedal 2: arm teleop started (relative mapping from home)")

    def _stop_arm_teleop(self):
        # Kill arm + tracker; arms hold their current pose. Leave /commander alone (hands still follow the glove).
        self.play_sound('stop_arm')
        self._kill_processes(['arm', 'tracker'])
        with self._lock:
            self.arm_teleop_on = False
        self.get_logger().info("⏸️  Pedal 2: arm teleop stopped (arms hold current pose)")

    def _kill_processes(self, names):
        with self._proc_lock:
            procs = [(n, self.teleop_processes.pop(n)) for n in names if n in self.teleop_processes]
        # 1) SIGINT (triggers each node's cleanup, e.g. arm_control disconnecting RM)
        for name, proc in procs:
            if proc.poll() is not None:
                continue
            self.get_logger().info(f"Stopping {name}: pid={proc.pid}")
            try:
                proc.send_signal(signal.SIGINT)
            except ProcessLookupError:
                pass
        # 2) Wait, escalate to SIGTERM on timeout
        deadline = time.monotonic() + self.shutdown_timeout
        for name, proc in procs:
            if proc.poll() is not None:
                continue
            try:
                proc.wait(timeout=max(0.0, deadline - time.monotonic()))
            except subprocess.TimeoutExpired:
                self.get_logger().warn(f"{name} did not exit in time, sending SIGTERM")
                self._killpg(name, proc, signal.SIGTERM)
        # 3) Still alive -> SIGKILL
        for name, proc in procs:
            if proc.poll() is not None:
                continue
            try:
                proc.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                self.get_logger().warn(f"{name} still alive, sending SIGKILL")
                self._killpg(name, proc, signal.SIGKILL)

    def _killpg(self, name, proc, sig):
        try:
            os.killpg(os.getpgid(proc.pid), sig)
        except ProcessLookupError:
            pass
        except Exception as exc:
            self.get_logger().warn(f"Failed to stop {name}: {exc}")

    # ---------- Recording service ----------
    def _call_recording_service(self, client, request, service_name, action_name):
        if not client.wait_for_service(timeout_sec=self.recording_service_timeout):
            self.get_logger().warn(f"{service_name} unavailable, skipping {action_name}")
            return False
        future = client.call_async(request)
        deadline = time.monotonic() + self.recording_service_timeout
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        if not future.done():
            self.get_logger().warn(f"{service_name} response timed out")
            return False
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warn(f"{service_name} call failed: {exc}")
            return False
        if response.success:
            self.get_logger().info(f"{action_name} succeeded: {response.message}")
            return True
        self.get_logger().warn(f"{action_name} failed: {response.message}")
        return False

    def _start_recording(self):
        req = SetBool.Request()
        req.data = True
        ok = self._call_recording_service(self.recording_client, req, '/toggle_recording', 'start recording')
        if ok:
            with self._lock:
                self.recording_state = 'recording'
        return ok

    def _stop_recording(self):
        req = SetBool.Request()
        req.data = False
        ok = self._call_recording_service(self.recording_client, req, '/toggle_recording', 'stop recording')
        if ok:
            with self._lock:
                self.recording_state = 'stopped'
        return ok

    def _discard(self):
        ok = self._call_recording_service(
            self.discard_recording_client, Trigger.Request(), '/discard_recording', 'discard recording')
        if ok:
            with self._lock:
                self.recording_state = 'discarded'
        return ok

    def destroy_node(self):
        if hasattr(self, 'kbd_listener'):
            self.kbd_listener.stop()
        try:
            self._kill_processes(['arm', 'tracker'])
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    node = TeleopCollectionNode()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
