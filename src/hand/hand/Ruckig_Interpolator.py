import numpy as np
from ruckig import Ruckig, InputParameter, OutputParameter, Result
from copy import copy

class SmoothJointInterpolator:
    """
    Time-synchronized, multi-DOF smooth joint interpolator built on the Ruckig library.
    (Modified: internal time management removed to fix time-base inconsistency.)
    """
    def __init__(self, dof, step, alpha):
        """
        Args:
            dof (int): number of degrees of freedom.
            step (float): control period in seconds, i.e. 1.0 / output_freq.
            alpha (float): low-pass filter coefficient for smoothing input noise.
        """

        self.dof = dof
        self.step = step
        self.alpha = alpha

        self.ruckig = Ruckig(self.dof, self.step)
        self.input_param = InputParameter(self.dof)
        self.output_param = OutputParameter(self.dof)

        # Default kinematic limits.
        self.input_param.max_velocity = [3.0] * self.dof
        self.input_param.max_acceleration = [6.0] * self.dof
        self.input_param.max_jerk = [12.0] * self.dof

        self.result = Result.Working
        self.param_initialized = False

    def set_kinematic_limits(self, max_velocity=None, max_acceleration=None, max_jerk=None):
        """Set kinematic limits (max velocity, acceleration, jerk)."""
        if max_velocity is not None:
            self.input_param.max_velocity = [max_velocity] * self.dof if isinstance(max_velocity, (int, float)) else max_velocity
        if max_acceleration is not None:
            self.input_param.max_acceleration = [max_acceleration] * self.dof if isinstance(max_acceleration, (int, float)) else max_acceleration
        if max_jerk is not None:
            self.input_param.max_jerk = [max_jerk] * self.dof if isinstance(max_jerk, (int, float)) else max_jerk

    def set_input_param(self, current_position, current_acceleration=None, current_velocity=None):
        """Set the current state."""
        self.input_param.current_position = current_position
        if current_acceleration is not None:
            self.input_param.current_acceleration = current_acceleration
        else:
            self.input_param.current_acceleration = np.zeros(self.dof)
        if current_velocity is not None:
            self.input_param.current_velocity = current_velocity
        else:
            self.input_param.current_velocity = np.zeros(self.dof)
        self.param_initialized = True

    @property
    def is_initialized(self):
        return self.param_initialized

    def update(self, target_pos, target_vel=None, target_acc=None):
        """Update the target joint positions."""
        if not self.is_initialized:
            raise ValueError("Interpolator is not initialized")
        
        self.input_param.target_position = target_pos
        self.input_param.target_velocity = np.zeros(self.dof) if target_vel is None else target_vel
        self.input_param.target_acceleration = np.zeros(self.dof) if target_acc is None else target_acc

        self.result = self.ruckig.update(self.input_param, self.output_param)
        wokring = Result.Working == self.result
        self.output_param.pass_to_input(self.input_param)
        pos = np.array(self.output_param.new_position)
        vel = np.array(self.output_param.new_velocity)
        acc = np.array(self.output_param.new_acceleration)
        # pass the updated delta movement to the input parameter as the new planning start point
        return pos, vel, acc, wokring
    def close(self):
        self.param_initialized = False

if __name__ == "__main__":
    interpolator = SmoothJointInterpolator(dof=3, step=0.01, alpha=0.5)
 
    # Set input parameters
    current_position = [0.0, 0.0, 0.5]
    current_velocity = [0.0, 0.0, 0.0]
    current_acceleration = [0.0, 0.0, 0.0]
 
    target_position = [0.5, 0.5, 0.5]
    target_velocity = [0.0, 0.0, 0.0]
    target_acceleration = [0.0, 0.0, 0.0]

    interpolator.set_kinematic_limits(
        max_velocity=[1.0, 1.0, 1.0],
        max_acceleration=[1.0, 1.0, 1.0],
        max_jerk=[1.0, 1.0, 1.0]
    )

    interpolator.set_input_param(
        current_position=current_position,
        current_velocity=current_velocity,
        current_acceleration=current_acceleration
    )
    first_output = None
    while interpolator.result == Result.Working:
        pos, vel, acc = interpolator.update(target_pos=target_position, target_vel=target_velocity, target_acc=target_acceleration)
        if not first_output:
            first_output = copy(interpolator.output_param)
        print(f"pos: {pos}, vel: {vel}, acc: {acc}")
    print(f'Calculation duration: {first_output.calculation_duration:0.1f} [µs]')
    print(f'Trajectory duration: {first_output.trajectory.duration:0.4f} [s]')
