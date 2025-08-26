from typing import Dict

import numpy as np
from scipy.spatial.transform import Rotation as R

from gello.robots.robot import Robot


class URRobot(Robot):
    """A class representing a UR robot."""

    def __init__(self, robot_ip: str = "192.168.1.10", no_gripper: bool = False):
        import rtde_control
        import rtde_receive

        try:
            self.robot = rtde_control.RTDEControlInterface(robot_ip)
            self.r_inter = rtde_receive.RTDEReceiveInterface(robot_ip)
            print("robot connected")
        except Exception as e:
            print(f"Failed to connect to robot at {robot_ip}: {e}")
            raise

        if not no_gripper:
            from gello.robots.robotiq_gripper import RobotiqGripper
            self.gripper = RobotiqGripper()
            self.gripper.connect(hostname=robot_ip, port=63352)
            print("gripper connected")
            self.gripper.activate(auto_calibrate=False)

        self._free_drive = False
        self.robot.endFreedriveMode()
        self._use_gripper = not no_gripper
        self.tcp_offset = self.robot.getTCPOffset()

        # Impedance control parameters
        self.imp_kp = 10000.0
        self.imp_kd = 2200.0
        self.ori_kp = 100.0
        self.ori_kd = 22.0
        self.imp_delta = 0.05
        self.forcemode_damping = 0.02
        self.dt = 1.0 / 500.0
        
        # Force mode configuration
        self.fm_task_frame = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.fm_selection_vector = [1, 1, 1, 1, 1, 1]
        self.fm_limits = [0.5, 0.5, 0.1, 1., 1., 1.]
        
        # Initialize force mode
        self.robot.forceModeSetDamping(self.forcemode_damping)
        self.robot.zeroFtSensor()

    def num_dofs(self) -> int:
        """Get the number of joints of the robot.

        Returns:
            int: The number of joints of the robot.
        """
        if self._use_gripper:
            return 7
        return 6

    def _get_gripper_pos(self) -> float:
        import time

        time.sleep(0.01)
        gripper_pos = self.gripper.get_current_position()
        assert 0 <= gripper_pos <= 255, "Gripper position must be between 0 and 255"
        return gripper_pos / 255

    def get_joint_state(self) -> np.ndarray:
        """Get the current state of the leader robot.

        Returns:
            T: The current state of the leader robot.
        """
        robot_joints = self.r_inter.getActualQ()
        if self._use_gripper:
            gripper_pos = self._get_gripper_pos()
            pos = np.append(robot_joints, gripper_pos)
        else:
            pos = robot_joints
        return pos

    def command_joint_state(self, joint_state: np.ndarray) -> None:
        """Command joints by mapping to EE impedance target via FK, then forceMode."""
        
        target_q = np.array(joint_state[:6], dtype=float)
        
        target_pose_xyz_rxryrz = self.robot.getForwardKinematics(target_q, self.tcp_offset)
        
        # Convert to quaternion format [x,y,z,qx,qy,qz,qw]
        target_pose_quat = self._pose_to_quat(target_pose_xyz_rxryrz)

        if self._use_gripper and joint_state.shape[0] >= 7:
            target_pose_quat = np.concatenate([target_pose_quat, [joint_state[6]]])
        
        self.command_ee_pos_quat(target_pose_quat)

    def freedrive_enabled(self) -> bool:
        """Check if the robot is in freedrive mode.

        Returns:
            bool: True if the robot is in freedrive mode, False otherwise.
        """
        return self._free_drive

    def set_freedrive_mode(self, enable: bool) -> None:
        """Set the freedrive mode of the robot.

        Args:
            enable (bool): True to enable freedrive mode, False to disable it.
        """
        if enable and not self._free_drive:
            self._free_drive = True
            self.robot.freedriveMode()
        elif not enable and self._free_drive:
            self._free_drive = False
            self.robot.endFreedriveMode()

    def get_observations(self) -> Dict[str, np.ndarray]:
        joints = self.get_joint_state()
        pos_quat = self._pose_to_quat(self.r_inter.getActualTCPPose())
        gripper_pos = np.array([joints[-1]])
        return {
            "joint_positions": joints,
            "joint_velocities": joints,
            "ee_pos_quat": pos_quat,
            "gripper_position": gripper_pos,
        }

    # -------------------- Helper methods --------------------
    def _pose_to_quat(self, pose_xyz_rxryrz: np.ndarray) -> np.ndarray:
        """Convert pose [x,y,z,rx,ry,rz] to [x,y,z,qx,qy,qz,qw]."""
        pos = np.array(pose_xyz_rxryrz[:3], dtype=float)
        rotvec = np.array(pose_xyz_rxryrz[3:], dtype=float)
        quat = R.from_rotvec(rotvec).as_quat()  # Returns [x,y,z,w]
        return np.concatenate([pos, quat])

    def command_ee_pos_quat(self, target_pose_quat: np.ndarray) -> None:
        """Impedance control step in task space via RTDE forceMode.

        Args:
            target_pose_quat: np.ndarray of shape (7,) -> [x, y, z, qx, qy, qz, qw]
        """
        curr_pose = self.r_inter.getActualTCPPose()
        curr_quat_pose = self._pose_to_quat(curr_pose)
        curr_vel = np.array(self.r_inter.getActualTCPSpeed(), dtype=float)
        
        # Position impedance
        diff_p = target_pose_quat[:3] - curr_quat_pose[:3]
        
        diff_p = np.clip(diff_p, a_min=-self.imp_delta, a_max=self.imp_delta)
        vel_delta = 2.0 * self.imp_delta / self.dt
        diff_d = np.clip(-curr_vel[:3], a_min=-vel_delta, a_max=vel_delta)
        force = self.imp_kp * diff_p + self.imp_kd * diff_d

        # Orientation impedance using scipy Rotation (robust to discontinuities)
        target_quat = target_pose_quat[3:7]  # Extract quaternion (4 values)
        curr_quat = curr_quat_pose[3:7]      # Extract quaternion (4 values)
        
        # Use scipy Rotation for robust orientation error calculation
        rot_diff = R.from_quat(target_quat) * R.from_quat(curr_quat).inv()
        rotvec_err = rot_diff.as_rotvec()
        ang_vel = curr_vel[3:]
        torque = self.ori_kp * rotvec_err + self.ori_kd * (-ang_vel)

        wrench = np.concatenate([force, torque]).astype(float)
        
        t_start = self.robot.initPeriod()
        ok = self.robot.forceMode(
            self.fm_task_frame,
            self.fm_selection_vector,
            wrench.tolist(),
            2,
            self.fm_limits,
        )
        if not ok:
            self.robot.forceModeStop()
            raise RuntimeError("forceMode failed")
        if self._use_gripper and target_pose_quat.shape[0] == 8:
            gripper_pos = target_pose_quat[-1] * 255.0
            self.gripper.move(gripper_pos, 255, 10)
        self.robot.waitPeriod(t_start)

    def cleanup(self) -> None:
        """Clean up robot state when shutting down."""
        try:
            print("Cleaning up robot state...")
            # Stop force mode first
            self.robot.forceModeStop()
            
            # Hold current position briefly to prevent drift
            current_joints = self.r_inter.getActualQ()
            self.robot.servoJ(current_joints, 0.5, 0.5, 0.1, 0.1, 300)
            print("Robot cleanup complete")
        except Exception as e:
            print(f"Error during robot cleanup: {e}")


def main():
    robot_ip = "192.168.1.11"
    ur = URRobot(robot_ip, no_gripper=True)
    print(ur)
    ur.set_freedrive_mode(True)
    print(ur.get_observations())


if __name__ == "__main__":
    main()
