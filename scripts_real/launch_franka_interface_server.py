import zerorpc
from polymetis import RobotInterface
from polymetis.gripper_interface import GripperInterface
import scipy.spatial.transform as st
import numpy as np
import torch

class FrankaInterface:
    def __init__(self):
        self.robot = RobotInterface('localhost')
        self.gripper = GripperInterface(ip_address="localhost")

    # --- Robot wrappers ---
    def get_ee_pose(self):
        data = self.robot.get_ee_pose()
        pos = data[0].numpy()
        quat_xyzw = data[1].numpy()
        rot_vec = st.Rotation.from_quat(quat_xyzw).as_rotvec()
        return np.concatenate([pos, rot_vec]).tolist()
    
    def get_joint_positions(self):
        return self.robot.get_joint_positions().numpy().tolist()
    
    def get_joint_velocities(self):
        return self.robot.get_joint_velocities().numpy().tolist()
    
    def move_to_joint_positions(self, positions, time_to_go):
        self.robot.move_to_joint_positions(
            positions=torch.Tensor(positions),
            time_to_go=time_to_go
        )
    
    # --- Gripper wrappers ---
    def get_gripper_state(self):
        state = self.gripper.get_state()
        return {
            "width": getattr(state, "width", None),
            "is_grasped": getattr(state, "is_grasped", None),
            "force": getattr(state, "force", None),
        }

    def goto_gripper(self, width: float, speed: float, force: float, blocking: bool = True):
        self.gripper.goto(width=width, speed=speed, force=force, blocking=blocking)

    def grasp_gripper(
        self,
        speed: float,
        force: float,
        grasp_width: float = 0.0,
        epsilon_inner: float = -1.0,
        epsilon_outer: float = -1.0,
        blocking: bool = True,
    ):
        self.gripper.grasp(
            speed=speed,
            force=force,
            grasp_width=grasp_width,
            epsilon_inner=epsilon_inner,
            epsilon_outer=epsilon_outer,
            blocking=blocking,
        )

    def open_gripper(self, width: float = 0.08, speed: float = 0.1, force: float = 10.0, blocking: bool = True):
        self.goto_gripper(width=width, speed=speed, force=force, blocking=blocking)

    def close_gripper(self, speed: float = 0.1, force: float = 10.0, grasp_width: float = 0.0, blocking: bool = True):
        self.grasp_gripper(speed=speed, force=force, grasp_width=grasp_width, blocking=blocking)

    # --- Cartesian impedance control ---

    def start_cartesian_impedance(self, Kx, Kxd):
        self.robot.start_cartesian_impedance(
            Kx=torch.Tensor(Kx),
            Kxd=torch.Tensor(Kxd)
        )

    def update_desired_ee_pose(self, pose):
        pose = np.asarray(pose)
        self.robot.update_desired_ee_pose(
            position=torch.Tensor(pose[:3]),
            orientation=torch.Tensor(st.Rotation.from_rotvec(pose[3:]).as_quat())
        )

    def terminate_current_policy(self):
        self.robot.terminate_current_policy()

s = zerorpc.Server(FrankaInterface())
s.bind("tcp://0.0.0.0:4242")
s.run()