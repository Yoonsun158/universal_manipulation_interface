# %%
import sys
import os

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.dirname(__file__)))
print(ROOT_DIR)
sys.path.append(ROOT_DIR)
os.chdir(ROOT_DIR)

# %%
import click
import time
import numpy as np
from multiprocessing.managers import SharedMemoryManager
import scipy.spatial.transform as st
from umi.real_world.spacemouse_shared_memory import Spacemouse
from umi.real_world.franka_hand_controller import FrankaHandController
from umi.common.precise_sleep import precise_wait
from umi.real_world.keystroke_counter import (
    KeystrokeCounter, Key, KeyCode
)
from umi.real_world.franka_interpolation_controller import FrankaInterpolationController


# %%
@click.command()
@click.option('-rh', '--robot_hostname', default='192.168.0.5')
@click.option('-gh', '--gripper_hostname', default='192.168.0.5')
@click.option('-gp', '--gripper_port', type=int, default=4242)
@click.option('-f', '--frequency', type=float, default=30) #(不能超过60)这个频率决定分发指令序列的时间间隔和命令延迟，设置过高可能会导致命令积压和系统不稳定。
@click.option('-gs', '--gripper_speed', type=float, default=200.0)
def main(robot_hostname, gripper_hostname, gripper_port, frequency, gripper_speed):
    max_pos_speed = 0.25
    max_rot_speed = 0.6
    # frankahand width in meters (was 90 mm for WSG)
    max_gripper_width = 0.08
    cube_diag = np.linalg.norm([1,1,1])
    tcp_offset = 0.13
    # tcp_offset = 0
    dt = 1/frequency
    command_latency = dt / 2

    with SharedMemoryManager() as shm_manager:
        with FrankaHandController(
            shm_manager=shm_manager,
            hostname=gripper_hostname,
            port=gripper_port,
            frequency=1000,
            # CLI gripper_speed is in mm/s, convert to m/s for FrankaHand
            move_max_speed=gripper_speed * 0.001,
            verbose=False
        ) as gripper, \
        KeystrokeCounter() as key_counter, \
        FrankaInterpolationController(
            shm_manager=shm_manager,
            robot_ip=robot_hostname,
            frequency=1000,
            Kx_scale=5.0,
            Kxd_scale=2.0,
            verbose=False
        ) as controller, \
        Spacemouse(
            shm_manager=shm_manager
        ) as sm:
            print('Ready!')
            # to account for recever interfance latency, use target pose
            # to init buffer.
            state = controller.get_state()
            # target_pose = state['TargetTCPPose']
            target_pose = state['ActualTCPPose']

            # print(target_pose)
            # exit()
        
            # target_pose = np.array([ 0.40328411,  0.00620825,  0.29310859, -2.26569407,  2.12426248, -0.00934497])
            # controller.servoL(target_pose, 5)
            # time.sleep(8)
            # exit()
        
            # FrankaHandController reports 'gripper_width' in meters
            gs = gripper.get_state()
            gripper_target_pos = gs.get('gripper_width', 0.0) if isinstance(gs, dict) else 0.0
            t_start = time.monotonic()
            gripper.restart_put(t_start-time.monotonic() + time.time())
            
            iter_idx = 0
            stop = False
            while not stop:
                state = controller.get_state()
                # print(target_pose - state['ActualTCPPose'])
                s = time.time()
                t_cycle_end = t_start + (iter_idx + 1) * dt
                t_sample = t_cycle_end - command_latency
                t_command_target = t_cycle_end + dt

                # handle key presses
                press_events = key_counter.get_press_events()
                for key_stroke in press_events:
                    # if key_stroke != None:
                    #     print(key_stroke)
                    if key_stroke == KeyCode(char='q'):
                        stop = True
                precise_wait(t_sample)
                sm_state = sm.get_motion_state_transformed()
                # print(sm_state)
                dpos = sm_state[:3] * (max_pos_speed / frequency)
                drot_xyz = sm_state[3:] * (max_rot_speed / frequency)

                drot = st.Rotation.from_euler('xyz', drot_xyz)
                target_pose[:3] += dpos
                target_pose[3:] = (drot * \
                        st.Rotation.from_rotvec(target_pose[3:])
                ).as_rotvec()

                dpos = 0
                # convert CLI gripper_speed (mm/s) to meters per cycle
                gripper_step = (gripper_speed * 0.001) / frequency
                if sm.is_button_pressed(0):
                    # close gripper
                    dpos = -gripper_step
                if sm.is_button_pressed(1):
                    dpos = gripper_step
                gripper_target_pos = np.clip(gripper_target_pos + dpos, 0, max_gripper_width)

                controller.schedule_waypoint(target_pose, 
                    t_command_target-time.monotonic()+time.time())
                gripper.schedule_waypoint(gripper_target_pos, 
                    t_command_target-time.monotonic()+time.time())

                precise_wait(t_cycle_end)
                iter_idx += 1
                # print(1/(time.time() -s))


    controller.terminate_current_policy()
# %%
if __name__ == '__main__':
    main()