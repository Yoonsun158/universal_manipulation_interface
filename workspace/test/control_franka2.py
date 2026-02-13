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
from umi.common.precise_sleep import precise_wait
from umi.real_world.franka_interpolation_controller import FrankaInterpolationController
from umi.real_world.franka_hand_controller import FrankaHandController
from umi.real_world.keystroke_counter import (
    KeystrokeCounter, Key, KeyCode
)



# %%
@click.command()
@click.option('-rh', '--robot_hostname', default='192.168.0.5')
@click.option('-gh', '--gripper_hostname', default='192.168.0.5')
@click.option('-gp', '--gripper_port', type=int, default=4242)
@click.option('-f', '--frequency', type=float, default=30)
@click.option('-gs', '--gripper_speed', type=float, default=200.0)
def main(robot_hostname, gripper_hostname, gripper_port, frequency, gripper_speed):
    # conservative default speeds to reduce impacts
    # max_pos_speed: linear speed limit (m/s)
    max_pos_speed = 0.05
    # max_rot_speed: angular speed limit (rad/s)
    max_rot_speed = 0.2
    max_gripper_width = 0.08  # meters
    cube_diag = np.linalg.norm([1,1,1])
    tcp_offset = 0.13
    # tcp_offset = 0
    dt = 1/frequency
    command_latency = dt / 2


    with SharedMemoryManager() as shm_manager:
        with KeystrokeCounter() as key_counter, \
             FrankaInterpolationController(
            shm_manager=shm_manager,
            robot_ip=robot_hostname,
            frequency=200,
            Kx_scale=5.0,
            Kxd_scale=2.0,
            verbose=False
        ) as controller, \
             FrankaHandController(
            shm_manager=shm_manager,
            hostname=gripper_hostname,
            port=gripper_port,
            frequency=frequency,
            verbose=False
        ) as gripper:
            try:
                print('Ready!')
                # to account for recever interfance latency, use target pose
                # to init buffer.
                state = controller.get_state()
                # target_pose = state['TargetTCPPose']
                target_pose = state['ActualTCPPose']

                # target_pose = np.array([ 0.40328411,  0.00620825,  0.29310859, -2.26569407,  2.12426248, -0.00934497])
                # controller.servoL(target_pose, 5)
                # time.sleep(8)
                # exit()

                # gripper width in meters (franka hand): [0.0, 0.08]
                max_gripper_width = 0.08
                # init gripper target width from controller ring buffer if available
                try:
                    gstate = gripper.get_state()
                    gripper_target_width = float(gstate['gripper_width']) if gstate and 'gripper_width' in gstate else max_gripper_width
                except Exception:
                    gripper_target_width = max_gripper_width
                # sync controller start time for gripper
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

                    # handle key presses (discrete steps per press)
                    press_events = key_counter.get_press_events()
                    for key_stroke in press_events:
                        if key_stroke == KeyCode(char='q'):
                            stop = True
                        # translation
                        elif key_stroke == KeyCode(char='w'):
                            target_pose[0] += max_pos_speed / frequency
                        elif key_stroke == KeyCode(char='s'):
                            target_pose[0] -= max_pos_speed / frequency
                        elif key_stroke == KeyCode(char='a'):
                            target_pose[1] -= max_pos_speed / frequency
                        elif key_stroke == KeyCode(char='d'):
                            target_pose[1] += max_pos_speed / frequency
                        elif key_stroke == KeyCode(char='r'):
                            target_pose[2] += max_pos_speed / frequency
                        elif key_stroke == KeyCode(char='f'):
                            target_pose[2] -= max_pos_speed / frequency
                        # rotation (small rotvec increments)
                        elif key_stroke == KeyCode(char='i'):
                            target_pose[3] += max_rot_speed / frequency
                        elif key_stroke == KeyCode(char='k'):
                            target_pose[3] -= max_rot_speed / frequency
                        elif key_stroke == KeyCode(char='j'):
                            target_pose[4] -= max_rot_speed / frequency
                        elif key_stroke == KeyCode(char='l'):
                            target_pose[4] += max_rot_speed / frequency
                        elif key_stroke == KeyCode(char='u'):
                            target_pose[5] += max_rot_speed / frequency
                        elif key_stroke == KeyCode(char='o'):
                            target_pose[5] -= max_rot_speed / frequency
                        # gripper control: '+' increase width, '-' decrease, 'c' close, 'p' open
                        elif key_stroke == KeyCode(char='+') or key_stroke == KeyCode(char='='):
                            gripper_target_width = np.clip(gripper_target_width + 0.002, 0.0, max_gripper_width)
                            gripper.schedule_waypoint(gripper_target_width, t_command_target-time.monotonic()+time.time())
                        elif key_stroke == KeyCode(char='-'):
                            gripper_target_width = np.clip(gripper_target_width - 0.002, 0.0, max_gripper_width)
                            gripper.schedule_waypoint(gripper_target_width, t_command_target-time.monotonic()+time.time())
                        elif key_stroke == KeyCode(char='c'):
                            gripper_target_width = 0.0
                            gripper.schedule_waypoint(gripper_target_width, t_command_target-time.monotonic()+time.time())
                        elif key_stroke == KeyCode(char='p'):
                            gripper_target_width = max_gripper_width
                            gripper.schedule_waypoint(gripper_target_width, t_command_target-time.monotonic()+time.time())

                    precise_wait(t_sample)

                    # schedule pose waypoint to controller
                    controller.schedule_waypoint(target_pose,
                        t_command_target-time.monotonic()+time.time())

                    precise_wait(t_cycle_end)
                    iter_idx += 1
                    # print(1/(time.time() -s))
            finally:
                pass

    controller.terminate_current_policy()
# %%
if __name__ == '__main__':
    main()