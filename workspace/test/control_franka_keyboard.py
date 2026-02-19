# %%
import sys
import os

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.dirname(__file__)))
sys.path.append(ROOT_DIR)
os.chdir(ROOT_DIR)

import click
import time
import numpy as np
from multiprocessing.managers import SharedMemoryManager
from umi.common.precise_sleep import precise_wait
from umi.real_world.franka_interpolation_controller import FrankaInterpolationController
from umi.real_world.franka_hand_controller import FrankaHandController
from umi.real_world.keystroke_counter import KeystrokeCounter, KeyCode


@click.command()
@click.option('-rh', '--robot_hostname', default='192.168.0.5')
@click.option('-gh', '--gripper_hostname', default='192.168.0.5')
@click.option('-gp', '--gripper_port', type=int, default=4242)
@click.option('-f', '--frequency', type=float, default=30) #（不能超过60）这个频率决定分发指令序列的时间间隔和命令延迟，设置过高可能会导致命令积压和系统不稳定。
def main(robot_hostname, gripper_hostname, gripper_port, frequency):
    max_pos_speed = 0.02
    max_rot_speed = 0.02
    max_gripper_width = 0.08
    dt = 1 / frequency
    command_latency = dt / 2

    with SharedMemoryManager() as shm_manager:
        with KeystrokeCounter() as key_counter, \
             FrankaInterpolationController(
                 shm_manager=shm_manager,
                 robot_ip=robot_hostname,
                 frequency=1000, 
                 Kx_scale=5.0,
                 Kxd_scale=2.0,
                 verbose=False
             ) as controller, \
             FrankaHandController(
                 shm_manager=shm_manager,
                 hostname=gripper_hostname,
                 port=gripper_port,
                 frequency=1000,
                 move_max_speed=0.1,
                 verbose=False
             ) as gripper:
            state = controller.get_state()
            target_pose = state['ActualTCPPose']

            try:
                gstate = gripper.get_state()
                gripper_target_width = float(gstate['gripper_width']) if gstate and 'gripper_width' in gstate else max_gripper_width
            except Exception:
                gripper_target_width = max_gripper_width

            t_start = time.monotonic()
            gripper.restart_put(t_start - time.monotonic() + time.time())

            iter_idx = 0
            stop = False
            print('Ready!')
            while not stop:
                state = controller.get_state()
                t_cycle_end = t_start + (iter_idx + 1) * dt
                t_sample = t_cycle_end - command_latency
                t_command_target = t_cycle_end + dt

                press_events = key_counter.get_press_events()
                for key_stroke in press_events:
                    if key_stroke == KeyCode(char='q'):
                        stop = True
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
                    elif key_stroke == KeyCode(char='+') or key_stroke == KeyCode(char='='):
                        gripper_target_width = np.clip(gripper_target_width + 0.01, 0.0, max_gripper_width)
                        gripper.schedule_waypoint(gripper_target_width, t_command_target - time.monotonic() + time.time())
                    elif key_stroke == KeyCode(char='-'):
                        gripper_target_width = np.clip(gripper_target_width - 0.01, 0.0, max_gripper_width)
                        gripper.schedule_waypoint(gripper_target_width, t_command_target - time.monotonic() + time.time())
                    elif key_stroke == KeyCode(char='c'):
                        gripper_target_width = 0.0
                        gripper.schedule_waypoint(gripper_target_width, t_command_target - time.monotonic() + time.time())
                    elif key_stroke == KeyCode(char='p'):
                        gripper_target_width = max_gripper_width
                        gripper.schedule_waypoint(gripper_target_width, t_command_target - time.monotonic() + time.time())

                precise_wait(t_sample)

                controller.schedule_waypoint(target_pose, t_command_target - time.monotonic() + time.time())

                precise_wait(t_cycle_end)
                iter_idx += 1

    controller.terminate_current_policy()


if __name__ == '__main__':
    main()