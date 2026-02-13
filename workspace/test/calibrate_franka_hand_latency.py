# %%
import sys
import os

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.dirname(__file__)))
sys.path.append(ROOT_DIR)
os.chdir(ROOT_DIR)

# %%
import click
import cv2
import time
import numpy as np
from collections import deque
from tqdm import tqdm
from multiprocessing.managers import SharedMemoryManager
from umi.real_world.franka_hand_controller import FrankaHandController
from umi.real_world.franka_interpolation_controller import FrankaInterface
from umi.common.precise_sleep import precise_sleep
from umi.common.latency_util_franka import get_latency
from matplotlib import pyplot as plt

# %%
@click.command()
@click.option('-h', '--hostname', default='192.168.0.5')
# @click.option('-p', '--port', type=int, default=1000)
@click.option('-f', '--frequency', type=float, default=1000)

def main(hostname, frequency):
    duration = 20.0
    sample_dt = 1.0 / 50
    k = int(duration / sample_dt)
    sample_t = np.linspace(0, duration, k, endpoint=True)
    # value = np.sin(sample_t * duration / 1.5) * 0.5 + 0.5
    P = 4 # 设置周期
    value = np.sin(2 * np.pi * sample_t / P) * 0.5 + 0.5
    # 将归一化的 value 映射到 [min_width, max_width]
    # min_width = 0.006  # 6 mm
    max_width = 0.077   # 77 mm (约为 Franka hand 最大开度)
    # width = min_width + value * (max_width - min_width)
    width = value * max_width

    # 记录目标和实际数据（目标时间戳在下发时生成并复用为 `timestamps`）
    actual_widths = []
    actual_timestamps = []

    # 使用 SharedMemoryManager 上下文并启动 FrankaHandController，批量下发时间戳轨迹
    with SharedMemoryManager() as shm_manager:
        with FrankaHandController(
            shm_manager=shm_manager,
            hostname=hostname,
            # port=1000,010
            frequency=int(frequency),
            move_max_speed=0.2,
            # get_max_k=int(k*1.2),
            get_max_k=int(frequency * (duration+1.0)),
            command_queue_size=int(k*1.2),
            launch_timeout=5,
            verbose=False
        ) as controller:
            controller.start_wait()

            # 清空 ring buffer 中旧数据
            controller.ring_buffer.clear()
            # 继续 schedule_waypoint .

            # 先发送一个初始点并等待控制器就绪
            controller.schedule_waypoint(width[0], time.time() + 0.3)
            precise_sleep(1.0)

            # 一次性下发所有带时间戳的轨迹点（由控制器本地按时间执行）
            timestamps = time.time() + sample_t + 1

            print(f"{len(timestamps)}, {len(width)}")

            for i in range(k):
                controller.schedule_waypoint(float(width[i]), float(timestamps[i]))
                time.sleep(0.0)
            # 等待执行完成并采样一段时间，让 ring buffer 收集数据
            precise_sleep(duration + 6.0)

            # 从控制器的 ring buffer 获取所有 gripper 状态
            states = controller.get_all_state()
            command_queue_size=int(frequency * (duration)),
    print(f"{len(states['gripper_width'])}, {len(states['gripper_receive_timestamp'])}")

    # # 从 states 中提取 gripper_width 和 gripper_receive_timestamp
    # actual_widths = np.array(states['gripper_width'])
    # actual_timestamps = np.array(states['gripper_receive_timestamp'])
    actual_widths = states['gripper_width']
    actual_timestamps = states['gripper_receive_timestamp']
    # # 去除nan
    # valid = ~np.isnan(actual_widths)
    # actual_widths = actual_widths[valid]
    # actual_timestamps = actual_timestamps[valid]

    # 延迟测量
    latency, info = get_latency(
        x_target=width,
        t_target=timestamps,
        x_actual=actual_widths,
        t_actual=actual_timestamps,
    )
    print(f"End-to-end latency: {latency}sec")

    # plot everything
    fig, axes = plt.subplots(1, 3)
    fig.set_size_inches(15, 5, forward=True)

    ax = axes[0]
    ax.plot(info['lags'], info['correlation'])
    ax.set_xlabel('lag')
    ax.set_ylabel('cross-correlation')
    ax.set_title("Cross Correlation")

    ax = axes[1]
    ax.plot(timestamps, width, label='target')
    ax.plot(actual_timestamps, actual_widths, label='actual')
    ax.set_xlabel('time')
    ax.set_ylabel('gripper-width')
    ax.legend()
    ax.set_title("Raw observation")

    ax=axes[2]
    t_samples = info['t_samples'] - info['t_samples'][0]
    ax.plot(t_samples, info['x_target'], label='target')
    ax.plot(t_samples-latency, info['x_actual'], label='actual-latency')
    ax.set_xlabel('time')
    ax.set_ylabel('gripper-width')
    ax.legend()
    ax.set_title(f"Aligned with latency={latency}")
    plt.show()

    # 初始化夹抓
    try:
        robot = FrankaInterface(hostname)
        # 将夹抓打开到最大宽度（单位：米），阻塞直到完成
        robot.goto_gripper(width=max_width, speed=0.08, force=10.0, blocking=True)
        robot.close()
        print("Gripper initialized (opened).")
    except Exception as e:
        print(f"Failed to initialize gripper: {e}")

# %%
if __name__ == '__main__':
    main()
