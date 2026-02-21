# %%
import sys
import os

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
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
from umi.real_world.franka_hand_controller import FrankaHandController, FrankaHandInterface
from umi.common.precise_sleep import precise_sleep
from umi.common.latency_util_discrete import get_latency
from matplotlib import pyplot as plt

# %%
@click.command()
@click.option('-h', '--hostname', default='192.168.0.5')
@click.option('-f', '--frequency', type=float, default=1000)
@click.option('-t', '--toggle_interval', type=float, default=3.0)
def main(hostname, frequency, toggle_interval):
    # 假设 period = 2.0
    period = toggle_interval * 2.0
    duration = 30.0
    # 采样间隔，可以设置得比周期小（例如 0.1s）以获得更平滑的信号
    sample_dt = toggle_interval

    k = int(duration / sample_dt)
    sample_t = np.linspace(0, duration, k, endpoint=True)
    
    # 生成方波：使用取模运算判断是否在半周期内
    value = ((sample_t % period) < (period / 2)).astype(float)
    
    max_width = 0.08   # 80 mm (Franka hand max open width)
    width = value * max_width

    # Record target and actual data
    actual_widths = []
    actual_timestamps = []

    # Use SharedMemoryManager and FrankaHandController
    with SharedMemoryManager() as shm_manager:
        with FrankaHandController(
            shm_manager=shm_manager,
            hostname=hostname,
            frequency=int(frequency),
            move_max_speed=0.2,
            get_max_k=int(frequency * (duration + 10.0)),
            command_queue_size=int(k * 1.5),
            launch_timeout=5,
            verbose=False
        ) as controller:
            controller.start_wait()

            # Clear old data from ring buffer
            controller.ring_buffer.clear()

            # Move to initial position and wait
            # controller.schedule_waypoint(width[0], time.time() + 0.3)
            # precise_sleep(1.0)

            # Schedule all waypoints at once
            timestamps = time.time() + sample_t + 1.0

            print(f"Scheduling {len(timestamps)} points...")
            for i in range(k):
                controller.schedule_waypoint(float(width[i]), float(timestamps[i]))
                time.sleep(0.0)
            
            # Wait for completion and some extra time for data collection
            print(f"Waiting for {duration + 2.0} seconds for execution...")
            precise_sleep(duration + 4.0)

            # Retrieve states from controller
            states = controller.get_all_state()

    actual_widths = states['gripper_position']
    actual_timestamps = states['gripper_receive_timestamp']

    if len(actual_widths) == 0:
        print("No data collected from gripper!")
        return

    # Calculate latency
    latency, info = get_latency(
        x_target=width,
        t_target=timestamps,
        x_actual=actual_widths,
        t_actual=actual_timestamps
    )
    print(f"Estimated latency: {latency} sec")

    # Plot results
    fig, axes = plt.subplots(1, 3, figsize=(18, 6))

    ax = axes[0]
    ax.plot(info['lags'], info['correlation'])
    ax.set_xlabel('Lag (sec)')
    ax.set_ylabel('Cross-correlation')
    ax.set_title("Cross-correlation vs Lag")

    ax = axes[1]
    ax.plot(timestamps, width, label='Target (Square Wave)')
    ax.plot(actual_timestamps, actual_widths, label='Actual')
    ax.set_xlabel('Timestamp')
    ax.set_ylabel('Gripper Width (m)')
    ax.legend()
    ax.set_title("Raw Data")

    ax = axes[2]
    t_samples = info['t_samples'] - info['t_samples'][0]
    ax.plot(t_samples, info['x_target'], label='Target (normalized)')
    ax.plot(t_samples - latency, info['x_actual'], label='Actual (aligned)')
    ax.set_xlabel('Time (sec)')
    ax.set_ylabel('Normalized Width')
    ax.legend()
    ax.set_title(f"Aligned Data (Latency={latency:.4f}s)")
    
    plt.tight_layout()
    plt.show()

    # Re-initialize or open gripper at the end
    try:
        robot = FrankaHandInterface(hostname)
        robot.goto_gripper(width=max_width, speed=0.08, force=10.0, blocking=True)
        robot.close()
        print("Gripper reset to open position.")
    except Exception as e:
        print(f"Failed to reset gripper: {e}")

# %%
if __name__ == '__main__':
    main()
