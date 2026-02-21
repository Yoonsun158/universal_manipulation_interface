# %%
import sys
import os
import json

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.dirname(__file__)))
sys.path.append(ROOT_DIR)
os.chdir(ROOT_DIR)

# %%
import cv2
import time
from multiprocessing.managers import SharedMemoryManager
from umi.real_world.uvc_camera import UvcCamera
from umi.common.usb_util import create_usb_list
from umi.common.precise_sleep import precise_wait

# %%
def load_points(points_file):
    """从 JSON 文件中加载点"""
    if not os.path.exists(points_file):
        print(f"点文件 {points_file} 不存在，跳过加载。")
        return []
    with open(points_file, "r", encoding="utf-8") as f:
        return json.load(f)

# %%
def draw_points(img, points, color=(0, 0, 255)):
    """在图像上绘制点"""
    for idx, (x, y) in enumerate(points):
        x = x + 160
        cv2.circle(img, (x, y), 5, color, -1)
        cv2.putText(img, str(idx), (x + 6, y - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)

# %%
def main():
    cv2.setNumThreads(1)

    dev_video_path = '/dev/video0'

    # enumerate USB device to find Elgato Capture Card
    device_list = create_usb_list()
    dev_usb_path = None
    for dev in device_list:
        if 'Elgato' in dev['description']:
            dev_usb_path = dev['path']
            print('Found :', dev['description'])
            break
    
    fps = 60
    dt = 1 / fps

    # 加载点
    points_file = os.path.join(ROOT_DIR,"workspace/test/points.json")
    points = load_points(points_file)
    show_points = True  # 显示点的开关

    with SharedMemoryManager() as shm_manager:
        with UvcCamera(
            shm_manager=shm_manager,
            dev_video_path=dev_video_path,
            # dev_usb_path=dev_usb_path
        ) as camera:
            print('Ready!')
            t_start = time.monotonic()
            iter_idx = 0
            while True:
                t_cycle_end = t_start + (iter_idx + 1) * dt

                data = camera.get()
                img = data['color']
                # img = cv2.resize(img, (960, 720))
                print(img.shape)
                # 如果显示开关开启，绘制点
                if show_points:
                    draw_points(img, points)

                # 显示视频帧
                cv2.imshow('frame', img)
                key = cv2.pollKey() & 0xFF

                # 按键处理
                if key == ord('q'):  # 按 'q' 退出
                    break
                elif key == ord('p'):  # 按 'p' 切换点的显示状态
                    show_points = not show_points
                    print(f"显示点: {'开启' if show_points else '关闭'}")

                precise_wait(t_cycle_end)
                iter_idx += 1
                            
# %%
if __name__ == '__main__':
    main()
