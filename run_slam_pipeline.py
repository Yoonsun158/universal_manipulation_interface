"""
Main script for UMI SLAM pipeline.
python run_slam_pipeline.py <session_dir>
"""

import sys
import os

ROOT_DIR = os.path.dirname(__file__)
sys.path.append(ROOT_DIR)
os.chdir(ROOT_DIR)

# %%
import pathlib
import click
import subprocess

# %%
@click.command()
@click.argument('session_dir', nargs=-1)
@click.option('-c', '--calibration_dir', type=str, default=None) # default=None表示，这是定义了一个可选的参数，如果没有default=None，则这个参数就是必需的，必须在命令行中提供，否则会报错。
def main(session_dir, calibration_dir):
    script_dir = pathlib.Path(__file__).parent.joinpath('scripts_slam_pipeline')
    if calibration_dir is None:
        calibration_dir = pathlib.Path(__file__).parent.joinpath('example', 'calibration')
    else:
        calibration_dir = pathlib.Path(calibration_dir)
    assert calibration_dir.is_dir()

    for session in session_dir: # session_dir是一个列表或元组，每个元素都是存储同一个场景采集数据的文件夹路径
        session = pathlib.Path(os.path.expanduser(session)).absolute() # os.path.expanduser()函数用于将路径中的~符号展开为用户的主目录路径，absolute()方法将路径转换为绝对路径

# %%
        print("############## 00_process_videos #############") # 视频预处理（如转码、调整分辨率）
        script_path = script_dir.joinpath("00_process_videos.py")
        assert script_path.is_file()
        cmd = [
            'python', str(script_path),
            str(session)
        ]
        result = subprocess.run(cmd)
        assert result.returncode == 0

# %%  # 从 GoPro 视频中提取 IMU 数据，生成一个包含时间戳、加速度计和陀螺仪数据的 CSV 文件，供后续 SLAM 处理使用
        print("############# 01_extract_gopro_imu ###########")
        script_path = script_dir.joinpath("01_extract_gopro_imu.py")
        assert script_path.is_file()
        cmd = [
            'python', str(script_path),
            str(session)
        ]
        result = subprocess.run(cmd)
        assert result.returncode == 0

# %%
        print("############# 02_create_map ###########") # 使用 mapping 文件夹中的视频创建 SLAM 地图
        script_path = script_dir.joinpath("02_create_map.py")
        assert script_path.is_file()
        demo_dir = session.joinpath('demos')
        mapping_dir = demo_dir.joinpath('mapping')
        assert mapping_dir.is_dir()
        map_path = mapping_dir.joinpath('map_atlas.osa')
        if not map_path.is_file():
            cmd = [
                'python', str(script_path),
                '--input_dir', str(mapping_dir),
                '--map_path', str(map_path)
            ]
            result = subprocess.run(cmd)
            assert result.returncode == 0
            assert map_path.is_file()

# %%
        print("############# 03_batch_slam ###########") # 批量处理 demos 目录下的所有动作视频
        script_path = script_dir.joinpath("03_batch_slam.py")
        assert script_path.is_file()
        cmd = [
            'python', str(script_path),
            '--input_dir', str(demo_dir),
            '--map_path', str(map_path)
        ]
        result = subprocess.run(cmd)
        assert result.returncode == 0

# %%
        print("############# 04_detect_aruco ###########") #提供世界坐标系的绝对参考, 检测视频中的 ArUco 码（二维码标记），需要提供相机内参和 ArUco 配置文件
        script_path = script_dir.joinpath("04_detect_aruco.py")
        assert script_path.is_file()
        camera_intrinsics = calibration_dir.joinpath('gopro_intrinsics_2_7k.json')
        aruco_config = calibration_dir.joinpath('aruco_config.yaml')
        assert camera_intrinsics.is_file()
        assert aruco_config.is_file()

        cmd = [
            'python', str(script_path),
            '--input_dir', str(demo_dir),
            '--camera_intrinsics', str(camera_intrinsics),
            '--aruco_yaml', str(aruco_config)
        ]
        result = subprocess.run(cmd)
        assert result.returncode == 0

# %%
        print("############# 05_run_calibrations ###########") # 运行校准脚本, 计算相机与夹爪/机械臂之间的相对位置关系
        script_path = script_dir.joinpath("05_run_calibrations.py")
        assert script_path.is_file()
        cmd = [
            'python', str(script_path),
            str(session)
        ]
        result = subprocess.run(cmd)
        assert result.returncode == 0

# %%
        print("############# 06_generate_dataset_plan ###########") # 生成数据集计划, 根据 SLAM 结果和校准结果，生成一个数据集计划文件，指导后续的数据处理和分析步骤
        script_path = script_dir.joinpath("06_generate_dataset_plan.py")
        assert script_path.is_file()
        cmd = [
            'python', str(script_path),
            '--input', str(session)
        ]
        result = subprocess.run(cmd)
        assert result.returncode == 0

## %%
if __name__ == "__main__":
    main(['example_demo_session'], 'example/calibration')
