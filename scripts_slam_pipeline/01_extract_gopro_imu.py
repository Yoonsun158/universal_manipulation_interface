"""
python scripts_slam_pipeline/01_extract_gopro_imu.py data_workspace/cup_in_the_wild/20240105_zhenjia_packard_2nd_conference_room
"""
# %%
import sys
import os

ROOT_DIR = os.path.dirname(os.path.dirname(__file__))
sys.path.append(ROOT_DIR)
os.chdir(ROOT_DIR)

# %% 
import pathlib # 提供了面向对象的文件路径操作，相比传统的字符串拼接更安全且跨平台
import click # 用于创建命令行接口的库，将脚本包装成带有参数的命令行程序（CLI）
import subprocess # 允许脚本启动并与外部程序进行交互
import multiprocessing # 提供了在 Python 中进行并行处理的工具，获取系统的 CPU 核心数，并利用 concurrent.futures.ThreadPoolExecutor 实现多线程异步执行
import concurrent.futures # 提供了一个高级接口，用于异步执行调用可调用对象（如函数）的任务，支持线程池和进程池两种方式，简化了并行编程的复杂性
from tqdm import tqdm # 用于显示循环进度的库，提供了一个可视化的进度条，帮助用户了解长时间运行的任务的完成情况

# %% 主要功能是从 GoPro 录制的视频文件中提取 IMU（惯性测量单元）数据（包括加速度计和陀螺仪数据），
# 并将其保存为 imu_data.json 文件。
@click.command()
@click.option('-d', '--docker_image', default="chicheng/openicc:latest")
@click.option('-n', '--num_workers', type=int, default=None)
@click.option('-np', '--no_docker_pull', is_flag=True, default=False, help="pull docker image from docker hub")
@click.argument('session_dir', nargs=-1)
def main(docker_image, num_workers, no_docker_pull, session_dir):
    if num_workers is None:
        num_workers = multiprocessing.cpu_count()

    # pull docker
    if not no_docker_pull:
        print(f"Pulling docker image {docker_image}")
        cmd = [
            'docker',
            'pull',
            docker_image
        ]
        p = subprocess.run(cmd)
        if p.returncode != 0:
            print("Docker pull failed!")
            exit(1) # 如果 Docker pull 失败，程序将打印错误消息并退出，返回状态码为 1，表示发生了错误。

    # 脚本会遍历 session_dir/demos 下所有的子目录。
    # 目标是寻找包含 raw_video.mp4 的文件夹（通常一个文件夹代表一次采集的动作演示）。
    for session in session_dir:
        input_dir = pathlib.Path(os.path.expanduser(session)).joinpath('demos')  #
        input_video_dirs = [x.parent for x in input_dir.glob('*/raw_video.mp4')]
        print(f'Found {len(input_video_dirs)} video dirs')

        # 进度条：使用 tqdm 显示整体的处理进度。
        # 使用 ThreadPoolExecutor 并行处理多个视频文件夹 
        with tqdm(total=len(input_video_dirs)) as pbar:
            # one chunk per thread, therefore no synchronization needed
            with concurrent.futures.ThreadPoolExecutor(max_workers=num_workers) as executor:
                futures = set()# 集合是一个无序且不包含重复元素的容器。从命名约定来看，这个集合通常用于存储 concurrent.futures.Future 对象，这些对象代表了尚未完成的异步或并行计算任务。
                for video_dir in tqdm(input_video_dirs): # ?
                    video_dir = video_dir.absolute()
                    # 如果在该目录下已经存在 imu_data.json，则跳过该文件夹，避免重复计算。
                    if video_dir.joinpath('imu_data.json').is_file():
                        print(f"imu_data.json already exists, skipping {video_dir.name}")
                        continue
                    mount_target = pathlib.Path('/data')

                    video_path = mount_target.joinpath('raw_video.mp4') # video_path 是 Docker 容器内的路径，指向挂载的 /data 目录下的 raw_video.mp4 文件。这个路径将被传递给 Docker 容器中的 Node.js 脚本 extract_metadata_single.js，用于从视频中提取 IMU 数据。
                    json_path = mount_target.joinpath('imu_data.json') # json_path 是 Docker 容器内的路径，指向挂载的 /data 目录下的 imu_data.json 文件。这个路径将被传递给 Docker 容器中的 Node.js 脚本 extract_metadata_single.js，用于将提取的 IMU 数据保存为 JSON 文件。

                    # run imu extractor
                    # 该命令会在 Docker 容器中运行一个 Node.js 脚本 extract_metadata_single.js，专门用于从 GoPro 视频中提取 IMU 数据。
                    cmd = [
                        'docker',
                        'run',
                        '--rm', # 在 Docker 容器停止运行后，立即自动将其从系统中删除。
                        '--volume', str(video_dir) + ':' + '/data', # 构建了 Docker 挂载卷（Volume） 的参数字符串，将宿主机上的 video_dir 目录挂载到 Docker 容器内的 /data 目录。这样，容器内的脚本就可以访问和处理 video_dir 中的视频文件了。
                        docker_image,
                        'node',
                        '/OpenImuCameraCalibrator/javascript/extract_metadata_single.js',
                        str(video_path),
                        str(json_path)
                    ]
                    
                    # 输出重定向：每个视频的提取日志会分别记录在该文件夹下的 
                    # extract_gopro_imu_stdout.txt 和 extract_gopro_imu_stderr.txt 中。
                    stdout_path = video_dir.joinpath('extract_gopro_imu_stdout.txt')
                    stderr_path = video_dir.joinpath('extract_gopro_imu_stderr.txt')

                    if len(futures) >= num_workers:
                        # limit number of inflight tasks
                        completed, futures = concurrent.futures.wait(futures, # completed：获取所有已进入 FINISHED 或 CANCELLED 状态的任务句柄。
                            return_when=concurrent.futures.FIRST_COMPLETED) # futures：接收剩余未完成的任务
                        pbar.update(len(completed))

                    futures.add(executor.submit
                        (
                        lambda x, stdo, stde: subprocess.run
                            (
                                x, 
                                cwd=str(video_dir),# 工作目录 (cwd)：通过 cwd=str(video_dir) 确保子进程在特定的视频路径下运行。
                                stdout=stdo.open('w'),
                                stderr=stde.open('w')
                            ), 
                        cmd, # cmd 赋值给 x
                        stdout_path, # stdout_path 赋值给 stdo
                        stderr_path # stderr_path 赋值给 stde
                        )
                    )
                    # print(' '.join(cmd))

                completed, futures = concurrent.futures.wait(futures)
                pbar.update(len(completed))

        print("Done! Result:")
        print([x.result() for x in completed])

# %%
if __name__ == "__main__":
    main()
