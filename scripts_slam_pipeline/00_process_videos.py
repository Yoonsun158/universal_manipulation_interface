"""
python scripts_slam_pipeline/00_process_videos.py data_workspace/toss_objects/20231113
"""
# %%
import sys
import os

ROOT_DIR = os.path.dirname(os.path.dirname(__file__))
sys.path.append(ROOT_DIR)
os.chdir(ROOT_DIR)

# %%
import pathlib # 用面向对象的方式处理路径（比 os.path 更好用）
import click 
import shutil # 用于文件操作，如移动、复制、删除等
from exiftool import ExifToolHelper # 调用系统的 exiftool 工具，用来读取视频里的隐藏信息（元数据，如相机序列号）。
from umi.common.timecode_util import mp4_get_start_datetime # 这是 UMI 自己的工具函数，用来读取视频开始录制的精确时间。从视频的元数据中获取视频的开始时间，返回一个 datetime 对象

# %%
@click.command(help='Session directories. Assumming mp4 videos are in <session_dir>/raw_videos')
@click.argument('session_dir', nargs=-1)
def main(session_dir):
    # 针对每一个 session_dir 进行处理，session_dir 是一个路径列表=['<session_dir_1>', '<session_dir_2>', ...]，
    # 每个路径对应一个 session 的目录
    for session in session_dir:
        session = pathlib.Path(os.path.expanduser(session)).absolute() # 将 session_dir 中的每个路径转换为绝对路径
        
        # hardcode subdirs
        # 构造目录<session_dir>/raw_videos 和 <session_dir>/demos
        input_dir = session.joinpath('raw_videos')  
        output_dir = session.joinpath('demos')
        
        # create raw_videos if don't exist
        # 如果不存在，建立 raw_videos 目录，并将<session_dir>下所有的 mp4 视频文件移动到 raw_videos 目录下
        if not input_dir.is_dir():
            input_dir.mkdir()
            print(f"{input_dir.name} subdir don't exits! Creating one and moving all mp4 videos inside.")
            for mp4_path in list(session.glob('**/*.MP4')) + list(session.glob('**/*.mp4')): # session.glob('**/*.MP4')是用来在 session 目录下递归查找所有扩展名为 .MP4 的文件，返回一个生成器对象，list()函数将其转换为一个列表。同样的，session.glob('**/*.mp4')是用来查找所有扩展名为 .mp4 的文件。然后将两个列表合并成一个列表，包含了所有扩展名为 .MP4 和 .mp4 的视频文件。
                out_path = input_dir.joinpath(mp4_path.name)
                shutil.move(mp4_path, out_path)
        
        # create mapping video if don't exist
        # 如果 raw_videos/mapping.mp4 不存在，找到<session_dir>下最大的 mp4 视频文件，
        # 并将其移动到 raw_videos 目录下，并重命名为 mapping.mp4
        mapping_vid_path = input_dir.joinpath('mapping.mp4')
        if (not mapping_vid_path.exists()) and not(mapping_vid_path.is_symlink()):
            max_size = -1
            max_path = None
            for mp4_path in list(input_dir.glob('**/*.MP4')) + list(input_dir.glob('**/*.mp4')):
                size = mp4_path.stat().st_size
                if size > max_size:
                    max_size = size
                    max_path = mp4_path
            shutil.move(max_path, mapping_vid_path)
            print(f"raw_videos/mapping.mp4 don't exist! Renaming largest file {max_path.name}.")
        
        # create gripper calibration video if don't exist
        # 如果 raw_videos/gripper_calibration 目录不存在，创建该目录，
        gripper_cal_dir = input_dir.joinpath('gripper_calibration')
        if not gripper_cal_dir.is_dir():
            gripper_cal_dir.mkdir()
            print("raw_videos/gripper_calibration don't exist! Creating one with the first video of each camera serial.")
            
            # %% 依据采集数据的操作顺序，我们把找到<session_dir>下每个相机序列号的第一个视频(除去 mapping 视频)，
            # 也就是相同相机拍摄的视频中开始录制时间最早的那个视频，
            # 作为该相机的校准视频，并将其移动到 raw_videos/gripper_calibration 目录下
            serial_start_dict = dict()
            serial_path_dict = dict()
            with ExifToolHelper() as et:
                for mp4_path in list(input_dir.glob('**/*.MP4')) + list(input_dir.glob('**/*.mp4')):
                    if mp4_path.name.startswith('map'): #用来判断当前视频文件的名字是否以 'map' 开头
                        continue
                    
                    start_date = mp4_get_start_datetime(str(mp4_path))# 获取当前视频的精确开始录制时间
                    meta = list(et.get_metadata(str(mp4_path)))[0] # 使用 ExifToolHelper 获取当前视频的元数据，返回一个包含所有元数据的字典。meta['QuickTime:CameraSerialNumber'] 是从元数据中获取相机序列号的值，作为当前视频的相机标识符。
                    cam_serial = meta['QuickTime:CameraSerialNumber']
                    
                    # 判断当前视频的相机序列号是否已经在 serial_start_dict 字典中出现过，
                    # 如果出现过，说明之前已经处理过一个相同相机序列号的视频了，
                    # 那么就比较当前视频的开始时间和之前记录的开始时间，选择较早的那个视频作为该相机序列号的代表视频。
                    if cam_serial in serial_start_dict: 
                        if start_date < serial_start_dict[cam_serial]:
                            serial_start_dict[cam_serial] = start_date
                            serial_path_dict[cam_serial] = mp4_path
                    else:
                        serial_start_dict[cam_serial] = start_date
                        serial_path_dict[cam_serial] = mp4_path
            
            for serial, path in serial_path_dict.items():
                print(f"Selected {path.name} for camera serial {serial}")
                out_path = gripper_cal_dir.joinpath(path.name)
                shutil.move(path, out_path)

        # %% 对每个 mp4 视频文件进行处理，首先获取视频的开始录制时间和相机序列号，
        # 然后根据这些信息构造输出目录的名字(除了 mapping 和 gripper_calibration)，并将视频文件移动到对应的输出目录下，
        # 并重命名为 raw_video.mp4。
        # 最后在原来视频文件的位置创建一个符号链接，指向新的视频文件位置。 
        
        # look for mp4 video in all subdirectories in input_dir
        input_mp4_paths = list(input_dir.glob('**/*.MP4')) + list(input_dir.glob('**/*.mp4'))
        print(f'Found {len(input_mp4_paths)} MP4 videos')

        with ExifToolHelper() as et:
            for mp4_path in input_mp4_paths:
                if mp4_path.is_symlink(): # 文件路径是否是一个符号链接（symlink），而非真实文件
                    print(f"Skipping {mp4_path.name}, already moved.")
                    continue
                
                # 构造每个视频文件所在目录的命名规则，
                # 一般格式为 demo_<相机序列号>_<视频开始时间>，
                # 其中视频开始时间的格式为 年.月.日_时.分.秒.毫秒
                start_date = mp4_get_start_datetime(str(mp4_path))
                meta = list(et.get_metadata(str(mp4_path)))[0]
                cam_serial = meta['QuickTime:CameraSerialNumber']
                out_dname = 'demo_' + cam_serial + '_' + start_date.strftime(r"%Y.%m.%d_%H.%M.%S.%f")

                # special folders
                # 重命名存放mapping和gripper_calibration视频文件所在目录
                if mp4_path.name.startswith('mapping'):
                    out_dname = "mapping"
                elif mp4_path.name.startswith('gripper_cal') or mp4_path.parent.name.startswith('gripper_cal'):
                    out_dname = "gripper_calibration_" + cam_serial + '_' + start_date.strftime(r"%Y.%m.%d_%H.%M.%S.%f")
                
                # create directory
                # 根据上面构造的目录名字，创建对应的目录，如果目录已经存在则不进行创建
                this_out_dir = output_dir.joinpath(out_dname)
                this_out_dir.mkdir(parents=True, exist_ok=True)
                
                # move videos
                # 将本循环中处理的视频文件移动到上面创建的目录下，
                # 并重命名为 raw_video.mp4
                vfname = 'raw_video.mp4'
                out_video_path = this_out_dir.joinpath(vfname)
                shutil.move(mp4_path, out_video_path)# 将当前视频文件移动到输出目录下，并重命名为 raw_video.mp4

                # create symlink back from original location
                # relative_to's walk_up argument is not avaliable until python 3.12
                # mp4_path = '/<session>/raw_videos/<***.MP4>'
                # out_video_path = '/<session>/demos/<demo_*_*>/raw_video.mp4'
                '''
                mp4_path.parent = /home/yuchen/project/universal_manipulation_interface(client)/example_demo_session/raw_videos
                mp4_path.parent.relative_to(session) = raw_videos
                mp4_path.parent.relative_to(session).parts) = ('raw_videos',)
                dots = ..
                out_video_path = /home/yuchen/project/universal_manipulation_interface(client)/example_demo_session/demos/demo_C3441328164125_2024.01.10_11.02.36.633583/raw_video.mp4
                rel_path = demos/demo_C3441328164125_2024.01.10_11.02.36.633583/raw_video.mp4
                symlink_path = ../demos/demo_C3441328164125_2024.01.10_11.02.36.633583/raw_video.mp4
                '''
                dots = os.path.join(*['..'] * len(mp4_path.parent.relative_to(session).parts))
                rel_path = str(out_video_path.relative_to(session))
                symlink_path = os.path.join(dots, rel_path)              
                mp4_path.symlink_to(symlink_path)

# %%
if __name__ == '__main__':
    if len(sys.argv) == 1: 
        main.main(['--help'])
    else:
        main()

        '''
    # 它利用了 Python 的 sys.argv 列表，该列表存储了从命令行传递给脚本的所有参数。
    # 在 Python 中，sys.argv 的第一个元素（索引为 0）始终是脚本本身的名称。
    # 因此，当 len(sys.argv) == 1 时，意味着用户在终端运行脚本时没有提供任何额外的参数。
    # 在这种情况下，程序会调用 main.main(['--help'])，其目的是自动打印帮助文档或使用说明，
    # 从而引导用户了解如何正确使用该工具。
    '''
