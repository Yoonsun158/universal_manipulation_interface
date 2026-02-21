# %%
import sys
import os

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.dirname(__file__)))
sys.path.append(ROOT_DIR)
os.chdir(ROOT_DIR)

# %%
import argparse
import json
import cv2


def annotate_first_frame(video_path: str, output_path: str):
    # 检查视频文件是否存在
    if not os.path.exists(video_path):
        raise FileNotFoundError(f"Video file not found: {video_path}")

    cap = cv2.VideoCapture(video_path)
    ok, frame = cap.read()
    print(frame.shape)  
    cap.release()
    if not ok:
        raise RuntimeError(f"Failed to read first frame from {video_path}")

    points = []
    base = frame.copy()

    def redraw():
        vis = base.copy()
        for idx, (x, y) in enumerate(points):
            cv2.circle(vis, (x, y), 4, (0, 0, 255), -1)
            cv2.putText(vis, str(idx), (x + 6, y - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
        cv2.imshow("select_points", vis)

    def on_mouse(event, x, y, _flags, _userdata):
        if event == cv2.EVENT_LBUTTONDOWN:
            points.append((x, y))
            redraw()

    cv2.namedWindow("select_points", cv2.WINDOW_NORMAL)
    cv2.setMouseCallback("select_points", on_mouse)
    redraw()

    print("操作说明：")
    print("  - 左键点击：添加点")
    print("  - Enter/Space：完成并保存")
    print("  - r：重置所有点")
    print("  - q/Esc：退出并不保存")

    while True:
        key = cv2.waitKey(0) & 0xFF
        if key in (13, 32):  # Enter or Space
            break
        if key in (ord('q'), 27):  # q or Esc
            points.clear()
            break
        if key in (ord('r'), ord('R')):
            points.clear()
            redraw()

    cv2.destroyAllWindows()

    if not points:
        print("未保存任何点。")
        return

    data = [[int(x), int(y)] for x, y in points]

    # 确保输出目录存在
    os.makedirs(os.path.dirname(output_path) or '.', exist_ok=True)
    with open(output_path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)
    print(f"已保存 {len(points)} 个点的坐标到 {output_path}")


def parse_args():
    parser = argparse.ArgumentParser(
        description="从视频的第一帧中选择点并保存以供后续使用。")
    parser.add_argument("--video", default="workspace/test/video.mp4", help="源视频的路径")
    parser.add_argument("--output", "-o", default="workspace/test/points.json",
                        help="保存选定点的 JSON 文件路径")
    return parser.parse_args()


def main():
    args = parse_args()
    try:
        annotate_first_frame(args.video, args.output)
    except Exception as e:
        print(f"发生错误：{e}")


if __name__ == "__main__":
    main()
