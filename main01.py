"""第一步：拍一张照片，存到 cermal/ 文件夹。"""

import cv2
import os


def main():
    # 1. 创建保存目录（相对于脚本所在目录）
    script_dir = os.path.dirname(os.path.abspath(__file__))
    save_dir = os.path.join(script_dir, "cermal")
    os.makedirs(save_dir, exist_ok=True)

    # 2. 打开摄像头（0 = 默认摄像头）
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)

    if not cap.isOpened():
        # V4L2 失败，回退默认后端
        cap.open(0)

    if not cap.isOpened():
        print("无法打开摄像头！检查 ls /dev/video*")
        return

    # 3. 读一帧
    ret, frame = cap.read()
    if not ret:
        print("读取帧失败！")
        cap.release()
        return

    # 4. 保存
    save_path = os.path.join(save_dir, "photo.jpg")
    cv2.imwrite(save_path, frame)
    print(f"已保存到 {save_path}")

    # 5. 释放摄像头
    cap.release()


if __name__ == "__main__":
    main()
