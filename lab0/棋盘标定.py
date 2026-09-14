from pathlib import Path

import cv2
import numpy as np


# ===================== 参数设置 =====================
# 棋盘：9×7 个方格 -> 8×6 个内角点
BOARD_SIZE = (8, 6)

# 如果只需要相机内参，设为 1.0 即可。
# 如果需要平移向量具有真实尺度，请填写屏幕上单个方格的实际边长，例如 25.0 mm。
SQUARE_SIZE = 1.0

BASE_DIR = Path(__file__).resolve().parent
IMAGE_DIR = BASE_DIR / "img"
OUTPUT_DIR = BASE_DIR / "calibration_result"
CORNERS_DIR = OUTPUT_DIR / "corners"

OUTPUT_DIR.mkdir(exist_ok=True)
CORNERS_DIR.mkdir(exist_ok=True)


def imread(path):
    """兼容 Windows 中文路径。"""
    data = np.fromfile(str(path), dtype=np.uint8)
    return cv2.imdecode(data, cv2.IMREAD_COLOR)


def imwrite(path, image):
    """兼容 Windows 中文路径。"""
    suffix = Path(path).suffix or ".jpg"
    success, encoded = cv2.imencode(suffix, image)
    if success:
        encoded.tofile(str(path))
    return success


# 亚像素角点优化停止条件
criteria = (
    cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
    50,
    0.001,
)

# 构造棋盘在世界坐标系中的三维点
# 例如：(0,0,0)、(1,0,0)、...、(7,5,0)
object_template = np.zeros(
    (BOARD_SIZE[0] * BOARD_SIZE[1], 3),
    dtype=np.float32,
)
object_template[:, :2] = (
    np.mgrid[0:BOARD_SIZE[0], 0:BOARD_SIZE[1]]
    .T.reshape(-1, 2)
)
object_template *= SQUARE_SIZE

object_points = []  # 每张图对应的三维点
image_points = []   # 每张图检测到的二维角点
valid_files = []
image_size = None

extensions = {".jpg", ".jpeg", ".png", ".bmp"}
image_files = sorted(
    path for path in IMAGE_DIR.iterdir()
    if path.suffix.lower() in extensions
)

if not image_files:
    raise FileNotFoundError(f"没有在 {IMAGE_DIR} 中找到图片")

print(f"共找到 {len(image_files)} 张图片")

for index, image_path in enumerate(image_files, start=1):
    image = imread(image_path)

    if image is None:
        print(f"[读取失败] {image_path.name}")
        continue

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    current_size = gray.shape[::-1]

    # 所有标定照片必须具有相同分辨率
    if image_size is None:
        image_size = current_size
    elif current_size != image_size:
        print(
            f"[尺寸不同，跳过] {image_path.name}: "
            f"{current_size}，要求 {image_size}"
        )
        continue

    flags = (
        cv2.CALIB_CB_ADAPTIVE_THRESH
        + cv2.CALIB_CB_NORMALIZE_IMAGE
    )

    found, corners = cv2.findChessboardCorners(
        gray,
        BOARD_SIZE,
        flags,
    )

    if not found:
        print(f"[未检测到棋盘] {image_path.name}")
        continue

    # 亚像素角点优化
    corners_subpix = cv2.cornerSubPix(
        gray,
        corners,
        winSize=(11, 11),
        zeroZone=(-1, -1),
        criteria=criteria,
    )

    object_points.append(object_template.copy())
    image_points.append(corners_subpix)
    valid_files.append(image_path)

    # 保存角点可视化结果
    preview = image.copy()
    cv2.drawChessboardCorners(
        preview,
        BOARD_SIZE,
        corners_subpix,
        found,
    )
    imwrite(
        CORNERS_DIR / f"{index:02d}_{image_path.stem}.jpg",
        preview,
    )

    print(f"[检测成功] {image_path.name}")


success_count = len(valid_files)
print(f"\n成功检测：{success_count}/{len(image_files)} 张")

if success_count < 8:
    raise RuntimeError(
        "有效图片少于 8 张，无法获得稳定结果。"
        "请检查 BOARD_SIZE，或重新拍摄清晰照片。"
    )


# ===================== 相机标定 =====================
rms, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
    object_points,
    image_points,
    image_size,
    None,
    None,
)

print("\n========== 标定结果 ==========")
print(f"OpenCV RMS 重投影误差：{rms:.6f} 像素")
print("\n相机内参矩阵 camera_matrix：")
print(camera_matrix)
print("\n畸变系数 dist_coeffs：")
print(dist_coeffs.ravel())


# ===================== 逐张计算重投影误差 =====================
total_squared_error = 0.0
total_point_count = 0
per_image_errors = []

print("\n========== 每张图片的重投影误差 ==========")

for path, obj_pts, img_pts, rvec, tvec in zip(
    valid_files,
    object_points,
    image_points,
    rvecs,
    tvecs,
):
    projected_points, _ = cv2.projectPoints(
        obj_pts,
        rvec,
        tvec,
        camera_matrix,
        dist_coeffs,
    )

    difference = img_pts.reshape(-1, 2) - projected_points.reshape(-1, 2)
    squared_error = np.sum(difference ** 2)
    point_count = len(obj_pts)
    image_rms = np.sqrt(squared_error / point_count)

    per_image_errors.append(image_rms)
    total_squared_error += squared_error
    total_point_count += point_count

    print(f"{path.name}: {image_rms:.4f} px")

overall_rms = np.sqrt(total_squared_error / total_point_count)
print(f"\n总体重投影 RMS：{overall_rms:.6f} px")


# ===================== 保存参数 =====================
np.savez(
    OUTPUT_DIR / "camera_calibration.npz",
    camera_matrix=camera_matrix,
    dist_coeffs=dist_coeffs,
    rvecs=np.asarray(rvecs),
    tvecs=np.asarray(tvecs),
    image_size=np.asarray(image_size),
    board_size=np.asarray(BOARD_SIZE),
    square_size=SQUARE_SIZE,
    rms=rms,
)

# 同时保存成 OpenCV YAML
yaml_path = OUTPUT_DIR / "camera_calibration.yaml"
storage = cv2.FileStorage(str(yaml_path), cv2.FILE_STORAGE_WRITE)
storage.write("image_width", image_size[0])
storage.write("image_height", image_size[1])
storage.write("board_width", BOARD_SIZE[0])
storage.write("board_height", BOARD_SIZE[1])
storage.write("square_size", SQUARE_SIZE)
storage.write("rms", rms)
storage.write("camera_matrix", camera_matrix)
storage.write("dist_coeffs", dist_coeffs)
storage.release()


# ===================== 生成一张去畸变示例 =====================
sample_image = imread(valid_files[0])

new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
    camera_matrix,
    dist_coeffs,
    image_size,
    alpha=1,
    newImgSize=image_size,
)

undistorted = cv2.undistort(
    sample_image,
    camera_matrix,
    dist_coeffs,
    None,
    new_camera_matrix,
)

imwrite(OUTPUT_DIR / "undistorted_sample.jpg", undistorted)

print("\n结果已保存到：", OUTPUT_DIR)
print("- camera_calibration.npz")
print("- camera_calibration.yaml")
print("- undistorted_sample.jpg")
print("- corners/（角点检测结果）")