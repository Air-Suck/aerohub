# Derived from fangxiangbiaostudyhard/yolo11_gemini2_detection-distance.
# Pinned commit: e7ea03c7be30f5675f463b84569f981aae3cd0be
# Licensed under Apache-2.0; see stage2/licenses/yolo11-gemini2-APACHE-2.0.txt.
# The four definitions below are copied verbatim. ROS adapters live in core.py.
from dataclasses import dataclass
from typing import Optional, Tuple
import numpy as np
import cv2

MIN_DEPTH_MM = 20.0
MAX_DEPTH_MM = 10000.0
CENTER_ROI_RATIO = 0.50
DEPTH_HIST_BIN_MM = 50.0
DEPTH_CLUSTER_WINDOW_MM = 120.0
MIN_VALID_DEPTH_POINTS = 20
SEG_MASK_ERODE_KERNEL = 3

@dataclass
class DepthEstimate:
    """一个目标区域的深度估计结果。"""

    # 最终选中的代表像素
    u: int
    v: int

    # 目标深度中位数
    depth_mm: float

    # 区域中的有效深度点数量
    valid_points: int

    # 主深度簇点数量
    cluster_points: int

def create_center_roi_mask(
    image_shape: Tuple[int, int],
    bbox: Tuple[int, int, int, int],
    ratio: float = CENTER_ROI_RATIO,
) -> np.ndarray:
    """
    在 Bounding Box 中创建中心 ROI Mask。

    ratio=0.5：
        只使用 bbox 中间 50% x 50% 区域测深。
    """

    image_h, image_w = image_shape
    x1, y1, x2, y2 = bbox

    box_width = max(1, x2 - x1)
    box_height = max(1, y2 - y1)

    center_x = (x1 + x2) / 2.0
    center_y = (y1 + y2) / 2.0

    roi_width = box_width * ratio
    roi_height = box_height * ratio

    roi_x1 = int(round(center_x - roi_width / 2.0))
    roi_y1 = int(round(center_y - roi_height / 2.0))
    roi_x2 = int(round(center_x + roi_width / 2.0))
    roi_y2 = int(round(center_y + roi_height / 2.0))

    roi_x1 = max(0, min(roi_x1, image_w - 1))
    roi_y1 = max(0, min(roi_y1, image_h - 1))
    roi_x2 = max(roi_x1 + 1, min(roi_x2, image_w))
    roi_y2 = max(roi_y1 + 1, min(roi_y2, image_h))

    mask = np.zeros((image_h, image_w), dtype=bool)
    mask[roi_y1:roi_y2, roi_x1:roi_x2] = True

    return mask

def estimate_depth_from_region(
    depth_map: np.ndarray,
    region_mask: np.ndarray,
) -> Optional[DepthEstimate]:
    """
    从某个目标区域中估计目标深度。

    流程：
        region mask
            -> 有效深度点
            -> 深度直方图
            -> 最大主峰
            -> 主峰附近 +/- window
            -> 深度中位数
            -> 选取一个真实有效的代表像素 (u, v)

    相比“整个 bbox 直接 median”，这种方法能明显减少背景干扰。
    """

    valid_mask = (
        region_mask
        & (depth_map > MIN_DEPTH_MM)
        & (depth_map < MAX_DEPTH_MM)
    )

    ys, xs = np.nonzero(valid_mask)

    if xs.size < MIN_VALID_DEPTH_POINTS:
        return None

    depths = depth_map[ys, xs].astype(np.float32)

    # --------------------------------------------------------------------------
    # 深度直方图
    # --------------------------------------------------------------------------

    bin_ids = np.floor(
        depths / DEPTH_HIST_BIN_MM
    ).astype(np.int32)

    unique_bins, counts = np.unique(
        bin_ids,
        return_counts=True,
    )

    if unique_bins.size == 0:
        return None

    peak_index = int(np.argmax(counts))
    peak_bin = int(unique_bins[peak_index])

    peak_depth_center = (
        peak_bin + 0.5
    ) * DEPTH_HIST_BIN_MM

    cluster_mask = (
        np.abs(depths - peak_depth_center)
        <= DEPTH_CLUSTER_WINDOW_MM
    )

    cluster_count = int(np.count_nonzero(cluster_mask))

    if cluster_count < MIN_VALID_DEPTH_POINTS:
        # 主峰点太少时退化为全部有效区域。
        cluster_depths = depths
        cluster_xs = xs
        cluster_ys = ys
    else:
        cluster_depths = depths[cluster_mask]
        cluster_xs = xs[cluster_mask]
        cluster_ys = ys[cluster_mask]

    if cluster_depths.size == 0:
        return None

    depth_mm = float(np.median(cluster_depths))

    # --------------------------------------------------------------------------
    # 代表像素：
    #
    # 先计算簇内像素坐标的中位数，再在真实簇像素中寻找离该位置最近的点。
    # 这样最终的 (u, v) 一定真的是目标深度簇中的有效像素，
    # 而不是一个可能落在空洞/背景上的“虚构中心点”。
    # --------------------------------------------------------------------------

    median_u = float(np.median(cluster_xs))
    median_v = float(np.median(cluster_ys))

    distance_sq = (
        (cluster_xs.astype(np.float32) - median_u) ** 2
        + (cluster_ys.astype(np.float32) - median_v) ** 2
    )

    representative_index = int(np.argmin(distance_sq))

    u = int(cluster_xs[representative_index])
    v = int(cluster_ys[representative_index])

    return DepthEstimate(
        u=u,
        v=v,
        depth_mm=depth_mm,
        valid_points=int(depths.size),
        cluster_points=int(cluster_depths.size),
    )


def prepare_segmentation_mask(
    raw_mask: np.ndarray,
    image_shape: Tuple[int, int],
) -> np.ndarray:
    """
    将 Ultralytics Mask 转成与 RGB/Depth 相同大小的 bool mask。

    retina_masks=True 时通常已经是原图尺寸；
    这里仍保留 resize 作为鲁棒性保护。

    另外会轻微腐蚀边缘，减少背景深度混入。
    """

    image_h, image_w = image_shape

    if raw_mask.shape != (image_h, image_w):
        raw_mask = cv2.resize(
            raw_mask.astype(np.float32),
            (image_w, image_h),
            interpolation=cv2.INTER_NEAREST,
        )

    mask = (raw_mask > 0.5).astype(np.uint8)

    if SEG_MASK_ERODE_KERNEL >= 3:
        kernel_size = int(SEG_MASK_ERODE_KERNEL)

        # 强制使用奇数 kernel
        if kernel_size % 2 == 0:
            kernel_size += 1

        kernel = np.ones(
            (kernel_size, kernel_size),
            dtype=np.uint8,
        )

        eroded = cv2.erode(mask, kernel, iterations=1)

        # 如果目标本身很小，腐蚀可能把 mask 全部吃掉。
        # 这时回退到原始 mask。
        if np.count_nonzero(eroded) >= MIN_VALID_DEPTH_POINTS:
            mask = eroded

    return mask.astype(bool)
