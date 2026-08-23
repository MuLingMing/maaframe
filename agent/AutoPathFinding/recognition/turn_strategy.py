# -*- coding: utf-8 -*-
"""转向策略独立模块。

借鉴 [MaaEnd MapTracker](https://github.com/MaaEnd/MaaEnd) 的设计：
- `RotationLowerThreshold` / `RotationUpperThreshold` 双阈值转向分级
- `rotationSpeed` 自适应（指数移动平均 0.618/0.382）
- "rotation is acceptable but can be improved" 转向分级注释语义

将原 `PathFindingReco._compute_turn_coordinates` 中混杂的逻辑拆分为：
- `compute_target_angle()` 统一计算目标角度
- `classify_turn()` 双阈值分级（fine_tune / large_turn）
- `compute_turn_distance()` 自适应滑动距离
- `compute_turn_coordinates()` 滑块起止点（裁剪防塌缩）
"""

from __future__ import annotations

import math
from typing import Tuple

from ..state import TurnGrade

# 屏幕中心坐标（720p 基准）
SCREEN_CENTER = (640, 360)
SCREEN_WIDTH = 1280
SCREEN_HEIGHT = 720

# 方向分箱的角度边界常量（度，atan2(-dy, dx) 坐标系）
# 规则：
# -   -45° ≤ angle <   45° → right   （目标在右）
# -    45° ≤ angle <  135° → forward （目标在上）
# -   135° ≤ angle <  180° 或 -180° ≤ angle < -135° → left  （目标在左）
# -  -135° ≤ angle <  -45° → backward（目标在下）
DIR_BOUNDARIES: tuple[float, ...] = (-135.0, -45.0, 45.0, 135.0)

DIR_CENTER_ANGLE: dict[str, float] = {
    "right": 0.0,
    "forward": 90.0,
    "left": 180.0,
    "backward": -90.0,
}


def compute_target_angle(
    target_center: tuple[float, float],
    screen_center: tuple[int, int] = SCREEN_CENTER,
) -> float:
    """计算目标相对屏幕中心的角度（度，atan2(-dy, dx) 坐标系）。

    使用 `-dy` 翻转以匹配游戏方向（屏幕上方 = forward）。

    参数:
    - target_center: 目标中心坐标 (x, y)
    - screen_center: 屏幕中心坐标，默认 720p

    返回值:
    - float: 角度（度），范围 [-180, 180]
    """
    tx, ty = target_center
    cx, cy = screen_center
    dx = tx - cx
    dy = ty - cy
    if dx == 0 and dy == 0:
        return 0.0
    return math.degrees(math.atan2(-dy, dx))


def bin_direction(angle: float) -> str:
    """将角度按固定边界分箱为方向字符串。

    参数:
    - angle: 角度（度），范围 [-180, 180]

    返回值:
    - str: 方向（forward / backward / left / right）
    """
    # angle ∈ [-180, 180]
    if -45.0 <= angle < 45.0:
        return "right"
    if 45.0 <= angle < 135.0:
        return "forward"
    if angle >= 135.0 or angle < -135.0:
        return "left"
    return "backward"


def dir_center_angle(direction: str) -> float | None:
    """返回各方向对应的分箱中心角度（度，atan2(-dy, dx) 坐标系）。"""
    return DIR_CENTER_ANGLE.get(direction)


def angle_diff(a: float, b: float) -> float:
    """返回 a 相对 b 的角度差（带环绕，范围 (-180, 180]）。"""
    diff = (a - b + 180.0) % 360.0 - 180.0
    return diff


def classify_turn(
    current_angle: float,
    last_angle: float | None,
    lower_threshold: float,
    upper_threshold: float,
) -> TurnGrade:
    """根据当前帧与上一帧的角度偏差判定转向分级。

    借鉴 MaaEnd MapTracker 的 rotation_lower_threshold / rotation_upper_threshold：
    - 偏差 < lower_threshold → 不需要转向（CENTERED，由调用方处理）
    - lower_threshold ≤ 偏差 < upper_threshold → FINE_TUNE
    - 偏差 ≥ upper_threshold → LARGE_TURN

    参数:
    - current_angle: 当前帧角度（度）
    - last_angle: 上一帧角度（度），None 表示无历史
    - lower_threshold: 微调阈值（度）
    - upper_threshold: 大幅调整阈值（度）

    返回值:
    - TurnGrade: 转向分级
    """
    if last_angle is None:
        # 无历史：直接用 upper 阈值判断
        delta = abs(angle_diff(current_angle, current_angle))  # = 0
    else:
        delta = abs(angle_diff(current_angle, last_angle))

    if delta < lower_threshold:
        return TurnGrade.FINE_TUNE
    if delta < upper_threshold:
        return TurnGrade.FINE_TUNE
    return TurnGrade.LARGE_TURN


def compute_turn_distance(
    offset_norm: float,
    turn_attempt: int,
    min_turn_distance: int,
    max_turn_distance: int,
    distance_missing_turn_scale: float,
    turn_min_floor_ratio: float,
    adaptive_speed: float = 1.0,
    speed_factor: float = 0.7,
) -> float:
    """计算转向滑动距离（像素）。

    设计（动态自适应）：
    - 基础距离 = offset_norm * speed_factor（让目标刚好走到中心附近，不越过去）
    - speed_factor < 1.0 保证滑动后目标落在中心**附近**但**不越过**到反方向
    - 上限保护：max_turn_distance（防止极端偏移下滑动过大）
    - 下限保护：min_turn_distance（防止极小偏移时滑动过短）
    - 自适应速度（借鉴 MapTracker EMA）乘以最终距离
    - 第 2 次起按 distance_missing_turn_scale 衰减

    参数:
    - offset_norm: 目标相对屏幕中心的欧氏距离
    - turn_attempt: 当前是第几次连续转向（从 1 开始）
    - min_turn_distance: 最短转向滑动距离（像素），下限保护
    - max_turn_distance: 最长转向滑动距离（像素），上限保护
    - distance_missing_turn_scale: 连续缺失距离时衰减系数
    - turn_min_floor_ratio: 滑动距离下限比例
    - adaptive_speed: 自适应速度系数（默认 1.0）
    - speed_factor: 滑动距离相对 offset_norm 的比例（默认 0.7）

    返回值:
    - float: 滑动距离（像素）
    """
    # 动态距离：滑动距离与目标偏移成正比，保证滑动后目标朝中心移动但不越过
    base_distance = offset_norm * speed_factor

    # 第 2 次起衰减（连续缺失距离时减小滑动）
    if turn_attempt > 1:
        base_distance *= distance_missing_turn_scale ** (turn_attempt - 1)

    # 应用自适应速度
    base_distance *= adaptive_speed

    # 上限保护：避免极端偏移下滑动过大
    if base_distance > max_turn_distance:
        base_distance = max_turn_distance

    # 下限保护：避免极小偏移时滑动过短
    floor = min_turn_distance * turn_min_floor_ratio
    if base_distance < floor:
        base_distance = floor

    return base_distance


def compute_turn_clip_rect(
    offset_x: float,
    offset_y: float,
    horizontal_clip: tuple[int, int, int, int],
    vertical_clip: tuple[int, int, int, int],
) -> tuple[int, int, int, int]:
    """根据目标偏移主轴选择水平/垂直裁剪区域。

    设计：
    - 水平方向（左/右）用上方水平条带
    - 垂直方向（上/下）用中央垂直条带
    - 选水平/垂直由主轴决定，使滑动向量整体落入对应长条区域

    参数:
    - offset_x: 目标相对屏幕中心的 x 偏移
    - offset_y: 目标相对屏幕中心的 y 偏移
    - horizontal_clip: 水平方向裁剪区域 (x, y, w, h)
    - vertical_clip: 垂直方向裁剪区域 (x, y, w, h)

    返回值:
    - tuple: 选定的裁剪区域
    """
    if abs(offset_x) >= abs(offset_y):
        return horizontal_clip
    return vertical_clip


def compute_turn_coordinates(
    target_center: tuple[float, float],
    turn_attempt: int,
    *,
    min_turn_distance: int,
    max_turn_distance: int,
    distance_missing_turn_scale: float,
    turn_min_floor_ratio: float,
    horizontal_clip: tuple[int, int, int, int],
    vertical_clip: tuple[int, int, int, int],
    adaptive_speed: float = 1.0,
    min_offset_ratio: float = 0.0,
    screen_center: tuple[int, int] = SCREEN_CENTER,
    speed_factor: float = 0.7,
) -> Tuple[Tuple[int, int], Tuple[int, int]] | None:
    """基于目标中心偏移计算转向滑动起止点。

    借鉴 MaaEnd MapTracker 的设计：
    - 滑动方向将目标朝屏幕中心"推"：手指 swipe (sx,sy)->(ex,ey) 后，
      屏幕中目标的位置变化 = swipe 向量本身。
      因此要让目标回到屏幕中心，需要 swipe 方向 = -offset。
    - 滑动距离与 offset_norm 成正比（动态自适应），保证滑动后目标朝中心移动
      而不越过中心到反方向。
    - 滑动向量以选定 clip rect 中心为基准放置
    - 起止点裁剪到 clip rect 内，避免硬裁剪导致滑动坍塌

    参数:
    - target_center: 目标中心坐标
    - turn_attempt: 当前是第几次连续转向（从 1 开始）
    - min_turn_distance: 最短滑动距离
    - max_turn_distance: 最长滑动距离
    - distance_missing_turn_scale: 衰减系数
    - turn_min_floor_ratio: 下限保护比例
    - horizontal_clip / vertical_clip: 水平/垂直裁剪区域
    - adaptive_speed: 自适应速度系数
    - min_offset_ratio: 触发转向的最小中心偏移比例（相对屏幕半宽）
    - screen_center: 屏幕中心坐标

    返回值:
    - ((start_x, start_y), (end_x, end_y)) 或 None（偏移过小/塌缩时）
    """
    tx, ty = target_center
    cx, cy = screen_center
    offset_x = tx - cx
    offset_y = ty - cy
    offset_norm = math.hypot(offset_x, offset_y)

    min_offset = (SCREEN_WIDTH / 2) * min_offset_ratio
    if offset_norm < min_offset:
        return None

    base_distance = compute_turn_distance(
        offset_norm,
        turn_attempt,
        min_turn_distance,
        max_turn_distance,
        distance_missing_turn_scale,
        turn_min_floor_ratio,
        adaptive_speed,
        speed_factor,
    )

    if offset_norm == 0:
        return None

    # swipe 单位向量：与目标偏移相反（把目标推向屏幕中心的方向）
    swx = -offset_x / offset_norm
    swy = -offset_y / offset_norm

    # 选 clip rect
    clip_x, clip_y, clip_w, clip_h = compute_turn_clip_rect(
        offset_x, offset_y, horizontal_clip, vertical_clip
    )
    rect_cx = clip_x + clip_w / 2
    rect_cy = clip_y + clip_h / 2

    half_dist = base_distance / 2
    # 起点：clip 中心向 swipe 反方向（目标方向）偏移 half_dist
    start_x = rect_cx - swx * half_dist
    start_y = rect_cy - swy * half_dist
    # 终点：clip 中心向 swipe 方向（目标反方向）偏移 half_dist
    # 即 swipe = end - start = (-offset_x, -offset_y)
    end_x = rect_cx + swx * half_dist
    end_y = rect_cy + swy * half_dist

    # 裁剪到选定 clip rect
    start_x = max(clip_x, min(start_x, clip_x + clip_w))
    start_y = max(clip_y, min(start_y, clip_y + clip_h))
    end_x = max(clip_x, min(end_x, clip_x + clip_w))
    end_y = max(clip_y, min(end_y, clip_y + clip_h))

    # 裁剪后若起止点重合，则本次转向无意义
    if (int(start_x), int(start_y)) == (int(end_x), int(end_y)):
        return None

    return (int(start_x), int(start_y)), (int(end_x), int(end_y))