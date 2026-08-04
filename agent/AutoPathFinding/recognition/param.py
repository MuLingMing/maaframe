# -*- coding: utf-8 -*-
"""
PathFindingReco 参数数据类

将识别器的 JSON 参数解析为类型安全的不可变对象，避免各方法中反复使用裸 dict。

设计参考 MaaEnd MapTracker：
- rotation_lower_threshold / rotation_upper_threshold 双阈值转向分级
- rotationSpeed 自适应（指数移动平均 0.618/0.382）
- stuckThreshold / stuckTimeout 卡住检测
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class PathFindingParam:
    """
    PathFindingReco 识别参数

    Attributes:
        ordered_templates: 按优先级排序后的模板列表
        roi: 识别区域 [x, y, w, h]
        threshold: 模板匹配阈值 [0, 1]
        selector_type: 选择器类型（priority/nearest/composite）
        distance_pattern: 距离 OCR 正则表达式
        distance_offset: 距离 OCR 区域偏移 [x, y, w, h]，与 MaaFramework roi_offset 语义一致
        distance_threshold: OCR 置信度阈值
        dead_zone: 方向判断死区半径（像素，欧氏距离）
        arrival_distance: 判定为到达目标的距离阈值（像素）
        stuck_threshold: 连续多少帧无进展才判定为卡住
        stuck_distance_tolerance: 距离变化小于此值视为无进展
        stuck_center_tolerance: 目标中心偏移小于此值视为无进展
        target_lock_timeout: 目标连续丢失多少帧后解锁
        target_lock_center_tolerance: 锁定目标跨帧匹配中心容差（像素）
        target_lock_score_threshold: 锁定目标跨帧匹配最低置信度
        enable_turning: 是否计算并返回转向坐标
        max_turn_attempts: 连续转向后仍未识别到距离，则停止转向
        min_turn_distance: 最短转向滑动距离（像素）
        max_turn_distance: 最长转向滑动距离（像素）
        distance_missing_turn_scale: 连续未识别到距离时转向滑动距离衰减系数
        enable_locked_fast_path: 是否在锁定状态下使用上一帧小 ROI 快速通道
        lock_roi_padding: 锁定态 ROI 相对上一帧 bbox 的扩展像素（单边）
        fast_path_max_turn_padding: 距离缺失时快速通道 ROI 的额外扩展（像素）
        direction_hysteresis: 方向分箱的角度滞回阈值（度），抑制 45° 边界的方向抖动
        fast_path_miss_limit: fast_path 连续 miss 多少帧后临时回退到全 ROI
        turn_min_floor_ratio: 滑动距离衰减下限（相对 min_turn_distance 的比例），避免连续衰减后转向距离过小
        rotation_lower_threshold: 微调阈值（度），偏差低于此值视为 FINE_TUNE 或不需要转向
        rotation_upper_threshold: 大幅调整阈值（度），偏差超过此值视为 LARGE_TURN
        rotation_adaptive_enabled: 是否启用自适应转向距离
        stuck_timeout_ms: 卡住阶段最长持续时间（毫秒），超过后判定导航失败
    """

    ordered_templates: list[str]
    roi: tuple[int, int, int, int]
    threshold: float
    selector_type: str
    distance_pattern: str
    distance_offset: list[int]
    distance_threshold: float
    dead_zone: int
    arrival_distance: int
    stuck_threshold: int
    stuck_distance_tolerance: int
    stuck_center_tolerance: int
    target_lock_timeout: int
    target_lock_center_tolerance: int
    target_lock_score_threshold: float
    enable_turning: bool
    max_turn_attempts: int
    min_turn_distance: int
    max_turn_distance: int
    distance_missing_turn_scale: float
    enable_locked_fast_path: bool
    lock_roi_padding: int
    fast_path_max_turn_padding: int
    direction_hysteresis: float
    fast_path_miss_limit: int
    turn_min_floor_ratio: float
    rotation_lower_threshold: float
    rotation_upper_threshold: float
    rotation_adaptive_enabled: bool
    stuck_timeout_ms: int