"""AutoPathFinding 状态类定义。

提供：
- LockedTarget：被锁定的目标，支持 lock_id 生成与 IOU 匹配。
- Phase：阶段状态机（借鉴 MaaEnd MapTracker 的 InferState）。
- TurnGrade：转向分级（fine_tune / large_turn）。
- RotationAdjuster：自适应转向速度（借鉴 MapTracker 的 rotationSpeed EMA 0.618/0.382）。
- StuckDetector：卡住检测（借鉴 MapTracker 的 stuckThreshold/stuckTimeout）。
- PathFindingState：按节点隔离的寻路状态（Reco 侧）。
- PathFinderState：按节点隔离的执行状态（Action 侧）。
"""

from __future__ import annotations

import math
import secrets
from dataclasses import dataclass, field
from enum import Enum
from typing import Tuple


class Phase(Enum):
    """寻路阶段状态机。

    借鉴 MaaEnd MapTracker 的 InferState 设计，将多状态维度收敛到单一阶段，
    便于 Pipeline 上层根据阶段做差异化处理。
    """

    SEEKING = "seeking"          # 未锁定目标，全 ROI 扫描
    TRACKING = "tracking"        # 锁定中，fast_path 小 ROI
    APPROACHING = "approaching"  # 目标进入 dead_zone 中心附近
    TURNING = "turning"          # 正在执行大角度转向
    STUCK = "stuck"              # 位置未变超时
    LOST = "lost"                # 目标丢失但未超时


class TurnGrade(Enum):
    """转向分级。

    借鉴 MaaEnd MapTracker 的 rotation_lower/upper_threshold 设计：
    - FINE_TUNE：小偏差微调，使用较短滑动
    - LARGE_TURN：大偏差大幅调整，使用较长滑动并触发"先停后转"
    """

    FINE_TUNE = "fine_tune"
    LARGE_TURN = "large_turn"


@dataclass
class RotationAdjuster:
    """自适应转向速度调节器。

    借鉴 MaaEnd MapTracker 的 `rotationSpeed` 自适应（指数移动平均 0.618/0.382）：
    - 每完成一次转向调整，根据实际偏差与目标偏差的比例更新速度
    - 速度被钳制在 [ROTATION_MIN_SPEED, ROTATION_MAX_SPEED] 之间
    - 初始速度为 1.0（无历史数据时不缩放）
    """

    DEFAULT_SPEED: float = 1.0
    MIN_SPEED: float = 0.5
    MAX_SPEED: float = 2.0
    DECAY: float = 0.618       # 历史速度权重（与 MapTracker 保持一致）
    INNOVATION: float = 0.382  # 新观测权重（1 - DECAY）

    speed: float = DEFAULT_SPEED
    samples: int = 0           # 已完成的转向调整次数（用于调试/测试）

    def update(self, ideal_speed: float) -> float:
        """根据本轮转向的理想速度更新 EMA。

        参数:
        - ideal_speed: 本轮 target.delta_rot / actual_delta_rot 的比值

        返回值:
        - float: 更新后的速度
        """
        # 仅在合理区间内更新，避免单次异常数据污染
        if ideal_speed < self.MIN_SPEED or ideal_speed > self.MAX_SPEED:
            return self.speed
        self.speed = self.speed * self.DECAY + ideal_speed * self.INNOVATION
        # 防御性钳制（理论上 update 区间不会越界）
        self.speed = max(self.MIN_SPEED, min(self.MAX_SPEED, self.speed))
        self.samples += 1
        return self.speed

    def reset(self) -> None:
        """重置为初始状态（新目标/新节点时调用）。"""
        self.speed = self.DEFAULT_SPEED
        self.samples = 0


@dataclass
class StuckDetector:
    """卡住检测器（借鉴 MapTracker stuckThreshold/stuckTimeout）。

    通过对比当前帧与上一帧的目标中心位置判断是否"位置未变"，
    累积达到 stuck_threshold 帧后进入 STUCK 阶段。
    """

    threshold_frames: int = 3   # 连续多少帧无进展判定卡住
    distance_tolerance: float = 5.0   # 距离容差（像素）
    center_tolerance: float = 10.0    # 中心偏移容差（像素）

    count: int = 0
    last_center: Tuple[float, float] | None = None
    last_distance: int | None = None
    # 卡住阶段开始时间（用于 stuck_timeout 兜底，目前仅记录）
    entered_at_ms: int | None = None

    def is_stuck_frame(
        self,
        current_center: Tuple[float, float],
        current_distance: int | None,
        now_ms: int | None = None,
    ) -> bool:
        """判断本帧是否无进展，并维护计数。

        参数:
        - current_center: 当前目标中心坐标
        - current_distance: 当前目标距离（可能为 None）
        - now_ms: 当前时间戳（毫秒），用于 stuck 进入时间记录

        返回值:
        - bool: 本帧累计是否达到卡住阈值
        """
        no_progress = self._check_no_progress(current_center, current_distance)
        if no_progress:
            if self.count == 0 and now_ms is not None:
                self.entered_at_ms = now_ms
            self.count += 1
        else:
            self.count = 0
            self.entered_at_ms = None

        # 更新历史
        self.last_center = current_center
        self.last_distance = current_distance

        return self.count >= self.threshold_frames

    def _check_no_progress(
        self,
        current_center: Tuple[float, float],
        current_distance: int | None,
    ) -> bool:
        """单帧无进展判定。

        优先使用距离信息：当距离未明显缩短（变化 ≤ tolerance）时视为无进展。
        无距离信息时回退到目标中心偏移判断。
        首帧（无历史数据）不判定为无进展。
        """
        if (
            isinstance(current_distance, int)
            and isinstance(self.last_distance, int)
        ):
            # 距离未缩短超过容差视为无进展（负值表示距离增加，也视为卡住）
            return (self.last_distance - current_distance) <= self.distance_tolerance

        if self.last_center is not None:
            dx = current_center[0] - self.last_center[0]
            dy = current_center[1] - self.last_center[1]
            return math.hypot(dx, dy) <= self.center_tolerance

        # 首帧无历史数据，不判定为卡住
        return False

    def reset(self) -> None:
        """重置检测器（新目标/新节点时调用）。"""
        self.count = 0
        self.last_center = None
        self.last_distance = None
        self.entered_at_ms = None


@dataclass
class LockedTarget:
    """被锁定的目标，用于跨帧保持选择连贯性。"""

    template: str
    center: tuple[float, float]
    bbox: tuple[int, int, int, int]
    score: float
    distance: int | None
    lock_id: str
    miss_count: int = 0
    # 连续未识别到距离的帧数；distance 被识别时清零。
    # 用于在选中新目标时优先选择距离稳定的目标。
    distance_miss_count: int = 0
    # 上一帧计算出的方向（用于方向分箱的角度滞回）
    last_direction: str | None = None
    # 上一帧目标相对屏幕中心的角度（度，atan2(-dy, dx) 坐标系），
    # 用于转向分级判定（fine_tune / large_turn）
    last_target_angle: float | None = None

    @staticmethod
    def generate_lock_id(template: str, center: tuple[float, float]) -> str:
        """生成带随机后缀的 lock_id，避免同名邻近目标冲突。"""
        suffix = secrets.token_hex(4)
        return f"{template}#{int(center[0])}#{int(center[1])}#{suffix}"

    def iou(self, other_bbox: tuple[int, int, int, int]) -> float:
        """计算当前 bbox 与另一 bbox 的 IOU。"""
        x1, y1, w1, h1 = self.bbox
        x2, y2, w2, h2 = other_bbox

        ax1, ay1, ax2, ay2 = x1, y1, x1 + w1, y1 + h1
        bx1, by1, bx2, by2 = x2, y2, x2 + w2, y2 + h2

        inter_x1 = max(ax1, bx1)
        inter_y1 = max(ay1, by1)
        inter_x2 = min(ax2, bx2)
        inter_y2 = min(ay2, by2)

        if inter_x1 >= inter_x2 or inter_y1 >= inter_y2:
            return 0.0

        inter_area = (inter_x2 - inter_x1) * (inter_y2 - inter_y1)
        area_a = w1 * h1
        area_b = w2 * h2
        union = area_a + area_b - inter_area
        return inter_area / union if union > 0 else 0.0

    def is_same_target(
        self,
        template: str,
        bbox: tuple[int, int, int, int],
        iou_threshold: float,
    ) -> bool:
        """基于模板名与 IOU 判断是否为同一目标。"""
        if self.template != template:
            return False
        return self.iou(bbox) >= iou_threshold


@dataclass
class PathFindingState:
    """按节点隔离的寻路状态（Reco 侧）。

    设计参考 MaaEnd MapTracker 的 InferState：
    - 单一 Phase 字段取代分散的标志位
    - 自适应转向/卡住检测由独立组件承载，便于单测
    """

    locked_target: LockedTarget | None = None
    # 当前阶段
    phase: Phase = Phase.SEEKING
    # 转向分级（仅在 phase == TURNING 时有义）
    turn_grade: TurnGrade | None = None
    # 自适应转向速度调节器（每个目标独立）
    rotation_adjuster: RotationAdjuster = field(default_factory=RotationAdjuster)
    # 卡住检测器（每个目标独立）
    stuck_detector: StuckDetector = field(default_factory=StuckDetector)
    # 历史数据（运动状态评估使用）
    last_distance: int | None = None
    last_center: tuple[float, float] | None = None
    stuck_count: int = 0
    turn_attempts_without_distance: int = 0
    # fast_path ROI 连续 miss 计数；达到 fast_path_miss_limit 时强制全 ROI 重新选择
    fast_path_miss_count: int = 0

    def reset_target_local_state(self) -> None:
        """新目标时重置与该目标绑定的自适应/卡住/历史状态。"""
        self.rotation_adjuster.reset()
        self.stuck_detector.reset()
        self.last_distance = None
        self.last_center = None
        self.stuck_count = 0
        self.turn_attempts_without_distance = 0
        self.fast_path_miss_count = 0
        self.phase = Phase.SEEKING
        self.turn_grade = None


@dataclass
class PathFinderState:
    """按节点隔离的执行状态（Action 侧）。"""

    last_lock_id: str | None = None
    # 卡住阶段开始时间（毫秒），用于 stuck_timeout 兜底
    stuck_started_at_ms: int | None = None