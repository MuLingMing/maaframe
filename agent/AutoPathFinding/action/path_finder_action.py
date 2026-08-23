# -*- coding: utf-8 -*-
"""
自动寻路执行器（重构版）

借鉴 [MaaEnd MapTracker](https://github.com/MaaEnd/MaaEnd) 的设计：
- 大幅转向（LARGE_TURN）前先释放按键，避免方向键卡住
- 卡住阶段（Phase.STUCK）触发 dodge
- 阶段状态机驱动的差异化执行

根据前序 PathFindingReco 返回的识别结果（direction / target.distance /
lock_id / turn_start / turn_end / turn_grade / phase），按
dodge → [release] → turn → move 顺序执行动作，并支持最大移动时间兜底。
复用 OperationRecording 的 PlatformFactory，自动适配 ADB/Desktop 平台。

功能说明：
1. 从 argv.reco_detail.best_result.detail 获取识别结果
2. 根据 lock_id 与 dodge_at_start 策略执行 dodge
3. 根据 turn_start/turn_end / turn_grade 与 enable_turning 执行 turn
   - LARGE_TURN 时先释放按键（参考 MapTracker rotation_upper_threshold 行为）
   - 可选 turn_duration_ms 显式控制 platform.turn 滑动时长
4. 根据 direction 执行 platform.move() 操作
5. 根据 target.distance 与 max_move_time 动态计算移动时长
6. 卡住阶段触发额外 dodge（借鉴 stuckThreshold）

执行流程：
1. _parse_param: 解析参数
2. _get_recognition_detail: 从 argv.reco_detail.best_result.detail 提取识别结果
3. _create_platform: 创建 OperationRecording 平台实例（PlatformFactory 缓存）
4. _execute_sequence: 执行 dodge → [release] → turn → move
5. finally: platform.release_all() 确保释放按键

返回值：CustomAction.RunResult(success=True/False)

状态检测说明：
- 到达 / 卡住等状态由 PathFindingReco 在识别阶段评估并返回 detail.phase
- PathFinderAction 仅负责根据 direction、turn 坐标、phase 和 dodge 策略执行动作
"""

import json
import logging
import time
from typing import Any, ClassVar

from maa.context import Context
from maa.custom_action import CustomAction
from maa.define import CustomRecognitionResult

from ..state import Phase, PathFinderState
from .param import PathFinderParam

logger = logging.getLogger(__name__)


class PathFinderAction(CustomAction):
    # 跨帧执行状态存储，按节点名隔离。生命周期跟随 Python 进程。
    _action_state: ClassVar[dict[str, PathFinderState]] = {}

    """
    自动寻路执行器

    参数格式（JSON）：
    {
        "move_duration": 500,
        "move_duration_far": 1500,
        "move_duration_near": 300,
        "distance_far": 200,
        "distance_near": 50,
        "max_move_time": 0,
        "dodge_at_start": "never",
        "dodge_direction": "forward",
        "enable_turning": true,
        "dodge_follows_direction": true,
        "dodge_release_ms": 50,
        "turn_duration_ms": null,
        "large_turn_release_ms": 100,
        "stuck_dodge_on_phase": true
    }

    字段说明：
    - move_duration: 无距离信息时的默认移动时长
    - move_duration_far: 距离 ≥ distance_far 时的移动时长
    - move_duration_near: 距离 ≤ distance_near 时的移动时长
    - distance_far: 远距离阈值
    - distance_near: 近距离阈值
    - max_move_time: 单次移动最大时长（毫秒），0 表示不限制
    - dodge_at_start: "never" 不闪避，"always" 每次闪避，"once_per_target" 同一 lock_id 只闪避一次
    - dodge_direction: 闪避方向，dodge_follows_direction=true 时忽略
    - enable_turning: 执行侧保险开关，false 时即使 Reco 返回 turn 坐标也不执行转向
    - dodge_follows_direction: 闪避方向跟随本帧 direction；false 时使用 dodge_direction
    - dodge_release_ms: dodge 与 move 之间的短暂 release 间隔，0 表示不释放
    - turn_duration_ms: 显式控制 platform.turn 滑动持续时间（毫秒），None 用 platform 默认
    - large_turn_release_ms: 大幅转向（LARGE_TURN）前先释放按键的时长（毫秒），
      0 表示不释放；借鉴 MapTracker rotation_upper_threshold 行为
    - stuck_dodge_on_phase: 当 Reco 返回 phase="stuck" 时是否额外触发 dodge

    Pipeline 使用示例：
    {
        "PathFinderAction": {
            "custom_action": "PathFinderAction",
            "custom_action_param": {
                "move_duration": 500,
                "max_move_time": 1000,
                "dodge_at_start": "once_per_target",
                "dodge_follows_direction": true
            },
            "next": ["AutoPathFinding"]
        }
    }
    """

    def run(
        self,
        context: Context,
        argv: CustomAction.RunArg,
    ) -> CustomAction.RunResult:
        if context.tasker.stopping:
            return CustomAction.RunResult(success=False)

        # 1. 解析参数
        param = self._parse_param(argv)

        # 2. 获取识别结果详情
        detail = self._get_recognition_detail(argv)
        if not detail:
            logger.error("Recognition detail is None")
            return CustomAction.RunResult(success=False)

        direction = detail.get("direction")
        if not direction:
            logger.error("Direction is empty in recognition detail")
            return CustomAction.RunResult(success=False)

        # 3. 提取识别结果字段
        target_info = detail.get("target", {})
        if isinstance(target_info, dict):
            distance = target_info.get("distance")
        else:
            distance = None

        lock_id = detail.get("lock_id")
        turn_start = detail.get("turn_start")
        turn_end = detail.get("turn_end")
        turn_grade = detail.get("turn_grade")
        phase_str = detail.get("phase")
        phase = self._resolve_phase(phase_str)

        # 4. 创建 OperationRecording 平台实例
        platform = self._create_platform(context)
        if not platform:
            return CustomAction.RunResult(success=False)

        try:
            # 5. 执行 dodge → turn → move
            success = self._execute_sequence(
                context,
                platform,
                direction,
                param,
                distance,
                lock_id,
                turn_start,
                turn_end,
                turn_grade,
                phase,
            )
        finally:
            platform.release_all()

        return CustomAction.RunResult(success=success)

    def _resolve_phase(self, phase_str: Any) -> Phase | None:
        """将 Reco 返回的 phase 字符串解析为 Phase 枚举。"""
        if not isinstance(phase_str, str):
            return None
        try:
            return Phase(phase_str)
        except ValueError:
            return None

    def _parse_param(self, argv: CustomAction.RunArg) -> PathFinderParam:
        try:
            raw_param = argv.custom_action_param
            if isinstance(raw_param, str):
                param = json.loads(raw_param)
            elif isinstance(raw_param, dict):
                param = raw_param
            else:
                param = {}
        except (json.JSONDecodeError, TypeError):
            param = {}

        return PathFinderParam(
            move_duration=param.get("move_duration", 500),
            move_duration_far=param.get("move_duration_far", 1500),
            move_duration_near=param.get("move_duration_near", 300),
            distance_far=param.get("distance_far", 200),
            distance_near=param.get("distance_near", 50),
            max_move_time=param.get("max_move_time", 0),
            dodge_at_start=param.get("dodge_at_start", "never"),
            dodge_direction=param.get("dodge_direction", "backward"),
            enable_turning=param.get("enable_turning", True),
            dodge_follows_direction=param.get("dodge_follows_direction", True),
            dodge_release_ms=param.get("dodge_release_ms", 50),
            move_start_delay_ms=param.get("move_start_delay_ms", 300),
            turn_duration_ms=param.get("turn_duration_ms", 200),
            large_turn_release_ms=param.get("large_turn_release_ms", 100),
            stuck_dodge_on_phase=param.get("stuck_dodge_on_phase", True),
        )

    def _get_recognition_detail(self, argv: CustomAction.RunArg) -> dict | None:
        reco_detail = argv.reco_detail
        if not reco_detail or not reco_detail.best_result:
            return None

        best_result = reco_detail.best_result
        if not isinstance(best_result, CustomRecognitionResult):
            raw = getattr(reco_detail, "raw_detail", None)
            if isinstance(raw, dict) and "detail" in raw:
                detail = raw["detail"]
                if isinstance(detail, str):
                    try:
                        return json.loads(detail)
                    except (json.JSONDecodeError, TypeError):
                        return None
                elif isinstance(detail, dict):
                    return detail
            return None

        detail = best_result.detail
        if isinstance(detail, str):
            try:
                return json.loads(detail)
            except (json.JSONDecodeError, TypeError):
                return None
        elif isinstance(detail, dict):
            return detail

        return None

    def _create_platform(self, context: Context):
        """通过 OperationRecording 的 PlatformFactory 创建平台实例。"""
        try:
            from OperationRecording.platforms import PlatformFactory

            controller = context.tasker.controller
            return PlatformFactory.create_from_config({}, controller)
        except Exception:
            logger.exception("Failed to create OperationRecording platform")
            return None

    def _get_state(self, node_name: str) -> PathFinderState:
        """按节点名获取执行状态，不存在则创建。"""
        if node_name not in self._action_state:
            self._action_state[node_name] = PathFinderState()
        return self._action_state[node_name]

    def _get_node_name(self, context: Context) -> str:
        try:
            detail = getattr(context, "run_task_detail", None)
            if detail is not None:
                node = getattr(detail, "current_node_name", None)
                if node:
                    return node
        except Exception:
            pass

        try:
            tasker = getattr(context, "tasker", None)
            if tasker is not None:
                node = getattr(tasker, "current_node_name", None)
                if node:
                    return node
        except Exception:
            pass

        return "PathFinderAction"

    def _execute_sequence(
        self,
        context: Context,
        platform,
        direction: str,
        param: PathFinderParam,
        distance: Any,
        lock_id: Any,
        turn_start: Any,
        turn_end: Any,
        turn_grade: Any,
        phase: Phase | None,
    ) -> bool:
        """
        执行 dodge → [release] → turn → move 序列

        借鉴 MaaEnd MapTracker：
        - LARGE_TURN 时先释放按键（force walk 类比）
        - phase == STUCK 时额外触发 dodge
        """
        node_name = self._get_node_name(context)

        # 1. dodge：按策略执行；卡住阶段额外触发一次
        should_dodge = self._should_dodge(node_name, param, lock_id)
        should_stuck_dodge = (
            param.stuck_dodge_on_phase
            and phase == Phase.STUCK
            and direction != "centered"
        )
        if should_dodge or should_stuck_dodge:
            if context.tasker.stopping:
                return False
            if direction != "centered":
                dodge_dir = (
                    direction if param.dodge_follows_direction else param.dodge_direction
                )
                platform.dodge(dodge_dir)
            else:
                platform.dodge(param.dodge_direction)

            if param.dodge_release_ms > 0:
                time.sleep(param.dodge_release_ms / 1000.0)
                platform.release_all()

        # 2. turn
        # 死区内不转向：避免中心附近抖动把目标推到屏幕外
        if (
            param.enable_turning
            and direction != "centered"
            and isinstance(turn_start, (list, tuple))
            and isinstance(turn_end, (list, tuple))
            and len(turn_start) == 2
            and len(turn_end) == 2
        ):
            if context.tasker.stopping:
                return False

            sx, sy = int(turn_start[0]), int(turn_start[1])
            ex, ey = int(turn_end[0]), int(turn_end[1])
            logger.debug(
                "[PathFinderAction] turn execute: grade=%s phase=%s "
                "swipe=(%d,%d)->(%d,%d) vec=(%+d,%+d) duration=%s",
                turn_grade,
                phase.value if phase else None,
                sx,
                sy,
                ex,
                ey,
                ex - sx,
                ey - sy,
                param.turn_duration_ms,
            )

            # 大幅转向：先释放按键，避免方向键与滑动冲突
            if turn_grade == "large_turn" and param.large_turn_release_ms > 0:
                platform.release_all()
                time.sleep(param.large_turn_release_ms / 1000.0)

            if param.turn_duration_ms is not None and param.turn_duration_ms > 0:
                platform.turn(
                    sx,
                    sy,
                    ex,
                    ey,
                    duration=param.turn_duration_ms / 1000.0,
                )
            else:
                platform.turn(
                    sx,
                    sy,
                    ex,
                    ey,
                )

        # 3. move
        # 死区内兜底：direction=centered 且 distance=None（无法判断到达距离）
        # 时仍需前进，否则目标在死区内会卡住不动。
        # distance 有效且 direction=centered 表示已到达目标，跳过 move。
        if direction == "centered":
            if distance is not None:
                logger.debug(
                    "[PathFinderAction] skip move: direction=centered, distance=%s (arrived)",
                    distance,
                )
                return True
            logger.debug(
                "[PathFinderAction] centered but distance missing: fallback forward move "
                "duration_ms=%d",
                param.move_duration,
            )
            if context.tasker.stopping:
                return False
            # 兜底前进：默认时长，避免在死区内卡住
            return platform.move("forward", param.move_duration / 1000.0)

        if context.tasker.stopping:
            return False

        # move 开始前的额外等待，确保 dodge/turn 动作完全结束、按键状态稳定
        if param.move_start_delay_ms > 0:
            time.sleep(param.move_start_delay_ms / 1000.0)
            if context.tasker.stopping:
                return False

        move_duration = self._resolve_duration(distance, param)
        return platform.move(direction, move_duration)

    def _should_dodge(
        self,
        node_name: str,
        param: PathFinderParam,
        lock_id: Any,
    ) -> bool:
        """
        判断当前周期是否执行闪避。

        - never: 不闪避
        - always: 每次移动前都闪避
        - once_per_target: 同一 lock_id 只闪避一次
        """
        if param.dodge_at_start == "never":
            return False
        if param.dodge_at_start == "always":
            return True

        # once_per_target
        if lock_id is None or not isinstance(lock_id, str):
            return False

        action_state = self._get_state(node_name)
        if action_state.last_lock_id != lock_id:
            action_state.last_lock_id = lock_id
            return True
        return False

    def _resolve_duration(self, distance: int | None, param: PathFinderParam) -> float:
        """根据目标距离计算移动时长（秒），并受 max_move_time 限制。"""
        if distance is None:
            duration_ms = param.move_duration
        else:
            far = param.distance_far
            near = param.distance_near

            if far <= near:
                duration_ms = param.move_duration
            else:
                far_ms = param.move_duration_far
                near_ms = param.move_duration_near

                if distance >= far:
                    duration_ms = far_ms
                elif distance <= near:
                    duration_ms = near_ms
                else:
                    ratio = (distance - near) / (far - near)
                    duration_ms = near_ms + ratio * (far_ms - near_ms)

        if param.max_move_time > 0:
            duration_ms = min(duration_ms, param.max_move_time)

        return duration_ms / 1000.0