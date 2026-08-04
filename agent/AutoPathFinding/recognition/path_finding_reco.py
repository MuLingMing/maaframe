# -*- coding: utf-8 -*-
"""
自动寻路识别器（重构版）

借鉴 [MaaEnd MapTracker](https://github.com/MaaEnd/MaaEnd) 的设计：
- 双阈值转向分级（rotation_lower_threshold / rotation_upper_threshold）
- 自适应转向速度（rotation_adjuster EMA 0.618/0.382）
- 阶段状态机（Phase）取代分散的标志位
- 卡住检测与 stuckTimeout 兜底
- 闭环控制（感知 → 决策 → 反馈）

使用 MaaFramework 内置 TemplateMatch 识别目标图标，支持多目标识别、
距离提取、方向计算、目标锁定与转向坐标计算。

功能说明：
1. 使用 MaaFramework 内置 TemplateMatch 识别目标图标
2. 支持三种选择器模式：priority（优先级）、nearest（就近）、composite（组合）
3. 支持索引和名称两种 selector_priority 格式
4. 提取目标图标下方的距离信息（OCR，可配置正则和偏移）
5. 计算目标相对于屏幕中心的方向（forward/backward/left/right/centered）
6. 评估目标运动状态（approaching/arrived/stuck）
7. 目标锁定/追踪/解锁，保证选择连贯性
8. 距离缺失时计算转向坐标（turn_start/turn_end）与转向分级（turn_grade）
9. 兼容 IfElseAction 分支（返回 hit_node="if"/"else"）

识别流程（职责分离）：
1. _parse_param: 解析参数并计算优先级顺序
2. _collect_targets: 收集所有匹配目标
3. _resolve_locked_target: 目标锁定/追踪/解锁
4. _select_target: 无锁定时使用 selector/ 目录下的选择器类选择最优目标
5. _calculate_direction: 计算移动方向（含角度滞回）
6. _calculate_turn: 计算转向坐标（含自适应距离）
7. _evaluate_movement_state: 评估运动状态（含阶段状态机）

返回值：CustomRecognition.AnalyzeResult
- hit=True,  hit_node="if"  → 识别到目标
- hit=False, hit_node="else" → 未识别到目标
- direction="centered" 表示目标已在屏幕中心死区内，无需移动
- phase / state 表示目标运动状态
- lock_id 用于跨帧锁定目标
- turn_start/turn_end 用于触发转向以恢复距离显示
- turn_grade 表示本帧转向分级
"""

import functools
import json
import logging
import math
import os
import re
import time
from typing import Any, ClassVar, Dict, List, Optional, Tuple, Union

import numpy as np
from maa.context import Context
from maa.custom_recognition import CustomRecognition
from maa.define import OCRResult, RectType, TemplateMatchResult
from maa.pipeline import JOCR, JRecognitionType, JTemplateMatch

from ..selector import (
    CompositeTargetSelector,
    NearestTargetSelector,
    PriorityTargetSelector,
    TargetInfo,
    TargetSelector,
)
from ..state import LockedTarget, Phase, PathFindingState, TurnGrade
from .param import PathFindingParam
from .turn_strategy import (
    SCREEN_CENTER,
    SCREEN_HEIGHT,
    SCREEN_WIDTH,
    angle_diff,
    bin_direction,
    classify_turn,
    compute_target_angle,
    compute_turn_coordinates,
    dir_center_angle,
)

logger = logging.getLogger(__name__)


@functools.lru_cache(maxsize=64)
def _expand_folder_cached(image_dir: str, folder_path: str) -> list[str] | None:
    """
    缓存的文件夹展开器。

    image_dir 作为缓存键的第一维，运行期切换 image_dir 时缓存自动失效。
    模板图通常运行时不变，缓存安全。
    """
    image_extensions = {".png", ".jpg", ".jpeg", ".bmp", ".webp"}
    files: list[str] = []
    for root, _, filenames in os.walk(folder_path):
        for filename in filenames:
            if os.path.splitext(filename.lower())[1] not in image_extensions:
                continue
            full_path = os.path.join(root, filename)
            rel_path = os.path.relpath(full_path, image_dir)
            files.append(rel_path.replace("\\", "/"))

    if not files:
        return None
    return sorted(files)


class PathFindingReco(CustomRecognition):
    """
    自动寻路识别器

    参数格式（JSON）：
    {
        "expected_templates": ["quest_icon.png", "npc_icon.png"],
        "roi": [0, 0, 1280, 720],
        "threshold": 0.8,
        "selector_type": "priority",
        "selector_priority": [0, 1],
        "distance_pattern": "(\\d+)米",
        "distance_offset": [-26, -30, 47, 57],
        "distance_threshold": 0.3,
        "dead_zone": 50,
        "arrival_distance": 30,
        "stuck_threshold": 3,
        "stuck_distance_tolerance": 5,
        "stuck_center_tolerance": 10,
        "target_lock_timeout": 3,
        "target_lock_center_tolerance": 100,
        "target_lock_score_threshold": 0.75,
        "enable_turning": true,
        "max_turn_attempts": 3,
        "min_turn_distance": 50,
        "max_turn_distance": 400,
        "distance_missing_turn_scale": 0.5,
        "enable_locked_fast_path": true,
        "lock_roi_padding": 200,
        "fast_path_max_turn_padding": 400,
        "direction_hysteresis": 15.0,
        "fast_path_miss_limit": 3,
        "turn_min_floor_ratio": 0.5,
        "rotation_lower_threshold": 8.0,
        "rotation_upper_threshold": 60.0,
        "rotation_adaptive_enabled": true,
        "stuck_timeout_ms": 10000
    }

    Pipeline 使用示例：
    {
        "AutoPathFinding": {
            "recognition": "Custom",
            "custom_recognition": "PathFindingReco",
            "custom_recognition_param": {
                "expected_templates": ["副本/目标点", "副本/NPC"],
                "selector_type": "priority",
                "selector_priority": [1, 0]
            },
            "action": "Custom",
            "custom_action": "IfElseAction",
            "custom_action_param": {
                "if": ["PathFinderAction"],
                "else": ["TaskComplete"]
            }
        }
    }
    """

    # 跨帧状态存储，按节点名隔离。生命周期跟随 Python 进程。
    _state: ClassVar[dict[str, PathFindingState]] = {}

    # 目标锁定 IOU 阈值
    DEFAULT_TARGET_LOCK_IOU_THRESHOLD: ClassVar[float] = 0.3

    # 转向起止点裁剪区域：四象限对称分布，根据目标偏移方向选择。
    # 720p 基准：[x, y, w, h]。
    # 水平方向（左/右）用上方水平条带；垂直方向（上/下）用中央垂直条带。
    TURN_CLIP_HORIZONTAL: ClassVar[tuple[int, int, int, int]] = (200, 80, 880, 120)
    TURN_CLIP_VERTICAL: ClassVar[tuple[int, int, int, int]] = (560, 60, 160, 600)

    # 触发转向的最小中心偏移比例（相对屏幕半宽）
    TURN_MIN_OFFSET_RATIO: ClassVar[float] = 0.0

    # 状态 → 字符串映射（兼容 IfElseAction）
    STATE_TO_STRING: ClassVar[dict[Phase, str]] = {
        Phase.SEEKING: "seeking",
        Phase.TRACKING: "tracking",
        Phase.APPROACHING: "approaching",
        Phase.TURNING: "turning",
        Phase.STUCK: "stuck",
        Phase.LOST: "lost",
    }

    def analyze(
        self,
        context: Context,
        argv: CustomRecognition.AnalyzeArg,
    ) -> CustomRecognition.AnalyzeResult | RectType | None:
        if context.tasker.stopping:
            return None

        # 1. 解析参数
        param = self._parse_param(argv.custom_recognition_param)

        # 2. 获取图片
        img = argv.image
        if img is None or img.size == 0:
            return None

        node_name = getattr(argv, "node_name", "PathFindingReco")
        state = self._get_state(node_name)

        # 3. 计算本帧 ROI（锁定态快速通道：小 ROI；否则全 ROI；连续 miss 时强制全 ROI）
        frame_roi = self._resolve_frame_roi(state, param)

        # 4. 收集目标
        targets = self._collect_targets(context, img, param, frame_roi)

        # fast_path miss 检测：连续 miss 时强制全 ROI 重新选择
        fast_path_miss = (
            frame_roi is not None and not targets and state.locked_target is not None
        )
        if fast_path_miss:
            state.fast_path_miss_count += 1
            if state.fast_path_miss_count >= param.fast_path_miss_limit:
                state.locked_target = None
                state.fast_path_miss_count = 0
                targets = self._collect_targets(context, img, param, None)
        else:
            state.fast_path_miss_count = 0

        # 5. 目标锁定/追踪/解锁
        selected = self._resolve_locked_target(state, targets, param)

        if selected is None:
            self._clear_state(node_name)
            return CustomRecognition.AnalyzeResult(
                box=None, detail={"hit": False, "hit_node": "else"}
            )

        if context.tasker.stopping:
            return None

        # 6. 对选中目标执行 OCR（nearest/composite 模式的性能优化）
        selected = self._extract_distance(context, img, selected, param)

        # 6.1 维护锁定目标的连续距离缺失计数
        if state.locked_target is not None:
            if selected.distance is not None:
                state.locked_target.distance_miss_count = 0
            else:
                state.locked_target.distance_miss_count += 1

        # 6.2 锁定丢失帧时把 distance 置 None，避免用过期距离算 move 时长
        if (
            state.locked_target is not None
            and state.locked_target.miss_count > 0
            and selected.distance is not None
        ):
            selected = TargetInfo(
                template=selected.template,
                center=selected.center,
                bbox=selected.bbox,
                score=selected.score,
                distance=None,
            )

        # 7. 计算移动方向（含角度滞回）
        direction = self._calculate_direction(
            selected.center, param.dead_zone, state, param.direction_hysteresis
        )

        # 8. 计算转向坐标与分级（含自适应距离）
        turn_start, turn_end, turn_grade = self._calculate_turn(
            state, selected, param
        )

        # 9. 评估运动状态与更新阶段
        state_eval, phase = self._evaluate_movement_state(
            node_name, selected, state, param
        )

        # 10. 更新锁定目标的状态
        if state.locked_target is not None:
            locked = state.locked_target
            locked.center = selected.center
            locked.bbox = selected.bbox
            locked.distance = selected.distance
            locked.score = selected.score
            locked.last_direction = direction
            locked.last_target_angle = compute_target_angle(selected.center)

        return CustomRecognition.AnalyzeResult(
            box=list(selected.bbox),
            detail={
                "hit": True,
                "hit_node": "if",
                "target": {
                    "template": selected.template,
                    "center": list(selected.center),
                    "bbox": list(selected.bbox),
                    "score": selected.score,
                    "distance": selected.distance,
                },
                "direction": direction,
                "state": state_eval,
                "phase": self.STATE_TO_STRING.get(phase, phase.value),
                "lock_id": state.locked_target.lock_id if state.locked_target else None,
                "turn_start": list(turn_start) if turn_start else None,
                "turn_end": list(turn_end) if turn_end else None,
                "turn_grade": turn_grade.value if turn_grade else None,
            },
        )

    def _get_state(self, node_name: str) -> PathFindingState:
        """按节点名获取寻路状态，不存在则创建。"""
        if node_name not in self._state:
            self._state[node_name] = PathFindingState()
        return self._state[node_name]

    def _resolve_frame_roi(
        self,
        state: PathFindingState,
        param: PathFindingParam,
    ) -> tuple[int, int, int, int] | None:
        """
        锁定态快速通道：根据上一帧锁定目标 bbox 计算本帧小 ROI。

        返回 None 时使用全 param.roi。
        """
        if not param.enable_locked_fast_path:
            return None
        locked = state.locked_target
        if locked is None or locked.miss_count > 0:
            return None

        bx, by, bw, bh = locked.bbox
        cx = bx + bw / 2
        cy = by + bh / 2
        # 距离缺失时适当扩大 ROI，避免转向后目标移出视野
        padding = param.lock_roi_padding
        if locked.distance is None:
            padding += param.fast_path_max_turn_padding

        roi_w = max(bw, 1) + 2 * padding
        roi_h = max(bh, 1) + 2 * padding
        roi_x = int(max(0, cx - roi_w / 2))
        roi_y = int(max(0, cy - roi_h / 2))
        roi_w = int(roi_w)
        roi_h = int(roi_h)

        # 与 param.roi 求交集，防止越界
        px, py, pw, ph = param.roi
        ix = max(roi_x, px)
        iy = max(roi_y, py)
        iw = min(roi_x + roi_w, px + pw) - ix
        ih = min(roi_y + roi_h, py + ph) - iy
        if iw <= 0 or ih <= 0:
            return None
        return (ix, iy, iw, ih)

    def _clear_state(self, node_name: str) -> None:
        """按节点名清空寻路状态。"""
        self._state.pop(node_name, None)

    def _parse_param(
        self, raw_param: Union[str, Dict[str, Any], None]
    ) -> PathFindingParam:
        if isinstance(raw_param, str):
            try:
                param = json.loads(raw_param)
            except json.JSONDecodeError:
                logger.warning("Failed to decode custom_recognition_param as JSON")
                param = {}
        elif isinstance(raw_param, dict):
            param = raw_param
        else:
            param = {}

        expected_templates = param.get("expected_templates", [])
        selector_priority = param.get("selector_priority", [])

        ordered_templates = self._calculate_priority_order(
            expected_templates, selector_priority
        )

        return PathFindingParam(
            ordered_templates=ordered_templates,
            roi=tuple(param.get("roi", [0, 0, SCREEN_WIDTH, SCREEN_HEIGHT])),
            threshold=param.get("threshold", 0.8),
            selector_type=param.get("selector_type", "priority"),
            distance_pattern=param.get("distance_pattern", r"(\d+)米"),
            distance_offset=param.get("distance_offset", [-26, -30, 47, 57]),
            distance_threshold=param.get("distance_threshold", 0.3),
            dead_zone=param.get("dead_zone", 50),
            arrival_distance=param.get("arrival_distance", 30),
            stuck_threshold=param.get("stuck_threshold", 3),
            stuck_distance_tolerance=param.get("stuck_distance_tolerance", 5),
            stuck_center_tolerance=param.get("stuck_center_tolerance", 10),
            target_lock_timeout=param.get("target_lock_timeout", 3),
            target_lock_center_tolerance=param.get("target_lock_center_tolerance", 100),
            target_lock_score_threshold=param.get("target_lock_score_threshold", 0.75),
            enable_turning=param.get("enable_turning", True),
            max_turn_attempts=param.get("max_turn_attempts", 3),
            min_turn_distance=param.get("min_turn_distance", 50),
            max_turn_distance=param.get("max_turn_distance", 400),
            distance_missing_turn_scale=param.get("distance_missing_turn_scale", 0.5),
            enable_locked_fast_path=param.get("enable_locked_fast_path", True),
            lock_roi_padding=param.get("lock_roi_padding", 200),
            fast_path_max_turn_padding=param.get("fast_path_max_turn_padding", 400),
            direction_hysteresis=param.get("direction_hysteresis", 15.0),
            fast_path_miss_limit=param.get("fast_path_miss_limit", 3),
            turn_min_floor_ratio=param.get("turn_min_floor_ratio", 0.5),
            rotation_lower_threshold=param.get("rotation_lower_threshold", 8.0),
            rotation_upper_threshold=param.get("rotation_upper_threshold", 60.0),
            rotation_adaptive_enabled=param.get("rotation_adaptive_enabled", True),
            stuck_timeout_ms=param.get("stuck_timeout_ms", 10000),
        )

    def _calculate_priority_order(
        self,
        expected_templates: List[str],
        selector_priority: List[Any],
    ) -> List[str]:
        if not selector_priority:
            return expected_templates.copy()
        if selector_priority and isinstance(selector_priority[0], int):
            return self._priority_order_by_index(expected_templates, selector_priority)
        return self._priority_order_by_name(expected_templates, selector_priority)

    def _priority_order_by_index(
        self,
        expected_templates: List[str],
        selector_priority: List[Any],
    ) -> List[str]:
        ordered: List[str] = []
        remaining = list(range(len(expected_templates)))

        for idx in selector_priority:
            if isinstance(idx, int) and 0 <= idx < len(expected_templates):
                ordered.append(expected_templates[idx])
                if idx in remaining:
                    remaining.remove(idx)
            else:
                logger.warning(
                    "Invalid selector_priority index %r for %d expected_templates",
                    idx,
                    len(expected_templates),
                )

        for idx in remaining:
            ordered.append(expected_templates[idx])

        return ordered

    def _priority_order_by_name(
        self,
        expected_templates: List[str],
        selector_priority: List[Any],
    ) -> List[str]:
        ordered: List[str] = []
        remaining = expected_templates.copy()

        for name in selector_priority:
            if isinstance(name, str) and name in remaining:
                ordered.append(name)
                remaining.remove(name)
            else:
                logger.warning(
                    "Invalid selector_priority name %r, available: %r",
                    name,
                    expected_templates,
                )

        ordered.extend(remaining)
        return ordered

    def _resolve_locked_target(
        self,
        state: PathFindingState,
        targets: List[TargetInfo],
        param: PathFindingParam,
    ) -> Optional[TargetInfo]:
        """返回选中的目标（基于锁定追踪或重新选择）。"""
        locked = state.locked_target

        if locked is not None:
            if targets:
                matched = self._track_locked_target(locked, targets, param)
                if matched is not None:
                    locked.miss_count = 0
                    return matched

            locked.miss_count += 1
            if locked.miss_count < param.target_lock_timeout:
                return TargetInfo(
                    template=locked.template,
                    center=locked.center,
                    bbox=locked.bbox,
                    score=locked.score,
                    distance=locked.distance,
                )
            else:
                state.locked_target = None
                locked = None

        if locked is None and targets:
            selected = self._select_target(targets, param)
            if selected is not None:
                lock_id = LockedTarget.generate_lock_id(
                    selected.template, selected.center
                )
                state.locked_target = LockedTarget(
                    template=selected.template,
                    center=selected.center,
                    bbox=selected.bbox,
                    score=selected.score,
                    distance=selected.distance,
                    lock_id=lock_id,
                    miss_count=0,
                )
                # 新目标：重置与该目标绑定的所有本地状态
                state.reset_target_local_state()
                return selected

        return None

    def _track_locked_target(
        self,
        locked: LockedTarget,
        targets: List[TargetInfo],
        param: PathFindingParam,
    ) -> Optional[TargetInfo]:
        """在候选中追踪已锁定目标。"""
        candidates = [
            t
            for t in targets
            if t.template == locked.template
            and t.score >= param.target_lock_score_threshold
        ]
        if not candidates:
            return None

        best = max(candidates, key=lambda t: locked.iou(t.bbox))
        if locked.iou(best.bbox) < self.DEFAULT_TARGET_LOCK_IOU_THRESHOLD:
            return None

        dx = best.center[0] - locked.center[0]
        dy = best.center[1] - locked.center[1]
        if math.hypot(dx, dy) > param.target_lock_center_tolerance:
            return None

        return best

    def _collect_targets(
        self,
        context: Context,
        img: np.ndarray,
        param: PathFindingParam,
        roi: tuple[int, int, int, int] | None = None,
    ) -> List[TargetInfo]:
        """收集所有匹配的目标（不含距离）。"""
        use_roi = roi if roi is not None else param.roi
        targets: List[TargetInfo] = []
        for template_name in param.ordered_templates:
            matched = self._match_template(
                context, img, template_name, use_roi, param.threshold
            )
            if matched:
                targets.extend(matched)
                if param.selector_type == "priority":
                    break
        return targets

    def _select_target(
        self,
        targets: List[TargetInfo],
        param: PathFindingParam,
    ) -> Optional[TargetInfo]:
        if not targets:
            return None
        selector = self._create_selector(param)
        return selector.select(targets)

    def _create_selector(self, param: PathFindingParam) -> TargetSelector:
        selector_type = param.selector_type

        if selector_type == "priority":
            return PriorityTargetSelector(priority=param.ordered_templates)
        elif selector_type == "nearest":
            return NearestTargetSelector(fallback_to_first=True)
        elif selector_type == "composite":
            return CompositeTargetSelector(type_priority=param.ordered_templates)
        else:
            logger.warning(
                "Unknown selector_type %r, falling back to priority", selector_type
            )
            return PriorityTargetSelector(priority=param.ordered_templates)

    def _resolve_image_dir(self) -> str | None:
        """解析资源图片目录。优先使用项目源码路径，回退到当前工作目录。"""
        candidates = [
            os.path.abspath(
                os.path.join(
                    os.path.dirname(__file__), "..", "..", "..", "..", "assets", "resource", "image"
                )
            ),
            os.path.abspath(os.path.join(os.getcwd(), "assets", "resource", "image")),
        ]
        for path in candidates:
            if os.path.isdir(path):
                return path
        return None

    def _expand_template_folder(self, template_name: str) -> list[str] | None:
        """若 template_name 对应文件夹，则展开为其中所有图片的相对路径列表。"""
        image_dir = self._resolve_image_dir()
        if image_dir is None:
            return None

        folder_path = os.path.join(image_dir, template_name)
        if not os.path.isdir(folder_path):
            return None

        return _expand_folder_cached(image_dir, folder_path)

    def _match_template(
        self,
        context: Context,
        img: np.ndarray,
        template_name: str,
        roi: tuple[int, int, int, int],
        threshold: float,
    ) -> list[TargetInfo]:
        """单个模板匹配（支持文件夹展开）。"""
        templates = self._expand_template_folder(template_name)
        if templates is None:
            templates = [template_name]
        if not templates:
            return []

        reco_param = JTemplateMatch(
            template=templates,
            roi=roi,
            threshold=[threshold] * len(templates),
            green_mask=True,
        )
        reco_detail = context.run_recognition_direct(
            JRecognitionType.TemplateMatch,
            reco_param,
            img,
        )

        targets: list[TargetInfo] = []
        if reco_detail is not None and reco_detail.hit:
            results = (
                reco_detail.filtered_results
                if reco_detail.filtered_results
                else reco_detail.all_results
            )
            for result in results:
                if not isinstance(result, TemplateMatchResult):
                    continue
                if result.score < threshold:
                    continue
                box = result.box
                if box is None:
                    continue
                if isinstance(box, (list, tuple)):
                    x, y, w, h = box[0], box[1], box[2], box[3]
                else:
                    x, y, w, h = box.x, box.y, box.w, box.h
                center = (x + w / 2, y + h / 2)
                targets.append(
                    TargetInfo(
                        template=template_name,
                        center=center,
                        bbox=(x, y, w, h),
                        score=result.score,
                    )
                )

        return targets

    def _extract_distance(
        self,
        context: Context,
        img: np.ndarray,
        target: TargetInfo,
        param: PathFindingParam,
    ) -> TargetInfo:
        """提取目标下方的距离信息"""
        x, y, w, h = target.bbox
        offset_x, offset_y, offset_w, offset_h = param.distance_offset
        roi = (
            max(0, x + offset_x),
            max(0, y + offset_y),
            w + offset_w,
            h + offset_h,
        )

        try:
            reco_param = JOCR(
                expected=[param.distance_pattern],
                roi=roi,
                threshold=param.distance_threshold,
            )
            reco_detail = context.run_recognition_direct(
                JRecognitionType.OCR,
                reco_param,
                img,
            )

            if reco_detail is not None and reco_detail.hit:
                best_result = reco_detail.best_result
                if isinstance(best_result, OCRResult):
                    text = best_result.text
                    if text:
                        match = re.search(param.distance_pattern, text)
                        if match:
                            distance_str = match.group(1)
                            if distance_str.isdigit():
                                return TargetInfo(
                                    template=target.template,
                                    center=target.center,
                                    bbox=target.bbox,
                                    score=target.score,
                                    distance=int(distance_str),
                                )
        except Exception as e:
            logger.warning(
                "Distance extraction failed for %s (roi=%s): %s",
                target.template,
                roi,
                e,
                exc_info=True,
            )

        return target

    def _calculate_turn(
        self,
        state: PathFindingState,
        selected: TargetInfo,
        param: PathFindingParam,
    ) -> Tuple[Optional[Tuple[int, int]], Optional[Tuple[int, int]], Optional[TurnGrade]]:
        """
        当 enable_turning=true 且已选定目标 distance is None 时计算转向坐标。

        返回 (turn_start, turn_end, turn_grade) 或 (None, None, None)。
        """
        if not param.enable_turning:
            state.turn_attempts_without_distance = 0
            state.turn_grade = None
            return None, None, None

        distance = selected.distance

        if distance is not None:
            # 距离已识别：重置计数 + 自适应速度，并返回
            state.turn_attempts_without_distance = 0
            state.turn_grade = None
            return None, None, None

        if selected.score < param.target_lock_score_threshold:
            return None, None, None

        # 距离缺失：计数 +1
        state.turn_attempts_without_distance += 1

        if state.turn_attempts_without_distance > param.max_turn_attempts:
            state.turn_grade = None
            return None, None, None

        # 计算转向分级（双阈值，借鉴 MapTracker）
        current_angle = compute_target_angle(selected.center)
        last_angle = (
            state.locked_target.last_target_angle if state.locked_target else None
        )
        grade = classify_turn(
            current_angle,
            last_angle,
            param.rotation_lower_threshold,
            param.rotation_upper_threshold,
        )
        state.turn_grade = grade
        state.phase = Phase.TURNING

        # 计算自适应转向距离
        adaptive_speed = (
            state.rotation_adjuster.speed if param.rotation_adaptive_enabled else 1.0
        )

        coords = compute_turn_coordinates(
            selected.center,
            state.turn_attempts_without_distance,
            min_turn_distance=param.min_turn_distance,
            max_turn_distance=param.max_turn_distance,
            distance_missing_turn_scale=param.distance_missing_turn_scale,
            turn_min_floor_ratio=param.turn_min_floor_ratio,
            horizontal_clip=self.TURN_CLIP_HORIZONTAL,
            vertical_clip=self.TURN_CLIP_VERTICAL,
            adaptive_speed=adaptive_speed,
            min_offset_ratio=self.TURN_MIN_OFFSET_RATIO,
        )

        if coords is None:
            return None, None, grade

        return coords[0], coords[1], grade

    def _evaluate_movement_state(
        self,
        node_name: str,
        selected: TargetInfo,
        state: PathFindingState,
        param: PathFindingParam,
    ) -> Tuple[str, Phase]:
        """
        评估运动状态与阶段（借鉴 MapTracker 闭环反馈）。

        返回 (state_string, phase)。
        - state_string: "approaching" / "arrived" / "stuck"
        - phase: 阶段状态机
        """
        distance = selected.distance
        if distance is not None and distance <= param.arrival_distance:
            self._clear_state(node_name)
            return "arrived", Phase.APPROACHING

        # 卡住检测（独立组件）
        now_ms = int(time.time() * 1000)
        # 同步 threshold/容差到 detector（参数可能通过 param 传入但 detector 已实例化）
        state.stuck_detector.threshold_frames = param.stuck_threshold
        state.stuck_detector.distance_tolerance = float(param.stuck_distance_tolerance)
        state.stuck_detector.center_tolerance = float(param.stuck_center_tolerance)
        is_stuck = state.stuck_detector.is_stuck_frame(
            selected.center, distance, now_ms=now_ms
        )
        state.stuck_count = state.stuck_detector.count
        state.last_distance = distance
        state.last_center = selected.center

        # 阶段判定
        if is_stuck:
            phase = Phase.STUCK
            state_eval = "stuck"
        elif state.locked_target is not None and state.locked_target.miss_count > 0:
            phase = Phase.LOST
            state_eval = "approaching"
        elif distance is not None and distance <= param.arrival_distance * 2:
            phase = Phase.APPROACHING
            state_eval = "approaching"
        elif state.locked_target is not None:
            phase = Phase.TRACKING
            state_eval = "approaching"
        else:
            phase = Phase.SEEKING
            state_eval = "approaching"

        # 若当前为转向阶段，保持 TURNING（不覆盖）
        if state.phase == Phase.TURNING and state.turn_grade is not None:
            phase = Phase.TURNING
            state_eval = "approaching"

        state.phase = phase
        return state_eval, phase

    def _calculate_direction(
        self,
        target_center: tuple[float, float],
        dead_zone: int,
        state: PathFindingState,
        hysteresis: float,
    ) -> str:
        """计算目标相对于屏幕中心的方向（含角度滞回）。"""
        tx, ty = target_center
        cx, cy = SCREEN_CENTER

        dx = tx - cx
        dy = ty - cy

        # 1. 圆形死区
        if math.hypot(dx, dy) <= dead_zone:
            return "centered"

        # 2. 角度分箱
        angle = compute_target_angle(target_center)
        new_dir = bin_direction(angle)

        # 3. 角度滞回：避免 45° 边界方向抖动
        last_dir = state.locked_target.last_direction if state.locked_target else None
        if (
            hysteresis > 0
            and last_dir is not None
            and last_dir != "centered"
            and new_dir != last_dir
        ):
            last_center_angle = dir_center_angle(last_dir)
            if last_center_angle is not None:
                delta = abs(angle_diff(angle, last_center_angle))
                if delta < hysteresis:
                    return last_dir

        return new_dir