# AutoPathFinding 实现设计方案（方案B）

## 1. 设计目标

基于已确认的需求分析，本次实现围绕以下三个核心目标展开：

1. **移动连续性**：在单次识别-动作周期内，按 `dodge → turn → move` 顺序执行，支持闪避触发疾跑、转向恢复距离显示、最大移动时间兜底。
2. **目标选择连贯性**：通过 `LockedTarget` 与 `PathFindingState`，按节点隔离地锁定已选目标，跨帧追踪，防止移动过程中目标跳动。
3. **参数扩展与向后兼容**：新增参数均有安全默认值，旧配置无需修改即可运行；类级常量保留在源码中便于测试调优，但不暴露为 Custom 参数。

## 2. 总体架构

保持 **Reco 负责“看屏幕并返回决策”、Action 负责“执行操作”** 的职责分离。

```
Pipeline 循环
    │
    ▼
PathFindingReco (Custom Recognition)
    ├─ 目标锁定/追踪/解锁
    ├─ 距离提取与方向计算
    ├─ 运动状态评估（arrived/stuck/approaching）
    ├─ 转向坐标计算（turn_start / turn_end）
    └─ 返回 detail: {target, direction, state, lock_id, turn_start, turn_end}
    │
    ▼
PathFinderAction (Custom Action)
    ├─ 解析参数
    ├─ 按 lock_id 判断是否 dodge（once_per_target）
    ├─ 按 turn_start/turn_end 与 enable_turning 判断是否 turn
    ├─ 计算并裁剪移动时长（max_move_time）
    ├─ 执行 dodge → turn → move
    └─ release_all()
    │
    ▼
  next 回到 PathFindingReco
```

## 3. 新增与修改文件清单

| 类型 | 文件路径                                                 | 说明                                                           |
| ---- | -------------------------------------------------------- | -------------------------------------------------------------- |
| 新增 | `agent/AutoPathFinding/state/__init__.py`                | 状态包入口                                                     |
| 新增 | `agent/AutoPathFinding/state/path_finding_state.py`      | `LockedTarget`、`PathFindingState`、`PathFinderState`          |
| 修改 | `agent/AutoPathFinding/recognition/param.py`             | 扩展 `PathFindingParam`（锁定 + 转向计算参数）                 |
| 修改 | `agent/AutoPathFinding/recognition/path_finding_reco.py` | 目标锁定、转向计算、状态迁移                                   |
| 修改 | `agent/AutoPathFinding/action/param.py`                  | 扩展 `PathFinderParam`（执行侧闪避/最大移动时间/转向执行开关） |
| 修改 | `agent/AutoPathFinding/action/path_finder_action.py`     | dodge / turn / max_move_time 执行序列                          |
| 修改 | `agent/AutoPathFinding/__init__.py`                      | 可选：导出公共类型                                             |
| 修改 | `deps/tools/custom_reco/PathFindingReco.schema.json`     | 新增 Reco 参数 Schema                                          |
| 修改 | `deps/tools/custom_act/PathFinderAction.schema.json`     | 新增 Action 执行参数 Schema                                    |
| 修改 | `agent/AutoPathFinding/README.md`                        | 更新使用说明（如存在）                                         |

## 4. 状态类设计

### 4.1 文件：`agent/AutoPathFinding/state/path_finding_state.py`

```python
from __future__ import annotations

import secrets
from dataclasses import dataclass


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
    """按节点隔离的寻路状态（Reco 侧）。"""

    locked_target: LockedTarget | None = None
    last_distance: int | None = None
    last_center: tuple[float, float] | None = None
    stuck_count: int = 0
    turn_attempts_without_distance: int = 0


@dataclass
class PathFinderState:
    """按节点隔离的执行状态（Action 侧）。"""

    last_lock_id: str | None = None
```

### 4.2 存储方式

- `PathFindingReco` 使用类级 `_state: ClassVar[dict[str, PathFindingState]] = {}`，按 `node_name` 隔离。
- `PathFinderAction` 使用类级 `_action_state: ClassVar[dict[str, PathFinderState]] = {}`，按 `node_name` 隔离。
- 生命周期跟随 Python 进程；到达、连续丢失、无目标时清空对应节点状态。

## 5. 参数职责划分

为避免 Reco 与 Action 参数混杂，明确职责边界：

- **`PathFindingParam`（Reco 侧）**：负责“要不要转向、转向距离怎么算”。包含 `enable_turning`、`max_turn_attempts`、`min_turn_distance`、`max_turn_distance`、`distance_missing_turn_scale`、锁定相关参数。
- **`PathFinderParam`（Action 侧）**：负责“要不要执行转向、闪避模式、最大移动时间”。包含 `enable_turning`（执行侧保险开关）、`dodge_at_start`、`dodge_direction`、`max_move_time`、保留的时长参数。

两侧 `enable_turning` 职责不同：

- Reco 的 `enable_turning=false`：不计算、不返回 `turn_start/turn_end`。
- Action 的 `enable_turning=false`：即使 Reco 返回了 `turn_start/turn_end`，也不执行 `platform.turn()`。

默认均为 `true`。关闭任意一侧即可关闭转向效果；同时关闭则完全不转向。

## 6. `PathFindingReco` 实现细节

### 6.1 参数扩展（`recognition/param.py`）

```python
@dataclass(frozen=True)
class PathFindingParam:
    # 保留现有字段 ...

    # 新增：目标锁定
    target_lock_timeout: int
    target_lock_center_tolerance: int
    target_lock_score_threshold: float

    # 新增：转向计算
    enable_turning: bool
    max_turn_attempts: int
    min_turn_distance: int
    max_turn_distance: int
    distance_missing_turn_scale: float
```

### 6.2 类级常量

```python
class PathFindingReco(CustomRecognition):
    # 目标锁定 IOU 阈值
    DEFAULT_TARGET_LOCK_IOU_THRESHOLD: ClassVar[float] = 0.3

    # 转向起止点裁剪区域：水平/垂直两套，按主轴动态选择
    TURN_CLIP_HORIZONTAL: ClassVar[tuple[int, int, int, int]] = (200, 80, 880, 120)
    TURN_CLIP_VERTICAL: ClassVar[tuple[int, int, int, int]] = (560, 60, 160, 600)

    # 触发转向的最小中心偏移比例（相对屏幕半宽）
    TURN_MIN_OFFSET_RATIO: ClassVar[float] = 0.0

    # Phase → 字符串映射（兼容 IfElseAction）
    STATE_TO_STRING: ClassVar[dict[Phase, str]] = {
        Phase.SEEKING: "seeking",
        Phase.TRACKING: "tracking",
        Phase.APPROACHING: "approaching",
        Phase.TURNING: "turning",
        Phase.STUCK: "stuck",
        Phase.LOST: "lost",
    }
```

### 6.3 `analyze` 主流程重构

```python
def analyze(self, context, argv):
    if context.tasker.stopping:
        return None

    param = self._parse_param(argv.custom_recognition_param)

    img = argv.image
    if img is None or img.size == 0:
        return None

    node_name = getattr(argv, "node_name", "PathFindingReco")
    state = self._get_state(node_name)

    # 1. 收集所有候选目标
    targets = self._collect_targets(context, img, param)

    # 2. 目标锁定/追踪/解锁
    selected = self._resolve_locked_target(state, targets, param)

    if selected is None:
        self._clear_state(node_name)
        return CustomRecognition.AnalyzeResult(
            box=None, detail={"hit": False, "hit_node": "else"}
        )

    # 3. 计算移动方向
    direction = self._calculate_direction(selected.center, param.dead_zone)

    # 4. 计算转向坐标
    turn_start, turn_end = self._calculate_turn(state, selected, param)

    # 5. 评估运动状态
    state_eval = self._evaluate_movement_state(node_name, selected, param)

    # 6. 更新锁定目标的距离/位置
    if state.locked_target is not None:
        locked = state.locked_target
        locked.center = selected.center
        locked.bbox = selected.bbox
        locked.distance = selected.distance
        locked.score = selected.score

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
            "lock_id": state.locked_target.lock_id if state.locked_target else None,
            "turn_start": list(turn_start) if turn_start else None,
            "turn_end": list(turn_end) if turn_end else None,
        },
    )
```

### 6.4 目标锁定/追踪/解锁逻辑

```python
def _resolve_locked_target(self, state, targets, param):
    """
    返回选中的目标（基于锁定追踪或重新选择）。
    - 有锁定目标时优先追踪；追踪成功返回当前匹配。
    - 追踪失败但 miss_count 未超时，返回锁定目标的历史位置以保持动作连续性。
    - 丢失超时后解锁并重新选择。
    - 无锁定时执行正常选择流程，新锁定时重置历史运动状态。
    """
    locked = state.locked_target

    if locked is not None:
        if targets:
            matched = self._track_locked_target(locked, targets, param)
            if matched is not None:
                locked.miss_count = 0
                return matched

        # 追踪失败：递增 miss_count
        locked.miss_count += 1
        if locked.miss_count < param.target_lock_timeout:
            # 未超时：使用历史位置继续，保持移动/转向连续性
            return TargetInfo(
                template=locked.template,
                center=locked.center,
                bbox=locked.bbox,
                score=locked.score,
                distance=locked.distance,
            )
        else:
            # 超时：解锁
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
            # 新目标：重置历史运动状态
            state.last_distance = None
            state.last_center = None
            state.stuck_count = 0
            return selected

    return None


def _track_locked_target(self, locked, targets, param):
    """
    在候选中追踪已锁定目标。
    优先选择：同名模板 + 与上一帧 bbox IOU 最大 + score >= threshold + IOU >= 常量阈值 + 中心容差通过。
    """
    candidates = [
        t for t in targets
        if t.template == locked.template and t.score >= param.target_lock_score_threshold
    ]
    if not candidates:
        return None

    # 按 IOU 降序，取最大 IOU
    best = max(candidates, key=lambda t: locked.iou(t.bbox))
    if locked.iou(best.bbox) < self.DEFAULT_TARGET_LOCK_IOU_THRESHOLD:
        return None

    # 额外中心容差校验
    dx = best.center[0] - locked.center[0]
    dy = best.center[1] - locked.center[1]
    if math.hypot(dx, dy) > param.target_lock_center_tolerance:
        return None

    return best
```

### 6.5 转向坐标计算

```python
def _calculate_turn(self, state, selected, param):
    """
    当 enable_turning=true 且已选定目标 distance is None 时计算 turn_start/turn_end。
    返回 (turn_start, turn_end) 或 (None, None)。
    """
    if not param.enable_turning:
        state.turn_attempts_without_distance = 0
        return None, None

    distance = selected.distance

    # 距离已识别：重置计数，不转向
    if distance is not None:
        state.turn_attempts_without_distance = 0
        return None, None

    # 距离缺失：计数 +1
    state.turn_attempts_without_distance += 1

    # 超过最大尝试次数：停止转向，直接移动
    if state.turn_attempts_without_distance > param.max_turn_attempts:
        return None, None

    return self._compute_turn_coordinates(
        selected.center,
        state.turn_attempts_without_distance,
        param,
    )


def _compute_turn_coordinates(
    self,
    target_center: tuple[float, float],
    turn_attempt: int,
    param: PathFindingParam,
) -> tuple[tuple[int, int], tuple[int, int]] | tuple[None, None]:
    """
    基于目标中心偏移计算转向滑动起止点。
    - 滑动方向与目标偏移相反（目标偏左则向右滑、偏右则向左滑）。
    - 第 1 次全额滑动，第 2 次起按 distance_missing_turn_scale 衰减。
    - 滑动向量以 TURN_CLIP_RECT 中心为基准放置，起止点裁剪到该区域内；
      若裁剪后起止点重合，则放弃本次转向。
    """
    tx, ty = target_center
    cx, cy = SCREEN_CENTER
    offset_x = tx - cx
    offset_y = ty - cy
    offset_norm = math.hypot(offset_x, offset_y)

    min_offset = (SCREEN_WIDTH / 2) * self.TURN_MIN_OFFSET_RATIO
    if offset_norm < min_offset:
        return None, None

    offset_ratio = min(1.0, offset_norm / (SCREEN_WIDTH / 2))
    base_distance = (
        param.min_turn_distance
        + offset_ratio * (param.max_turn_distance - param.min_turn_distance)
    )

    # 第 2 次起衰减
    if turn_attempt > 1:
        base_distance *= param.distance_missing_turn_scale ** (turn_attempt - 1)

    if offset_norm == 0:
        return None, None

    # 滑动方向与目标偏移相反
    ux = -offset_x / offset_norm
    uy = -offset_y / offset_norm

    # 以 TURN_CLIP_RECT 中心为滑动中心，避免以屏幕中心计算后被硬裁剪坍塌
    clip_x, clip_y, clip_w, clip_h = self.TURN_CLIP_RECT
    rect_cx = clip_x + clip_w / 2
    rect_cy = clip_y + clip_h / 2

    half_dist = base_distance / 2
    start_x = rect_cx - ux * half_dist
    start_y = rect_cy - uy * half_dist
    end_x = rect_cx + ux * half_dist
    end_y = rect_cy + uy * half_dist

    # 裁剪到 TURN_CLIP_RECT
    start_x = max(clip_x, min(start_x, clip_x + clip_w))
    start_y = max(clip_y, min(start_y, clip_y + clip_h))
    end_x = max(clip_x, min(end_x, clip_x + clip_w))
    end_y = max(clip_y, min(end_y, clip_y + clip_h))

    # 裁剪后若起止点重合，则本次转向无意义
    if (int(start_x), int(start_y)) == (int(end_x), int(end_y)):
        return None, None

    return (int(start_x), int(start_y)), (int(end_x), int(end_y))
```

### 6.6 运动状态评估调整

```python
def _evaluate_movement_state(self, node_name, selected, param):
    distance = selected.distance
    if distance is not None and distance <= param.arrival_distance:
        self._clear_state(node_name)
        return "arrived"

    state = self._get_state(node_name)
    last_distance = state.last_distance
    last_center = state.last_center
    stuck_count = state.stuck_count

    if self._is_stuck(selected.center, last_center, distance, last_distance, param):
        stuck_count += 1
    else:
        stuck_count = 0

    state.last_distance = distance
    state.last_center = selected.center
    state.stuck_count = stuck_count

    if stuck_count >= param.stuck_threshold:
        return "stuck"
    return "approaching"
```

## 7. `PathFinderAction` 实现细节

### 7.1 参数扩展（`action/param.py`）

```python
@dataclass(frozen=True)
class PathFinderParam:
    # 保留现有字段 ...

    max_move_time: int
    dodge_at_start: str        # "never" / "always" / "once_per_target"
    dodge_direction: str
    enable_turning: bool       # 执行侧保险开关：是否执行 Reco 建议的转向
```

### 7.2 `run` 主流程重构

```python
def run(self, context, argv):
    if context.tasker.stopping:
        return CustomAction.RunResult(success=False)

    param = self._parse_param(argv)
    detail = self._get_recognition_detail(argv)
    if not detail:
        logger.error("Recognition detail is None")
        return CustomAction.RunResult(success=False)

    direction = detail.get("direction")
    if not direction:
        logger.error("Direction is empty in recognition detail")
        return CustomAction.RunResult(success=False)

    target_info = detail.get("target", {}) if isinstance(detail.get("target"), dict) else {}
    distance = target_info.get("distance")

    lock_id = detail.get("lock_id")
    turn_start = detail.get("turn_start")
    turn_end = detail.get("turn_end")

    platform = self._create_platform(context)
    if not platform:
        return CustomAction.RunResult(success=False)

    try:
        success = self._execute_sequence(
            context,
            platform,
            direction,
            param,
            distance,
            lock_id,
            turn_start,
            turn_end,
        )
    finally:
        platform.release_all()

    return CustomAction.RunResult(success=success)
```

### 7.3 执行序列：`dodge → turn → move`

```python
def _execute_sequence(
    self,
    context,
    platform,
    direction,
    param,
    distance,
    lock_id,
    turn_start,
    turn_end,
):
    node_name = self._get_node_name(context)

    # 1. dodge
    if self._should_dodge(node_name, param, lock_id):
        if context.tasker.stopping:
            return False
        platform.dodge(param.dodge_direction)

    # 2. turn（即使 direction == "centered" 也执行，以恢复距离显示）
    if (
        param.enable_turning
        and isinstance(turn_start, (list, tuple))
        and isinstance(turn_end, (list, tuple))
        and len(turn_start) == 2
        and len(turn_end) == 2
    ):
        if context.tasker.stopping:
            return False
        platform.turn(
            int(turn_start[0]), int(turn_start[1]),
            int(turn_end[0]), int(turn_end[1]),
        )

    # 3. move
    if direction == "centered":
        return True

    if context.tasker.stopping:
        return False

    move_duration = self._resolve_duration(distance, param)
    return platform.move(direction, move_duration)


def _should_dodge(self, node_name, param, lock_id):
    if param.dodge_at_start == "never":
        return False
    if param.dodge_at_start == "always":
        return True

    # once_per_target
    if lock_id is None:
        return False

    action_state = self._get_state(node_name)
    if action_state.last_lock_id != lock_id:
        action_state.last_lock_id = lock_id
        return True
    return False
```

### 7.4 最大移动时间限制

```python
def _resolve_duration(self, distance, param):
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
```

### 7.5 节点名获取

```python
def _get_node_name(self, context):
    """
    获取当前 Pipeline 节点名，用于按节点隔离 Action 状态。
    优先从 context.run_task_detail 或 context.tasker 获取；无法获取时返回默认值。
    """
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
```

## 8. JSON Schema 更新

### 8.1 `PathFindingReco.schema.json`

新增字段：

| 字段                           | Schema 类型 | 默认值 | 说明                                   |
| ------------------------------ | ----------- | ------ | -------------------------------------- |
| `target_lock_timeout`          | integer     | 3      | 目标连续丢失多少帧后解锁               |
| `target_lock_center_tolerance` | integer     | 100    | 锁定目标跨帧匹配中心容差（像素）       |
| `target_lock_score_threshold`  | number      | 0.75   | 锁定目标跨帧匹配最低置信度             |
| `enable_turning`               | boolean     | true   | 是否计算并返回转向坐标                 |
| `max_turn_attempts`            | integer     | 3      | 连续转向后仍未识别到距离，则停止转向   |
| `min_turn_distance`            | integer     | 50     | 最短转向滑动距离（像素）               |
| `max_turn_distance`            | integer     | 400    | 最长转向滑动距离（像素）               |
| `distance_missing_turn_scale`  | number      | 0.5    | 连续未识别到距离时转向滑动距离衰减系数 |

### 8.2 `PathFinderAction.schema.json`

新增字段：

| 字段              | Schema 类型 | 默认值    | 说明                                       |
| ----------------- | ----------- | --------- | ------------------------------------------ |
| `max_move_time`   | integer     | 0         | 单次移动最大时长（毫秒），0 不限制         |
| `dodge_at_start`  | string enum | "never"   | `never` / `always` / `once_per_target`     |
| `dodge_direction` | string      | "backward" | 闪避方向（默认 backward：游戏单点闪避默认向后翻滚）|
| `enable_turning`  | boolean     | true      | 是否执行 Reco 建议的转向（执行侧保险开关） |
| `turn_duration_ms`| integer     | 200       | 显式控制 `platform.turn` 滑动持续时间（毫秒），0 用 platform 默认 |
| `dodge_release_ms` | integer | 50     | dodge 与 move 之间的短暂释放时长（毫秒），0 表示不释放     |
| `move_start_delay_ms` | integer | 300 | move 开始前的额外等待时长（毫秒），给游戏反应时间         |

## 9. 状态流转

### 9.1 锁定状态机

```
无锁定 ──识别到目标──► 锁定目标（生成 lock_id，重置历史状态）
  ▲                      │
  │                      │ 每帧追踪成功
  │                      ▼
  │                  保持锁定
  │                      │ 追踪失败但 miss_count < timeout：返回历史位置
  │                      │ 追踪失败且 miss_count >= timeout：解锁
  └──────────────────────┘
```

### 9.2 转向计数器

```
distance 识别成功 ──► turn_attempts_without_distance = 0
       │
       ▼ distance 缺失
   计数 +1
       │
       ├── <= max_turn_attempts ──► 计算并返回 turn_start/turn_end
       │
       └── > max_turn_attempts  ──► 不返回 turn 坐标，直接移动
```

## 10. 边界条件与错误处理

| 场景                                                    | 处理策略                                                         |
| ------------------------------------------------------- | ---------------------------------------------------------------- |
| `context.tasker.stopping`                               | 立即返回 `success=False`，不执行任何动作。                       |
| 识别结果缺少 `direction`                                | Action 记录错误并返回失败。                                      |
| `direction == "centered"`                               | 跳过 `move`，但仍执行 `dodge/turn`。                             |
| `turn_start` / `turn_end` 格式异常                      | 跳过转向，继续执行后续动作。                                     |
| `lock_id` 为 None 但 `dodge_at_start="once_per_target"` | 不触发 dodge。                                                   |
| `max_move_time` 为 0                                    | 不限制移动时长。                                                 |
| `far <= near`                                           | 回退到 `move_duration`，与现有逻辑一致。                         |
| 目标锁定后候选为空                                      | `miss_count++`，未超时返回历史位置；超时解锁并返回 `hit=False`。 |
| 两个同名目标相邻                                        | 通过 IOU + 中心容差 + 置信度三重校验，降低跟踪错误概率。         |
| Reco `enable_turning=false`                             | 不计算、不返回 `turn_start/turn_end`。                           |
| Action `enable_turning=false`                           | 即使 Reco 返回坐标也不执行 `turn`。                              |

## 11. 测试策略

### 11.1 单元测试重点

- `LockedTarget.iou()` 计算正确性。
- `_track_locked_target()` 在多个同名候选中选择 IOU 最大者。
- `_compute_turn_coordinates()` 方向正确、衰减正确、裁剪正确。
- `_should_dodge()` 在 `once_per_target` 模式下按 `lock_id` 变化触发。
- `_resolve_duration()` 在 `max_move_time` 限制下正确截断。
- 目标追踪单帧丢失时返回历史位置，`miss_count` 递增。

### 11.2 集成测试重点

- 旧配置不加任何新参数，单目标寻路行为与当前版本一致。
- 多目标场景下，选中目标后 5 帧内 `lock_id` 不跳动。
- 锁定后选择器优先级失效：即使出现更高优先级模板也不切换。
- `enable_turning=false` 时即使距离缺失也不调用 `turn`。
- 连续无距离时第 2 次滑动距离低于第 1 次。
- `max_turn_attempts=3` 时，第 4 帧起直接移动。
- `dodge_at_start="always"` 每次移动前都 dodge；`"once_per_target"` 同一 `lock_id` 只 dodge 一次。
- `max_move_time=1000` 时远距离单次移动不超过 1 秒。
- 目标 centered 但距离缺失时，仍执行 turn 但不执行 move。
- 单帧目标丢失时，动作不中断（使用历史位置）。

### 11.3 回归测试重点

- `hit_node="if"/"else"` 语义不变。
- `state="arrived"` 时清空锁定。
- `state="stuck"` 时保持锁定。
- 720p 坐标基准不变。
- **死区内不转向**（方向 `centered` 时 `_calculate_turn` 返回 `(None, None, None)`；Action 同样跳过 `platform.turn`）。

---

## 12. 重构变更（借鉴 MaaEnd MapTracker）

> 参考实现：[MaaEnd MapTracker](https://github.com/MaaEnd/MaaEnd)（PR #986 转向优化、PR #2836 InferState 重构）。

### 12.1 借鉴点对照

| MaaEnd MapTracker 设计 | 当前 AutoPathFinding 实现 |
|---|---|
| `rotation_lower_threshold` 微调阈值 | `PathFindingReco.rotation_lower_threshold: float = 8.0` |
| `rotation_upper_threshold` 大幅调整阈值（先停后转） | `PathFindingReco.rotation_upper_threshold: float = 60.0` + `PathFinderAction.large_turn_release_ms: int = 100` |
| `rotationSpeed` 自适应（EMA 0.618/0.382） | `RotationAdjuster` 数据类（`DECAY=0.618, INNOVATION=0.382`），通过 `rotation_adaptive_enabled` 开关 |
| `MovementWalk/Run/Sprint` 状态机 | `Phase` 枚举（`seeking/tracking/approaching/turning/stuck/lost`），借鉴 `InferState` 设计 |
| `stuckThreshold` + `stuckTimeout` 卡住检测 | `StuckDetector` 数据类 + `stuck_timeout_ms: int = 10000`，借鉴 `StuckDetector` 设计 |
| 转向分级 `rotation is acceptable but can be improved` | `TurnGrade` 枚举（`FINE_TUNE` / `LARGE_TURN`），通过 `classify_turn()` 双阈值判定 |
| 死区内不转向（避免中心抖动推到屏幕外） | Reco `_calculate_turn(direction="centered")` 返回 `(None, None, None)`；Action `_execute_sequence` 在 `direction == "centered"` 时跳过 `platform.turn`，**双重防护**。 |
| **滑动方向语义修复** | `compute_turn_coordinates` 中 `swipe = end - start` 必须 = `-offset`（把目标推向屏幕中心）。原版误以为起点要"指向目标方向"，导致 swipe 向量与目标偏移同向，目标被推离屏幕中心。修复后通过 `logger.debug` 输出 swipe vec / expected vec 对照日志，便于现场排查。 |
| **centered 死区内兜底前进** | Action 在 `direction == "centered"` 且 `distance is None`（OCR 失败）时不能再 return True，否则角色原地不动。改为 centered + distance 有效 → 视为到达（跳过 move）；centered + distance 缺失 → 兜底前进 `move_duration` 时长。 |
| **滑动距离动态自适应** | `compute_turn_distance` 改为 `offset_norm * turn_speed_factor`（默认 0.7），保证滑动后目标朝屏幕中心移动而不越过中心到反方向。原版在 [min=50, max=400] 区间线性插值，大偏移下滑动距离过大（200-400px），相当于旋转视角 60°-90°。新增 `turn_speed_factor` 参数（默认 0.7），`max_turn_distance` 仍作为上限保护。 |

### 12.2 状态机迁移

| 旧字段 | 新字段 |
|---|---|
| `PathFindingState.stuck_count` (裸字段) | `PathFindingState.stuck_detector: StuckDetector`（独立组件） |
| `PathFindingState.turn_attempts_without_distance` | 保留，并新增 `PathFindingState.turn_grade: TurnGrade` |
| 无阶段字段 | `PathFindingState.phase: Phase` |
| 无自适应速度 | `PathFindingState.rotation_adjuster: RotationAdjuster` |
| 无 fast_path 计数 | `PathFindingState.fast_path_miss_count: int`（连续 miss 触发回退全 ROI） |
| 无统一重置入口 | `PathFindingState.reset_target_local_state()`（新目标时一次性重置 detector/adjuster/history/phase） |
| `LockedTarget.last_direction` | 保留，并新增 `LockedTarget.last_target_angle: float \| None` |
| 无连续距离缺失计数 | `LockedTarget.distance_miss_count: int`（选中新目标时优先选择距离稳定的目标） |
| `PathFinderState.last_lock_id` | 保留，并新增 `stuck_started_at_ms`（卡住超时兜底） |

### 12.3 模块拆分

| 文件 | 变更 |
|---|---|
| `state/path_finding_state.py` | 新增 `Phase`/`TurnGrade`/`RotationAdjuster`/`StuckDetector` 类 |
| `recognition/turn_strategy.py` | **新增**独立转向算法模块（`compute_target_angle`/`bin_direction`/`dir_center_angle`/`angle_diff`/`classify_turn`/`compute_turn_distance`/`compute_turn_clip_rect`/`compute_turn_coordinates`） |
| `recognition/path_finding_reco.py` | 拆分出转向计算到 `turn_strategy.py`；集成阶段状态机与卡住检测；clip rect 改为水平/垂直两套按主轴选择 |
| `recognition/param.py` | 新增 `rotation_lower_threshold`/`rotation_upper_threshold`/`rotation_adaptive_enabled`/`stuck_timeout_ms`/`direction_hysteresis`/`turn_min_floor_ratio`/`enable_locked_fast_path`/`lock_roi_padding`/`fast_path_max_turn_padding`/`fast_path_miss_limit` |
| `action/param.py` | 新增 `large_turn_release_ms`/`stuck_dodge_on_phase` |
| `action/path_finder_action.py` | LARGE_TURN 时先释放按键；phase=STUCK 时触发 dodge |

### 12.4 新增返回值字段

`PathFindingReco.detail` 新增两个字段：

- `phase: str` — 阶段状态机（`seeking` / `tracking` / `approaching` / `turning` / `stuck` / `lost`）
- `turn_grade: str | None` — 转向分级（`fine_tune` / `large_turn` / `None`）

### 12.5 兼容性

- 所有旧参数保持向后兼容（默认值与旧版一致）。
- 旧 Pipeline JSON 配置无需修改即可运行。
- 旧测试在签名变更处已做最小适配（`_calculate_turn` 返回 3 元组；`_evaluate_movement_state` 增加 `state` 形参）。

### 12.6 Fast-Path ROI 优化

借鉴 MaaEnd MapTracker 锁定态下小 ROI 扫描的做法，平衡识别耗时与连续性：

- `enable_locked_fast_path=true` 时，锁定态下根据上一帧锁定目标的 `bbox` 计算小 ROI（`lock_roi_padding`）。
- 距离缺失时（可能正在转向），额外叠加 `fast_path_max_turn_padding`，避免目标在滑动后移出 ROI。
- 连续 `fast_path_miss_limit` 帧小 ROI 内未识别到目标时，强制回退一次全 ROI 扫描，防止持续小 ROI 漏识别。
- 回退全 ROI 仍失败时，按既有 `target_lock_timeout` 逻辑递增 `miss_count`，超时后解锁。
