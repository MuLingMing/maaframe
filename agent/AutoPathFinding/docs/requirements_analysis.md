# AutoPathFinding 需求分析报告

## 1. 需求复述

用户希望对 `agent/AutoPathFinding` 模块在现有基础上继续完善，必要时可进行整体重构，以实现以下三类效果：

1. **移动连续性**
   - 新增**转向动作**：当已选定的目标未识别到距离时，通过坐标滑动调整视角，使距离文本可见；支持上下左右任意方向。
   - 已选定目标后，不再考虑其他候选目标是否缺失距离；只关注已选定目标本身。
   - 第一次因距离缺失触发转向时全额滑动；若连续多次仍未识别到距离，则按衰减系数自动降低滑动距离。
   - 新增**闪避动作**：在移动开始时执行闪避，用于触发疾跑状态，从而支持更长时间的连续移动。
   - 新增**最大移动时间**参数：限制单次移动的最长持续时间，避免单帧移动过久。
2. **目标选择连贯性**
   - `PathFindingReco` 返回选中的目标后，后续识别/动作周期应以该目标为唯一移动目标，不再因更高优先级模板出现或距离变化而重新选择，直到目标到达、丢失或显式释放。
3. **保留并扩展现有参数**
   - 保留现有的 `move_duration` / `move_duration_far` / `move_duration_near` / `distance_far` / `distance_near` 参数体系。
   - 新增“移动开始时是否闪避”的参数选择。

## 2. 现有实现概述

当前 `AutoPathFinding` 模块由两部分组成：

- **`PathFindingReco`（Custom Recognition）**
  - 使用 `TemplateMatch` 识别目标图标，支持 `priority` / `nearest` / `composite` 三种选择器。
  - 使用 `OCR` 提取目标下方距离文本。
  - 基于角度分箱 + 圆形死区计算方向（`forward` / `backward` / `left` / `right` / `centered`）。
  - 通过类级 `_path_state` 按节点名跨帧跟踪运动状态（`approaching` / `arrived` / `stuck`）。
  - 返回 `detail` 给 `PathFinderAction`，包含 `target`、`direction`、`state`。
- **`PathFinderAction`（Custom Action）**
  - 从 Reco 返回的 `detail` 中读取 `direction` 和 `target.distance`。
  - 复用 `OperationRecording` 的 `PlatformFactory` 创建平台实例。
  - 调用 `platform.move(direction, duration)` 执行一次移动，最后通过 `release_all()` 释放按键。
  - 移动时长根据距离线性插值计算。

当前架构遵循“识别 → 操作 → 识别”的 Pipeline 循环，每次 Action 都是一次性的移动 + 释放，目标选择也在每次 Reco 时重新进行。

## 3. 差距分析

| 用户目标 | 当前能力 | 差距 |
| --- | --- | --- |
| 移动连续性 | 每次 Action 仅移动一次并释放 | 缺少转向、闪避、最大移动时长控制 |
| 目标选择连贯性 | 每帧 Reco 都重新选择目标 | 缺少目标锁定机制，可能中途切换目标 |
| 参数扩展 | 只有距离相关时长参数 | 缺少闪避、转向、最大移动时间等参数 |

## 4. 可行方案

### 方案 A：最小化补丁（在现有结构上直接叠加）

**做法**

- 在 `PathFindingReco` 中新增类级 `_target_lock` 字典，按 `node_name` 锁定已选目标。
- 在 `PathFinderAction` 中直接按 `direction` 判断是否先调用 `platform.turn`，再调用 `platform.dodge`，最后 `platform.move`。
- 新增参数直接扩展两个 `Param` 数据类。

**优点**

- 改动范围小，实现快。
- 对现有 Pipeline 配置影响最小。

**缺点**

- 转向是开环执行，执行后无法在同一帧内验证目标是否回到屏幕中心。
- 状态分散在 Reco/Action 中，长期维护成本高。
- `once_per_target` 闪避需要 Action 维护额外状态，容易与 Reco 不同步。

**适用场景**

- 快速验证效果，或暂时不追求复杂状态管理。

### 方案 B：引入统一状态对象（推荐）

**做法**

- 新增 `PathFindingState` 类，按 `node_name` 集中管理：
  - 目标锁定信息（`LockedTarget`）
  - 历史运动状态（用于 stuck/arrived）
  - 上一次转向/闪避记录
- `PathFindingReco` 负责：
  - 目标锁定/追踪/解锁
  - 转向建议（在 `detail` 中返回 `turn_start` / `turn_end` 坐标）
- `PathFinderAction` 负责：
  - 根据 `detail` 执行 `dodge` → `turn` → `move` 序列
  - 用 `max_move_time` 限制移动时长

**优点**

- 状态集中，职责清晰，便于后续扩展（例如加入障碍物检测、路径修正）。
- Reco 仍专注于“看屏幕并返回决策”，Action 仍专注于“执行操作”。
- 更容易实现 `once_per_target` 闪避（通过 `lock_id` 比较）。

**缺点**

- 需要新增文件/类，改动量中等。
- 需要仔细处理状态生命周期（到达/丢失/节点切换）。

**适用场景**

- 作为长期维护的基础架构，推荐采用。

### 方案 C：Action 内部闭环循环

**做法**

- `PathFinderAction` 内部循环截图 → 识别 → 转向/移动，直到到达、`max_move_time` 耗尽或卡住。
- 将 Reco 的部分逻辑下沉到 Action 中，或直接在 Action 中重复调用识别。

**优点**

- 连续性最强，可以在一次 Action 内多次修正方向。

**缺点**

- 破坏 Reco/Action 职责分离，Action 变得臃肿。
- 与 Pipeline 的“识别 → 操作 → 识别”模型冲突，难以利用 Pipeline 分支处理 `arrived` / `stuck`。
- 调试和测试困难。

**适用场景**

- 不推荐，除非现有 Pipeline 模型无法满足极端连续性需求。

## 5. 推荐方案

**采用方案 B：引入统一状态对象，同时保持 Reco/Action 职责分离。**

理由：

1. 满足移动连续性（转向 + 闪避 + 最大移动时间）而不破坏架构。
2. 通过 `PathFindingState` 和 `LockedTarget` 自然实现目标选择连贯性。
3. 状态集中后，`once_per_target` 闪避、目标解锁、到达判定等逻辑更容易维护。
4. 向后兼容：新参数均有安全默认值，旧配置不修改即可运行。

## 6. 详细设计

### 6.1 新增数据类型与文件位置

推荐新增文件：`agent/AutoPathFinding/state/path_finding_state.py`

```python
from dataclasses import dataclass
import secrets

@dataclass
class LockedTarget:
    """被锁定的目标，用于跨帧保持选择连贯性"""
    template: str
    center: tuple[float, float]
    bbox: tuple[int, int, int, int]
    score: float
    distance: int | None
    lock_id: str
    miss_count: int = 0

    @staticmethod
    def generate_lock_id(template: str, center: tuple[float, float]) -> str:
        """生成带随机后缀的 lock_id，避免同名邻近目标冲突"""
        suffix = secrets.token_hex(4)
        return f"{template}#{int(center[0])}#{int(center[1])}#{suffix}"

@dataclass
class PathFindingState:
    """按节点隔离的寻路状态（Reco 侧）"""
    locked_target: LockedTarget | None = None
    last_distance: int | None = None
    last_center: tuple[float, float] | None = None
    stuck_count: int = 0
    turn_attempts_without_distance: int = 0

@dataclass
class PathFinderState:
    """按节点隔离的执行状态（Action 侧）"""
    last_lock_id: str | None = None
```

- `PathFindingReco` 使用类级 `_state: ClassVar[dict[str, PathFindingState]]` 按 `node_name` 存储。
- `PathFinderAction` 使用类级 `_action_state: ClassVar[dict[str, PathFinderState]]` 按 `node_name` 存储，用于判断 `once_per_target` 闪避。

### 6.2 目标锁定与解锁逻辑（PathFindingReco）

**锁定流程**

1. 检查当前节点是否已有 `locked_target`。
2. 若有，优先在上一帧 `bbox` 扩展 `target_lock_center_tolerance` 的区域内，用相同模板重新匹配。
   - 若同一区域存在多个同名匹配，选择**与上一帧 bbox IOU 最大**的结果。
   - 匹配成功且 `score >= target_lock_score_threshold`、IOU >= `DEFAULT_TARGET_LOCK_IOU_THRESHOLD`：更新锁定目标的 `center` / `bbox` / `distance`，`miss_count = 0`。
   - 匹配失败：`miss_count += 1`；当 `miss_count >= target_lock_timeout` 时清空锁定。
3. 若无锁定目标，执行现有选择流程；选择成功后创建 `LockedTarget` 并锁定，`lock_id` 在锁定时刻生成并保持不变。

**锁定后的选择器行为**

- 目标锁定后，原 `selector_type` / `selector_priority` 暂时失效，Reco 以追踪锁定目标为主。
- 解锁后重新进入完整选择流程。

**解锁条件**

- `state == "arrived"`：到达目标，清空锁定。
- 目标连续丢失达到 `target_lock_timeout` 帧。
- 当前帧完全未识别到任何目标（`targets` 为空）。
- 外部 Pipeline 显式切换到其他节点时无需处理，状态按 `node_name` 自然隔离。

**stuck 状态下的锁定**

- 状态为 `stuck` 时**不解锁**，仍保持当前目标锁定。
- 由外部 Pipeline 根据 `state == "stuck"` 执行脱卡逻辑，脱卡完成后继续寻路。

**返回值扩展**

在现有 `detail` 中增加：

```json
{
  "target": { ... },
  "direction": "forward",
  "state": "approaching",
  "lock_id": "quest_icon.png#1150#420#a3f7",
  "turn_start": [640, 360],
  "turn_end": [840, 360]
}
```

- `lock_id`：用于 Action 判断是否是新目标，锁定期间保持不变。
- `turn_start` / `turn_end`：可选，存在时表示 Reco 建议的转向滑动起点/终点（720p 坐标）。
  - 不存在或为空时，Action 不执行转向。
  - 转向方向由坐标差决定：向右滑动表示摄像机右转，目标左移回中心；向下滑动表示摄像机下转，目标上移回中心。

### 6.3 转向动作（PathFinderAction）

**参考实现**

转向参考 `OperationRecording` 的 `TurnAction`：直接通过 `platform.turn(start_x, start_y, end_x, end_y, duration=None)` 执行坐标滑动，由平台默认配置控制滑动速度。

**触发条件（已澄清）**

- `enable_turning == true`。
- **已选定目标**（`PathFindingReco` 返回的 `target`）的 `distance is None`。
- 已选定目标后，不再考虑其他候选目标是否缺失距离。
- 当 `turn_attempts_without_distance >= max_turn_attempts` 时，即使距离仍缺失，也停止转向直接移动。

**`max_turn_attempts` 计数语义**

```python
# 每一帧触发转向时：
state.turn_attempts_without_distance += 1
if state.turn_attempts_without_distance > max_turn_attempts:
    # 不再返回 turn_start / turn_end，直接移动
    turn_start = turn_end = None
else:
    # 计算并返回 turn_start / turn_end
    ...

# 当已选定目标的 distance 被成功识别时：
state.turn_attempts_without_distance = 0
```

- 第 1 次触发转向时，`turn_attempts_without_distance == 1`，执行转向。
- 第 `max_turn_attempts` 次触发时，仍然执行转向。
- 第 `max_turn_attempts + 1` 次触发时，停止转向，直接移动。
- 一旦已选定目标的距离重新出现，计数立即重置为 0。

**自动计算转向坐标**

```python
import math

offset_x = target_center_x - SCREEN_CENTER_X
offset_y = target_center_y - SCREEN_CENTER_Y
offset_norm = math.hypot(offset_x, offset_y)

# 即使目标接近屏幕中心也继续滑动；微小抖动可通过 TURN_MIN_OFFSET_RATIO 常量抑制
if offset_norm >= (SCREEN_WIDTH / 2) * TURN_MIN_OFFSET_RATIO:
    offset_ratio = min(1.0, offset_norm / (SCREEN_WIDTH / 2))
    base_distance = min_turn_distance + offset_ratio * (max_turn_distance - min_turn_distance)

    # 第一次因距离缺失触发转向时全额滑动；后续若仍未识别到距离，逐次衰减
    if distance is None and state.turn_attempts_without_distance > 1:
        base_distance *= distance_missing_turn_scale ** (state.turn_attempts_without_distance - 1)

    # 滑动方向与目标偏移相反：目标偏左则向右滑，目标偏上则向下滑
    dx = -offset_x / offset_norm * base_distance
    dy = -offset_y / offset_norm * base_distance
else:
    dx = dy = 0

# 以屏幕中心为基准，向两侧延伸，再裁剪到 TURN_CLIP_RECT 区域
turn_start = (SCREEN_CENTER_X - dx / 2, SCREEN_CENTER_Y - dy / 2)
turn_end = (SCREEN_CENTER_X + dx / 2, SCREEN_CENTER_Y + dy / 2)
```

- `turn_start` / `turn_end` 最终需裁剪到 `TURN_CLIP_RECT`（格式 `[x, y, w, h]`）。
- 滑动向量同时包含 x、y 分量，因此支持左上、右下等任意方向。

> 说明：转向是开环动作，执行后无法立即获得新的截图反馈；真正的闭环修正由下一次 Pipeline 循环完成。

### 6.4 闪避动作（PathFinderAction）

**参数**

- `dodge_at_start`：
  - `"never"`：不闪避。
  - `"always"`：每次 Action 开始时都闪避。
  - `"once_per_target"`：仅在锁定目标发生变化（`lock_id` 改变）时闪避一次。
- `dodge_direction`：闪避方向，默认 `"forward"`。

**执行状态隔离**

```python
class PathFinderAction:
    _action_state: ClassVar[dict[str, PathFinderState]] = {}
```

- 每次执行时比较当前 `detail.lock_id` 与 `_action_state[node_name].last_lock_id`。
- `once_per_target` 模式下，若 `lock_id` 变化则触发 dodge，并更新 `last_lock_id`。

**执行序列**

```
if dodge_at_start 满足条件:
    platform.dodge(dodge_direction)
if enable_turning 且 turn_start / turn_end 存在:
    platform.turn(turn_start[0], turn_start[1], turn_end[0], turn_end[1])
platform.move(direction, actual_duration)
```

### 6.5 最大移动时间

在 `_resolve_duration` 中：

```python
calculated = ...  # 现有线性插值结果
if param.max_move_time > 0:
    return min(calculated, param.max_move_time / 1000.0)
return calculated
```

## 7. 参数设计

### 7.1 PathFinderAction 参数

| 参数 | 类型 | 默认值 | 说明 |
| --- | --- | --- | --- |
| `move_duration` | `int` | `500` | 无距离时的默认移动时长（毫秒） |
| `move_duration_far` | `int` | `1500` | 远距离移动时长（毫秒） |
| `move_duration_near` | `int` | `300` | 近距离移动时长（毫秒） |
| `distance_far` | `int` | `200` | 远距离阈值（像素） |
| `distance_near` | `int` | `50` | 近距离阈值（像素） |
| `max_move_time` | `int` | `0` | 单次移动最大时长（毫秒），`0` 不限制 |
| `dodge_at_start` | `str` | `"never"` | `never` / `always` / `once_per_target` |
| `dodge_direction` | `str` | `"forward"` | 闪避方向 |
| `enable_turning` | `bool` | `true` | 转向总开关；`false` 时不执行转向 |
| `distance_missing_turn_scale` | `float` | `0.5` | 连续未识别到距离时，转向滑动距离的衰减系数 |
| `min_turn_distance` | `int` | `50` | 最短转向滑动距离（像素） |
| `max_turn_distance` | `int` | `400` | 最长转向滑动距离（像素） |

### 7.2 PathFindingReco 参数

| 参数 | 类型 | 默认值 | 说明 |
| --- | --- | --- | --- |
| `target_lock_timeout` | `int` | `3` | 目标连续丢失多少帧后解锁 |
| `target_lock_center_tolerance` | `int` | `100` | 锁定目标跨帧匹配的中心容差（像素） |
| `target_lock_score_threshold` | `float` | `0.75` | 锁定目标跨帧匹配的最低置信度 |
| `max_turn_attempts` | `int` | `3` | 连续转向后仍未识别到距离，则停止转向直接移动 |

### 7.3 类级常量（可在源码中调整，不作为 Custom 参数暴露）

| 常量 | 类型 | 默认值 | 说明 |
| --- | --- | --- | --- |
| `DEFAULT_TARGET_LOCK_IOU_THRESHOLD` | `float` | `0.3` | 锁定目标跨帧匹配的最低 IOU |
| `TURN_MIN_OFFSET_RATIO` | `float` | `0.0` | 触发转向的最小中心偏移比例；`0.0` 表示即使目标接近中心也尝试滑动 |
| `TURN_CLIP_RECT` | `list[int]` | `[800, 50, 300, 200]` | 转向滑动起止点的裁剪区域 `[x, y, w, h]`（720p 基准） |

> 注：`turn_screen_edge_ratio` 与 `ensure_distance_visible` 已移除；转向统一由 `enable_turning` 控制。

## 8. 向后兼容

- 所有新增参数均有默认值，未配置时行为与当前版本一致。
- `PathFinderAction` 的 `direction` 计算逻辑不变；`centered` 时仍然不执行移动。
- `PathFindingReco` 返回的 `hit_node` / `state` 语义不变，现有 `IfElseAction` 分支无需修改。
- `selector/` 下的类仍作为内部工具，不注册到 `custom.json`。

## 9. 风险评估与应对措施

| 风险 | 概率 | 影响 | 应对措施 |
| --- | --- | --- | --- |
| 转向动作开环执行，可能转多或转少 | 中 | 中 | 通过 `min_turn_distance` / `max_turn_distance` 限制滑动距离；`distance_missing_turn_scale` 逐次衰减；`max_turn_attempts` 防止无限转向 |
| 闪避不一定能触发所有游戏的疾跑 | 中 | 中 | `dodge_at_start` 默认 `never`，用户按需开启；方向可配置 |
| 目标锁可能跟踪到错误同名目标 | 低 | 中 | 使用中心容差 + IOU + 置信度三重校验；丢失后重新选择 |
| 锁定目标长期无法到达导致 stuck | 中 | 低 | 现有 stuck 机制继续生效，到达/卡住后清空锁定 |
| 状态生命周期管理不当导致内存增长 | 低 | 中 | 到达/丢失/无目标时清空对应 `node_name` 状态；不同节点自然隔离 |
| 新增参数导致 Schema 不同步 | 中 | 低 | 实现后同步更新 `deps/tools/custom_act/PathFinderAction.json` 与 `deps/tools/custom_reco/PathFindingReco.json` |

## 10. 实施建议

### 10.1 推荐文件拆分

```
agent/AutoPathFinding/
├── state/
│   ├── __init__.py
│   └── path_finding_state.py      # PathFindingState / LockedTarget / PathFinderState
├── action/
│   ├── param.py                   # PathFinderParam（扩展新参数）
│   └── path_finder_action.py      # 增加 dodge / turn / max_move_time
├── recognition/
│   ├── param.py                   # PathFindingParam（扩展锁定/转向参数）
│   └── path_finding_reco.py       # 增加目标锁定、转向坐标计算
└── docs/
    ├── requirements_checklist.md
    └── requirements_analysis.md
```

### 10.2 实施顺序

1. **重构状态管理**：新增 `state/path_finding_state.py`，将现有 `_path_state` 迁移到 `PathFindingState`。
2. **实现目标锁定**：在 `PathFindingReco` 中加入锁定/追踪/解锁逻辑（含 IOU 校验），确保多目标场景下目标不跳动。
3. **实现转向坐标计算**：根据目标偏移计算 `turn_start` / `turn_end`，支持上下左右任意方向。
4. **扩展 Action**：加入 `dodge` / `turn` / `max_move_time`，保持 `dodge → turn → move` 执行序列清晰。
5. **同步文档与 Schema**：
   - 更新 `README.md`。
   - 更新 `deps/tools/custom_act/PathFinderAction.json`。
   - 更新 `deps/tools/custom_reco/PathFindingReco.json`。
6. **测试闭环**：
   - 旧配置回归测试。
   - 多目标锁定测试（含同名目标邻近场景）。
   - 转向/闪避/最大移动时间功能测试。
   - 到达/丢失/卡住状态测试。

## 11. 已澄清问题

| 问题 | 用户决策 |
| --- | --- |
| 转向触发条件 | `enable_turning=true` 且**已选定目标**自身距离缺失时触发；已选定后不再考虑其他候选目标 |
| 转向实现方式 | 参考 `OperationRecording` 的 `TurnAction`，使用坐标滑动；支持上下左右 |
| `max_turn_attempts` 计数 | 第 1 次触发计数为 1并执行；第 `max_turn_attempts` 次仍执行；第 `max_turn_attempts + 1` 次停止转向 |
| `max_turn_attempts` 重置条件 | 当不再满足转向触发条件时重置，即已选定目标的距离被成功识别时 |
| 连续转向无距离 | 第一次全额滑动，后续按 `distance_missing_turn_scale` 逐次衰减 |
| 目标接近中心但距离缺失 | 继续滑动；最小偏移阈值 `TURN_MIN_OFFSET_RATIO` 作为类常量，默认 `0.0` |
| 锁定匹配 IOU | `DEFAULT_TARGET_LOCK_IOU_THRESHOLD` 作为类常量，默认 `0.3` |
| 转向起止点裁剪 | 使用 `TURN_CLIP_RECT` 类常量，默认 `[800, 50, 300, 200]` |
| 转向开关 | 仅保留 `enable_turning` 作为总开关；`ensure_distance_visible` 已移除 |
| 避障 | 作为后续插件处理，本次不实现 |
| 闪避模式 | `never` / `always` / `once_per_target` |
| `max_move_time` 默认值 | `0` 表示不限制 |
| `lock_id` | 加入随机后缀，避免同名邻近目标冲突 |
