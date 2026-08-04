# AutoPathFinding 需求清单

## 1. 需求复述

用户希望对 `agent/AutoPathFinding` 模块在现有基础上进行完善（可考虑整体重构），核心目标有三点：

1. **移动连续性**：单次/连续寻路过程中，动作应尽可能连贯。
   - 增加**转向动作**：参考 `OperationRecording` 的 `TurnAction`，通过坐标滑动调整视角；仅当 `enable_turning=true` 且**已选定目标**自身距离缺失时触发；支持上下左右任意方向。
   - 已选定目标后，不再考虑其他候选目标是否缺失距离。
   - 第一次因距离缺失触发转向时全额滑动；若连续多次仍未识别到距离，则按 `distance_missing_turn_scale` 自动降低滑动距离。
   - 新增 `max_turn_attempts` 参数：连续 N 帧转向后仍未识别到距离，则停止转向直接移动。
   - 增加**闪避动作**，用于在移动开始时触发疾跑状态，从而支持长时间连续移动。
   - 增加**最大移动时间**参数，限制单次移动的最长持续时间。
2. **目标选择连贯性**：`PathFindingReco` 选定目标后，后续识别/动作周期应继续以该目标为移动目标，直到目标丢失、到达或显式释放，避免在移动过程中频繁切换目标。
3. **保留并扩展参数**：保留现有的移动持续时间参数体系，同时新增“移动开始时是否闪避”的参数选择。

## 2. 功能需求清单

| 编号 | 需求 | 优先级 | 验收标准 |
| --- | --- | --- | --- |
| FR-001 | 保留现有 `move_duration` / `move_duration_far` / `move_duration_near` / `distance_far` / `distance_near` 参数及线性插值逻辑 | 高 | 现有 Pipeline JSON 配置无需修改即可继续运行 |
| FR-002 | 新增 `max_move_time` 参数，限制单次 `PathFinderAction` 移动的最长持续时间 | 高 | 计算时长大于 max 时取 max；为 0/不配置时不限制 |
| FR-003 | 新增转向动作：参考 `OperationRecording.TurnAction`，使用坐标滑动；`enable_turning=true` 且已选定目标距离缺失时触发 | 高 | 触发条件唯一且明确；支持上下左右；返回 `turn_start` / `turn_end` |
| FR-004 | `enable_turning=false` 时不执行任何转向 | 高 | `false` 时即使距离缺失，`PathFinderAction` 也不调用 `turn` |
| FR-005 | 已选定目标后，不再因其他候选目标缺失距离而触发转向 | 高 | `nearest` / `composite` 模式下，仅检查已选定目标的距离 |
| FR-006 | 连续未识别到距离时，按 `distance_missing_turn_scale` 逐次降低滑动距离 | 中 | 第 2 次起滑动距离低于第 1 次 |
| FR-007 | 新增 `max_turn_attempts` 参数：连续 N 帧转向后仍未识别到距离，则停止转向直接移动 | 中 | 第 `max_turn_attempts + 1` 次起直接移动；已选定目标距离被识别到时重置计数 |
| FR-008 | 新增闪避动作：移动序列开始时可选执行一次 `platform.dodge` | 高 | 参数开启时，`PathFinderAction` 先 dodge 再 move |
| FR-009 | 新增 `dodge_at_start` 参数，支持 `never` / `always` / `once_per_target` | 高 | 默认 `never`，向后兼容；`once_per_target` 在同一 `lock_id` 生命周期内只触发一次 |
| FR-010 | 目标锁定：`PathFindingReco` 选定目标后，跨帧保持同一目标 | 高 | 同节点后续识别优先匹配已锁定目标，丢失超过阈值才重新选择；锁定后原选择器优先级暂时失效 |
| FR-011 | 目标锁解锁条件：到达、目标连续丢失 N 帧、模板不再匹配 | 高 | 解锁后下一帧可重新选择新目标；`stuck` 状态不解锁 |
| FR-012 | 锁定匹配使用 IOU 辅助校验，防止同名邻近目标跟踪错 | 中 | 重检测时优先选择 IOU 最高的候选；IOU 低于类常量阈值时视为丢失 |
| FR-013 | `lock_id` 包含随机后缀，避免同名邻近目标 ID 冲突 | 中 | 同一目标锁定期间 `lock_id` 保持不变 |
| FR-014 | 锁定状态按节点隔离，避免不同寻路节点互相污染 | 中 | Reco 与 Action 均使用 `node_name` 作为状态键 |
| FR-015 | 状态/参数兼容 `IfElseAction`：返回 `hit_node` / `state` 语义不变 | 高 | 现有基于 `state` 的分支 Pipeline 继续可用 |

## 3. 非功能需求

| 编号 | 需求 | 优先级 |
| --- | --- | --- |
| NFR-001 | 720p 坐标基准不变 | 高 |
| NFR-002 | `selector/` 下的类仍为内部工具，不在 `custom.json` 注册 | 高 |
| NFR-003 | 所有新增参数和状态保持类型安全，不使用 `type: ignore` | 高 |
| NFR-004 | 调试输出最小化，仅保留关键错误日志 | 中 |
| NFR-005 | 向后兼容：未配置新参数时行为与当前版本一致 | 高 |
| NFR-006 | JSON Schema 同步更新，提供字段悬停提示 | 中 |
| NFR-007 | 代码结构保持 Reco 负责识别/状态、Action 负责执行的职责分离 | 高 |

## 4. 参数清单

### 4.1 PathFinderAction 新增/变更参数

| 参数 | 类型 | 默认值 | 说明 |
| --- | --- | --- | --- |
| `move_duration` | `int` | `500` | 无距离信息时的默认移动时长（毫秒），保留 |
| `move_duration_far` | `int` | `1500` | 远距离移动时长（毫秒），保留 |
| `move_duration_near` | `int` | `300` | 近距离移动时长（毫秒），保留 |
| `distance_far` | `int` | `200` | 远距离阈值（像素），保留 |
| `distance_near` | `int` | `50` | 近距离阈值（像素），保留 |
| `max_move_time` | `int` | `0` | 单次移动最大持续时间（毫秒），`0` 表示不限制 |
| `dodge_at_start` | `str` | `"never"` | 移动开始时是否闪避：`never` / `always` / `once_per_target` |
| `dodge_direction` | `str` | `"forward"` | 闪避方向，仅在支持方向的平台有意义 |
| `enable_turning` | `bool` | `true` | 转向总开关；`false` 时不执行转向 |
| `distance_missing_turn_scale` | `float` | `0.5` | 连续未识别到距离时，转向滑动距离的衰减系数 |
| `min_turn_distance` | `int` | `50` | 最短转向滑动距离（像素） |
| `max_turn_distance` | `int` | `400` | 最长转向滑动距离（像素） |

### 4.2 PathFindingReco 新增/变更参数

| 参数 | 类型 | 默认值 | 说明 |
| --- | --- | --- | --- |
| `target_lock_timeout` | `int` | `3` | 目标连续丢失多少帧后解锁 |
| `target_lock_center_tolerance` | `int` | `100` | 跨帧匹配锁定目标时的中心位置容差（像素） |
| `target_lock_score_threshold` | `float` | `0.75` | 匹配锁定目标时的最低置信度 |
| `max_turn_attempts` | `int` | `3` | 连续转向后仍未识别到距离，则停止转向直接移动 |

### 4.3 类级常量（非 Custom 参数，可在源码中调整）

| 常量 | 类型 | 默认值 | 说明 |
| --- | --- | --- | --- |
| `DEFAULT_TARGET_LOCK_IOU_THRESHOLD` | `float` | `0.3` | 锁定目标跨帧匹配的最低 IOU |
| `TURN_MIN_OFFSET_RATIO` | `float` | `0.0` | 触发转向的最小中心偏移比例 |
| `TURN_CLIP_RECT` | `list[int]` | `[800, 50, 300, 200]` | 转向滑动起止点的裁剪区域 `[x, y, w, h]`（720p 基准） |

## 5. 验收检查表

- [ ] 旧配置回归：不加任何新参数，现有单目标寻路 Pipeline 行为与当前版本一致。
- [ ] 多目标锁定：多目标场景下，选中目标后 5 帧内 `lock_id` 不跳动。
- [ ] 锁定后选择器失效：锁定目标后，即使出现更高优先级模板，也不切换目标。
- [ ] IOU 校验：两个同名目标相邻时，锁定目标不会跳到另一个目标上。
- [ ] 转向开关：`enable_turning=false` 时即使距离缺失也不调用 `turn`。
- [ ] 转向触发：`enable_turning=true` 且已选定目标无距离时，`PathFinderAction` 先 `turn` 再 `move`。
- [ ] 不依赖其他候选：`nearest` / `composite` 模式下，其他候选无距离但已选定目标有距离时，不触发转向。
- [ ] 转向方向：目标偏左时向右滑动，目标偏上时向下滑动。
- [ ] 转向衰减：`enable_turning=true` 且连续无距离时，第 2 次滑动距离低于第 1 次。
- [ ] 转向防抖：`max_turn_attempts=3` 且连续 3 帧已选定目标无距离时，第 4 帧起直接移动。
- [ ] 闪避模式：`dodge_at_start="always"` 每次移动前都 dodge；`"once_per_target"` 同一 `lock_id` 只 dodge 一次。
- [ ] `max_move_time`：配置 1000ms 后，远距离目标单次移动不超过 1 秒。
- [ ] 到达/丢失解锁：到达或连续丢失后，下一帧可重新选择新目标。
- [ ] `stuck` 不解锁：`state == "stuck"` 时仍保持目标锁定，由外部 Pipeline 处理脱卡。
- [ ] JSON Schema 与 README 已同步更新。
