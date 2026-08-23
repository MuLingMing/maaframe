# AutoPathFinding 自动寻路模块

## 1. 模块概述

### 1.1 功能定义

根据游戏截图识别指定目标图标，结合距离信息判断目标位置，自动执行移动操作到达目标点。

### 1.2 核心能力

- 目标图标识别（模板匹配）
- 距离信息读取（OCR）
- 多目标选择（接口化，可扩展）
- 目标锁定与跨帧追踪（保证选择连贯性）
- 转向动作（距离缺失时通过滑动视角恢复距离显示）
- 闪避动作（移动开始时触发，可用于启用疾跑状态）
- 自动移动（平台适配，复用 OperationRecording）
- 运动状态评估（到达 / 卡住 / 接近中）
- 最大移动时间兜底

### 1.3 技术栈

- **识别**：MaaFramework 内置 TemplateMatch + OCR
- **平台**：复用 OperationRecording 的 PlatformFactory
- **调度**：MaaFramework Pipeline
- **分辨率基准**：720p (1280x720)

---

## 2. 架构设计

### 2.1 组件架构

```
┌─────────────────────────────────────────────────────────┐
│                    Pipeline 协调层                       │
│  (JSON 定义识别流程、分支逻辑、任务调度)                 │
└─────────────────────────────────────────────────────────┘
                          │
        ┌─────────────────┴─────────────────┐
        ▼                                   ▼
┌─────────────────────┐           ┌─────────────────────┐
│  PathFindingReco    │           │  PathFinderAction   │
│  (Custom Recognition)│           │  (Custom Action)    │
├─────────────────────┤           ├─────────────────────┤
│ • 模板匹配识别目标   │           │ • 从 Reco 提取方向  │
│ • OCR 提取距离       │    ───►   │ • 按 lock_id 闪避   │
│ • 选择并锁定目标     │           │ • 按坐标转向        │
│ • 计算移动方向       │           │ • 计算并执行移动    │
│ • 计算转向坐标       │           │ • release_all 释放  │
│ • 评估运动状态       │           │                     │
└─────────────────────┘           └─────────────────────┘
        │                                   │
        └─────────────────┬─────────────────┘
                          ▼
                ┌─────────────────────┐
                │   PlatformFactory   │
                │ (OperationRecording) │
                └─────────────────────┘
                          │
                          ▼
                ┌─────────────────────┐
                │     Selector/       │
                │ (策略插件，可扩展)  │
                └─────────────────────┘
```

### 2.2 设计原则

- **Reco 职责**：根据截图返回信息（目标位置、距离、方向、运动状态、锁定 ID、转向坐标）
- **Action 职责**：根据 Reco 返回的信息按 `dodge → turn → move` 顺序执行动作
- **选择器**：通过 `selector/` 目录下的可插拔类实现多目标选择策略
- **状态隔离**：通过 `state/` 目录下的类按 Pipeline 节点名隔离跨帧状态
- **平台层**：复用 OperationRecording 的 PlatformFactory，不重复实现
- **卡住/到达检测**：由 PathFindingReco 跨帧跟踪并在 `detail.state` 中返回，Pipeline 根据状态决定分支；Action 内不做检测

### 2.3 组件清单

| 组件                        | 类型               | 职责                                                      |
| --------------------------- | ------------------ | --------------------------------------------------------- |
| **PathFindingReco**         | Custom Recognition | 识别目标、提取距离、锁定追踪、计算方向/转向、评估运动状态 |
| **PathFinderAction**        | Custom Action      | 执行 dodge、turn、move，控制最大移动时间                  |
| **TargetSelector**          | Python 接口        | 目标选择策略（抽象基类）                                  |
| **PriorityTargetSelector**  | 预置实现           | 按模板优先级选择                                          |
| **NearestTargetSelector**   | 预置实现           | 按距离最近选择                                            |
| **CompositeTargetSelector** | 预置实现           | 按类型优先级 + 距离组合选择                               |
| **LockedTarget**            | 状态类             | 被锁定目标的信息与 IOU 匹配                               |
| **PathFindingState**        | 状态类             | Reco 侧按节点隔离的寻路状态                               |
| **PathFinderState**         | 状态类             | Action 侧按节点隔离的执行状态                             |
| **PlatformFactory**         | 复用               | 平台操作适配（OperationRecording）                        |

---

## 3. 使用说明

### 3.1 快速开始

#### 3.1.1 注册组件

在 `agent/custom.json` 中注册组件（平铺格式）：

```json
{
    "PathFindingReco": {
        "type": "recognition",
        "class": "PathFindingReco",
        "file_path": "AutoPathFinding/recognition/path_finding_reco.py"
    },
    "PathFinderAction": {
        "type": "action",
        "class": "PathFinderAction",
        "file_path": "AutoPathFinding/action/path_finder_action.py"
    }
}
```

> 说明：`selector/` 与 `state/` 目录下的类是 `PathFindingReco` / `PathFinderAction` 内部使用的 Python 模块，**不需要**在 `custom.json` 中注册。

#### 3.1.2 准备模板图片

将目标图标模板放入 `assets/resource/image/` 目录：

```
assets/resource/image/
├── quest_icon.png      # 任务图标
├── npc_icon.png        # NPC 图标
└── shop_icon.png       # 商店图标
```

### 3.2 PathFindingReco 参数

| 参数                           | 类型             | 默认值               | 说明                                                                          |
| ------------------------------ | ---------------- | -------------------- | ----------------------------------------------------------------------------- |
| `expected_templates`           | `list[str]`      | `[]`                 | 期望匹配的模板列表（必填），支持单张图片或文件夹路径                          |
| `roi`                          | `list[int]`      | `[0, 0, 1280, 720]`  | 识别区域 [x, y, w, h]                                                         |
| `threshold`                    | `float`          | `0.8`                | 匹配阈值 (0-1)                                                                |
| `selector_type`                | `str`            | `"priority"`         | 选择器类型：`priority` / `nearest` / `composite`                              |
| `selector_priority`            | `list[int\|str]` | `[]`                 | 选择器优先级（支持索引或名称格式）                                            |
| `distance_pattern`             | `str`            | `"(\d+)米"`          | 距离 OCR 正则表达式                                                           |
| `distance_offset`              | `list[int]`      | `[-26, -30, 47, 57]` | 距离 OCR 区域偏移 [x, y, w, h]                                                |
| `distance_threshold`           | `float`          | `0.3`                | OCR 置信度阈值                                                                |
| `dead_zone`                    | `int`            | `50`                 | 方向判断死区（像素，欧氏距离），目标在死区内时返回 `direction="centered"`     |
| `arrival_distance`             | `int`            | `30`                 | 到达判定距离阈值（像素），OCR 距离 ≤ 此值时 `state="arrived"`                 |
| `stuck_threshold`              | `int`            | `3`                  | 卡住判定连续帧数，连续达到此值时 `state="stuck"`                              |
| `stuck_distance_tolerance`     | `int`            | `5`                  | 卡住距离容差（像素），相邻两帧距离缩短值 ≤ 此值视为无进展                     |
| `stuck_center_tolerance`       | `int`            | `10`                 | 卡住中心偏移容差（像素），无距离信息时相邻两帧中心偏移 ≤ 此值视为无进展       |
| `target_lock_timeout`          | `int`            | `3`                  | 目标连续丢失多少帧后解锁                                                      |
| `target_lock_center_tolerance` | `int`            | `100`                | 锁定目标跨帧匹配中心容差（像素）                                              |
| `target_lock_score_threshold`  | `float`          | `0.75`               | 锁定目标跨帧匹配最低置信度                                                    |
| `enable_turning`               | `bool`           | `true`               | 是否计算并返回转向坐标                                                        |
| `max_turn_attempts`            | `int`            | `3`                  | 连续转向后仍未识别到距离，则停止转向；第 `max_turn_attempts + 1` 帧起直接移动 |
| `min_turn_distance`            | `int`            | `50`                 | 最短转向滑动距离（像素）                                                      |
| `max_turn_distance`            | `int`            | `400`                | 最长转向滑动距离（像素）                                                      |
| `distance_missing_turn_scale`  | `float`          | `0.5`                | 连续未识别到距离时转向滑动距离衰减系数                                        |
| `direction_hysteresis`          | `float`          | `15.0`               | 方向分箱的角度滞回阈值（度），抑制 45° 边界方向抖动；设为 0 关闭滞回          |
| `turn_speed_factor`             | `float`          | `0.7`                | 滑动距离相对 `offset_norm` 的比例（< 1.0 保证滑动后目标朝中心移动但不越过中心到反方向）；动态自适应算法的核心参数 |
| `enable_locked_fast_path`       | `bool`           | `true`               | 锁定态下使用小 ROI 快速通道；关闭后每帧全 ROI 扫描                            |
| `lock_roi_padding`              | `int`            | `200`                | 锁定态 fast_path ROI 相对 bbox 的额外 padding（像素）                          |
| `fast_path_max_turn_padding`    | `int`            | `400`                | 距离缺失时 fast_path ROI 的额外 padding，避免转向后目标移出视野              |
| `fast_path_miss_limit`          | `int`            | `3`                  | fast_path ROI 连续 miss 多少帧后临时回退到全 ROI                              |
| `turn_min_floor_ratio`          | `float`          | `0.5`                | 滑动距离衰减下限（相对 min_turn_distance 的比例），避免连续衰减后转向距离过小 |
| `rotation_lower_threshold`      | `float`          | `8.0`                | 转向微调阈值（度），偏差低于此值视为 FINE_TUNE（借鉴 MaaEnd MapTracker）       |
| `rotation_upper_threshold`      | `float`          | `60.0`               | 转向大幅调整阈值（度），偏差超过此值视为 LARGE_TURN，先释放按键再转向          |
| `rotation_adaptive_enabled`     | `bool`           | `true`               | 是否启用自适应转向距离（EMA 0.618/0.382，借鉴 MapTracker `rotationSpeed`）     |
| `stuck_timeout_ms`              | `int`            | `10000`              | 卡住阶段最长持续时间（毫秒），超过后判定导航失败（借鉴 MapTracker `stuckTimeout`） |

#### 返回值格式

```python
# 命中目标
{
    "box": [x, y, w, h],           # 目标边界框
    "detail": {
        "hit": True,
        "hit_node": "if",          # 用于 IfElseAction 分支
        "target": {
            "template": "quest_icon.png",
            "center": [640.0, 360.0],  # 浮点坐标
            "bbox": [600, 320, 80, 80],
            "score": 0.95,
            "distance": 150            # 可选
        },
        "direction": "forward",        # forward/backward/left/right/centered
        "state": "approaching",        # approaching/arrived/stuck
        "phase": "tracking",           # 阶段状态机：seeking/tracking/approaching/turning/stuck/lost
        "lock_id": "quest_icon.png#640#360#a1b2c3d4",  # 当前锁定目标 ID
        "turn_start": [900, 150],      # 转向滑动起点 [x, y]，可选
        "turn_end": [1100, 250],       # 转向滑动终点 [x, y]，可选
        "turn_grade": "large_turn"     # 转向分级：fine_tune / large_turn
    }
}

# 未命中目标
{
    "box": None,
    "detail": {
        "hit": False,
        "hit_node": "else"
    }
}
```

### 3.3 PathFinderAction 参数

| 参数                 | 类型   | 默认值      | 说明                                                       |
| -------------------- | ------ | ----------- | ---------------------------------------------------------- |
| `move_duration`      | `int`  | `500`       | 无距离信息时的默认移动持续时间（毫秒）                     |
| `move_duration_far`  | `int`  | `1500`      | 远距离移动持续时间（毫秒），距离 ≥ `distance_far` 时使用   |
| `move_duration_near` | `int`  | `300`       | 近距离移动持续时间（毫秒），距离 ≤ `distance_near` 时使用  |
| `distance_far`       | `int`  | `200`       | 远距离阈值（像素）                                         |
| `distance_near`      | `int`  | `50`        | 近距离阈值（像素）                                         |
| `max_move_time`      | `int`  | `0`         | 单次移动最大时长（毫秒），`0` 表示不限制                   |
| `dodge_at_start`     | `str`  | `"never"`   | 移动开始时闪避模式：`never` / `always` / `once_per_target` |
| `dodge_direction`    | `str`  | `"backward"` | 闪避方向：`forward` / `backward` / `left` / `right`；游戏单点闪避默认向后翻滚 |
| `enable_turning`     | `bool` | `true`      | 是否执行 Reco 建议的转向（执行侧保险开关）                 |
| `dodge_follows_direction` | `bool` | `true`   | 闪避方向是否跟随本帧移动方向；true 时忽略 `dodge_direction` |
| `dodge_release_ms`   | `int`  | `50`       | dodge 与 move 之间的短暂 release 间隔（毫秒），0 表示不释放 |
| `move_start_delay_ms` | `int` | `300`      | move 开始前的额外等待时长（毫秒），给游戏反应时间           |
| `turn_duration_ms`   | `int`         | `200`  | 显式控制 `platform.turn` 滑动持续时间（毫秒），0 用 platform 默认 |
| `large_turn_release_ms` | `int`      | `100`  | 大幅转向（LARGE_TURN）前先释放按键的时长（毫秒），0 表示不释放；借鉴 MapTracker `rotation_upper_threshold` 行为 |
| `stuck_dodge_on_phase` | `bool`      | `true` | 当 Reco 返回 `phase="stuck"` 时是否额外触发 dodge（借鉴 MapTracker `stuckThreshold` 行为） |

距离在 `(distance_near, distance_far)` 之间时，移动时长按线性插值计算；`distance` 为 `None` 时回退到 `move_duration`。`max_move_time > 0` 时，实际移动时长不会超过该值。

### 3.4 选择器类型

#### 3.4.1 priority（按优先级选择）

按优先级顺序逐个识别，命中即停（短路求值）。

```json
{
    "selector_type": "priority",
    "expected_templates": [
        "quest_icon.png",
        "npc_icon.png"
    ]
}
```

#### 3.4.2 nearest（按距离选择）

全量识别所有模板，提取距离信息，选择距离最近的目标。

```json
{
    "selector_type": "nearest",
    "distance_pattern": "(\\d+)米",
    "distance_offset": [
        -10,
        -10,
        30,
        30
    ]
}
```

#### 3.4.3 composite（组合条件选择）

按优先级顺序识别，同优先级内选择距离最近的目标。

```json
{
    "selector_type": "composite",
    "expected_templates": [
        "quest_icon.png",
        "npc_icon.png"
    ],
    "distance_pattern": "(\\d+)米"
}
```

#### 3.4.4 文件夹作为模板

`expected_templates` 中的每一项既可以是单张图片，也可以是文件夹路径。文件夹内所有图片会被等效为**同一个模板**，选择器和优先级均按文件夹名处理。

```json
{
    "selector_type": "priority",
    "expected_templates": [
        "icons/quest",
        "npc_icon.png"
    ]
}
```

上例中，`assets/resource/image/icons/quest/` 目录下的所有图片被视为一个名为 `icons/quest` 的模板，命中任意一张即视为命中该模板。

#### 3.4.5 selector_priority 参数

支持**索引格式**和**名称格式**两种写法，用于调整 `expected_templates` 的优先级顺序。

**索引格式**（0-based）：

```json
{
    "expected_templates": [
        "A",
        "B",
        "C"
    ],
    "selector_priority": [
        1,
        2
    ]
}
```

优先级：B > C > A

**名称格式**：

```json
{
    "expected_templates": [
        "A",
        "B",
        "C"
    ],
    "selector_priority": [
        "B",
        "C"
    ]
}
```

优先级：B > C > A

---

## 4. Pipeline 配置示例

> **提示**：完整示例中的 `selector_priority` 字段可省略，未设置时使用 `expected_templates` 原始顺序。

### 4.1 基础寻路

```json
{
    "AutoPathFinding": {
        "recognition": "Custom",
        "custom_recognition": "PathFindingReco",
        "custom_recognition_param": {
            "expected_templates": ["quest_icon.png"],
            "selector_type": "priority",
            "selector_priority": ["quest_icon.png"]
        },
        "next": ["PathFinderAction"]
    },
    "PathFinderAction": {
        "custom_action": "PathFinderAction",
        "custom_action_param": {
            "move_duration": 500,
            "max_move_time": 1000,
            "dodge_at_start": "once_per_target",
            "dodge_direction": "forward",
            "enable_turning": true
        },
        "next": ["AutoPathFinding"]
    }
}
```

### 4.2 带 IfElseAction 的条件分支

```json
{
    "AutoPathFinding": {
        "recognition": "Custom",
        "custom_recognition": "PathFindingReco",
        "custom_recognition_param": {
            "expected_templates": ["quest_icon.png"],
            "selector_type": "priority",
            "selector_priority": ["quest_icon.png"]
        },
        "next": ["HandlePathFinding"]
    },
    "HandlePathFinding": {
        "custom_action": "IfElseAction",
        "if": ["PathFinderAction"],
        "else": ["TaskComplete"]
    },
    "PathFinderAction": {
        "custom_action": "PathFinderAction",
        "next": ["AutoPathFinding"]
    },
    "TaskComplete": {
        "next": []
    }
}
```

### 4.3 多目标选择

```json
{
    "AutoPathFinding": {
        "recognition": "Custom",
        "custom_recognition": "PathFindingReco",
        "custom_recognition_param": {
            "expected_templates": [
                "quest_icon.png",
                "npc_icon.png",
                "shop_icon.png"
            ],
            "selector_type": "composite",
            "selector_priority": [
                "quest_icon.png",
                "npc_icon.png",
                "shop_icon.png"
            ]
        },
        "next": ["PathFinderAction"]
    },
    "PathFinderAction": {
        "custom_action": "PathFinderAction",
        "next": ["AutoPathFinding"]
    }
}
```

---

## 5. 数据类型定义

### 5.1 TargetInfo

```python
@dataclass
class TargetInfo:
    """识别到的目标信息"""
    template: str                      # 匹配的模板名称
    center: tuple[float, float]        # 中心坐标 (x, y)，浮点精度
    bbox: tuple[int, int, int, int]    # 边界框 (x, y, w, h)
    score: float                       # 匹配置信度
    distance: int | None = None        # 距离值（如果存在）
```

### 5.2 LockedTarget

```python
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
```

### 5.3 TargetSelector 接口

```python
class TargetSelector(ABC):
    """目标选择器接口"""

    @abstractmethod
    def select(self, targets: list[TargetInfo]) -> TargetInfo | None:
        """从目标列表中选择一个目标"""
        ...
```

---

## 6. 平台适配

### 6.1 复用 OperationRecording

直接使用 OperationRecording 的 PlatformFactory，无需重复实现。

```python
from OperationRecording.platforms import PlatformFactory
# ↑ 必须用包级导入（确保 OperationRecording.platforms 包被加载，
#   触发 @register_platform 装饰器执行，否则 platform_registry 为空）

# 创建平台实例（带缓存）
platform = PlatformFactory.create_from_config({}, context.tasker.controller)

# 使用平台操作
platform.dodge("forward")                # 闪避
platform.turn(x1, y1, x2, y2)            # 调整视角
platform.move("forward", duration=0.5)   # 移动
platform.release_all()                   # 释放所有操作
```

**平台缓存**：`create_from_config` 内部使用 `WeakKeyDictionary` 缓存 platform 实例，同一 controller 多次调用返回同一 platform，避免高频调用时的重复创建并保留 platform 内部状态（`_active_contacts`、`_active_directions`）。测试或重连场景可调用 `PlatformFactory.clear_cache()` 显式清空。

### 6.2 可用的平台操作

| OperationRecording 方法                         | AutoPathFinding 用途                                         |
| ----------------------------------------------- | ------------------------------------------------------------ |
| `platform.move(direction, duration)`            | 移动角色（forward/backward/left/right），`centered` 时不调用 |
| `platform.turn(start_x, start_y, end_x, end_y)` | 调整视角使目标居中或恢复距离显示                             |
| `platform.dodge(direction)`                     | 闪避，可用于触发疾跑状态                                     |
| `platform.click(x, y)`                          | 点击屏幕目标位置                                             |
| `platform.swipe(...)`                           | 滑动操作                                                     |
| `platform.release_all()`                        | 释放所有操作                                                 |

---

## 7. 识别流程详解

### 7.1 统一识别流程

```
┌─────────────────────────────────────────────────────────┐
│  1. 解析参数                                             │
│     - expected_templates: 模板列表                       │
│     - selector_priority: 计算最终优先级顺序               │
│     - selector_type: 选择器类型                          │
└─────────────────────────────────────────────────────────┘
                          │
                          ▼
┌─────────────────────────────────────────────────────────┐
│  2. 按优先级顺序识别                                     │
│     - priority: 短路识别（命中即停）                      │
│     - nearest: 全量识别                                  │
│     - composite: 全量识别（按优先级分组）                  │
└─────────────────────────────────────────────────────────┘
                          │
                          ▼
┌─────────────────────────────────────────────────────────┐
│  3. 距离提取（命中即提取）                                │
│     - 按 MaaFramework roi_offset 语义叠加 distance_offset│
│     - 识别 OCR，匹配 distance_pattern                    │
│     - 绑定距离到 TargetInfo                              │
└─────────────────────────────────────────────────────────┘
                          │
                          ▼
┌─────────────────────────────────────────────────────────┐
│  4. 目标锁定/追踪/解锁                                   │
│     - 已锁定：按 IOU + 中心容差 + 置信度追踪              │
│     - 追踪失败：miss_count++，未超时返回历史位置          │
│     - 超时或初次识别：选择并锁定新目标                    │
└─────────────────────────────────────────────────────────┘
                          │
                          ▼
┌─────────────────────────────────────────────────────────┐
│  5. 计算方向                                             │
│     - 角度分箱 + 圆形死区                                 │
│     - 返回 forward/backward/left/right/centered         │
└─────────────────────────────────────────────────────────┘
                          │
                          ▼
┌─────────────────────────────────────────────────────────┐
│  6. 计算转向坐标                                         │
│     - 仅当选中目标 distance is None 且 enable_turning    │
│     - 超过 max_turn_attempts 后停止转向                   │
│     - 返回 turn_start / turn_end                         │
└─────────────────────────────────────────────────────────┘
                          │
                          ▼
┌─────────────────────────────────────────────────────────┐
│  7. 评估运动状态                                         │
│     - 跨帧跟踪目标距离与中心位置                          │
│     - 返回 approaching / arrived / stuck                │
└─────────────────────────────────────────────────────────┘
```

### 7.2 目标锁定与追踪

锁定目标后，选择器优先级失效：即使出现更高优先级的模板，也不会切换目标。追踪使用同名模板 + IOU + 中心容差 + 置信度四重校验，降低两个同名目标相邻时跟踪错误的概率。

```python
def _track_locked_target(self, locked, targets, param):
    candidates = [
        t for t in targets
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
```

### 7.3 转向坐标计算

当已锁定目标距离缺失时，计算与目标偏移相反的滑动向量，使目标进入屏幕中心区域以恢复距离显示。滑动向量以 `TURN_CLIP_RECT` 中心为基准放置，并将起止点裁剪到该区域内，防止因硬裁剪导致滑动坍塌。

```python
def _compute_turn_coordinates(self, target_center, turn_attempt, param):
    tx, ty = target_center
    cx, cy = SCREEN_CENTER
    offset_x = tx - cx
    offset_y = ty - cy
    offset_norm = math.hypot(offset_x, offset_y)

    offset_ratio = min(1.0, offset_norm / (SCREEN_WIDTH / 2))
    base_distance = (
        param.min_turn_distance
        + offset_ratio * (param.max_turn_distance - param.min_turn_distance)
    )

    # 第 2 次起衰减
    if turn_attempt > 1:
        base_distance *= param.distance_missing_turn_scale ** (turn_attempt - 1)

    # 滑动方向与目标偏移相反
    ux = -offset_x / offset_norm
    uy = -offset_y / offset_norm

    # 以 TURN_CLIP_RECT 中心为滑动中心
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

### 7.4 运动状态检测

由 PathFindingReco 在识别阶段跨帧跟踪目标状态，并通过 `detail.state` 返回。

状态定义：

- `approaching`：正在接近目标（默认状态）
- `arrived`：OCR 距离 ≤ `arrival_distance`，判定已到达
- `stuck`：连续 `stuck_threshold` 帧距离/中心几乎无变化，判定卡住

判定逻辑：

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

跨帧状态按 `node_name` 隔离，生命周期跟随 Python 进程。到达目标时会自动清空对应状态，避免切换节点后历史数据污染。

### 7.5 状态处理

PathFindingReco 返回的 `detail.state` 可与 Pipeline 分支配合使用，典型用法：

```json
{
    "AutoPathFinding": {
        "recognition": "Custom",
        "custom_recognition": "PathFindingReco",
        "next": ["HandleState"]
    },
    "HandleState": {
        "custom_action": "IfElseAction",
        "custom_action_param": {
            "if": ["ArrivedHandler"],
            "else": ["PathFinderAction"]
        }
    }
}
```

> 说明：IfElseAction 默认按 `hit_node` 分支（命中目标为 `if`，未命中为 `else`）。如需按 `state` 分支，需要自定义 Action 读取 `reco_detail.best_result.detail.state`。

**处理建议**：

- `arrived`：结束寻路，执行后续任务（如交互、进入副本）
- `stuck`：尝试脱卡（如释放方向键、跳一下、切换路径）
- `approaching`：继续执行 `PathFinderAction` 移动

---

## 8. 执行流程详解

### 8.1 Action 执行序列

PathFinderAction 按 `dodge → turn → move` 顺序执行：

1. **dodge**：根据 `dodge_at_start` 策略决定是否闪避。
    - `never`：不闪避
    - `always`：每次执行都闪避
    - `once_per_target`：同一 `lock_id` 只闪避一次
2. **turn**：当 Reco 返回 `turn_start` / `turn_end` 且 Action 的 `enable_turning=true` 时执行。
3. **move**：根据 `direction` 执行移动，`direction="centered"` 时跳过移动。

### 8.2 最大移动时间

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

---

## 9. 文件结构

```
agent/AutoPathFinding/
├── __init__.py                     # 模块初始化
├── README.md                       # 本文档
├── action/
│   ├── __init__.py
│   ├── param.py                    # PathFinderAction 参数数据类
│   └── path_finder_action.py       # 自定义动作：执行 dodge/turn/move
├── recognition/
│   ├── __init__.py
│   ├── param.py                    # PathFindingReco 参数数据类
│   ├── path_finding_reco.py        # 自定义识别：识别目标并计算决策
│   └── turn_strategy.py            # 转向算法独立模块（双阈值/自适应距离）
├── selector/
│   ├── __init__.py
│   ├── base.py                     # TargetSelector 接口
│   ├── types.py                    # TargetInfo 数据类型
│   ├── priority_selector.py        # 按优先级选择
│   ├── nearest_selector.py         # 按距离选择
│   └── composite_selector.py       # 组合条件选择
└── state/
    ├── __init__.py                 # 状态包入口
    └── path_finding_state.py       # LockedTarget / Phase / TurnGrade / RotationAdjuster
                                     # / StuckDetector / PathFindingState / PathFinderState
```

---

## 10. 扩展指南

### 10.1 自定义选择器

实现 `TargetSelector` 接口即可：

```python
from agent.AutoPathFinding.selector import TargetSelector, TargetInfo

class MyCustomSelector(TargetSelector):
    def select(self, targets: list[TargetInfo]) -> TargetInfo | None:
        # 自定义选择逻辑
        ...
```

### 10.2 调整类级常量

以下常量定义在 `PathFindingReco` 类中，可在源码中调整以适配不同游戏或场景，但不作为 Custom 参数暴露：

| 常量                                | 类型    | 默认值                | 说明                             |
| ----------------------------------- | ------- | --------------------- | -------------------------------- |
| `DEFAULT_TARGET_LOCK_IOU_THRESHOLD` | `float` | `0.3`                 | 锁定目标跨帧匹配最小 IOU         |
| `TURN_CLIP_HORIZONTAL`              | `tuple` | `(200, 80, 880, 120)` | 水平转向（左/右）滑动起止点裁剪区域 [x,y,w,h]，720p 基准 |
| `TURN_CLIP_VERTICAL`                | `tuple` | `(560, 60, 160, 600)` | 垂直转向（上/下）滑动起止点裁剪区域 [x,y,w,h]，720p 基准 |
| `TURN_MIN_OFFSET_RATIO`             | `float` | `0.0`                 | 触发转向的最小中心偏移比例（相对屏幕半宽） |
| `STATE_TO_STRING`                   | `dict`  | 6 项 Phase 映射        | `Phase` 枚举 → detail.phase 字符串映射（兼容 IfElseAction） |

---

## 11. API 接口规范

### 11.1 Custom Recognition 接口规范

**PathFindingReco** 遵循 MaaFramework Custom Recognition 规范：

```python
class PathFindingReco(CustomRecognition):
    def analyze(
        self,
        context: Context,
        argv: CustomRecognition.AnalyzeArg,
    ) -> Union[CustomRecognition.AnalyzeResult, Optional[RectType]]:
        """
        参数:
        - context: MaaFramework 上下文对象
        - argv: 识别参数，包含 image、roi、custom_recognition_param 等

        返回值:
        - CustomRecognition.AnalyzeResult: 识别结果
        - Optional[RectType]: 识别到的位置
        - None: 识别失败
        """
```

**关键 API 使用**：

- `argv.image`: 获取输入图片
- `argv.custom_recognition_param`: 获取自定义参数（JSON 字符串或 dict）
- `context.run_recognition_direct()`: 调用 MaaFramework 内置识别器（TemplateMatch、OCR 等）
- `CustomRecognition.AnalyzeResult(box=..., detail={...})`: 返回识别结果

**RecognitionDetail 类型安全访问**：

`context.run_recognition_direct()` 返回 `RecognitionDetail`，其 `best_result` 是联合类型，需要根据算法类型进行类型检查：

```python
from maa.define import TemplateMatchResult, OCRResult

# TemplateMatch 结果
reco_detail = context.run_recognition_direct(JRecognitionType.TemplateMatch, reco_param, img)
if reco_detail is not None and reco_detail.hit:
    best_result = reco_detail.best_result
    if isinstance(best_result, TemplateMatchResult):
        box = best_result.box   # Rect 或 list
        score = best_result.score  # float

# OCR 结果
reco_detail = context.run_recognition_direct(JRecognitionType.OCR, reco_param, img)
if reco_detail is not None and reco_detail.hit:
    best_result = reco_detail.best_result
    if isinstance(best_result, OCRResult):
        text = best_result.text  # str
        score = best_result.score  # float
```

**RecognitionResult 联合类型**：

- `TemplateMatchResult`: box, score
- `OCRResult`: box, score, text
- `FeatureMatchResult`: box, count
- `CustomRecognitionResult`: box, detail

### 11.2 Custom Action 接口规范

**PathFinderAction** 遵循 MaaFramework Custom Action 规范：

```python
class PathFinderAction(CustomAction):
    def run(
        self,
        context: Context,
        argv: CustomAction.RunArg,
    ) -> CustomAction.RunResult:
        """
        参数:
        - context: MaaFramework 上下文对象
        - argv: 运行参数，包含识别结果和自定义参数

        返回值:
        - CustomAction.RunResult: 执行结果
        """
```

**关键 API 使用**：

- `argv.custom_action_param`: 获取自定义参数（JSON 字符串）
- `argv.reco_detail.best_result.detail`: 获取前序识别器返回的 detail 字典
- `context.tasker.stopping`: 检查是否收到停止信号
- `context.tasker.controller`: 获取控制器（用于 platform 创建）
- `CustomAction.RunResult(success=True/False)`: 返回执行结果

### 11.3 与 IfElseAction 集成

PathFindingReco 返回的 `detail` 字典包含 `hit_node` 字段，可与 IfElseAction 配合使用：

```json
{
    "AutoPathFinding": {
        "recognition": "Custom",
        "custom_recognition": "PathFindingReco",
        "custom_recognition_param": {
            "expected_templates": ["quest_icon.png"],
            "selector_type": "priority",
            "selector_priority": ["quest_icon.png"]
        },
        "action": "Custom",
        "custom_action": "IfElseAction",
        "custom_action_param": {
            "if": ["PathFinderAction"],
            "else": ["TaskComplete"]
        }
    }
}
```

**返回值映射**：

- `hit=True, hit_node="if"` → IfElseAction 执行 `if` 分支
- `hit=False, hit_node="else"` → IfElseAction 执行 `else` 分支

---

## 12. 风险评估

| 风险                         | 概率 | 影响 | 应对措施                                             |
| ---------------------------- | ---- | ---- | ---------------------------------------------------- |
| 模板匹配误识别               | 中   | 中   | 动态阈值 + `green_mask` 过滤任务追踪标记             |
| OCR 识别不准确               | 中   | 中   | 正则匹配 + `isdigit` 容错处理                        |
| 1 像素抖动导致方向频繁切换   | 低   | 中   | 角度分箱 + 圆形死区（`math.hypot`）                  |
| 目标在死区内仍继续前进       | 低   | 中   | 死区内返回 `direction="centered"`，Action 不执行移动 |
| 平台操作不兼容               | 低   | 高   | 复用 OperationRecording，已验证                      |
| 平台实例重复创建             | 低   | 中   | `PlatformFactory` 内部 `WeakKeyDictionary` 缓存      |
| 距离缺失导致卡住误判         | 低   | 中   | 无距离时回退到中心偏移判断                           |
| 到达阈值设置不当导致提前结束 | 低   | 中   | 按实际游戏距离调整 `arrival_distance`                |
| 两个同名目标相邻导致跟踪错   | 低   | 中   | IOU + 中心容差 + 置信度三重校验                      |
| 连续转向未识别距离导致停留   | 低   | 中   | `max_turn_attempts` 上限，超限时直接移动             |
| **死区内转向抖动推到屏幕外** | 中   | 高   | Reco 与 Action **双重防护**：`_calculate_turn(direction="centered")` 直接返回 `(None, None, None)`；`_execute_sequence` 同样跳过 turn。目标已居中时绝不滑动视角。 |
| **滑动方向反了，目标被推到屏幕外** | 中   | 高   | **根因**：原版 swipe `start → end` 方向 = +offset（指向目标方向），但移动端 swipe 后屏幕上目标位置变化 = swipe 向量本身，导致目标被推离屏幕中心。**修复**：`compute_turn_coordinates` 改用 `swipe = end - start = -offset`，使目标朝屏幕中心移动。Reco 与 Action 增加 `logger.debug` 输出滑动向量与期望向量对照，便于现场排查。 |
| **direction=centered 且 distance=None 时角色原地不动** | 中   | 高   | **根因**：原版 Action 在 `direction == "centered"` 时直接 return，不调 move；但 distance 缺失时本应继续前进靠近目标（distance OCR 失败 ≠ 已到达）。**修复**：centered + `distance is not None` → 跳过 move（视为已到达）；centered + `distance is None` → 兜底前进 `move_duration` 时长。 |
| **滑动距离过大把目标推过中心** | 中   | 高   | **根因**：原版 `compute_turn_distance` 在 [min=50, max=400] 之间按 offset_norm 线性插值，大偏移时滑动 200-400px（屏幕尺寸 720p 下相当于旋转视角 60°-90°），导致目标被推到屏幕反方向（如 target 在屏幕下方被推到屏幕更下方）。**修复**：滑动距离改为 `offset_norm * turn_speed_factor`（默认 0.7），与目标偏移成正比，保证滑动后目标朝中心移动而不越过中心。`max_turn_distance` 仍作为上限保护。 |

---

## 13. 参考资料

- [MaaFramework Pipeline 协议](https://github.com/MaaXYZ/MaaFramework/blob/main/docs/en_us/3.1-PipelineProtocol.md)
- [MaaFramework Custom Recognition/Action](https://github.com/MaaXYZ/MaaFramework/blob/main/docs/en_us/3.2-CustomAction.md)
- [OperationRecording 模块](../OperationRecording/)
