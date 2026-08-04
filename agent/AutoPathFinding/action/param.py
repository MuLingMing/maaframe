# -*- coding: utf-8 -*-
"""
PathFinderAction 参数数据类

将执行器的 JSON 参数解析为类型安全的不可变对象，避免方法中反复使用裸 dict。

设计参考 MaaEnd MapTracker：
- 大幅转向时强制 "walk" 模式（rotation_upper_threshold）
- 卡住时触发 dodge（stuckThreshold 类比）
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class PathFinderParam:
    """
    PathFinderAction 执行参数

    Attributes:
        move_duration: 无距离信息时的默认移动时长（毫秒）
        move_duration_far: 距离 ≥ distance_far 时的移动时长（毫秒）
        move_duration_near: 距离 ≤ distance_near 时的移动时长（毫秒）
        distance_far: 远距离阈值（像素）
        distance_near: 近距离阈值（像素）
        max_move_time: 单次移动最大时长（毫秒），0 表示不限制
        dodge_at_start: 移动开始时闪避模式（never/always/once_per_target）
        dodge_direction: 闪避方向
        enable_turning: 是否执行 Reco 建议的转向（执行侧保险开关）
        dodge_follows_direction: 闪避方向是否跟随本帧移动方向；true 时忽略 dodge_direction
        dodge_release_ms: dodge 与 move 之间的短暂释放时长（毫秒），0 表示不释放
        turn_duration_ms: 显式转向滑动持续时间（毫秒），None 表示用 platform 默认
        large_turn_release_ms: 大幅转向（LARGE_TURN）前先释放按键的时长（毫秒），0 表示不释放
        stuck_dodge_on_phase: 卡住阶段是否触发 dodge
    """

    move_duration: int
    move_duration_far: int
    move_duration_near: int
    distance_far: int
    distance_near: int
    max_move_time: int
    dodge_at_start: str
    dodge_direction: str
    enable_turning: bool
    dodge_follows_direction: bool
    dodge_release_ms: int
    turn_duration_ms: int | None
    large_turn_release_ms: int
    stuck_dodge_on_phase: bool