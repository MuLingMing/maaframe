# -*- coding: utf-8 -*-
"""
按优先级选择目标
"""

from .base import TargetSelector
from .types import TargetInfo


class PriorityTargetSelector(TargetSelector):
    """
    按模板优先级选择目标

    按照 priority 列表的顺序，选择第一个匹配的目标。

    参数：
        priority: 模板优先级列表，从高到低排列

    示例：
        selector = PriorityTargetSelector(priority=["quest", "npc"])
        selected = selector.select(targets)
    """

    def __init__(self, priority: list[str]):
        self.priority = priority

    def select(self, targets: list[TargetInfo]) -> TargetInfo | None:
        if not targets:
            return None

        # 同一模板可能有多候选（多尺度、相似位置），按 score 取最高。
        for ptype in self.priority:
            best = max(
                (t for t in targets if t.template == ptype),
                key=lambda t: t.score,
                default=None,
            )
            if best is not None:
                return best

        return max(targets, key=lambda t: t.score)
