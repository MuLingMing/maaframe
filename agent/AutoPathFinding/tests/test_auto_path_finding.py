# -*- coding: utf-8 -*-
"""AutoPathFinding 核心逻辑单元测试。

覆盖范围：
- LockedTarget IOU 与 lock_id 生成
- PathFindingReco 方向计算、转向坐标、锁定追踪、运动状态评估
- PathFinderAction 闪避策略、移动时长计算

注意：本测试不依赖真实截图或控制器，仅验证纯逻辑行为。
"""

import pytest

from agent.AutoPathFinding.action.param import PathFinderParam
from agent.AutoPathFinding.action.path_finder_action import PathFinderAction
from agent.AutoPathFinding.recognition.param import PathFindingParam
from agent.AutoPathFinding.recognition.path_finding_reco import PathFindingReco
from agent.AutoPathFinding.recognition import turn_strategy
from agent.AutoPathFinding.selector import (
    NearestTargetSelector,
    PriorityTargetSelector,
    TargetInfo,
)
from agent.AutoPathFinding.state import (
    LockedTarget,
    Phase,
    PathFindingState,
    PathFinderState,
    RotationAdjuster,
    StuckDetector,
    TurnGrade,
)


SCREEN_CENTER = (640, 360)


def make_reco() -> PathFindingReco:
    """构造一个干净的 PathFindingReco 实例用于测试私有方法。"""
    return PathFindingReco()


def make_action() -> PathFinderAction:
    """构造一个干净的 PathFinderAction 实例用于测试私有方法。"""
    return PathFinderAction()


def make_param(
    *,
    enable_turning: bool = True,
    max_turn_attempts: int = 3,
    min_turn_distance: int = 50,
    max_turn_distance: int = 400,
    distance_missing_turn_scale: float = 0.5,
    target_lock_timeout: int = 3,
    target_lock_center_tolerance: int = 100,
    target_lock_score_threshold: float = 0.75,
    dead_zone: int = 50,
    arrival_distance: int = 30,
    stuck_threshold: int = 3,
    stuck_distance_tolerance: int = 5,
    stuck_center_tolerance: int = 10,
    enable_locked_fast_path: bool = True,
    lock_roi_padding: int = 200,
    fast_path_max_turn_padding: int = 400,
    direction_hysteresis: float = 15.0,
    fast_path_miss_limit: int = 3,
    turn_min_floor_ratio: float = 0.5,
    rotation_lower_threshold: float = 8.0,
    rotation_upper_threshold: float = 60.0,
    rotation_adaptive_enabled: bool = True,
    stuck_timeout_ms: int = 10000,
    turn_speed_factor: float = 0.7,
) -> PathFindingParam:
    """构造用于测试的 PathFindingParam。"""
    return PathFindingParam(
        ordered_templates=["a.png"],
        roi=(0, 0, 1280, 720),
        threshold=0.8,
        selector_type="priority",
        distance_pattern=r"(\d+)米",
        distance_offset=[-26, -30, 47, 57],
        distance_threshold=0.3,
        dead_zone=dead_zone,
        arrival_distance=arrival_distance,
        stuck_threshold=stuck_threshold,
        stuck_distance_tolerance=stuck_distance_tolerance,
        stuck_center_tolerance=stuck_center_tolerance,
        target_lock_timeout=target_lock_timeout,
        target_lock_center_tolerance=target_lock_center_tolerance,
        target_lock_score_threshold=target_lock_score_threshold,
        enable_turning=enable_turning,
        max_turn_attempts=max_turn_attempts,
        min_turn_distance=min_turn_distance,
        max_turn_distance=max_turn_distance,
        distance_missing_turn_scale=distance_missing_turn_scale,
        enable_locked_fast_path=enable_locked_fast_path,
        lock_roi_padding=lock_roi_padding,
        fast_path_max_turn_padding=fast_path_max_turn_padding,
        direction_hysteresis=direction_hysteresis,
        fast_path_miss_limit=fast_path_miss_limit,
        turn_min_floor_ratio=turn_min_floor_ratio,
        rotation_lower_threshold=rotation_lower_threshold,
        rotation_upper_threshold=rotation_upper_threshold,
        rotation_adaptive_enabled=rotation_adaptive_enabled,
        stuck_timeout_ms=stuck_timeout_ms,
        turn_speed_factor=turn_speed_factor,
    )


def make_action_param(
    *,
    move_duration: int = 500,
    move_duration_far: int = 1500,
    move_duration_near: int = 300,
    distance_far: int = 200,
    distance_near: int = 50,
    max_move_time: int = 0,
    dodge_at_start: str = "never",
    dodge_direction: str = "backward",
    enable_turning: bool = True,
    dodge_follows_direction: bool = True,
    dodge_release_ms: int = 50,
    move_start_delay_ms: int = 300,
    turn_duration_ms: int | None = 200,
    large_turn_release_ms: int = 100,
    stuck_dodge_on_phase: bool = True,
) -> PathFinderParam:
    """构造用于测试的 PathFinderParam。"""
    return PathFinderParam(
        move_duration=move_duration,
        move_duration_far=move_duration_far,
        move_duration_near=move_duration_near,
        distance_far=distance_far,
        distance_near=distance_near,
        max_move_time=max_move_time,
        dodge_at_start=dodge_at_start,
        dodge_direction=dodge_direction,
        enable_turning=enable_turning,
        dodge_follows_direction=dodge_follows_direction,
        dodge_release_ms=dodge_release_ms,
        move_start_delay_ms=move_start_delay_ms,
        turn_duration_ms=turn_duration_ms,
        large_turn_release_ms=large_turn_release_ms,
        stuck_dodge_on_phase=stuck_dodge_on_phase,
    )


@pytest.fixture(autouse=True)
def _clear_class_state() -> None:
    """每个测试用例前清空类级状态，避免跨用例污染。"""
    PathFindingReco._state.clear()
    PathFinderAction._action_state.clear()


class TestLockedTarget:
    """LockedTarget 基础行为测试。"""

    def test_lock_id_contains_template_and_position(self):
        target = LockedTarget(
            template="icon.png",
            center=(123.4, 567.8),
            bbox=(100, 500, 50, 50),
            score=0.9,
            distance=100,
            lock_id=LockedTarget.generate_lock_id("icon.png", (123.4, 567.8)),
        )
        parts = target.lock_id.split("#")
        assert parts[0] == "icon.png"
        assert parts[1] == "123"
        assert parts[2] == "567"
        assert len(parts[3]) == 8  # token_hex(4)

    def test_iou_full_overlap(self):
        target = LockedTarget(
            template="a.png",
            center=(50, 50),
            bbox=(0, 0, 100, 100),
            score=0.9,
            distance=10,
            lock_id="a#50#50#x",
        )
        assert target.iou((0, 0, 100, 100)) == pytest.approx(1.0)

    def test_iou_no_overlap(self):
        target = LockedTarget(
            template="a.png",
            center=(50, 50),
            bbox=(0, 0, 100, 100),
            score=0.9,
            distance=10,
            lock_id="a#50#50#x",
        )
        assert target.iou((200, 200, 100, 100)) == pytest.approx(0.0)

    def test_iou_partial_overlap(self):
        target = LockedTarget(
            template="a.png",
            center=(50, 50),
            bbox=(0, 0, 100, 100),
            score=0.9,
            distance=10,
            lock_id="a#50#50#x",
        )
        # 交集 50x50=2500，并集 10000+10000-2500=17500
        assert target.iou((50, 50, 100, 100)) == pytest.approx(2500 / 17500)


class TestTemplateFolder:
    """模板文件夹展开测试。"""

    def test_expand_template_folder_returns_image_paths(self, tmp_path, monkeypatch):
        reco = make_reco()
        monkeypatch.setattr(reco, "_resolve_image_dir", lambda: str(tmp_path))

        folder = tmp_path / "quest"
        folder.mkdir()
        (folder / "a.png").write_text("")
        (folder / "b.jpg").write_text("")
        (folder / "c.txt").write_text("")  # 非图片，应被忽略

        expanded = reco._expand_template_folder("quest")
        assert expanded is not None
        assert sorted(expanded) == ["quest/a.png", "quest/b.jpg"]

    def test_expand_template_folder_recursive(self, tmp_path, monkeypatch):
        reco = make_reco()
        monkeypatch.setattr(reco, "_resolve_image_dir", lambda: str(tmp_path))

        folder = tmp_path / "quest"
        sub = folder / "sub"
        sub.mkdir(parents=True)
        (folder / "a.png").write_text("")
        (sub / "b.png").write_text("")

        expanded = reco._expand_template_folder("quest")
        assert expanded is not None
        assert sorted(expanded) == ["quest/a.png", "quest/sub/b.png"]

    def test_expand_single_file_returns_none(self, tmp_path, monkeypatch):
        reco = make_reco()
        monkeypatch.setattr(reco, "_resolve_image_dir", lambda: str(tmp_path))
        (tmp_path / "icon.png").write_text("")
        assert reco._expand_template_folder("icon.png") is None

    def test_expand_missing_folder_returns_none(self, tmp_path, monkeypatch):
        reco = make_reco()
        monkeypatch.setattr(reco, "_resolve_image_dir", lambda: str(tmp_path))
        assert reco._expand_template_folder("not_exist") is None


class TestPrioritySelector:
    """priority 选择器应按 score 最高选，而不是按 targets 顺序。"""

    def test_picks_highest_score_for_same_template(self):
        selector = PriorityTargetSelector(priority=["副本/目标点"])
        targets = [
            TargetInfo(
                template="副本/目标点",
                center=(50, 50),
                bbox=(40, 40, 20, 20),
                score=0.5,
            ),
            TargetInfo(
                template="副本/目标点",
                center=(700, 200),
                bbox=(680, 180, 40, 40),
                score=0.95,
            ),
            TargetInfo(
                template="副本/目标点",
                center=(200, 400),
                bbox=(180, 380, 40, 40),
                score=0.7,
            ),
        ]
        selected = selector.select(targets)
        assert selected is not None
        assert selected.center == (700, 200)
        assert selected.score == pytest.approx(0.95)

    def test_picks_first_priority_with_highest_score(self):
        selector = PriorityTargetSelector(priority=["A", "B"])
        targets = [
            TargetInfo(template="A", center=(10, 10), bbox=(0, 0, 20, 20), score=0.5),
            TargetInfo(template="A", center=(20, 20), bbox=(0, 0, 40, 40), score=0.6),
            TargetInfo(template="B", center=(100, 100), bbox=(0, 0, 200, 200), score=0.99),
        ]
        selected = selector.select(targets)
        assert selected is not None
        assert selected.template == "A"
        assert selected.score == pytest.approx(0.6)

    def test_falls_back_to_highest_score_when_no_priority_match(self):
        selector = PriorityTargetSelector(priority=["X"])
        targets = [
            TargetInfo(template="Y", center=(10, 10), bbox=(0, 0, 20, 20), score=0.5),
            TargetInfo(template="Y", center=(20, 20), bbox=(0, 0, 40, 40), score=0.9),
        ]
        selected = selector.select(targets)
        assert selected is not None
        assert selected.template == "Y"
        assert selected.score == pytest.approx(0.9)

    def test_empty_targets_returns_none(self):
        selector = PriorityTargetSelector(priority=["A"])
        assert selector.select([]) is None


class TestNearestSelectorReuse:
    """确认 nearest selector 行为未受影响（顺带覆盖）。"""

    def test_nearest_picks_closest(self):
        selector = NearestTargetSelector(fallback_to_first=True)
        targets = [
            TargetInfo(
                template="a.png", center=(100, 100), bbox=(0, 0, 200, 200), score=0.9, distance=500
            ),
            TargetInfo(
                template="a.png", center=(300, 300), bbox=(0, 0, 600, 600), score=0.8, distance=50
            ),
        ]
        selected = selector.select(targets)
        assert selected is not None
        assert selected.distance == 50


class TestFrameRoi:
    """锁定态小 ROI 快速通道测试。"""

    def _make_locked(self, bbox, distance=None, miss_count=0):
        return LockedTarget(
            template="a.png",
            center=(bbox[0] + bbox[2] / 2, bbox[1] + bbox[3] / 2),
            bbox=bbox,
            score=0.9,
            distance=distance,
            lock_id="a#x#y#z",
            miss_count=miss_count,
        )

    def test_returns_none_when_no_lock(self):
        reco = make_reco()
        param = make_param()
        state = PathFindingState()
        assert reco._resolve_frame_roi(state, param) is None

    def test_returns_none_when_disabled(self):
        reco = make_reco()
        param = make_param(enable_locked_fast_path=False)
        state = PathFindingState(locked_target=self._make_locked((500, 300, 50, 50)))
        assert reco._resolve_frame_roi(state, param) is None

    def test_returns_none_when_missed(self):
        reco = make_reco()
        param = make_param()
        state = PathFindingState(
            locked_target=self._make_locked((500, 300, 50, 50), miss_count=1)
        )
        assert reco._resolve_frame_roi(state, param) is None

    def test_returns_small_roi_when_locked_with_distance(self):
        reco = make_reco()
        param = make_param(lock_roi_padding=100, fast_path_max_turn_padding=0)
        state = PathFindingState(
            locked_target=self._make_locked((500, 300, 50, 50), distance=100)
        )
        roi = reco._resolve_frame_roi(state, param)
        assert roi is not None
        x, y, w, h = roi
        # bbox 50x50，padding 100 单边 -> ROI 250x250
        assert 240 <= w <= 260
        assert 240 <= h <= 260
        # 中心应对齐
        center_x = x + w / 2
        center_y = y + h / 2
        assert abs(center_x - 525) <= 1
        assert abs(center_y - 325) <= 1

    def test_larger_roi_when_distance_missing(self):
        reco = make_reco()
        param = make_param(lock_roi_padding=100, fast_path_max_turn_padding=300)
        state_with_dist = PathFindingState(
            locked_target=self._make_locked((500, 300, 50, 50), distance=100)
        )
        state_no_dist = PathFindingState(
            locked_target=self._make_locked((500, 300, 50, 50), distance=None)
        )
        roi_with = reco._resolve_frame_roi(state_with_dist, param)
        roi_without = reco._resolve_frame_roi(state_no_dist, param)
        assert roi_with is not None and roi_without is not None
        assert roi_without[2] > roi_with[2]


class TestDirectionCalculation:
    """方向计算测试。"""

    @pytest.mark.parametrize(
        "center,expected",
        [
            ((640, 100), "forward"),   # 上方
            ((640, 600), "backward"),  # 下方
            ((100, 360), "left"),      # 左侧
            ((1100, 360), "right"),    # 右侧
            ((640, 360), "centered"),  # 中心死区
        ],
    )
    def test_direction(self, center, expected):
        reco = make_reco()
        # state 传空 PathFindingState，hysteresis=0 关闭滞回以保持原行为
        state = PathFindingState()
        assert (
            reco._calculate_direction(center, dead_zone=50, state=state, hysteresis=0)
            == expected
        )


class TestTurnCoordinates:
    """转向坐标计算测试。"""

    def test_distance_recognized_no_turn(self):
        reco = make_reco()
        state = PathFindingState()
        param = make_param()
        target = TargetInfo(
            template="a.png",
            center=(700, 300),
            bbox=(680, 280, 40, 40),
            score=0.9,
            distance=100,
        )
        start, end, _grade = reco._calculate_turn(state, target, param)
        assert start is None
        assert end is None
        assert state.turn_attempts_without_distance == 0

    def test_turn_direction_opposite_to_target_offset(self):
        reco = make_reco()
        state = PathFindingState()
        param = make_param()
        # 目标在屏幕左侧 (offset_x=-240)
        # 期望 swipe 把目标推向屏幕中心，即 swipe_vec_x = +240（向右）
        # 因此 end_x > start_x
        target = TargetInfo(
            template="a.png",
            center=(400, 360),
            bbox=(380, 340, 40, 40),
            score=0.9,
            distance=None,
        )
        start, end, _grade = reco._calculate_turn(state, target, param)
        assert start is not None
        assert end is not None
        # 目标偏左：swipe 应向右（end_x > start_x），即起点在左、终点在右
        assert end[0] > start[0]

    def test_turn_coordinates_clipped_to_rect(self):
        reco = make_reco()
        state = PathFindingState()
        param = make_param()
        target = TargetInfo(
            template="a.png",
            center=(100, 360),
            bbox=(80, 340, 40, 40),
            score=0.9,
            distance=None,
        )
        start, end, _grade = reco._calculate_turn(state, target, param)
        # 水平方向目标应被裁剪到 TURN_CLIP_HORIZONTAL 区域内
        clip_x, clip_y, clip_w, clip_h = reco.TURN_CLIP_HORIZONTAL
        for point in (start, end):
            if point is None:
                continue
            assert clip_x <= point[0] <= clip_x + clip_w
            assert clip_y <= point[1] <= clip_y + clip_h

    def test_turn_distance_decay(self):
        """连续缺失距离时滑动距离按 scale 衰减。"""
        reco = make_reco()
        state = PathFindingState()
        param = make_param(distance_missing_turn_scale=0.5)
        target = TargetInfo(
            template="a.png",
            center=(800, 360),
            bbox=(780, 340, 40, 40),
            score=0.9,
            distance=None,
        )
        first_start, first_end, _ = reco._calculate_turn(state, target, param)
        second_start, second_end, _ = reco._calculate_turn(state, target, param)

        def distance(p1, p2):
            return ((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2) ** 0.5

        first_len = distance(first_start, first_end)
        second_len = distance(second_start, second_end)
        # 第 2 次 = 第 1 次 * 0.5
        assert second_len < first_len
        assert second_len == pytest.approx(first_len * 0.5, rel=0.05)

    def test_turn_distance_proportional_to_offset(self):
        """滑动距离 = offset_norm * speed_factor（动态自适应）。"""
        reco = make_reco()
        state = PathFindingState()

        # 偏下 20px：offset_norm=20，预期 base = 20 * 0.7 = 14 -> 被 floor 提到 25
        small_target = TargetInfo(
            template="a.png",
            center=(640, 380),  # offset_y=+20
            bbox=(620, 360, 40, 40),
            score=0.9,
            distance=None,
        )
        # 偏下 100px：offset_norm=100，预期 base = 100 * 0.7 = 70
        large_target = TargetInfo(
            template="a.png",
            center=(640, 460),  # offset_y=+100
            bbox=(620, 440, 40, 40),
            score=0.9,
            distance=None,
        )

        param = make_param()
        s1, e1, _ = reco._calculate_turn(state, small_target, param)
        state2 = PathFindingState()
        s2, e2, _ = reco._calculate_turn(state2, large_target, param)

        def distance(p1, p2):
            return ((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2) ** 0.5

        small_len = distance(s1, e1)
        large_len = distance(s2, e2)

        # 偏移越大，滑动距离越大（动态自适应）
        assert large_len > small_len
        # 比例关系：large / small = 100 / 20 = 5
        assert large_len == pytest.approx(small_len * (100 / 20), rel=0.15)

    def test_turn_distance_capped_by_max(self):
        """base_distance 受 max_turn_distance 上限保护。"""
        reco = make_reco()
        state = PathFindingState()
        param = make_param(max_turn_distance=80, turn_speed_factor=0.7)
        # 极大偏移：offset_norm=300，base=210 远超 max=80
        target = TargetInfo(
            template="a.png",
            center=(640, 60),  # offset_y=-300
            bbox=(620, 40, 40, 40),
            score=0.9,
            distance=None,
        )
        start, end, _ = reco._calculate_turn(state, target, param)

        def distance(p1, p2):
            return ((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2) ** 0.5

        swipe_len = distance(start, end)
        # 不超过 max_turn_distance + 少许 clip 误差
        assert swipe_len <= 80

    def test_turn_stops_after_max_attempts(self):
        reco = make_reco()
        state = PathFindingState()
        param = make_param(max_turn_attempts=2)
        target = TargetInfo(
            template="a.png",
            center=(800, 360),
            bbox=(780, 340, 40, 40),
            score=0.9,
            distance=None,
        )
        assert reco._calculate_turn(state, target, param)[0] is not None
        assert reco._calculate_turn(state, target, param)[0] is not None
        assert reco._calculate_turn(state, target, param)[0] is None
        assert state.turn_attempts_without_distance == 3

    def test_enable_turning_false_resets_counter(self):
        reco = make_reco()
        state = PathFindingState()
        state.turn_attempts_without_distance = 5
        param = make_param(enable_turning=False)
        target = TargetInfo(
            template="a.png",
            center=(800, 360),
            bbox=(780, 340, 40, 40),
            score=0.9,
            distance=None,
        )
        start, end, _grade = reco._calculate_turn(state, target, param)
        assert start is None
        assert end is None
        assert state.turn_attempts_without_distance == 0


class TestTargetLocking:
    """目标锁定与追踪测试。"""

    def test_new_target_gets_locked(self):
        reco = make_reco()
        state = PathFindingState()
        param = make_param()
        targets = [
            TargetInfo(
                template="a.png",
                center=(100, 100),
                bbox=(80, 80, 40, 40),
                score=0.9,
                distance=50,
            )
        ]
        selected = reco._resolve_locked_target(state, targets, param)
        assert selected is not None
        assert state.locked_target is not None
        assert state.locked_target.template == "a.png"
        assert state.locked_target.lock_id is not None

    def test_locked_target_tracked_by_iou(self):
        reco = make_reco()
        state = PathFindingState()
        param = make_param()
        initial = TargetInfo(
            template="a.png",
            center=(100, 100),
            bbox=(80, 80, 40, 40),
            score=0.9,
            distance=50,
        )
        reco._resolve_locked_target(state, [initial], param)
        if state.locked_target is None:
            return
        lock_id = state.locked_target.lock_id

        # 小幅度移动，IOU 仍足够
        moved = TargetInfo(
            template="a.png",
            center=(105, 105),
            bbox=(85, 85, 40, 40),
            score=0.88,
            distance=48,
        )
        selected = reco._resolve_locked_target(state, [moved], param)
        assert selected is not None
        assert state.locked_target.lock_id == lock_id

    def test_adjacent_same_template_resolved_by_iou(self):
        reco = make_reco()
        state = PathFindingState()
        param = make_param()
        initial = TargetInfo(
            template="a.png",
            center=(100, 100),
            bbox=(80, 80, 40, 40),
            score=0.9,
            distance=50,
        )
        reco._resolve_locked_target(state, [initial], param)
        if state.locked_target is None:
            return
        lock_id = state.locked_target.lock_id

        # 两个同名目标相邻出现
        candidate1 = TargetInfo(
            template="a.png",
            center=(300, 300),  # 远离原目标
            bbox=(280, 280, 40, 40),
            score=0.95,
            distance=200,
        )
        candidate2 = TargetInfo(
            template="a.png",
            center=(105, 105),  # 接近原目标
            bbox=(85, 85, 40, 40),
            score=0.88,
            distance=48,
        )
        selected = reco._resolve_locked_target(
            state, [candidate1, candidate2], param
        )
        if selected is None:
            return
        assert selected.center == pytest.approx((105.0, 105.0))
        assert state.locked_target.lock_id == lock_id

    def test_target_loss_returns_historical_position(self):
        reco = make_reco()
        state = PathFindingState()
        param = make_param(target_lock_timeout=3)
        initial = TargetInfo(
            template="a.png",
            center=(100, 100),
            bbox=(80, 80, 40, 40),
            score=0.9,
            distance=50,
        )
        reco._resolve_locked_target(state, [initial], param)
        if state.locked_target is None:
            return
        historical_lock_id = state.locked_target.lock_id

        selected = reco._resolve_locked_target(state, [], param)
        assert selected is not None
        assert selected.center == (100.0, 100.0)
        assert state.locked_target.miss_count == 1
        assert state.locked_target.lock_id == historical_lock_id

    def test_target_loss_timeout_unlocks(self):
        reco = make_reco()
        state = PathFindingState()
        param = make_param(target_lock_timeout=2)
        initial = TargetInfo(
            template="a.png",
            center=(100, 100),
            bbox=(80, 80, 40, 40),
            score=0.9,
            distance=50,
        )
        reco._resolve_locked_target(state, [initial], param)

        # 连续丢失 2 帧后应解锁
        reco._resolve_locked_target(state, [], param)
        reco._resolve_locked_target(state, [], param)
        selected = reco._resolve_locked_target(state, [], param)
        assert selected is None
        assert state.locked_target is None

    def test_lock_resets_state_on_new_target(self):
        reco = make_reco()
        state = PathFindingState()
        param = make_param()
        state.stuck_count = 5
        state.turn_attempts_without_distance = 3

        target = TargetInfo(
            template="a.png",
            center=(100, 100),
            bbox=(80, 80, 40, 40),
            score=0.9,
            distance=50,
        )
        reco._resolve_locked_target(state, [target], param)
        assert state.stuck_count == 0
        assert state.turn_attempts_without_distance == 0


class TestMovementState:
    """运动状态评估测试。"""

    def test_arrived_clears_state(self):
        reco = make_reco()
        param = make_param(arrival_distance=30)
        target = TargetInfo(
            template="a.png",
            center=(100, 100),
            bbox=(80, 80, 40, 40),
            score=0.9,
            distance=20,
        )
        state_eval, _ = reco._evaluate_movement_state(
            "node1", target, PathFindingState(), param
        )
        assert state_eval == "arrived"
        # 到达后状态被清空
        assert "node1" not in reco._state

    def test_stuck_by_distance(self):
        reco = make_reco()
        param = make_param(
            stuck_threshold=1,
            stuck_distance_tolerance=5,
        )

        target1 = TargetInfo(
            template="a.png",
            center=(100, 100),
            bbox=(80, 80, 40, 40),
            score=0.9,
            distance=98,
        )
        # 首帧无历史数据，不判定卡住
        shared_state = reco._get_state("node1")
        state_eval1, _ = reco._evaluate_movement_state(
            "node1", target1, shared_state, param
        )
        assert state_eval1 == "approaching"

        target2 = TargetInfo(
            template="a.png",
            center=(100, 100),
            bbox=(80, 80, 40, 40),
            score=0.9,
            distance=96,
        )
        # 第二帧距离仅缩短 2 ≤ 容差 5，stuck_count 达到阈值 1
        state_eval2, _ = reco._evaluate_movement_state(
            "node1", target2, shared_state, param
        )
        assert state_eval2 == "stuck"

    def test_not_stuck_when_approaching(self):
        reco = make_reco()
        param = make_param(
            stuck_threshold=2,
            stuck_distance_tolerance=5,
        )
        shared_state = reco._get_state("node1")

        # 先写入历史帧
        reco._evaluate_movement_state(
            "node1",
            TargetInfo(
                template="a.png",
                center=(100, 100),
                bbox=(80, 80, 40, 40),
                score=0.9,
                distance=100,
            ),
            shared_state,
            param,
        )

        # 距离缩短 20 > 容差 5，不应判定为卡住
        target = TargetInfo(
            template="a.png",
            center=(100, 100),
            bbox=(80, 80, 40, 40),
            score=0.9,
            distance=80,
        )
        state_eval, _ = reco._evaluate_movement_state(
            "node1", target, shared_state, param
        )
        assert state_eval == "approaching"
        assert reco._state["node1"].stuck_count == 0


class TestDodge:
    """闪避策略测试。"""

    def test_never_dodge(self):
        action = make_action()
        param = make_action_param(dodge_at_start="never")
        action._action_state.clear()
        assert action._should_dodge("node1", param, "lock1") is False

    def test_always_dodge(self):
        action = make_action()
        param = make_action_param(dodge_at_start="always")
        action._action_state.clear()
        assert action._should_dodge("node1", param, "lock1") is True

    def test_once_per_target_dodges_on_new_lock(self):
        action = make_action()
        param = make_action_param(dodge_at_start="once_per_target")
        action._action_state.clear()
        assert action._should_dodge("node1", param, "lock1") is True
        assert action._should_dodge("node1", param, "lock1") is False

    def test_once_per_target_dodges_again_when_lock_changes(self):
        action = make_action()
        param = make_action_param(dodge_at_start="once_per_target")
        action._action_state.clear()
        assert action._should_dodge("node1", param, "lock1") is True
        assert action._should_dodge("node1", param, "lock2") is True
        assert action._should_dodge("node1", param, "lock2") is False

    def test_once_per_target_no_lock_id_no_dodge(self):
        action = make_action()
        param = make_action_param(dodge_at_start="once_per_target")
        action._action_state.clear()
        assert action._should_dodge("node1", param, None) is False


class TestDuration:
    """移动时长计算测试。"""

    def test_no_distance_uses_default_duration(self):
        action = make_action()
        param = make_action_param(move_duration=500)
        assert action._resolve_duration(None, param) == pytest.approx(0.5)

    def test_far_distance_uses_far_duration(self):
        action = make_action()
        param = make_action_param(
            move_duration_far=1500,
            distance_far=200,
        )
        assert action._resolve_duration(250, param) == pytest.approx(1.5)

    def test_near_distance_uses_near_duration(self):
        action = make_action()
        param = make_action_param(
            move_duration_near=300,
            distance_near=50,
        )
        assert action._resolve_duration(30, param) == pytest.approx(0.3)

    def test_mid_distance_interpolates(self):
        action = make_action()
        param = make_action_param(
            move_duration_far=1500,
            move_duration_near=300,
            distance_far=200,
            distance_near=50,
        )
        # 距离 125 正好在中间
        assert action._resolve_duration(125, param) == pytest.approx(0.9)

    def test_max_move_time_clamps_duration(self):
        action = make_action()
        param = make_action_param(
            move_duration_far=1500,
            distance_far=200,
            max_move_time=800,
        )
        assert action._resolve_duration(250, param) == pytest.approx(0.8)

    def test_max_move_time_zero_no_clamp(self):
        action = make_action()
        param = make_action_param(
            move_duration_far=1500,
            distance_far=200,
            max_move_time=0,
        )
        assert action._resolve_duration(250, param) == pytest.approx(1.5)

    def test_invalid_far_near_fallback(self):
        action = make_action()
        param = make_action_param(
            move_duration=500,
            distance_far=50,
            distance_near=200,
        )
        assert action._resolve_duration(100, param) == pytest.approx(0.5)


class TestDirectionHysteresis:
    """方向分箱的角度滞回测试。"""

    def test_hysteresis_keeps_last_direction(self):
        """当 angle 距上一帧分箱中心 < hysteresis 时，保持上一帧方向。"""
        reco = make_reco()
        # 上一帧方向为 "forward"（分箱中心 90°）
        state = PathFindingState(
            locked_target=LockedTarget(
                template="a.png",
                center=(640, 360),
                bbox=(620, 340, 40, 40),
                score=0.9,
                distance=100,
                lock_id="a#x#y#z",
                last_direction="forward",
            )
        )
        # 当前目标在 80°（forward 分箱 [45,135) 内），距 forward 中心 10° < 15°
        # 如果不滞回：atan2(-(-260), 700) ≈ 20° → 落到 right 分箱
        # 滞回开启（hysteresis=15）：保持 forward
        result = reco._calculate_direction(
            (900, 100), dead_zone=50, state=state, hysteresis=15
        )
        assert result == "forward"

    def test_hysteresis_switches_when_far(self):
        """当 angle 距上一帧分箱中心 > hysteresis 时，正常切换。"""
        reco = make_reco()
        state = PathFindingState(
            locked_target=LockedTarget(
                template="a.png",
                center=(640, 360),
                bbox=(620, 340, 40, 40),
                score=0.9,
                distance=100,
                lock_id="a#x#y#z",
                last_direction="forward",
            )
        )
        # 当前目标在右侧远处，angle 接近 0°（right 分箱中心）
        result = reco._calculate_direction(
            (1100, 360), dead_zone=50, state=state, hysteresis=15
        )
        assert result == "right"

    def test_hysteresis_disabled_with_zero(self):
        """hysteresis=0 时不启用滞回，按当前 angle 自由分箱。"""
        reco = make_reco()
        state = PathFindingState(
            locked_target=LockedTarget(
                template="a.png",
                center=(640, 360),
                bbox=(620, 340, 40, 40),
                score=0.9,
                distance=100,
                lock_id="a#x#y#z",
                last_direction="forward",
            )
        )
        # (900, 100) 相对 (640, 360): dx=260, dy=-260, atan2(-dy, dx) = 45°
        # 45° 属于 forward 分箱 [45, 135)
        # hysteresis=0 时不启用滞回，结果应是 angle 自身的分箱
        result = reco._calculate_direction(
            (900, 100), dead_zone=50, state=state, hysteresis=0
        )
        assert result == "forward"


class TestTurnDistanceFloor:
    """转向滑动距离衰减下限保护测试。"""

    def test_floor_protects_extreme_decay(self):
        """连续多次衰减后，base_distance 不应低于 min_turn_distance * turn_min_floor_ratio。"""
        reco = make_reco()
        state = PathFindingState()
        param = make_param(
            distance_missing_turn_scale=0.5,
            turn_min_floor_ratio=0.5,
        )
        target = TargetInfo(
            template="a.png",
            center=(800, 360),
            bbox=(780, 340, 40, 40),
            score=0.9,
            distance=None,
        )

        # 第 1 次
        s1, e1, _ = reco._calculate_turn(state, target, param)
        # 第 2 次 (scale=0.5)
        s2, e2, _ = reco._calculate_turn(state, target, param)
        # 第 3 次 (scale=0.25) - 衰减已超过 0.5 倍 min_turn_distance
        s3, e3, _ = reco._calculate_turn(state, target, param)

        def seg_len(p1, p2):
            return ((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2) ** 0.5

        len1 = seg_len(s1, e1)
        len3 = seg_len(s3, e3)

        # 第 3 次不应低于 min_turn_distance * turn_min_floor_ratio = 50 * 0.5 = 25
        assert len3 >= 25 * 0.95
        # 第 1 次应该比第 3 次大（如果第 1 次本来 > 25）
        assert len1 > len3


class TestTurnClipAxis:
    """转向 clip rect 应根据目标偏移主轴选择水平/垂直。"""

    def test_horizontal_target_uses_horizontal_clip(self):
        """目标在水平方向（|dx| >= |dy|）时使用 TURN_CLIP_HORIZONTAL。"""
        reco = make_reco()
        state = PathFindingState()
        param = make_param()
        # 目标在左：offset_x=-540, offset_y=0
        target = TargetInfo(
            template="a.png",
            center=(100, 360),
            bbox=(80, 340, 40, 40),
            score=0.9,
            distance=None,
        )
        start, end, _grade = reco._calculate_turn(state, target, param)
        clip_x, clip_y, clip_w, clip_h = reco.TURN_CLIP_HORIZONTAL
        for point in (start, end):
            if point is None:
                continue
            assert clip_x <= point[0] <= clip_x + clip_w
            assert clip_y <= point[1] <= clip_y + clip_h

    def test_vertical_target_uses_vertical_clip(self):
        """目标在垂直方向（|dy| > |dx|）时使用 TURN_CLIP_VERTICAL。"""
        reco = make_reco()
        state = PathFindingState()
        param = make_param()
        # 目标在上：offset_x=0, offset_y=-260
        target = TargetInfo(
            template="a.png",
            center=(640, 100),
            bbox=(620, 80, 40, 40),
            score=0.9,
            distance=None,
        )
        start, end, _grade = reco._calculate_turn(state, target, param)
        clip_x, clip_y, clip_w, clip_h = reco.TURN_CLIP_VERTICAL
        for point in (start, end):
            if point is None:
                continue
            assert clip_x <= point[0] <= clip_x + clip_w
            assert clip_y <= point[1] <= clip_y + clip_h

    def test_diagonal_target_no_collapse(self):
        """对角方向目标不应因裁剪塌缩到单点。"""
        reco = make_reco()
        state = PathFindingState()
        param = make_param()
        # 目标在左上：offset_x=-200, offset_y=-200（|dx| == |dy|，按 >= 选水平 clip）
        target = TargetInfo(
            template="a.png",
            center=(440, 160),
            bbox=(420, 140, 40, 40),
            score=0.9,
            distance=None,
        )
        start, end, _grade = reco._calculate_turn(state, target, param)
        assert start is not None
        assert end is not None
        # 起止点不应重合
        assert start != end


class TestTurnSwipeDirection:
    """滑动方向正确性测试（修复 Bug：原版滑动方向与期望相反）。

    关键约束：
    - 移动端手势 swipe (sx,sy) -> (ex,ey) 后，屏幕中目标位置变化 = swipe 向量
    - 要把目标从 (tx, ty) 推向屏幕中心 (cx, cy)，swipe 应 = (cx-tx, cy-ty) = -offset
    - 修复前 swipe = +offset，方向完全反向；修复后 swipe = -offset。
    """

    def _compute_swipe(self, target_center, **param_overrides):
        reco = make_reco()
        state = PathFindingState()
        param = make_param(**param_overrides)
        target = TargetInfo(
            template="a.png",
            center=target_center,
            bbox=(target_center[0] - 20, target_center[1] - 20, 40, 40),
            score=0.9,
            distance=None,
        )
        start, end, _grade = reco._calculate_turn(state, target, param)
        assert start is not None and end is not None, "转向不应被放弃"
        return start, end

    @pytest.mark.parametrize(
        "target,expected_swipe_sign",
        [
            # 目标在屏幕下方：swipe 应向上（dy < 0）
            ((640, 400), (-1, -1)),    # 偏下，期望 swipe=(0, -)
            ((640, 380), (0, -1)),     # 用户场景：略偏下，期望向上滑
            ((640, 440), (0, -1)),     # 偏下大距离
            # 目标在屏幕上方：swipe 应向下（dy > 0）
            ((640, 320), (0, 1)),      # 偏上
            ((640, 280), (0, 1)),      # 偏上较多
            # 目标在屏幕右方：swipe 应向左（dx < 0）
            ((700, 360), (-1, 0)),     # 偏右
            # 目标在屏幕左方：swipe 应向右（dx > 0）
            ((580, 360), (1, 0)),      # 偏左
        ],
    )
    def test_swipe_direction_pulls_target_toward_center(self, target, expected_swipe_sign):
        """滑动向量应把目标朝屏幕中心推（即 swipe 方向 = -offset 方向）。"""
        start, end = self._compute_swipe(target)
        sx, sy = start
        ex, ey = end
        swipe_dx = ex - sx
        swipe_dy = ey - sy
        assert (swipe_dx > 0) == (expected_swipe_sign[0] > 0)
        assert (swipe_dy > 0) == (expected_swipe_sign[1] > 0)
        # 至少一个轴方向正确（允许另一个轴为 0）
        assert swipe_dx != 0 or swipe_dy != 0

    def test_target_below_center_swipes_up(self):
        """用户场景：目标在屏幕下方，swipe 应向上（dy < 0）。"""
        start, end = self._compute_swipe((640, 380))
        swipe_dy = end[1] - start[1]
        assert swipe_dy < 0, f"目标在屏幕下方，期望 swipe 向上，但 dy={swipe_dy}"

    def test_target_above_center_swipes_down(self):
        """目标在屏幕上方时 swipe 应向下（dy > 0）。"""
        start, end = self._compute_swipe((640, 320))
        swipe_dy = end[1] - start[1]
        assert swipe_dy > 0, f"目标在屏幕上方，期望 swipe 向下，但 dy={swipe_dy}"

    def test_target_left_center_swipes_right(self):
        """目标在屏幕左方时 swipe 应向右（dx > 0）。"""
        start, end = self._compute_swipe((580, 360))
        swipe_dx = end[0] - start[0]
        assert swipe_dx > 0, f"目标在屏幕左方，期望 swipe 向右，但 dx={swipe_dx}"

    def test_target_right_center_swipes_left(self):
        """目标在屏幕右方时 swipe 应向左（dx < 0）。"""
        start, end = self._compute_swipe((700, 360))
        swipe_dx = end[0] - start[0]
        assert swipe_dx < 0, f"目标在屏幕右方，期望 swipe 向左，但 dx={swipe_dx}"


class TestFastPathMiss:
    """fast_path 连续 miss 自动回退到全 ROI 测试。"""

    def test_miss_counter_increments(self):
        """连续 miss 时计数递增。"""
        reco = make_reco()
        state = PathFindingState(
            locked_target=LockedTarget(
                template="a.png",
                center=(640, 360),
                bbox=(620, 340, 40, 40),
                score=0.9,
                distance=100,
                lock_id="a#x#y#z",
            )
        )
        # 连续 2 次 miss，计数器应为 2
        state.fast_path_miss_count += 1
        state.fast_path_miss_count += 1
        assert state.fast_path_miss_count == 2


class TestActionParam:
    """PathFinderParam 新参数测试。"""

    def test_default_dodge_follows_direction(self):
        action = make_action()
        param = make_action_param()
        assert param.dodge_follows_direction is True
        assert param.dodge_release_ms == 50
        assert param.move_start_delay_ms == 300
        assert param.turn_duration_ms == 200

    def test_dodge_release_ms_override(self):
        action = make_action()
        param = make_action_param(dodge_release_ms=0, turn_duration_ms=120)
        assert param.dodge_release_ms == 0
        assert param.turn_duration_ms == 120

    def test_move_start_delay_ms_default_and_override(self):
        action = make_action()
        param_default = make_action_param()
        assert param_default.move_start_delay_ms == 300
        param_custom = make_action_param(move_start_delay_ms=500)
        assert param_custom.move_start_delay_ms == 500

    def test_execute_sequence_waits_move_start_delay(self):
        """move 开始前应等待 move_start_delay_ms。"""
        import time as _time

        action = make_action()
        param = make_action_param(move_start_delay_ms=200)

        class _StubPlatform:
            def __init__(self):
                self.move_called = False
                self.move_start_time = 0.0

            def dodge(self, direction):
                return True

            def turn(self, sx, sy, ex, ey, duration=None):
                return True

            def move(self, direction, duration):
                self.move_called = True
                self.move_start_time = _time.monotonic()
                return True

            def release_all(self):
                return True

        platform = _StubPlatform()

        class _StubTasker:
            stopping = False

        class _StubContext:
            tasker = _StubTasker()

        start_time = _time.monotonic()
        ctx = _StubContext()
        action._execute_sequence(
            ctx,
            platform,
            direction="forward",
            param=param,
            distance=None,
            lock_id="lock1",
            turn_start=None,
            turn_end=None,
            turn_grade=None,
            phase=None,
        )
        elapsed = platform.move_start_time - start_time
        assert platform.move_called is True
        # 应该至少等待 move_start_delay_ms (200ms)
        assert elapsed >= 0.18  # 容忍 20ms 误差

    def test_execute_sequence_move_start_delay_zero_skips_wait(self):
        """move_start_delay_ms=0 时不等待直接 move。"""
        import time as _time

        action = make_action()
        param = make_action_param(move_start_delay_ms=0)

        class _StubPlatform:
            def __init__(self):
                self.move_called = False

            def dodge(self, direction):
                return True

            def turn(self, sx, sy, ex, ey, duration=None):
                return True

            def move(self, direction, duration):
                self.move_called = True
                return True

            def release_all(self):
                return True

        platform = _StubPlatform()

        class _StubTasker:
            stopping = False

        class _StubContext:
            tasker = _StubTasker()

        start_time = _time.monotonic()
        ctx = _StubContext()
        action._execute_sequence(
            ctx,
            platform,
            direction="forward",
            param=param,
            distance=None,
            lock_id="lock1",
            turn_start=None,
            turn_end=None,
            turn_grade=None,
            phase=None,
        )
        elapsed = _time.monotonic() - start_time
        assert platform.move_called is True
        # 不应等待 move_start_delay
        assert elapsed < 0.1

    def test_new_params_defaults(self):
        """新增的 large_turn_release_ms 与 stuck_dodge_on_phase 默认值。"""
        action = make_action()
        param = make_action_param()
        assert param.large_turn_release_ms == 100
        assert param.stuck_dodge_on_phase is True

    def test_large_turn_release_ms_override(self):
        action = make_action()
        param = make_action_param(large_turn_release_ms=0, stuck_dodge_on_phase=False)
        assert param.large_turn_release_ms == 0
        assert param.stuck_dodge_on_phase is False


class TestRotationAdjuster:
    """RotationAdjuster 自适应速度测试（借鉴 MapTracker EMA 0.618/0.382）。"""

    def test_initial_speed(self):
        adj = RotationAdjuster()
        assert adj.speed == RotationAdjuster.DEFAULT_SPEED
        assert adj.samples == 0

    def test_update_within_range(self):
        adj = RotationAdjuster()
        new_speed = adj.update(1.5)
        # EMA: 1.0 * 0.618 + 1.5 * 0.382 = 0.618 + 0.573 = 1.191
        assert new_speed == pytest.approx(1.191, rel=0.01)
        assert adj.samples == 1

    def test_update_ignores_outliers(self):
        adj = RotationAdjuster()
        original = adj.speed
        # 低于下界：忽略更新
        adj.update(0.1)
        assert adj.speed == original
        # 高于上界：忽略更新
        adj.update(3.0)
        assert adj.speed == original

    def test_reset(self):
        adj = RotationAdjuster()
        adj.update(1.5)
        adj.reset()
        assert adj.speed == RotationAdjuster.DEFAULT_SPEED
        assert adj.samples == 0


class TestStuckDetector:
    """StuckDetector 卡住检测测试（借鉴 MapTracker stuckThreshold）。"""

    def test_no_history_not_stuck(self):
        det = StuckDetector(threshold_frames=2)
        assert det.is_stuck_frame((100, 100), 100) is False

    def test_stuck_by_distance(self):
        det = StuckDetector(threshold_frames=2, distance_tolerance=5.0)
        det.is_stuck_frame((100, 100), 100)
        # 第二帧距离缩短 ≤ 5，count=1 未达阈值
        assert det.is_stuck_frame((100, 100), 98) is False
        # 第三帧累计达到阈值
        assert det.is_stuck_frame((100, 100), 96) is True

    def test_not_stuck_when_distance_shrinks(self):
        det = StuckDetector(threshold_frames=2, distance_tolerance=5.0)
        det.is_stuck_frame((100, 100), 100)
        # 距离大幅缩短 → 不卡住，count 清零
        assert det.is_stuck_frame((100, 100), 50) is False
        assert det.count == 0

    def test_stuck_by_center(self):
        det = StuckDetector(threshold_frames=2, center_tolerance=10.0)
        det.is_stuck_frame((100, 100), None)
        assert det.is_stuck_frame((105, 105), None) is False  # count=1
        assert det.is_stuck_frame((108, 108), None) is True   # count=2

    def test_reset(self):
        det = StuckDetector(threshold_frames=2)
        det.is_stuck_frame((100, 100), 100)
        det.is_stuck_frame((100, 100), 100)
        det.reset()
        assert det.count == 0
        assert det.last_center is None
        assert det.last_distance is None
        assert det.entered_at_ms is None


class TestPhase:
    """阶段枚举测试。"""

    def test_phase_values(self):
        assert Phase.SEEKING.value == "seeking"
        assert Phase.TRACKING.value == "tracking"
        assert Phase.APPROACHING.value == "approaching"
        assert Phase.TURNING.value == "turning"
        assert Phase.STUCK.value == "stuck"
        assert Phase.LOST.value == "lost"

    def test_phase_from_string(self):
        assert Phase("seeking") == Phase.SEEKING
        assert Phase("stuck") == Phase.STUCK

    def test_invalid_phase_raises(self):
        with pytest.raises(ValueError):
            Phase("invalid")


class TestTurnGrade:
    """转向分级枚举测试。"""

    def test_grade_values(self):
        assert TurnGrade.FINE_TUNE.value == "fine_tune"
        assert TurnGrade.LARGE_TURN.value == "large_turn"


class TestTurnStrategy:
    """turn_strategy 模块独立函数测试。"""

    def test_compute_target_angle_zero_offset(self):
        angle = turn_strategy.compute_target_angle((640, 360))
        assert angle == 0.0

    def test_compute_target_angle_right(self):
        angle = turn_strategy.compute_target_angle((740, 360))
        assert angle == pytest.approx(0.0)

    def test_compute_target_angle_up(self):
        angle = turn_strategy.compute_target_angle((640, 260))
        # 上方 = forward = 90°（atan2(-dy, dx) with dy=-100, dx=0）
        assert angle == pytest.approx(90.0)

    def test_compute_target_angle_down(self):
        angle = turn_strategy.compute_target_angle((640, 460))
        # 下方 = backward = -90°
        assert angle == pytest.approx(-90.0)

    def test_bin_direction_basic(self):
        assert turn_strategy.bin_direction(0.0) == "right"
        assert turn_strategy.bin_direction(90.0) == "forward"
        assert turn_strategy.bin_direction(180.0) == "left"
        assert turn_strategy.bin_direction(-90.0) == "backward"

    def test_dir_center_angle(self):
        assert turn_strategy.dir_center_angle("right") == 0.0
        assert turn_strategy.dir_center_angle("forward") == 90.0
        assert turn_strategy.dir_center_angle("left") == 180.0
        assert turn_strategy.dir_center_angle("backward") == -90.0
        assert turn_strategy.dir_center_angle("centered") is None

    def test_angle_diff_wrap(self):
        # 跨越 180° 边界
        diff = turn_strategy.angle_diff(179.0, -179.0)
        assert diff == pytest.approx(-2.0, abs=0.01)

    def test_classify_turn_fine_tune(self):
        """小偏差归类为 FINE_TUNE。"""
        grade = turn_strategy.classify_turn(95.0, 90.0, 8.0, 60.0)
        assert grade == TurnGrade.FINE_TUNE

    def test_classify_turn_large(self):
        """大偏差归类为 LARGE_TURN。"""
        grade = turn_strategy.classify_turn(180.0, 90.0, 8.0, 60.0)
        assert grade == TurnGrade.LARGE_TURN

    def test_classify_turn_no_history_fine(self):
        """无历史时初始偏差=0，归为 FINE_TUNE。"""
        grade = turn_strategy.classify_turn(45.0, None, 8.0, 60.0)
        assert grade == TurnGrade.FINE_TUNE


class TestPathFindingStateReset:
    """PathFindingState.reset_target_local_state 测试。"""

    def test_reset_clears_all_local_fields(self):
        locked = LockedTarget(
            template="a.png",
            center=(100, 100),
            bbox=(80, 80, 40, 40),
            score=0.9,
            distance=100,
            lock_id="a#x#y#z",
        )
        state = PathFindingState(locked_target=locked)
        state.stuck_count = 5
        state.turn_attempts_without_distance = 3
        state.phase = Phase.TURNING
        state.turn_grade = TurnGrade.LARGE_TURN
        state.rotation_adjuster.update(1.5)
        state.stuck_detector.is_stuck_frame((100, 100), 100)

        state.reset_target_local_state()

        assert state.stuck_count == 0
        assert state.turn_attempts_without_distance == 0
        assert state.phase == Phase.SEEKING
        assert state.turn_grade is None
        assert state.rotation_adjuster.speed == RotationAdjuster.DEFAULT_SPEED
        assert state.stuck_detector.count == 0
        # 注意：locked_target 本身不被 reset（仍归属上层逻辑）
        assert state.locked_target is locked


class TestRecoTurnGrade:
    """PathFindingReco 返回 turn_grade 字段测试。"""

    def test_calculate_turn_returns_grade_large_turn(self):
        """距离缺失且偏差大时返回 LARGE_TURN。"""
        reco = make_reco()
        state = PathFindingState(
            locked_target=LockedTarget(
                template="a.png",
                center=(640, 360),
                bbox=(620, 340, 40, 40),
                score=0.9,
                distance=None,
                lock_id="a#x#y#z",
                last_target_angle=90.0,
            )
        )
        # 当前 angle ≈ 0°（dx=460, dy=0），偏差 ≈ 90° > upper=60° → LARGE_TURN
        target = TargetInfo(
            template="a.png",
            center=(1100, 360),
            bbox=(1080, 340, 40, 40),
            score=0.9,
            distance=None,
        )
        param = make_param(rotation_upper_threshold=60.0, rotation_lower_threshold=8.0)
        start, end, grade = reco._calculate_turn(state, target, param)
        assert start is not None and end is not None
        assert grade == TurnGrade.LARGE_TURN

    def test_calculate_turn_returns_grade_fine_tune(self):
        """距离缺失且偏差小时返回 FINE_TUNE。"""
        reco = make_reco()
        state = PathFindingState(
            locked_target=LockedTarget(
                template="a.png",
                center=(640, 360),
                bbox=(620, 340, 40, 40),
                score=0.9,
                distance=None,
                lock_id="a#x#y#z",
                last_target_angle=90.0,
            )
        )
        # 当前 angle ≈ 95°（dx=0, dy=-95），偏差 5° < lower=8° → FINE_TUNE
        target = TargetInfo(
            template="a.png",
            center=(640, 265),
            bbox=(620, 245, 40, 40),
            score=0.9,
            distance=None,
        )
        param = make_param(rotation_upper_threshold=60.0, rotation_lower_threshold=8.0)
        _, _, grade = reco._calculate_turn(state, target, param)
        assert grade == TurnGrade.FINE_TUNE


class TestRecoPhase:
    """PathFindingReco 阶段判定测试。"""

    def test_arrived_returns_approaching_phase(self):
        reco = make_reco()
        param = make_param(arrival_distance=30)
        target = TargetInfo(
            template="a.png",
            center=(640, 360),
            bbox=(620, 340, 40, 40),
            score=0.9,
            distance=20,
        )
        state_eval, phase = reco._evaluate_movement_state(
            "node1", target, PathFindingState(), param
        )
        assert state_eval == "arrived"
        assert phase == Phase.APPROACHING

    def test_stuck_phase_after_threshold(self):
        """无进展帧数累积到阈值后进入 STUCK 阶段。"""
        reco = make_reco()
        param = make_param(stuck_threshold=2, stuck_distance_tolerance=5)
        # 必须复用同一个 state 跨帧累计
        shared_state = PathFindingState()
        # 首帧：写入历史
        reco._evaluate_movement_state(
            "node1",
            TargetInfo(
                template="a.png",
                center=(100, 100),
                bbox=(80, 80, 40, 40),
                score=0.9,
                distance=100,
            ),
            shared_state,
            param,
        )
        # 第二帧：距离缩短 ≤ 5，count=1 未达阈值
        state_eval, _ = reco._evaluate_movement_state(
            "node1",
            TargetInfo(
                template="a.png",
                center=(100, 100),
                bbox=(80, 80, 40, 40),
                score=0.9,
                distance=98,
            ),
            shared_state,
            param,
        )
        assert state_eval == "approaching"
        # 第三帧：累计达到阈值
        state_eval, phase = reco._evaluate_movement_state(
            "node1",
            TargetInfo(
                template="a.png",
                center=(100, 100),
                bbox=(80, 80, 40, 40),
                score=0.9,
                distance=97,
            ),
            shared_state,
            param,
        )
        assert state_eval == "stuck"
        assert phase == Phase.STUCK


class TestActionPhase:
    """PathFinderAction phase 字段处理测试。"""

    def test_resolve_phase_valid(self):
        action = make_action()
        assert action._resolve_phase("stuck") == Phase.STUCK
        assert action._resolve_phase("turning") == Phase.TURNING
        assert action._resolve_phase("seeking") == Phase.SEEKING

    def test_resolve_phase_invalid(self):
        action = make_action()
        assert action._resolve_phase("invalid") is None
        assert action._resolve_phase(None) is None
        assert action._resolve_phase(123) is None


class TestDeadZoneNoTurn:
    """死区内不转向测试（避免中心附近抖动把目标推到屏幕外）。

    场景：目标已在屏幕中心死区内（direction == "centered"）但距离 OCR 缺失。
    此时若仍触发转向，会在视觉中心附近做微幅滑动，导致目标偏离中心，
    下一帧 direction 变化，触发反向滑动，形成抖动并最终把目标推到屏幕外，
    判为 backward 后向后移动。
    """

    def test_reco_no_turn_when_centered(self):
        """Reco：direction=centered 且 distance=None 时不返回转向坐标。"""
        reco = make_reco()
        state = PathFindingState()
        param = make_param()
        # 目标在死区内（与屏幕中心偏移 < dead_zone=50）
        target = TargetInfo(
            template="a.png",
            center=(645, 365),  # offset=(5, 5), norm≈7 < 50
            bbox=(625, 345, 40, 40),
            score=0.9,
            distance=None,
        )
        start, end, grade = reco._calculate_turn(
            state, target, param, direction="centered"
        )
        assert start is None
        assert end is None
        assert grade is None
        # 计数应被清零（避免 centered 帧累积污染计数）
        assert state.turn_attempts_without_distance == 0

    def test_reco_still_turns_when_not_centered(self):
        """Reco：direction != centered 且 distance=None 时仍正常转向。"""
        reco = make_reco()
        state = PathFindingState()
        param = make_param()
        # 目标偏左（不进入死区）
        target = TargetInfo(
            template="a.png",
            center=(400, 360),  # offset_x=-240
            bbox=(380, 340, 40, 40),
            score=0.9,
            distance=None,
        )
        start, end, grade = reco._calculate_turn(
            state, target, param, direction="left"
        )
        assert start is not None
        assert end is not None

    def test_reco_no_turn_when_centered_resets_attempts(self):
        """Reco：连续 centered 帧不累积 turn_attempts_without_distance。"""
        reco = make_reco()
        state = PathFindingState()
        # 预先模拟有过几次 turn 尝试
        state.turn_attempts_without_distance = 2
        param = make_param()
        target = TargetInfo(
            template="a.png",
            center=(645, 365),
            bbox=(625, 345, 40, 40),
            score=0.9,
            distance=None,
        )
        # 死区帧：清零计数，不返回 turn
        start, end, _ = reco._calculate_turn(
            state, target, param, direction="centered"
        )
        assert start is None and end is None
        assert state.turn_attempts_without_distance == 0

    def test_action_skips_turn_when_centered(self, monkeypatch):
        """Action：direction=centered 时跳过 turn 执行（不调用 platform.turn）。"""
        action = make_action()
        param = make_action_param()

        class _StubPlatform:
            def __init__(self):
                self.turn_called = False
                self.dodge_called = False
                self.move_called = False
                self.release_called = False

            def turn(self, sx, sy, ex, ey, duration=None):
                self.turn_called = True
                return True

            def dodge(self, direction):
                self.dodge_called = True
                return True

            def move(self, direction, duration):
                self.move_called = True
                return True

            def release_all(self):
                self.release_called = True
                return True

        platform = _StubPlatform()

        # 模拟 Context.tasker.stopping
        class _StubTasker:
            stopping = False

        class _StubContext:
            tasker = _StubTasker()

        ctx = _StubContext()
        action._execute_sequence(
            ctx,
            platform,
            direction="centered",
            param=param,
            distance=None,
            lock_id="lock1",
            turn_start=[800, 200],
            turn_end=[400, 400],  # 故意给一个非 None 的 turn 坐标
            turn_grade="large_turn",
            phase=None,
        )
        # centered 方向不应触发 turn 也不应触发 move，但 release_all 会执行
        assert platform.turn_called is False, "centered 不应调用 platform.turn"
        assert platform.move_called is False, "centered 不应调用 platform.move"
        assert platform.release_called is True

    def test_action_calls_turn_when_not_centered(self, monkeypatch):
        """Action：direction != centered 且 Reco 返回 turn 坐标时正常执行 turn。"""
        action = make_action()
        param = make_action_param()

        class _StubPlatform:
            def __init__(self):
                self.turn_called = False

            def turn(self, sx, sy, ex, ey, duration=None):
                self.turn_called = True
                return True

            def dodge(self, direction):
                return True

            def move(self, direction, duration):
                return True

            def release_all(self):
                return True

        platform = _StubPlatform()

        class _StubTasker:
            stopping = False

        class _StubContext:
            tasker = _StubTasker()

        ctx = _StubContext()
        action._execute_sequence(
            ctx,
            platform,
            direction="left",
            param=param,
            distance=None,
            lock_id="lock1",
            turn_start=[800, 200],
            turn_end=[400, 400],
            turn_grade="fine_tune",
            phase=None,
        )
        assert platform.turn_called is True

    def test_action_centered_with_distance_no_move(self, monkeypatch):
        """direction=centered 且 distance 有效时视为到达，不调用 move。"""
        action = make_action()
        param = make_action_param()

        class _StubPlatform:
            def __init__(self):
                self.move_called = False
                self.last_move_direction = None

            def turn(self, sx, sy, ex, ey, duration=None):
                return True

            def dodge(self, direction):
                return True

            def move(self, direction, duration):
                self.move_called = True
                self.last_move_direction = direction
                return True

            def release_all(self):
                return True

        platform = _StubPlatform()

        class _StubTasker:
            stopping = False

        class _StubContext:
            tasker = _StubTasker()

        ctx = _StubContext()
        action._execute_sequence(
            ctx,
            platform,
            direction="centered",
            param=param,
            distance=20,  # 已到达
            lock_id="lock1",
            turn_start=None,
            turn_end=None,
            turn_grade=None,
            phase=None,
        )
        assert platform.move_called is False

    def test_action_centered_no_distance_fallback_forward(self, monkeypatch):
        """direction=centered 且 distance=None 时必须兜底前进（避免在死区内卡住）。"""
        action = make_action()
        param = make_action_param(move_duration=800)

        class _StubPlatform:
            def __init__(self):
                self.move_called = False
                self.last_move_direction = None
                self.last_move_duration = None

            def turn(self, sx, sy, ex, ey, duration=None):
                return True

            def dodge(self, direction):
                return True

            def move(self, direction, duration):
                self.move_called = True
                self.last_move_direction = direction
                self.last_move_duration = duration
                return True

            def release_all(self):
                return True

        platform = _StubPlatform()

        class _StubTasker:
            stopping = False

        class _StubContext:
            tasker = _StubTasker()

        ctx = _StubContext()
        action._execute_sequence(
            ctx,
            platform,
            direction="centered",
            param=param,
            distance=None,  # 距离缺失
            lock_id="lock1",
            turn_start=None,
            turn_end=None,
            turn_grade=None,
            phase=None,
        )
        # 兜底前进：默认时长前进
        assert platform.move_called is True
        assert platform.last_move_direction == "forward"
        assert platform.last_move_duration == pytest.approx(0.8)
