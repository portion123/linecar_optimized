"""转弯/恢复契约：运行实际C状态机，固定输入不能替代实车动力学。"""
import math
import unittest
from native_api import fw, step, pwm

TRACK, SEARCH, STOP, FAST, CONFIRM, APPROACH, EXIT, SLOW, ALIGN = 1,2,8,11,12,14,15,16,17

def warm(mask=0x18):
    fw.Native_Start(mask)
    for tick in range(20,421,20): step(tick,mask,(5,5))
    return 420

def corner(tick=420,mask=0x3F):
    for _ in range(4): tick+=20; step(tick,mask,(3,3))
    for _ in range(2): tick+=20; step(tick,0,(3,3))
    assert fw.Native_State()==APPROACH
    return tick

def turn(mask=0x3F):
    tick=corner(warm(),mask)
    for _ in range(200):
        tick+=20; step(tick,0,(3,3))
        if fw.Native_State()!=APPROACH: break
    assert fw.Native_State()==FAST
    return tick

def align(mask=0x3F):
    tick=turn(mask); direction=fw.Native_Dir()
    for _ in range(180):
        tick+=20
        # 先达到合理转角，再从预期侧收敛到中心。
        progress=-direction*fw.Native_Angle()
        sensor=0 if progress<math.radians(72) else (0x0C if direction<0 else 0x30)
        if progress>=math.radians(75): sensor=0x18
        step(tick,sensor,(direction*2,-direction*2))
        if fw.Native_State()==ALIGN: return tick
        assert fw.Track_IsRunning(), (fw.Native_State(),progress)
    raise AssertionError('转弯未进入ALIGN')

def exit_line(mask=0x3F):
    tick=align(mask)
    for _ in range(80):
        tick+=20; step(tick,0x18,(2,2))
        if fw.Native_State()==EXIT: return tick
    raise AssertionError('ALIGN未进入EXIT')

class RecoveryTests(unittest.TestCase):
    def test_01_center_line_has_no_bias(self):
        warm()
        self.assertEqual(fw.Native_State(),TRACK)
        self.assertEqual(fw.Native_LeftTarget(),fw.Native_RightTarget())
        self.assertEqual(fw.Native_LeftCommand(),fw.Native_RightCommand())
        self.assertEqual(fw.Native_Hint(),0)

    def test_02_left_right_gradual_inputs_are_mirrors_and_smooth(self):
        traces=[]
        for masks in ([0x18,0x0C,0x06,0x03,0x01],[0x18,0x30,0x60,0xC0,0x80]):
            tick=warm(); trace=[]; old=(fw.Native_LeftCommand(),fw.Native_RightCommand())
            for mask in masks:
                for _ in range(4):
                    tick+=20; step(tick,mask,(5,5))
                    pair=(fw.Native_LeftCommand(),fw.Native_RightCommand())
                    self.assertLessEqual(max(abs(x-y) for x,y in zip(pair,old)),9.0001)
                    trace.append((fw.Native_State(),pair)); old=pair
            traces.append(trace)
        for (state,left),(other,right) in zip(*traces):
            self.assertEqual(state,other)
            self.assertAlmostEqual(left[0],right[1],places=4)
            self.assertAlmostEqual(left[1],right[0],places=4)

    def test_03_clear_left_and_right_corners_select_correct_direction(self):
        for mask,direction in ((0x3F,-1),(0xFC,1)):
            corner(warm(),mask)
            self.assertEqual(fw.Native_Dir(),direction)
            self.assertEqual(fw.Native_LeftTarget(),28)
            self.assertEqual(fw.Native_RightTarget(),28)

    def test_04_ambiguous_corner_uses_valid_right_hint(self):
        tick=warm(0x20)
        self.assertEqual(fw.Native_Hint(),1)
        corner(tick,0xFF)
        self.assertEqual(fw.Native_Dir(),1)

    def test_05_expired_hint_is_not_used(self):
        tick=warm(0x20)
        for _ in range(40): tick+=20; step(tick,0x18,(5,5))
        self.assertEqual(fw.Native_Hint(),0)
        corner(tick,0xFF)
        self.assertEqual(fw.Native_Dir(),0)

    def test_06_one_or_two_center_reads_do_not_complete_turn(self):
        for mask in (0x3F,0xFC):
            tick=turn(mask); direction=fw.Native_Dir()
            while -direction*fw.Native_Angle()<math.radians(72):
                tick+=20; step(tick,0,(direction*2,-direction*2))
            for _ in range(2): tick+=20; step(tick,0x18,(direction*2,-direction*2))
            self.assertEqual(fw.Native_State(),SLOW)
            tick+=20; step(tick,0,(direction*2,-direction*2))
            self.assertEqual(fw.Native_State(),SLOW)

    def test_07_correct_turn_align_exit_track(self):
        for mask in (0x3F,0xFC):
            tick=exit_line(mask)
            for _ in range(70):
                tick+=20; step(tick,0x18,(3,3))
                self.assertIn(fw.Native_State(),(EXIT,TRACK))
            self.assertEqual(fw.Native_State(),TRACK)
            self.assertGreater(min(pwm()),0)

    def test_08_exit_white_20_to_100ms_does_not_reverse_search(self):
        for mask in (0x3F,0xFC):
            tick=exit_line(mask); direction=fw.Native_Dir()
            for _ in range(5):
                tick+=20; step(tick,0,(1,1))
                self.assertEqual(fw.Native_State(),EXIT)
                self.assertEqual(fw.Native_Dir(),direction)
                self.assertGreaterEqual(min(pwm()),0)
            tick+=20; step(tick,0x18,(2,2))
            self.assertEqual(fw.Native_State(),EXIT)

    def test_09_permanent_loss_stops_in_finite_time(self):
        for initial in (0x18,0x04,0x20):
            tick=warm(initial)
            for _ in range(650):
                tick+=20; step(tick,0)
                if not fw.Track_IsRunning(): break
            self.assertFalse(fw.Track_IsRunning())
            self.assertEqual(pwm(),(0,0))
            self.assertLessEqual(tick,13420)

    def test_10_false_recovery_does_not_renew_session(self):
        tick=warm(0x20)
        for _ in range(4): tick+=20; step(tick,0,(3,3))
        self.assertEqual(fw.Native_State(),SEARCH)
        started=fw.Native_RecoveryStart(); attempts=[]; seen=set()
        for _ in range(600):
            state=fw.Native_State(); seen.add(state)
            # 搜索时短暂稳定中心，进入ALIGN/EXIT后随即丢线，始终没有稳定TRACK。
            mask=0x18 if state in (SEARCH,ALIGN) else 0
            tick+=20; step(tick,mask)
            self.assertEqual(fw.Native_RecoveryStart(),started)
            attempts.append(fw.Native_RecoveryAttempts())
            if not fw.Track_IsRunning(): break
        self.assertTrue({SEARCH,ALIGN,EXIT}.issubset(seen))
        self.assertEqual(fw.Native_State(),STOP)
        self.assertEqual(fw.Native_StopReason(),13)
        self.assertEqual(pwm(),(0,0))
        self.assertEqual(attempts,sorted(attempts))

    def test_11_full_corner_mirrors_state_transitions(self):
        traces=[]
        for mask in (0x3F,0xFC):
            tick=turn(mask); direction=fw.Native_Dir(); trace=[]; right_residual=0.0
            for _ in range(200):
                state=fw.Native_State()
                if state in (FAST,SLOW):
                    progress=-direction*fw.Native_Angle()
                    sensor=0 if progress<math.radians(72) else 0x18
                    counts=(direction*2,-direction*2)
                else:
                    sensor=0x18
                    # 两轮实测count/rev不同，同count不是相同物理位移；按标定量化。
                    right_residual+=3*265.35/251.0
                    counts=(3,int(right_residual)); right_residual-=counts[1]
                tick+=20; step(tick,sensor,counts)
                trace.append((fw.Native_State(),-direction*fw.Native_Angle()))
                if fw.Native_State()==TRACK: break
            self.assertEqual(fw.Native_State(),TRACK)
            traces.append(trace)
        self.assertEqual([t[0] for t in traces[0]],[t[0] for t in traces[1]])
        for left,right in zip(*traces): self.assertAlmostEqual(left[1],right[1],delta=.012)

    def test_12_stable_forward_track_clears_session(self):
        tick=exit_line()
        for _ in range(160): tick+=20; step(tick,0x18,(3,3))
        self.assertEqual(fw.Native_State(),TRACK)
        self.assertFalse(fw.Native_RecoveryActive())
        self.assertEqual(fw.Native_RecoveryAttempts(),0)

    def test_13_attempt_exhaustion_cannot_reapply_pwm(self):
        warm()
        for _ in range(4): fw.Native_RestartScan(1)
        self.assertTrue(fw.Track_IsRunning())
        started=fw.Native_RecoveryStart()
        fw.Native_RestartScan(1)
        self.assertEqual(fw.Native_State(),STOP)
        self.assertEqual(fw.Native_StopReason(),13)
        self.assertEqual(fw.Native_RecoveryStart(),started)
        self.assertEqual(pwm(),(0,0))

    def test_14_session_time_budget_across_clock_wrap(self):
        tick=warm(); fw.Native_SetClock(0xFFFFFFF0)
        fw.Native_RecoveryAge(11990)
        step(0x100000004,0x18,(2,2))
        self.assertEqual(fw.Native_State(),STOP)
        self.assertEqual(fw.Native_StopReason(),10)
        self.assertEqual(pwm(),(0,0))

    def test_15_session_total_angle_budget(self):
        tick=warm(); fw.Native_RestartScan(1)
        for _ in range(20): tick+=20; step(tick,0)
        # 匹配当前命令方向，排除反向故障，使累计角度保护直接接受极端输入。
        step(tick+20,0,(1000,-1000))
        self.assertEqual(fw.Native_State(),STOP)
        self.assertEqual(fw.Native_StopReason(),11)
        self.assertEqual(pwm(),(0,0))

    def test_16_session_distance_budget(self):
        tick=warm(); tick+=20; step(tick,0,(2,2))
        self.assertTrue(fw.Native_RecoveryActive())
        step(tick+20,0x18,(2000,2000))
        self.assertEqual(fw.Native_State(),STOP)
        self.assertEqual(fw.Native_StopReason(),12)
        self.assertEqual(pwm(),(0,0))

    def test_17_oled_skips_control_deadline_margin(self):
        fw.Native_Start(0x18)
        fw.Native_Idle(1); calls=fw.Native_OLEDServices()
        fw.Native_Idle(17); fw.Native_Idle(18); fw.Native_Idle(19)
        self.assertEqual(fw.Native_OLEDServices(),calls)
        fw.Native_Idle(20)
        self.assertGreater(fw.Native_OLEDServices(),calls)

    def test_18_motion_modes_are_explicit(self):
        warm(); self.assertEqual(fw.Native_Mode(),0)
        turn(); self.assertEqual(fw.Native_Mode(),2)
        align(); self.assertEqual(fw.Native_Mode(),0)
        tick=warm()
        for _ in range(40):
            tick+=20; step(tick,0xC0,(2,2))
            if fw.Native_State()==SEARCH: break
        self.assertEqual(fw.Native_Mode(),1)

    def test_19_single_corner_frame_is_not_confirmed(self):
        tick=warm(); tick+=20; step(tick,0xFF,(2,2))
        self.assertLess(fw.Native_Confidence(),4)
        for _ in range(3): tick+=20; step(tick,0,(2,2))
        self.assertNotEqual(fw.Native_State(),APPROACH)

    def test_20_continuous_all_black_has_exit_path(self):
        tick=warm()
        for _ in range(120):
            tick+=20; step(tick,0xFF,(2,2))
            if not fw.Track_IsRunning(): break
        self.assertEqual(fw.Native_State(),STOP)
        self.assertEqual(fw.Native_StopReason(),4)
        self.assertEqual(pwm(),(0,0))

    def test_21_exit_track_transition_does_not_drop_back_to_42rpm(self):
        tick=exit_line(); previous=None
        for _ in range(80):
            tick+=20; step(tick,0x18,(3,3))
            if fw.Native_State()==TRACK:
                pair=(fw.Native_LeftCommand(),fw.Native_RightCommand())
                if previous is not None:
                    self.assertGreaterEqual(min(pair),min(previous)-.001)
                previous=pair
        self.assertIsNotNone(previous)
        self.assertGreaterEqual(min(previous),59.9)

    def test_22_manual_stop_in_all_added_phases(self):
        for factory in (lambda: corner(warm()),turn,align,exit_line):
            factory(); fw.Track_HandleKey(1)
            self.assertFalse(fw.Track_IsRunning()); self.assertEqual(pwm(),(0,0))

    def test_23_bench_low_speed_limits_and_timeout(self):
        for left,right in ((5,0),(-5,0),(0,5),(0,-5)):
            fw.Native_Reset(); fw.Track_StartSpeedTest(left,right)
            self.assertEqual(fw.Native_State(),19)
            for tick in range(20,3021,20):
                # 稀疏正确编码器反馈，无零窗口误报；3s后必须主动停车。
                counts=(0 if tick%60 else (1 if left>0 else -1 if left<0 else 0),
                        0 if tick%60 else (1 if right>0 else -1 if right<0 else 0))
                step(tick,0,counts)
            self.assertEqual(fw.Native_State(),STOP)
            self.assertEqual(fw.Native_StopReason(),18)
            self.assertEqual(pwm(),(0,0))
        fw.Native_Reset(); fw.Track_StartSpeedTest(61,0)
        self.assertFalse(fw.Track_IsRunning()); self.assertEqual(pwm(),(0,0))

    def test_24_turn_debug_uses_current_sensor_error_and_keeps_history(self):
        for mask in (0x3F,0xFC):
            tick=turn(mask); history=fw.Native_Error(); direction=fw.Native_Dir()
            tick+=20; step(tick,0x04,(direction*2,-direction*2))
            self.assertEqual(fw.Native_DebugError(),-30)
            self.assertEqual(fw.Native_Error(),history)
            tick+=20; step(tick,0x20,(direction*2,-direction*2))
            self.assertEqual(fw.Native_DebugError(),30)
            self.assertEqual(fw.Native_Error(),history)


if __name__=='__main__': unittest.main(verbosity=2)
