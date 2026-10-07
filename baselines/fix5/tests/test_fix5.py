"""FIX5 contracts, executed against real production C helpers.

Logic tests bypass the trajectory, motor and encoder models.  A tolerance of
1e-4 is used only for mirrored float logic; PWM assertions allow one integer
rounding unit while the internal float slew is checked at 1e-4.  The low-speed
checks establish software semantics and do not establish a real 5 RPM motor
capability.  Physical scenarios and fixed-seed comparisons live separately.
"""
import ctypes as C
from functools import wraps
import math
from pathlib import Path
import random
import re
import subprocess
import tempfile
import unittest

import native_api
from native_api import fw, step, pwm

TRACE, FILTER, NONLINEAR, TREND, ADAPTIVE, LOW_SPEED, TURN, ALIGN = (1,2,4,8,16,32,64,128)


def bind(lib, name, args=(), result=C.c_int):
    if hasattr(lib,name):
        function=getattr(lib,name)
        function.argtypes=list(args)
        function.restype=result


def configure(lib):
    bind(lib,'Fix5_Enabled',result=C.c_uint)
    bind(lib,'Fix5_Config',(C.c_uint,),C.c_float)
    for name in ('Correction','RequestedBase','Base','ErrorRate','AlignBase','AlignGain'):
        bind(lib,'Fix5_'+name,result=C.c_float)
    for name in ('Integral','Output'):
        bind(lib,'Fix5_'+name,(C.c_uint,),C.c_float)
    bind(lib,'Fix5_Position',(C.c_float,),C.c_float)
    bind(lib,'Fix5_TrendReset',(C.c_float,),None)
    bind(lib,'Fix5_TrendPush',(C.c_float,),C.c_float)
    bind(lib,'Fix5_SetBase',(C.c_float,),None)
    bind(lib,'Fix5_Adaptive',(C.c_float,C.c_float),C.c_float)
    bind(lib,'Fix5_Sync',(C.c_uint,C.c_float,C.c_float,C.c_uint,C.c_float),None)
    bind(lib,'Fix5_LogicReset',result=None)
    bind(lib,'Fix5_LogicFollow',(C.c_float,),None)
    bind(lib,'Fix5_SpeedFrame',(C.c_uint,C.c_float,C.c_float))
    bind(lib,'Fix5_GainScale',(C.c_float,C.c_uint),C.c_float)
    bind(lib,'Fix5_Majority',(C.c_uint,C.c_uint,C.c_uint),C.c_uint)
    bind(lib,'Fix5_Turn',(C.c_float,),C.c_float)
    bind(lib,'Fix5_TurnCommand',(C.c_float,C.c_float),C.c_float)
    bind(lib,'Fix5_TurnFrame',(C.c_float,),None)
    bind(lib,'Fix5_AlignReset',(C.c_float,),None)
    bind(lib,'Fix5_AlignFrame',(C.c_float,),None)
    bind(lib,'Fix5_AlignControlledReset',(C.c_float,),None)
    bind(lib,'Fix5_AlignControlledFrame',(C.c_float,),None)
    bind(lib,'Fix5_Stop',(C.c_uint,),None)
    bind(lib,'Fix5_FaultPath',(C.c_uint,),None)
    bind(lib,'Fix5_TraceCount',result=C.c_uint)
    bind(lib,'Fix5_TraceSize',result=C.c_uint)
    bind(lib,'Fix5_TraceField',(C.c_uint,C.c_uint),C.c_int32)
    bind(lib,'Track_ClearTrace',result=None)

configure(fw)


def requires(flags):
    def decorator(function):
        @wraps(function)
        def checked(self,*args,**kwargs):
            if fw.Fix5_Enabled()&flags != flags:
                self.skipTest('feature disabled in this build')
            return function(self,*args,**kwargs)
        return checked
    return decorator


def cfg(index):
    return fw.Fix5_Config(index)


def trace():
    return [tuple(fw.Fix5_TraceField(i,field) for field in range(18))
            for i in range(fw.Fix5_TraceCount())]


class TraceStopTests(unittest.TestCase):
    @requires(TRACE)
    def test_18a_ring_overwrite_bounds_and_ram(self):
        fw.Native_Start(0x18)
        for tick in range(20,10001,20): step(tick,0x18,(5,5))
        count=fw.Fix5_TraceCount()
        self.assertGreaterEqual(count,150); self.assertLessEqual(count,250)
        self.assertLessEqual(count*fw.Fix5_TraceSize()+16,8192)
        records=trace()
        stamps=[record[0]&0xFFFFFFFF for record in records]
        self.assertEqual(stamps,list(range(10000-(count-1)*20,10001,20)))
        self.assertEqual(fw.Fix5_TraceField(count,0),-2147483648)
        self.assertEqual(fw.Fix5_TraceField(65535,0),-2147483648)
        self.assertTrue(all(record[1]==1 for record in records))
        fw.Track_ClearTrace(); self.assertEqual(fw.Fix5_TraceCount(),0)

    @requires(TRACE)
    def test_18b_stop_is_idempotent_first_cause_and_frozen(self):
        fw.Native_Start(0x18)
        for tick in range(20,401,20): step(tick,0x18,(5,5))
        fw.Fix5_Stop(2)
        self.assertEqual(fw.Fix5_ActualState(),8)
        self.assertEqual(fw.Native_StopReason(),2)
        first=trace()
        fw.Fix5_Stop(3)
        for tick in range(420,801,20): step(tick,0xFF,(5,5))
        self.assertEqual(fw.Native_StopReason(),2)
        self.assertEqual(trace(),first)
        self.assertEqual(pwm(),(0,0))
        self.assertEqual((fw.Native_LeftTarget(),fw.Native_RightTarget()),(0.0,0.0))
        self.assertEqual((fw.Native_LeftCommand(),fw.Native_RightCommand()),(0.0,0.0))

    @requires(TRACE)
    def test_18c_user_key_has_distinct_reason_and_restart_rearms(self):
        self.assertEqual(fw.Fix5_StopReasonNone(),0)
        user=fw.Fix5_StopReasonKey()
        self.assertNotEqual(user,0)
        self.assertNotIn(user,range(2,19))
        for key in (1,2,3,5):
            fw.Native_Start(0x18); step(20,0x18,(5,5))
            fw.Track_HandleKey(key)
            self.assertEqual(fw.Fix5_ActualState(),8)
            self.assertEqual(fw.Native_StopReason(),user)
            self.assertEqual(pwm(),(0,0))
        fw.Native_Sensors(0x18); fw.Track_Start()
        self.assertEqual(fw.Native_StopReason(),0)
        self.assertEqual(fw.Fix5_TraceCount(),0)
        for tick in range(40,121,20): step(tick,0x18,(5,5))
        self.assertGreater(fw.Fix5_TraceCount(),0)
        fw.Fix5_Stop(3)
        self.assertEqual(fw.Native_StopReason(),3)
        fw.Fix5_Stop(2)
        self.assertEqual(fw.Native_StopReason(),3)

    @requires(TRACE)
    def test_18d_all_reason_codes_stop_requests_and_lock_state(self):
        # The runtime scenario tests exercise the originating paths; this checks
        # the unified atomic semantics for every existing non-NONE reason.
        for reason in range(1,19):
            fw.Native_Start(0x18); step(20,0x18,(5,5))
            fw.Fix5_Stop(reason)
            self.assertEqual(fw.Fix5_ActualState(),8)
            self.assertEqual(fw.Native_StopReason(),reason)
            self.assertEqual(pwm(),(0,0))
            self.assertEqual((fw.Native_LeftTarget(),fw.Native_RightTarget()),(0.0,0.0))
            self.assertEqual((fw.Native_LeftCommand(),fw.Native_RightCommand()),(0.0,0.0))
            step(40,0x18,(5,5))
            self.assertEqual(fw.Fix5_ActualState(),8)

    @requires(TRACE)
    def test_18e_encoder_and_schedule_fault_have_explicit_causes(self):
        fw.Native_Start(0x18)
        for tick in range(20,2001,20):
            step(tick,0x18,(0,0))
            if not fw.Track_IsRunning(): break
        self.assertEqual(fw.Fix5_ActualState(),8)
        self.assertEqual(fw.Native_StopReason(),2)
        self.assertLessEqual(tick,1000)
        self.assertEqual(trace()[-1][1],8)
        self.assertEqual(trace()[-1][15],2)
        fw.Native_Start(0x18); step(100,0x18,(0,0))
        self.assertEqual(fw.Fix5_ActualState(),8)
        self.assertEqual(fw.Native_StopReason(),3)
        self.assertEqual(trace()[-1][1],8)
        self.assertEqual(trace()[-1][15],3)

    @requires(TRACE)
    def test_18f_every_existing_software_stop_path_has_its_own_cause(self):
        for reason in range(1,19):
            fw.Fix5_FaultPath(reason)
            self.assertEqual(fw.Fix5_ActualState(),8,reason)
            self.assertEqual(fw.Native_StopReason(),reason,reason)
            self.assertEqual(pwm(),(0,0),reason)
            self.assertEqual((fw.Native_LeftTarget(),fw.Native_RightTarget()),(0.0,0.0),reason)
            self.assertEqual((fw.Native_LeftCommand(),fw.Native_RightCommand()),(0.0,0.0),reason)

    @requires(TRACE)
    def test_20_clock_wrap_preserves_controller_and_trace_cadence(self):
        outputs=[]
        for initial in (0,0xFFFFFFF0):
            fw.Native_Start(0x18); fw.Native_SetClock(initial)
            rows=[]
            for frame in range(1,30):
                step(initial+frame*20,0x18,(5,5))
                rows.append((fw.Fix5_ActualState(),fw.Native_LeftTarget(),
                             fw.Native_RightTarget(),*pwm(),fw.Native_StopReason()))
            stamps=[row[0]&0xFFFFFFFF for row in trace()]
            self.assertEqual(stamps,[(initial+i*20)&0xFFFFFFFF for i in range(1,30)])
            outputs.append(rows)
        self.assertEqual(outputs[0],outputs[1])


class FilterFollowTests(unittest.TestCase):
    @requires(FILTER|ADAPTIVE)
    def test_01_center_500_cycles_single_frame_noise_is_quiet(self):
        fw.Native_Start(0x18); corrections=[]
        for frame in range(1,501):
            mask=(0x10 if frame%82==41 else 0x08 if frame%82==0 else 0x18)
            step(frame*20,mask,(5,5))
            self.assertEqual(fw.Fix5_ActualState(),1)
            self.assertEqual(fw.Native_LeftTarget(),fw.Native_RightTarget())
            corrections.append(fw.Fix5_Correction())
        self.assertLessEqual(max(map(abs,corrections)),1e-4)
        signs=[1 if x>1e-4 else -1 if x<-1e-4 else 0 for x in corrections]
        self.assertEqual(sum(a*b<0 for a,b in zip(signs,signs[1:])),0)
        self.assertGreaterEqual(fw.Fix5_Base(),.95*cfg(0))

    @requires(FILTER)
    def test_02_single_frame_noise_does_not_change_line_error(self):
        fw.Native_Start(0x30)
        for tick,mask in enumerate((0x30,0x30,0x20,0x30),1):
            step(tick*20,mask,(5,5))
            self.assertEqual(fw.Fix5_Raw(),mask)
            self.assertEqual(fw.Fix5_Filtered(),0x30)
            self.assertAlmostEqual(fw.Native_Error(),2.0,delta=1e-4)
            self.assertEqual(fw.Fix5_ActualState(),1)

    @requires(FILTER)
    def test_03_majority_truth_table_and_random_bit_reference(self):
        for pattern in range(8):
            inputs=tuple(255 if pattern&(1<<i) else 0 for i in range(3))
            expected=255 if pattern.bit_count()>=2 else 0
            self.assertEqual(fw.Fix5_Majority(*inputs),expected)
        generator=random.Random(0xF105)
        for _ in range(1024):
            inputs=[generator.randrange(256) for _ in range(3)]
            expected=sum(1<<bit for bit in range(8)
                         if sum(bool(value&(1<<bit)) for value in inputs)>=2)
            self.assertEqual(fw.Fix5_Majority(*inputs),expected)

    @requires(FILTER)
    def test_03b_stable_step_reaches_filter_within_two_cycles(self):
        fw.Native_Start(0x18)
        for tick in (20,40,60): step(tick,0x18,(5,5))
        step(80,0x0C,(5,5)); step(100,0x0C,(5,5))
        self.assertEqual(fw.Fix5_Filtered(),0x0C)
        self.assertAlmostEqual(fw.Native_Error(),-2.0,delta=1e-4)

    @requires(NONLINEAR)
    def test_03c_nonlinear_is_continuous_odd_monotone_and_has_no_dead_zone(self):
        self.assertGreater(fw.Fix5_Position(.25),0.0)
        self.assertLess(fw.Fix5_Position(.25),.25)
        fw.Fix5_LogicReset()
        for _ in range(20): fw.Fix5_LogicFollow(.25)
        self.assertGreater(fw.Fix5_Correction(),0.0)
        self.assertLess(fw.Fix5_Correction(),cfg(7)*.25)
        previous=fw.Fix5_Position(0.0)
        for i in range(1,701):
            error=i*.01; value=fw.Fix5_Position(error)
            self.assertGreaterEqual(value,previous-1e-6)
            self.assertAlmostEqual(value,-fw.Fix5_Position(-error),delta=1e-4)
            previous=value
        for boundary in (cfg(14),cfg(15)):
            low=fw.Fix5_Position(boundary-.0001)
            exact=fw.Fix5_Position(boundary)
            high=fw.Fix5_Position(boundary+.0001)
            self.assertLess(abs(high-low),.001)
            self.assertLessEqual(low,exact); self.assertLessEqual(exact,high)

    @requires(ADAPTIVE|TREND)
    def test_04_progressive_error_slows_and_slew_remains_bounded(self):
        fw.Fix5_LogicReset(); fw.Fix5_SetBase(cfg(0))
        for _ in range(60): fw.Fix5_LogicFollow(0.0)
        base=fw.Fix5_Base(); correction=0.0
        old=(fw.Native_LeftCommand(),fw.Native_RightCommand())
        for error in (.5,1.0,2.0,3.0,4.0):
            fw.Fix5_LogicFollow(error)
            current=(fw.Native_LeftCommand(),fw.Native_RightCommand())
            self.assertLessEqual(max(abs(a-b) for a,b in zip(old,current)),cfg(2)+1e-4)
            self.assertLessEqual(fw.Fix5_Base(),base+1e-4)
            base=fw.Fix5_Base(); old=current
            new=abs(fw.Fix5_Correction())
            self.assertGreaterEqual(new,correction-1e-4); correction=new

    @requires(ADAPTIVE|TREND)
    def test_05_recentering_recovers_base_at_configured_rate(self):
        fw.Fix5_LogicReset(); fw.Fix5_SetBase(cfg(10))
        fw.Fix5_TrendReset(4.0)
        old=fw.Fix5_Base()
        for error in (4.0,3.0,2.0,1.0,0.0)+((0.0,)*30):
            fw.Fix5_LogicFollow(error); current=fw.Fix5_Base()
            self.assertGreaterEqual(current,old-1e-4)
            self.assertLessEqual(current-old,cfg(12)+1e-4)
            old=current
        self.assertGreaterEqual(old,.95*cfg(0))

    @requires(ADAPTIVE|TREND)
    def test_06_same_error_outward_trend_slows_earlier(self):
        rates=[]; bases=[]
        for history in ((0.0,1.0,2.0),(4.0,3.0,2.0)):
            fw.Fix5_TrendReset(history[0])
            for error in history: rate=fw.Fix5_TrendPush(error)
            rates.append(rate)
            fw.Fix5_SetBase(cfg(0))
            bases.append(fw.Fix5_Adaptive(2.0,rate))
        self.assertGreater(rates[0],0.0); self.assertLess(rates[1],0.0)
        # Direct desired-base histories can both reach the fast decel bound on
        # their first tick; repeated updates expose the trend contribution.
        converged=[]
        for rate in rates:
            fw.Fix5_SetBase(cfg(0))
            for _ in range(15): base=fw.Fix5_Adaptive(2.0,rate)
            converged.append(base)
        self.assertLessEqual(bases[0],bases[1]+1e-4)
        self.assertLess(converged[0],converged[1]-.01)

    @requires(ADAPTIVE|TREND)
    def test_07_mirrored_logic_swaps_targets_and_negates_correction(self):
        traces=[]
        history=(0.0,.5,1.0,2.0,3.0,4.0,3.0,2.0,1.0,0.0,-.5,-1.0,0.0)
        for sign in (-1,1):
            fw.Fix5_LogicReset(); rows=[]
            for error in history:
                fw.Fix5_LogicFollow(sign*error)
                rows.append((fw.Fix5_ActualState(),fw.Native_Mode(),fw.Fix5_Base(),
                             fw.Fix5_Correction(),fw.Fix5_ErrorRate(),
                             fw.Native_LeftTarget(),fw.Native_RightTarget(),
                             fw.Native_LeftCommand(),fw.Native_RightCommand()))
            traces.append(rows)
        for left,right in zip(*traces):
            self.assertEqual(left[:2],right[:2])
            self.assertAlmostEqual(left[2],right[2],delta=1e-4)
            self.assertAlmostEqual(left[3],-right[3],delta=1e-4)
            self.assertAlmostEqual(left[4],-right[4],delta=1e-4)
            for a,b in ((5,6),(6,5),(7,8),(8,7)):
                self.assertAlmostEqual(left[a],right[b],delta=1e-4)

    @requires(TREND)
    def test_07b_error_rate_is_bounded_and_resets_between_sessions(self):
        fw.Fix5_TrendReset(0.0)
        for error in (7.0,-7.0,7.0,-7.0)*20:
            self.assertLessEqual(abs(fw.Fix5_TrendPush(error)),cfg(16)+1e-4)
        fw.Fix5_TrendReset(2.0)
        self.assertAlmostEqual(fw.Fix5_TrendPush(2.0),0.0,delta=1e-4)

    @requires(TREND)
    def test_07c_trend_has_signed_units_per_second_and_exact_mirrors(self):
        histories=[]
        for sign in (-1,1):
            fw.Fix5_TrendReset(0.0); rates=[]
            for index in range(1,13): rates.append(fw.Fix5_TrendPush(sign*index*.5))
            self.assertAlmostEqual(rates[-1],sign*25.0,delta=1e-4)
            histories.append(rates)
        for left,right in zip(*histories):
            self.assertAlmostEqual(left,-right,delta=1e-4)


class RecoveryContractTests(unittest.TestCase):
    def test_17_existing_24_recovery_tests_are_unchanged(self):
        # Reuse the original test bodies and their original tolerances.  This
        # wrapper makes the Recovery contract part of both the default and
        # explicitly all-on FIX5 suites without duplicating its implementation.
        from test_recovery import RecoveryTests
        suite=unittest.defaultTestLoader.loadTestsFromTestCase(RecoveryTests)
        self.assertEqual(suite.countTestCases(),24)
        result=unittest.TestResult(); suite.run(result)
        details='\n'.join(message for _,message in result.failures+result.errors)
        self.assertFalse(result.failures or result.errors,details)
        self.assertFalse(result.skipped)
        self.assertEqual(result.testsRun,24)


class SpeedContinuityTests(unittest.TestCase):
    def test_08_nonzero_5_8_12_18rpm_never_becomes_zero(self):
        for wheel in (0,1):
            for sign in (-1,1):
                for target in (5.0,8.0,12.0,18.0):
                    fw.Native_Reset(); old=0.0
                    for _ in range(80):
                        value=fw.Fix5_SpeedFrame(wheel,sign*target,sign*target)
                        self.assertGreater(sign*value,0)
                        current=fw.Fix5_Output(wheel)
                        self.assertLessEqual(current-old,cfg(3)+1e-4)
                        self.assertLessEqual(old-current,cfg(4)+1e-4)
                        self.assertLessEqual(abs(fw.Fix5_Integral(wheel)),cfg(5)+1e-4)
                        self.assertTrue(math.isfinite(current)); old=current
                    self.assertEqual(fw.Fix5_SpeedFrame(wheel,0.0,0.0),0)

    @requires(LOW_SPEED)
    def test_09_low_speed_gains_and_pwm_are_continuous_at_breakpoints(self):
        for threshold in (cfg(18),cfg(19)):
            for integral in (0,1):
                values=[fw.Fix5_GainScale(threshold+i*.001,integral) for i in range(-20,21)]
                self.assertTrue(all(b>=a-1e-6 for a,b in zip(values,values[1:])))
                self.assertLess(max(abs(b-a) for a,b in zip(values,values[1:])),.001)
            for wheel in (0,1):
                fw.Native_Reset()
                for _ in range(30): fw.Fix5_SpeedFrame(wheel,threshold,threshold)
                old=fw.Fix5_Output(wheel)
                for i in range(-20,21):
                    target=threshold+i*.001
                    fw.Fix5_SpeedFrame(wheel,target,target)
                    new=fw.Fix5_Output(wheel)
                    self.assertLess(abs(new-old),.02); old=new
        self.assertAlmostEqual(fw.Fix5_GainScale(cfg(19),0),fw.Fix5_GainScale(cfg(19)+10,0),delta=1e-6)
        self.assertAlmostEqual(fw.Fix5_GainScale(cfg(19),1),fw.Fix5_GainScale(cfg(19)+10,1),delta=1e-6)

    def test_10_100ms_no_pulses_does_not_store_an_unbounded_start_kick(self):
        for wheel in (0,1):
            for target in (5.0,8.0,12.0,18.0):
                fw.Native_Reset(); values=[]
                for _ in range(5):
                    values.append(abs(fw.Fix5_SpeedFrame(wheel,target,0.0)))
                self.assertLessEqual(abs(fw.Fix5_Integral(wheel)),.4)
                for frame in range(40):
                    actual=target*min(1.0,(frame+1)/8)
                    values.append(abs(fw.Fix5_SpeedFrame(wheel,target,actual)))
                    self.assertLessEqual(abs(fw.Fix5_Integral(wheel)),cfg(5)+1e-4)
                self.assertLessEqual(max(values),cfg(6))
                self.assertLessEqual(max(b-a for a,b in zip(values,values[1:])),math.ceil(cfg(3))+1)
                self.assertLessEqual(max(a-b for a,b in zip(values,values[1:])),math.ceil(cfg(4))+1)
                self.assertLessEqual(max(values[-5:])-min(values[-5:]),1)

    def test_11_four_direction_offset_parameters_are_independent(self):
        macros=('LEFT_DRIVE_OFFSET_PWM','LEFT_REVERSE_DRIVE_OFFSET_PWM',
                'RIGHT_DRIVE_OFFSET_PWM','RIGHT_REVERSE_DRIVE_OFFSET_PWM')
        directions=((0,1),(0,-1),(1,1),(1,-1))
        root=native_api.ROOT
        original=(root/'hardware/CarConfig.h').read_text()
        def outputs(lib):
            bind(lib,'Fix5_SpeedFrame',(C.c_uint,C.c_float,C.c_float))
            result=[]
            for wheel,sign in directions:
                lib.Native_Reset()
                for _ in range(30): value=lib.Fix5_SpeedFrame(wheel,sign*18.0,sign*18.0)
                result.append(value)
            return result
        reference=outputs(fw)
        with tempfile.TemporaryDirectory(prefix='fix5_offsets_') as directory:
            directory=Path(directory)
            for changed,macro in enumerate(macros):
                header=directory/(macro+'.h')
                # Preserve independent numeric values; modifying a forward
                # macro also affecting a reverse alias is a real failure.
                replacement=r'\g<1>(\2 + 3.0f)'
                modified,count=re.subn(r'(^#define\s+'+macro+r'\s+)([^/\n]+)',replacement,
                                       original,count=1,flags=re.MULTILINE)
                self.assertEqual(count,1)
                feature_names=('TRACK_TRACE_ENABLE','TRACK_SENSOR_FILTER_ENABLE',
                               'TRACK_NONLINEAR_FOLLOW_ENABLE','TRACK_ERROR_TREND_ENABLE',
                               'TRACK_ADAPTIVE_SPEED_ENABLE','TRACK_LOW_SPEED_ZONE_ENABLE',
                               'TRACK_TURN_CONTINUITY_ENABLE','TRACK_ALIGN_TREND_ENABLE')
                # Match the loaded reference feature configuration even when
                # this test was invoked with a preincluded matrix header.
                switches=''.join('#define '+name+' '+str(int(bool(fw.Fix5_Enabled()&(1<<bit))))+'\n'
                                 for bit,name in enumerate(feature_names))
                header.write_text(switches+modified)
                library=directory/(macro+'.so')
                command=list(native_api.COMMAND)
                command[command.index('-o')+1]=str(library)
                command[1:1]=['-include',str(header)]
                subprocess.run(command,cwd=root,check=True,capture_output=True)
                actual=outputs(C.CDLL(str(library)))
                for index,(before,after) in enumerate(zip(reference,actual)):
                    if index==changed:
                        self.assertGreater(abs(after),abs(before)+1)
                    else:
                        self.assertEqual(after,before,(macro,directions[index]))

    def test_12_reversal_passes_zero_without_old_integral_or_false_fault(self):
        for wheel in (0,1):
            fw.Native_Reset()
            for _ in range(20): fw.Fix5_SpeedFrame(wheel,20.0,20.0)
            fw.Native_SetIntegral(wheel,10)
            self.assertEqual(fw.Fix5_SpeedFrame(wheel,0.0,12.0),0)
            self.assertAlmostEqual(fw.Fix5_Integral(wheel),0.0,delta=1e-4)
            values=[]
            for actual in (8.0,4.0,0.0,-8.0,-16.0,-20.0)+((-20.0,)*20):
                values.append(fw.Fix5_SpeedFrame(wheel,-20.0,actual))
                self.assertLessEqual(values[-1],0)
                self.assertLessEqual(abs(fw.Fix5_Integral(wheel)),cfg(5)+1e-4)
            self.assertEqual(values[0],0)
            self.assertLess(max(map(abs,values)),15)
            fw.Native_Reset()
            for _ in range(20): self.assertFalse(fw.Native_FaultFrame(wheel,2,20,12))
            for _ in range(10): self.assertFalse(fw.Native_FaultFrame(wheel,1,-20,-9))
            for _ in range(40): self.assertFalse(fw.Native_FaultFrame(wheel,-1,-20,-9))


class TurnAlignHandoffTests(unittest.TestCase):
    @requires(TURN)
    def test_13_turn_requested_speed_is_continuous_monotone_and_slewed(self):
        values=[fw.Fix5_Turn(angle) for angle in (0,20,40,50,60,70)]
        self.assertTrue(all(a>=b for a,b in zip(values,values[1:])))
        self.assertGreater(min(values),0.0)
        for angle,requested in zip((0,20,40,50,60,70),values):
            fw.Fix5_TurnFrame(float(angle))
            self.assertAlmostEqual(fw.Native_LeftTarget(),requested,delta=1e-4)
            self.assertAlmostEqual(fw.Native_RightTarget(),-requested,delta=1e-4)
            self.assertEqual(fw.Native_Mode(),2)
            self.assertLessEqual(abs(fw.Native_LeftCommand()),4.0+1e-4)
            self.assertLessEqual(abs(fw.Native_RightCommand()),4.0+1e-4)
        previous=fw.Fix5_Turn(0.0)
        for index in range(1,9001):
            current=fw.Fix5_Turn(index*.01)
            self.assertLessEqual(current,previous+1e-5)
            self.assertLess(abs(current-previous),.05); previous=current
        command=0.0
        for requested in values:
            old=command; command=fw.Fix5_TurnCommand(command,requested)
            self.assertLessEqual(command-old,4.0+1e-4)
            self.assertLessEqual(old-command,8.0+1e-4)

    @requires(ALIGN)
    def test_14_align_improving_error_keeps_original_strategy(self):
        fw.Fix5_AlignReset(5.0)
        for error in (5.0,4.0,3.0,2.0):
            fw.Fix5_AlignFrame(error)
            self.assertAlmostEqual(fw.Fix5_AlignBase(),cfg(8),delta=1e-4)
            self.assertAlmostEqual(fw.Fix5_AlignGain(),cfg(9),delta=1e-4)
        fw.Fix5_AlignControlledReset(5.0)
        for error in (5.0,4.0,3.0,2.0):
            fw.Fix5_AlignControlledFrame(error)
            self.assertAlmostEqual(fw.Fix5_RequestedBase(),cfg(8),delta=1e-4)
            self.assertEqual(fw.Fix5_ActualState(),17)

    @requires(ALIGN)
    def test_15_align_persistent_worsening_slows_and_strengthens_gain(self):
        fw.Fix5_AlignReset(1.0)
        for index in range(int(cfg(20))): fw.Fix5_AlignFrame(float(index+2))
        self.assertLess(fw.Fix5_AlignBase(),cfg(8))
        self.assertGreater(fw.Fix5_AlignGain(),cfg(9))
        self.assertGreater(fw.Fix5_AlignBase(),0.0)
        self.assertLessEqual(fw.Fix5_AlignGain(),cfg(9)*1.3)
        expected=fw.Fix5_AlignBase(); fw.Fix5_AlignControlledReset(1.0)
        old=(fw.Native_LeftCommand(),fw.Native_RightCommand())
        for index in range(int(cfg(20))):
            fw.Fix5_AlignControlledFrame(float(index+2))
            current=(fw.Native_LeftCommand(),fw.Native_RightCommand())
            self.assertLessEqual(max(abs(a-b) for a,b in zip(old,current)),6.0+1e-4)
            old=current
        self.assertAlmostEqual(fw.Fix5_RequestedBase(),expected,delta=1e-4)
        self.assertLessEqual(abs(fw.Fix5_Correction()),10.0+1e-4)

    @requires(ALIGN)
    def test_16_align_frequent_sign_flips_damp_gain(self):
        fw.Fix5_AlignReset(2.0)
        for error in (-2.0,2.0,-2.0,2.0,-2.0,2.0): fw.Fix5_AlignFrame(error)
        self.assertLess(fw.Fix5_AlignGain(),cfg(9))
        self.assertGreater(fw.Fix5_AlignGain(),0.0)
        gain=fw.Fix5_AlignGain(); fw.Fix5_AlignControlledReset(2.0)
        for error in (-2.0,2.0,-2.0,2.0,-2.0,2.0):
            fw.Fix5_AlignControlledFrame(error)
        self.assertAlmostEqual(abs(fw.Fix5_Correction()),2.0*gain,delta=1e-4)
        self.assertEqual(fw.Fix5_ActualState(),17)

    @requires(ADAPTIVE)
    def test_19a_exit_low_base_enters_track_without_lower_clamp_jump(self):
        fw.Fix5_LogicReset()
        fw.Fix5_Sync(0,26.0,28.0,1,27.0)
        self.assertAlmostEqual(fw.Fix5_Base(),27.0,delta=1e-4)
        self.assertLess(fw.Fix5_Base(),cfg(10))
        previous=fw.Fix5_Base()
        for _ in range(20):
            current=fw.Fix5_Adaptive(0.0,0.0)
            self.assertGreaterEqual(current,previous)
            self.assertLessEqual(current-previous,cfg(12)+1e-4)
            previous=current
        self.assertGreater(previous,cfg(10))

    @requires(ADAPTIVE)
    def test_19b_spin_and_pivot_do_not_pollute_forward_base(self):
        fw.Fix5_LogicReset()
        fw.Fix5_Sync(0,28.0,32.0,0,0.0)
        self.assertAlmostEqual(fw.Fix5_Base(),30.0,delta=1e-4)
        for mode,left,right in ((2,30.0,-30.0),(1,0.0,30.0),(1,30.0,0.0)):
            fw.Fix5_Sync(mode,left,right,0,0.0)
            self.assertAlmostEqual(fw.Fix5_Base(),30.0,delta=1e-4)
        fw.Fix5_Sync(0,30.0,34.0,0,0.0)
        self.assertAlmostEqual(fw.Fix5_Base(),32.0,delta=1e-4)

    @requires(ADAPTIVE)
    def test_19c_start_uses_configured_forward_base(self):
        fw.Fix5_LogicReset()
        self.assertAlmostEqual(fw.Fix5_Base(),cfg(11),delta=1e-4)


if __name__=='__main__':
    unittest.main(verbosity=2)
