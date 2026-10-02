# FIX5 纯软件稳定性增强报告

本次针对STM32F103ZE小车做小步纯软件增强，默认保留RAM黑匣子、首因STOP/重启复位、有界误差趋势观测、四方向独立补偿参数及ALIGN两条轻趋势规则，并修复原四种致命异常处理未撤PWM的缺口。普通TRACK滤波、非线性P、自适应base、低速PI缩放和turn曲线均已实现并测试，但开启后发生模型回归，因此默认关闭；预测也默认关闭，只供明确启用的实验。当前默认模型成功率、误差、直线翻转和完成时间等于基线；本次没有证据宣称默认直线更静、缓弯更稳、实车5 RPM更连续或右转问题已解决。

保留 TRACK → CORNER_CONFIRM → CORNER_APPROACH → TURN_FAST/TURN_SLOW → ALIGN → EXIT → TRACK，以及 SEARCH、Recovery Session 与有限安全停车。硬件映射、几何、编码器标定、方向宏、原模型和原测试通过门槛保持不变；失败数据保留，不扩大安全预算换通过率。

## 1. 阶段 0 基线与测量口径

修改前源码已保存为 `fix4_baseline`。另提供[完整基线归档及各阶段控制器快照](checkpoints/README.md)，含fix4_baseline、fix5_stage0及fix5_step1～8的commit和控制器SHA256[manifest](checkpoints/manifest.json)，压缩包脱离.git后仍可核验阶段源码。完整源码确认见[当前基线源码审核](source_audit_baseline.md)；旧报告的早期原源码扫描不能替代本轮审核。

| 基线组别 | 通过 / 总数 | 失败与含义 |
|---|---:|---|
| 原有单元 | 94 / 94 | 无失败。保留原断言与门槛。 |
| 标准几何 | 34 / 34 | 原矩形 TRACK 前探头误差<35 mm；圆弧全程误差<25 mm。另记录所有状态误差。 |
| 高摩擦压力 | 9 / 10 | 唯一失败 `pressure_square_-1`：22.64 s安全STOP，reason=9，未完成圈数要求。 |
| 30个固定扰动种子 | 27 / 30成功 | 3次安全STOP、0次失控；不把保护停车计为赛道完成成功。 |

30种子基线的 TRACK 最大前探头横向偏差：样本最大13.80 mm，中位数10.91 mm、p90为12.73 mm。全30项运行终止时间中位数42.82 s，包含3次较早STOP；成功27项的完成时间为最小41.84 s、中位数43.12 s、最大44.06 s。失败停车时间不混称完成时间。直线correction翻转次数分布为最小9、中位数50、p90为59、最大73。标准/压力/随机均执行真实C控制器，传感器、编码器量化、电机响应与轨迹由独立模型提供。

指标同时记录：成功率、失控次数、保护停车次数、最大横向偏差、完成时间、直线 correction 符号翻转次数。整数/枚举逐周期精确比对；浮点行为基线采用同GCC、同 `-O2`、`-ffp-contract=off` 构建，禁止 `-ffast-math` 和 `-march=native`。golden仅比较state、传感器解读、line error、requested L/R RPM、PWM、turn/recovery状态、stop_reason等行为字段，排除Trace索引、日志序号、墙钟时间等观测字段。新增算法不能通过改golden让回归通过。

原有测试包含 `CAR_ENCODER_FAULT=9`、`CAR_CONTROL_FAULT=18` 的数值状态契约。原94项在FIX5全关构建上94/94通过；在最终默认构建上90项通过、4项失败，均因故障状态已统一为 `CAR_STOP=8`，旧断言仍要求9或18。具体为ControlTests的test_11/test_14/test_18和SafetyBoundsTests的test_fault_stop_stays_latched_until_explicit_restart，详见[默认旧测试日志](final/legacy_default.log)。这是明确且保留的数值ABI变化，不能称默认原94项全部通过。统一STOP、故障原因、目标/PWM归零及锁止重启由新测试验证；人工USER_KEY停车仍清四方向负载补偿，以保持原校准API契约。

| 基础设施证据 | 结果 |
|---|---|
| 全现有场景逐周期golden | 已导出142327条规范记录，其中141076条Native_Step周期记录，另含Start/Stop与直接速度/故障逻辑观测；[最终源码全关比对](final/all_off/comparison.json)精确一致，float容差0。 |
| 基线GCC warning/编译 | GCC14.2.0、`-O2 -Wall -Wextra -Wdouble-promotion -ffp-contract=off -Werror`；18个Keil引用C单元/63个文件引用和6旧宏分支检查通过。证据：[metadata](baseline/metadata.json)、[build](baseline/build.log)。 |
| 独立ASan/UBSan构建 | 最终全关/默认/全功能（预测仍0）各40470周期、合计121410周期，无越界或UB报告；不参与行为或性能基线。证据见第9节。 |
| baseline固定种子列表/扰动分布 | [runner结果](baseline/random_results.json)，10500～10529。电机gain±5%、deadPWM±0.5%、breakaway±1%、响应±6%、初始偏移±4 mm/航向±1°、时变dead摩擦±0.4%、单帧噪声概率0.005；30种子共317帧噪声。 |

## 2. 上一版实际已确认的功能

下表行号来自 `fix4_baseline`，对应修改前源码；完整状态出口、停车路径与数值审查见[source_audit_baseline.md](source_audit_baseline.md)。

| 已有功能 | 修改前真实源码位置 | FIX5处理原则 |
|---|---|---|
| Recovery Session/整体失线预算 | `hardware/Track.c:611`、`:618`、`:654`；`hardware/CarConfig.h:227` | 保留12 s、4次、300°、700 mm，不因瞬间回线清零。 |
| 禁止默认转向 | `hardware/CarConfig.h:138`、`:256`；`hardware/Track.c:592` | `CORNER_DEFAULT_DIR=0`，有证据才选方向。 |
| 左右镜像方向 | `hardware/Track.c:115`、`:500`、`:530`、`:718`、`:732` | 保留误差/票/目标/yaw符号关系，未改方向宏。 |
| TURN_FAST/SLOW | `hardware/Track.c:716`；`hardware/CarConfig.h:175` | 保留状态与50°/70°门槛；只可连续化RPM规划。 |
| ALIGN | `hardware/Track.c:686`、`:777` | 保留稳定接线及时间/距离/追加yaw保护。 |
| EXIT分阶段提速 | `hardware/Track.c:804`、`:825` | 原28→40→60与原稳定确认保留。 |
| target RPM slew | `hardware/Track.c:460`、`:469` | 原状态专用斜率与换向过零保留。 |
| PWM slew | `hardware/Track.c:372` | 原增3%/减4%每20 ms与即时安全撤驱动保留。 |
| 低速多周期测速 | `hardware/Track.c:437`；`hardware/CarConfig.h:99` | 5帧窗口和8～24 RPM连续混合保留。 |
| 四方向offset/负载记忆 | `hardware/Track.c:358`；`hardware/CarConfig.h:84`、`:116` | 路径独立；本轮已将反向正向别名改为同数值独立常量，并做4个改参构建、16组方向比较。 |
| 编码器反向窗口 | `hardware/Track.c:399`；`hardware/CarConfig.h:112` | 零count不抹证据，宽容从实际反向PWM开始。 |
| 状态滞回 | `hardware/Track.c:511`、`:675`、`:793`、`:825`、`:835` | 保留计分、连续确认、重新武装时间+距离；不新建模块。 |
| anti-windup | `system/PID.c:40`、`:44`；`hardware/Track.c:371`、`:374` | 条件积分、饱和/斜率冻结、有界负载转移保留。 |
| 换向过零 | `hardware/Track.c:463`、`:315`、`:322`、`:332`；`hardware/Motor.c:26` | 先零命令/撤驱动，清换向瞬态，只恢复对应方向负载。 |

确认的硬件值：PF0～PF7，黑低电平；左/右251.0/265.35 count/rev；轮径44 mm、轮距133 mm、探头前置175 mm；Encoder(-1,+1)、Motor(+1,-1)。没有为模型通过率修改这些数值。

## 3. FIX5实际修改文件

生产源码只改 `hardware/Track.c`、`hardware/Track.h`、`hardware/CarConfig.h`、`user/stm32f10x_it.c`。前三者承载FIX5增量，异常文件只增加 `FaultMotorStop`，在HardFault/MemManage/BusFault/UsageFault死循环前直接将TIM2 CCR3/CCR4写0。不新增固件C单元，原 `.uvprojx` 引用保持；Motor/Encoder/PWM/GraySensor/PID、几何和方向实现不改。

测试基础设施修改 `tests/native_api.py`、`tests/native/harness.c`、`tests/test_geometry.py`、`tests/test_oled_buffer.py`、`tests/run_verification.py`、`scripts/check_build.py`；原单元断言未改，几何文件只增加观察指标/场景标签，OLED测试只补编译告警参数。新增 `tests/test_fix5.py`、`tests/test_fault_stop.py`、`tests/native/fix5_observers.inc`、`tests/fix5_units.py`、`tests/fix5_random.py`、`tests/fix5_sanitizer.c`、`tests/run_fix5_sanitizer.py`、`tests/run_fix5_mutations.py`、`scripts/fix5_golden.py`、`scripts/fix5_stage.py`、`scripts/fix5_matrix.py`、`scripts/fix5_verify.py`。`tests/geometry_results.json`、`tests/pressure_results.json`是重跑输出。

辅助文件修改 `README.md`、`.gitignore`、`SHA256SUMS.txt`，分别说明默认/实验边界及复验入口、忽略生成构建产物、核验交付文件。`docs/fix5/`新增此报告、基线/最终源码审核、不可覆盖golden、各阶段成功/失败日志、编译/内存检查证据及checkpoints归档/manifest；它们不属于固件源码。

## 4. 分步实施与保留决定

每步只有编译、相关新增测试、关键回归与同种子基线比较通过后才保留；退化先修复/回退，再进入下一步。功能关闭时不改变规范控制输出。

| Step | 功能 | 开关 | 新增测试 | 回归测试 | 基线变化 | 是否保留 |
|---|---|---|---|---|---|---|
| 1 | RAM Trace与统一停车/首因/重启 | TRACK_TRACE_ENABLE，默认1 | 7 / 7（含18种真实停车来源） | 全关原94 / 94；全关golden142327记录精确；ASan40470周期 | 标准34 / 34、压力9 / 10、随机27 / 30，行为指标无退化；默认旧断言4项数值ABI差异见第9节 | 保留，tag `fix5_step1` |
| 2 | 3帧数字多数滤波，raw安全证据 | TRACK_SENSOR_FILTER_ENABLE，默认0 | 滤波3 / 3；Trace7 / 7 | [最终回退比较](step2_final/comparison.json)：全关golden精确；34 / 34、9 / 10、27 / 30，0回归 | 开启候选出现标准/随机新增STOP；具体失败见下表 | 开启行为回退；保留实验实现，tag `fix5_step2` |
| 3 | 中心非零小增益、连续三段P | TRACK_NONLINEAR_FOLLOW_ENABLE，默认0 | 本步相关逻辑+Trace 8 / 8 | [最终回退比较](step3_final/comparison.json)：全关golden精确；34 / 34、9 / 10、27 / 30，0回归 | 修复后压力10 / 10，但标准33 / 34和随机23 / 30仍退化 | 开启行为回退；保留实验实现，tag `fix5_step3` |
| 4 | 有界4样本error_rate观测；P-only预测默认关 | TRACK_ERROR_TREND_ENABLE=1；TRACK_ERROR_PREDICT_ENABLE=0 | 趋势2 / 2；Trace7 / 7 | [本步比较](step4/comparison.json)：全关golden精确；原94全关通过 | 标准34 / 34、压力9 / 10、随机27 / 30；请求轮速不变 | 保留趋势观测；预测只供实验，tag `fix5_step4` |
| 5 | TRACK自适应base、快减慢升、状态交接 | TRACK_ADAPTIVE_SPEED_ENABLE，默认0 | 相关逻辑5 / 5（启动/趋势/恢复/交接） | [最终回退比较](step5_final/comparison.json)：全关golden精确；34 / 34、9 / 10、27 / 30，0回归 | 候选标准30 / 34、压力8 / 10、随机22 / 30；正常新增失败 | 开启行为回退；保留实验实现，tag `fix5_step5` |
| 6 | 每轮连续低速Kp/Ki缩放、四方向独立性 | TRACK_LOW_SPEED_ZONE_ENABLE，默认0；方向常量独立修复常开 | 低速相关5 / 5 | [最终回退比较](step6_final/comparison.json)：全关golden精确；34 / 34、9 / 10、27 / 30，0回归 | 低速缩放候选标准28 / 34、压力8 / 10、随机17 / 30；独立常量默认数值不变 | 缩放默认回退；保留四方向参数修复，tag `fix5_step6` |
| 7 | 转速在原50°/70°前各10°区间连续降至下档 | TRACK_TURN_CONTINUITY_ENABLE，默认0 | 最终test_13连续/单调/slew扫描通过；默认构建该项skip | [最终回退比较](step7_final/comparison.json)：全关golden精确；34 / 34、9 / 10、27 / 30，0回归 | 两曲线均新增旧成功种子STOP，拒绝以其它种子恢复抵消 | 开启行为回退；保留实验短区间曲线，tag `fix5_step7` |
| 8 | ALIGN连续恶化减base/适度增P、频繁翻转降P | TRACK_ALIGN_TREND_ENABLE，默认1 | 新规则3 / 3 | [最终比较](step8/comparison.json)：原94全关通过；golden142327记录精确；34 / 34、9 / 10、27 / 30 | 无新增STOP/失控，误差/时间/翻转指标与基线相同 | 保留，仍经原target slew/限幅 |

Trace的host sizeof为32字节，200条数组6400字节，加两uint16索引/计数与三uint8管理状态7字节，共6407字节显式存储（约6.26 KiB），覆盖4 s；小于8 KB。这不包含链接器对齐填充，ARM map尚未测。写入只在主循环20 ms任务内一次，ISR不写，不动态分配，不写Flash，不运行期printf。控制任务内的故障先记录最终原因样本，再冻结；控制周期外按键停车立即冻结，不额外写一条Trace，用户原因由 `Track_GetDebug` 读取。重新启动清原因并解冻。

未保留默认运行行为的候选也保留证据，不能以压力场景改善抵消标准场景新增STOP：

| 候选 | 标准几何 | 压力 | 30种子 | 新增旧成功失败（标准/压力/随机） | 结论 |
|---|---:|---:|---:|---:|---|
| [Step 2原3帧滤波](step2/comparison.json) | 27 / 34 | 8 / 10 | 20 / 30 | 7 / 1 / 7 | 普通控制链额外延迟；有停车和误差超标，退化。 |
| [Step 2修raw几何/edge](step2_repair/comparison.json) | 28 / 34 | 8 / 10 | 20 / 30 | 6 / 1 / 7 | 仍有标准场景新增失败，拒绝默认开。 |
| [Step 2只滤中心](step2_center/comparison.json) | 27 / 34 | 8 / 10 | 21 / 30 | 7 / 1 / 6 | 仍退化，拒绝。 |
| [Step 2缩位置tau至20 ms](step2_latency/comparison.json) | 25 / 34 | 9 / 10 | 17 / 30 | 9 / 0 / 10 | 改原位置响应仍未消除退化，拒绝。 |
| [Step 3初始P](step3/comparison.json) | 32 / 34 | 8 / 10 | 24 / 30 | 2 / 1 / 3 | 正常/随机新增STOP，拒绝。 |
| [Step 3仅P映射并保留原D](step3_preserve_d/comparison.json) | 33 / 34 | 10 / 10 | 23 / 30 | 1 / 0 / 4 | `circle_-1_radius200`新增SEARCH STOP；压力另有2项明显TRACK误差恶化。 |
| [Step 5自适应base](step5/comparison.json) | 30 / 34 | 8 / 10 | 22 / 30 | 4 / 1 / 6 | 一个旧失败恢复，但6个旧成功随机新增失败；逻辑通过不抵消。 |
| [Step 6低速Kp/Ki缩放](step6/comparison.json) | 28 / 34 | 8 / 10 | 17 / 30 | 6 / 1 / 10 | 低速数值逻辑通过，完整负载/接线回归失败。 |
| [Step 7全角度线性降速](step7/comparison.json) | 34 / 34 | 9 / 10 | 28 / 30 | 0 / 0 / 1 | 旧成功seed10501新增ALIGN STOP(reason7)；两个旧失败恢复不能抵消。 |
| [Step 7短区间过渡](step7_blend/comparison.json) | 34 / 34 | 9 / 10 | 25 / 30 | 0 / 0 / 3 | 旧成功10501/10509/10512新增STOP；不继续按种子调参。 |

所有候选新增失控为0；新增失败不全是停车。例如Step 2的 `loaded_circle_+1_radius600`误差25.01 mm和 `pressure_circle_+1_radius200`误差26.53 mm均在未STOP时超出原25 mm成功门槛；Step 5对应压力圆误差27.42 mm。comparison中的regressions还包含明显误差恶化，不能把净成功数变化、新增失败数和regressions数量混为一项。候选中时间比>1.30的记录均对应基线原失败场景，比较的是终止/完成时间，不能称原成功完成速度退化超过30%。

Step 2/3/5/6/7的控制变化均默认0，只供明确开启后的实验和独立逻辑测试。默认交付没有引入新的传感器多数抗噪、非线性纠偏、自适应base、低速gain缩放或turn平滑行为，不能宣称默认直线/弯道/低速已经因此改善。四方向offset/前馈独立定义保留，数值仍为原估计，修改单个方向不再连带其它方向。各默认回退比较链接见分步表，候选失败目录最终列入证据表。

Step 7候选中的 `pressure_square_-1` 仍失败，部分候选因高摩擦更早触发编码器reason2；较早保护停止不等于右转接线成功，也不是原压力失败已解决。默认恢复原turn后保留基线22.64 s、reason9有限扫描停车。TURN_FAST/SLOW、70°最低认线角、105°转角、15°局部反扫与Recovery/SEARCH硬限制均未扩大。

趋势使用4个error样本，单位sensor-position error/s，限幅±100 error/s；主路径只做观测写Trace，默认开启不改变轮速。预测风险：原位置PD已经对滤波误差求一阶微分，若把 `error+K*error_rate` 再送原PD，会重复一阶趋势且产生二阶差分项。因此最终实验实现启用PRED时把line PID的Kd设为0，再加 `Kp×20 ms×error_rate` 的P偏置，替代原D，不叠加也不对predicted再次求导；关闭PRED时保留原PD。预测默认0，本轮31项新测试的all-on构建仍是PRED0；预测源码审查/单开编译不能充当预测运行效果、完整模型或默认稳定性证明。

自适应实验参数为MAX60 RPM、MIN36 RPM、起步28 RPM，base每20 ms恢复≤2 RPM、降速≤4 RPM，error系数4.5 RPM/error，向外恶化rate系数0.08 RPM/(error/s)。rate惩罚只取误差向外增大的部分，回正不机械同罚。普通TRACK另保内轮请求≥12 RPM，接近零内轮由显式pivot/turn状态表达。状态交接只读请求RPM，非TRACK优先显式base，双轮同向前进可取请求平均，spin/pivot保持最近有效forward base；从EXIT低于MIN回TRACK时不钳高状态量，只限制目标并按恢复slew爬升。上述语义逻辑验证通过，但物理回归未通过，因此默认不用此规划。

ALIGN只新增两条规则：|error|以>0.05 error增量连续增大3次时，base由24降至22 RPM、P gain为原115%；6帧120 ms内至少3次有效正负翻转（只计|E|>1）时，gain为当前70%。真正改善时不额外降速，改善后解除恶化标记；幅值不再变化时保持已经触发的谨慎策略。无新状态、无更改中心稳定计数/退出门槛；requested仍限总差速20 RPM，执行仍过原slew。直接逻辑测试确证规则，当前模型未出现完成率改善，不能把保留规则表述成已解决压力右角。

## 5. 普通循迹数据流

```text
Raw IR (once / 20 ms, PF0..7)
  -> Light Filter (FILTER=0: bypass; 1: 3-frame bit majority)
  -> Line Error (contiguous black blocks, mirrored signed error)
  -> Error Trend (TREND=1: 4 samples, bounded error/s observation)
  -> Base Speed (ADAPTIVE=0: original; 1: fast fall / slow rise)
  -> Follow Correction (NONLINEAR=0: original PD; 1: continuous P)
  -> Left / Right Requested RPM (base +/- correction, motion mode)
  -> Target Slew (original state limits, reversal through zero)
  -> Speed Control (original estimator / PI / feedforward / anti-windup)
  -> PWM Slew (original per-cycle rise / fall limits)
  -> Motor (original GPIO / direction signs / TIM2 PWM)

raw -> existing evidence counters -> state machine / Recovery / STOP
20 ms main task -> RAM Trace -> STOP freeze -> explicit restart
PRED=1 experiment: P lookahead replaces original line D (Kd=0)
fatal handlers -> direct TIM2 CCR3=0, CCR4=0 -> original latch loop
```

这是功能开启的数据链。默认 FILTER/NONLINEAR/ADAPTIVE/LOW_SPEED/TURN/PRED为0，直接使用对应原路径；TREND默认1仅观测。默认真正改变控制的增量为统一STOP语义和ALIGN轻规则，不是新增控制状态机。

## 6. 状态与raw/filtered使用

基线GraySensor每20 ms只读一次GPIO，无周期内多次采样。因此时间多数无需新增采样中断或延时。历史以启动首次有效mask初始化，真实稳定阶跃应在第二次控制样本反映，即约20～40 ms。

| 状态 | filtered用途 | raw用途 | 理由 |
|---|---|---|---|
| TRACK | 控制line error、趋势、实验非线性P/base；侧块/歧义分类 | 失线/全白全黑、线形/边界、再武装和Recovery稳定清零 | 实验控制可过滤单帧，但安全计数不叠多数延迟。 |
| CORNER_CONFIRM | 满足filtered宽带条件时才增加置信分；继续follow时的控制位置 | 宽带几何、方向票、白线确认、原计时/推进预算 | raw宽带才能建立/维持候选，残留filtered不能清除真实lost计时。 |
| CORNER_APPROACH | 仅可保留观测，不参与推进控制 | Recovery证据；推进控制实际用编码器距离、两轮实速和yaw | 几何推进不依赖滤波位置，也不修改175 mm探头前置。 |
| TURN_FAST/SLOW | sensor_filtered只记录，不用于接线/控制error | AcquireLine原6帧计数、raw位置趋势、最小转角与范围；debug/Trace line_error | 原接线已有确认，不能叠3帧延迟；现场error与实际控制来源一致。 |
| ALIGN | 差速error及轻趋势规则 | 丢线计时、中心稳定计数、原time/distance/yaw预算 | 控制平滑，进入EXIT仍用严格raw连续确认。 |
| EXIT | 普通follow纠偏位置 | 白线宽容、稳定提速/回TRACK条件、Recovery证据 | 噪声不打舵，稳定接线后才恢复速度。 |
| SEARCH | sensor_filtered只记录，原搜索RPM逻辑不借滤波放宽预算 | 找线计数/raw位置/方向线索/局部角度/时间/次数；debug/Trace line_error | 搜索原多帧证据保持，失败按原预算停止。 |

上述filtered位置/置信路径只有明确设置FILTER=1才执行。默认FILTER=0，各状态控制、几何、确认都使用原raw路径，Trace的sensor_filtered此时等于sensor_raw。原已有多帧滞回继续抗偶发状态触发，不能将其算为FIX5新增默认多数滤波。

实验FILTER=1时第二次DecodeLine会把side_observed、side_error、line_ambiguous更新为filtered分类，而sensor_mask/count/raw_error仍为raw；分类和安全证据不是同一帧来源。该组合未通过完整模型防回归，保留为实验局限。最终还修正TURN/SEARCH的debug和Trace error，明确取实际接线/控制所用raw值。

## 7. 稳定性变化如何得到证据

实际保留的改善是可追溯停车、首因/重启正确、四方向参数调节互不连带，以及ALIGN恶化/摆动时的保守规则。模型成功率、误差、直线翻转和完成时间未改善；普通循迹输入与速度候选因回归默认关。下表区分独立逻辑证据和未得到的默认/实车结论。

| 目标 | 实际证据 | 默认/实车结论 |
|---|---|---|
| 直线更安静 | FILTER+ADAPTIVE逻辑构建500周期含指定单帧噪声，correction幅值≤1e-4、翻转0，状态保持TRACK，base恢复≥95% | 默认这两项关闭，随机直线翻转与基线相同；没有默认或实车改善证据。 |
| 进弯提前收速度 | 自适应同E不同趋势、渐进偏线、回正恢复/交接逻辑通过 | 完整模型新增失败，默认回退；趋势默认仅观测。 |
| 低速更连续 | 5/8/12/18非零目标保留，实验缩放连续；100 ms无脉冲积分有界，过零回归通过 | 非零低速和原测速本就存在；缩放默认关，真实5 RPM能力需A实测。 |
| 左右对称、方向调参独立 | 逻辑镜像float≤1e-4；4个方向offset改参构建/16组比较通过 | 独立常量修复可用；CPR不同，四方向真实摩擦/响应仍未标定。 |
| 单帧噪声更难影响控制 | 实验多数真值、1024个固定随机8-bit参考、指定序列和bypass mutation检测通过 | 实验FILTER完整模型退化，默认不用；安全/接线计数维持raw。 |
| ALIGN更谨慎 | 改善保持、连续恶化减base/增P、频繁翻转降P共3项逻辑通过，默认完整模型无新增失败 | 模型完成率未提升；未解决压力右角和3个随机ALIGN停车。 |
| 原状态机与停车更可追溯 | 原全部预算保留；软件18类真实停车首因/冻结/重启检查，致命异常独立最小撤PWM修复 | 所测host路径更易诊断；故障停车不是完成圈数，也不代表ARM异常实测。 |

如果某项模型退化，则默认关/回退并保留失败证据。完成时间只观察；超过基线×1.30必须解释，不能用慢5%作为单独回退标准。

## 8. 最重要的实车参数（最多8个）

以下8项已与最终CarConfig.h核对。反向offset原为正向别名，本轮已改为同数值独立定义；这些offset仍是原估计初值，不等于实车四方向已标定。默认关闭功能的参数会明确标注，调它不会改变当前默认普通TRACK行为。

| 参数 | 当前值 | 建议范围 | 增大效果 | 减小效果 | 什么时候调 |
|---|---:|---|---|---|---|
| LEFT_DRIVE_OFFSET_PWM | 6.478873% | 实测最低稳定PWM附近；初调原值±2%，每次约0.5% | 左正向更易持续驱动，过大易超速 | 更柔和，可能停启 | A左正向出现一卡一冲且符号正确时。 |
| LEFT_REVERSE_DRIVE_OFFSET_PWM | 6.478873%（独立定义） | 同上，单独测左反向 | 左反向更易驱动 | 左反向更易停启 | A左反向、单左角内轮异常。 |
| RIGHT_DRIVE_OFFSET_PWM | 4.672686% | 同上，单独测右正向 | 右正向更易驱动 | 右正向更易停启 | A右正向、单左角外轮异常。 |
| RIGHT_REVERSE_DRIVE_OFFSET_PWM | 4.672686%（独立定义） | 同上，单独测右反向 | 右反向更易驱动 | 右反向更易停启 | A右反向、单右角内轮异常。 |
| SPEED_VERY_LOW_RPM | 8 RPM（LOW=24；缩放默认关闭） | 依据四方向最低持续RPM和量化稀疏范围，大于0且小于LOW | 弱PI区扩大，量化冲击降低但闭环回正更慢 | 正常PI更早参与 | A记录四方向最低持续转速后，开启实验缩放前。 |
| TRACK_MIN_FOLLOW_BASE_RPM | 36 RPM（adaptive默认关闭） | 必须让常规TRACK内轮留在实测持续区；不能机械设得越低越好 | 普通进弯更快，内轮较少落极低速 | 进弯更保守，过低内轮停启 | B/C结合base−correction与A低速能力；实验开关明确开启时才生效。 |
| CORNER_FIND_RPM | 18 RPM | 12～24 RPM，且对应方向能持续驱动 | 更快找线，易扫过接线窗口 | 接线更柔和，过低停启 | D/E在70°后错过线且A已通过时。 |
| TRACK_ALIGN_KP | 4 RPM/error | 2～5，保持原差速限幅 | 回正更强，过大可能反复摆动 | 更柔和，可能回正不足 | D/E已接线但ALIGN偏差扩大/翻转时。 |

先修方向、最低持续速度与机械阻力，再调位置/速度规划。几何和CPR必须重新实际测量才能改；本次不改。安全预算不是优先调参项。

## 9. 完整测试、Mutation与编译矩阵

最终同源码完整复验已在2026-10-02 02:24:13～02:25:12 UTC执行，真实命令、退出码和逐项统计见[summary.json](final/summary.json)及[完整执行日志](final/verification.log)。18项必需检查全部满足，`required_checks_passed=true`；`all_tests_passed=false`，因为默认旧单元仍有下列4项数值ABI断言失败，原压力/随机失败也完整保留。不能将前者写成“所有测试全部通过”。

| 最终组别 | 通过 / 总数 | 与基线差异 / 限制 |
|---|---|---|
| 原有单元（全关） | 94 / 94 | 最终源码重跑，原测试断言与门槛不变，[日志](final/all_off/units.log)。 |
| 原有单元（最终默认） | 90 / 94；4项数值ABI失败 | [日志](final/legacy_default.log)；仅统一STOP的state8替代旧state9/18，未改原断言。修复前真实USER负载复位回归另见下文。 |
| 新增FIX5（最终默认） | 17通过 / 31；14 skip；0失败/错误 | [日志](final/logic_default.log)和[统计](final/logic_results.json)。关闭实验功能的14项skip不算通过；覆盖默认趋势/ALIGN、STOP/Trace、非零低速、换向和四方向独立性。 |
| 新增FIX5（所有实验功能开、PRED=0） | 31 / 31；0 skip/失败/错误 | [日志](final/logic_all_on.log)。逻辑层独立于plant，含滤波/镜像/连续增益/base交接；此结果不推翻完整模型退化。 |
| 原Recovery wrapper | 内嵌24 / 24，两种新测试构建均执行 | `test_17_existing_24_recovery_tests_are_unchanged`按原断言检查全部预算，wrapper本身在31项中计1项，不重复计为另24个新增测试。 |
| 四方向offset独立性 | 4个单独改参构建 / 16组比较通过 | 每次只改变一个L+/L−/R+/R−offset，只有对应方向输出变化；默认四个初值不变。 |
| 致命异常独立撤PWM | host 4 / 4 | [日志](final/fault_stop.log)；`tests/test_fault_stop.py`执行真实四个handler，只用共享假TIM2地址；CCR3/4归零，其余寄存器保持，handler持续锁止。不是ARM异常触发或真实PWM输出实测。 |
| 全关golden | 142327 / 142327记录 | [最终比较](final/all_off/comparison.json)：单元11293、几何/压力70500、随机60534；规范字段精确一致，float容差0，观测字段不参与。 |
| 标准几何 | 34 / 34 | 原几何与原门槛，0安全STOP/失控/新增回归，[结果](final/geometry_results.json)。 |
| 高摩擦压力 | 9 / 10 | 基线失败pressure_square_-1保留，1安全STOP、0失控/新增回归，[结果](final/pressure_results.json)。 |
| 30种子最终 | 27 / 30成功 | 3安全STOP、0失控/新增回归，各指标同基线，[结果](final/random_results.json)。 |
| GCC warnings/引用检查 | 最终18个C单元 / 63个引用 / 6个旧宏分支通过 | [日志](final/build.log)及summary内build字段；使用 `-Wall -Wextra -Wdouble-promotion -Werror -ffp-contract=off`，未放开新增应用double提升。原OLED字体missing-braces及64位host/32位MCU地址转换例外在结果中单列。 |
| ASan/UBSan | 3 / 3构建，每项40470周期 | [全关](final/sanitizer_all_off.json)、[默认](final/sanitizer_default.json)、[全部实验功能且预测0](final/sanitizer_all_features.json)。无越界或UB报告；每项30k固定随机输入与200条Trace读取（Trace关闭项0条）；源码hash运行前后一致。不评价物理成功率或时间。 |
| 编译矩阵 | 最终12配置 / 24检查全部通过 | [最终矩阵](compile_matrix.json)和[日志](final/compile_matrix.log)：默认、全关、全部实验开且PRED0及9开关单独开。首轮14/22（8项观察器/配置头不一致而失败）保存在[修复前记录](final/compile_matrix_before_repair.json)。 |
| Mutation | 修复验证脚本后8 / 8检出 | [最终结果](final/mutations.json)；reference全通过、mutant各1断言失败、无error/skip；首轮7/8及修复原因保留，下表详述。 |
| STM32 ARMCC5链接/HEX | 未执行 | 没有Keil ARMCC或真实板卡，host GCC不代表ARM通过。 |
| 实车/烧录 | 未执行 | 没有真实小车，不能声称右转完全解决或5 RPM稳定。 |

默认旧单元首次执行89/94，除4项数值ABI变化外，`test_calibration...test_11_load_compensation_survives_inner_stop_and_is_bounded`发现USER停车后旧负载补偿未清，这是实际API回归。已修正EnterStop仅对USER_KEY清load_compensation及四方向direction_load；重跑该项通过，故障停车仍保留现场。修复前结果保存在[日志](final/legacy_default_before_repair.log)，不能把5个原失败都解释为ABI差异。

统一host复验入口是 `python3 scripts/fix5_verify.py`（旧 `tests/run_verification.py` 转入同一入口）。它按一个源码快照顺序检查引用/严格警告、12配置矩阵、全关golden、默认标准/压力/随机模型、默认与实验新逻辑、默认旧单元的四个精确ABI诊断、四个致命异常handler、Mutation及3个独立sanitizer；任一新增失败、产物缺失或运行期间源文件hash变化都不能通过。12配置是全关、默认三个开关、全部实验开且PRED0及9个开关各单开；每配置检查真实harness共享库编译和user/main.c语法。GCC主机矩阵不包含ARM汇编、ARMCC链接或硬件时序测量。

交付前还把源码导出到无`.git`、无host库/profile的独立目录，重新执行整个入口：[导出复验](final/export_replay.json)18/18必需检查通过，117份源hash与主验证完全相同；[运行日志](final/export_replay.log)。导出树的Git状态明确不可用、clean=null，不伪造“干净”状态；工作区干净Mutation证据是另一次提交后的真实Git运行。精确golden要求Linux及同一 `gcc (Debian 14.2.0-19) 14.2.0`，基线SHA或编译器变更会被拒绝。

本轮实际核验117项源码/测试/工程文件的运行前后SHA256完全一致，`source_changed_during_verification=[]`；8项基线输入（metadata、原单元manifest、三组模型结果、三份golden）前后hash也一致。生成配置头的hash和默认9开关均核验，final默认数据与baseline逐项相同，所有新增regressions为空。完整hash、命令时间戳、退出码及产物路径均保存在summary；这证明复验针对同一实际源码快照，不依靠覆盖旧golden或运行中修改配置。

最终默认与基线30种子比较，误差、运行终止时间、完成时间和直线翻转完全相同，没有任何时间超过基线×1.30的默认场景：

| 指标/口径 | 基线 | FIX5默认 |
|---|---:|---:|
| 30种子成功率 | 27/30=90% | 27/30=90% |
| 安全停车率 / 失控率 | 3/30=10% / 0/30 | 3/30=10% / 0/30 |
| 最大TRACK前探头误差，p50 / p90 / 样本max | 10.91 / 12.73 / 13.80 mm | 相同 |
| 最大所有状态前探头误差，p50 / p90 / 样本max | 199.225 / 200.67 / 201.83 mm | 相同；推进/旋转阶段可远离线，不能混称TRACK误差 |
| 成功27项完成时间，min / p50 / max | 41.84 / 43.12 / 44.06 s | 相同 |
| 全30项运行终止时间，p50 / p90 / max | 42.82 / 43.86 / 44.06 s | 相同；含失败STOP时间 |
| 直线correction符号翻转，p50 / p90 / max | 50 / 59 / 73次 | 相同 |

保留失败不能隐藏：`pressure_square_-1`只完成0.488圈，22.64 s进入STOP reason9，PWM0；TRACK最大误差13.64 mm，全状态204.02 mm。原高摩擦假设为左/右dead13/15%、起转20/22%、响应0.22/0.28 s。第一右角接回，第二角约99.8°接线后ALIGN再次丢线，15°附近扫描耗尽而停；原轨迹见[压力失败记录](../压力失败轨迹.txt)，本轮[结果](final/pressure_results.json)与基线一致。ALIGN轻规则没有使该压力场景完成。优先实测右转对应左正/右反持续能力、慢转惯性和ALIGN回正，不扩大搜索范围。

随机失败 `10508`、`10514`、`10522`分别在9.46/29.02/9.50 s进入ALIGN保护STOP reason7，与基线相同；没有原成功种子退化。它们属于当前模型中接线未能在原ALIGN预算内稳定的未解决场景，不算成功。当前模型没有新增失控，仍不能推导实车稳定性。

新增需求到测试的映射如下；31项执行计数和skip已在上表列出。实验功能的独立逻辑通过不改变第4节完整模型退化的事实。

| 用户编号 | 实际测试或证据 |
|---|---|
| 1中心500周期噪声 | `FilterFollowTests.test_01_center_500_cycles_single_frame_noise_is_quiet`，只有显式FILTER+ADAPTIVE构建执行。 |
| 2单帧指定序列 | `test_02_single_frame_noise_does_not_change_line_error`。 |
| 3多数真值/逐bit参考/阶跃 | `test_03_majority_truth_table_and_random_bit_reference`（8真值+1024固定随机）、`test_03b_stable_step_reaches_filter_within_two_cycles`。 |
| 4进弯平滑/5回正恢复/6同E不同趋势 | `test_04_progressive_error_slows_and_slew_remains_bounded`、`test_05_recentering_recovers_base_at_configured_rate`、`test_06_same_error_outward_trend_slows_earlier`。 |
| 7镜像/非线性连续/有界趋势 | `test_07_mirrored_logic_swaps_targets_and_negates_correction`、`test_03c_nonlinear_is_continuous_odd_monotone_and_has_no_dead_zone`、`test_07b/07c`趋势限幅/重置/单位/镜像。 |
| 8～12低速/阈值连续/100 ms无脉冲/四方向独立/换向 | `SpeedContinuityTests.test_08`～`test_12`；四offset分别独立编译改值，观察四方向输出。 |
| 13转弯连续/单调/slew | `TurnAlignHandoffTests.test_13_turn_requested_speed_is_continuous_monotone_and_slewed`，扫描0/20/40/50/60/70°及拐点邻域。 |
| 14改善/15恶化/16翻转 | `test_14_align_improving_error_keeps_original_strategy`、`test_15_align_persistent_worsening_slows_and_strengthens_gain`、`test_16_align_frequent_sign_flips_damp_gain`。 |
| 17Recovery所有旧预算 | 原 `RecoveryTests.test_09`～`test_16`、`test_20`等，原断言全关保持；新增真实停车来源检查开启统一STOP。 |
| 18Trace/STOP完整契约 | `TraceStopTests.test_18a`～`test_18f`，覆盖回卷、冻结、首因、USER独立、重启及18种真实停车来源。 |
| 19EXIT低base/pivot/spin/启动交接 | `test_19a_exit_low_base_enters_track_without_lower_clamp_jump`、`test_19b_spin_and_pivot_do_not_pollute_forward_base`、`test_19c_start_uses_configured_forward_base`。 |
| 20时间戳回绕 | `test_20_clock_wrap_preserves_controller_and_trace_cadence`及旧时钟/Recovery回绕测试。 |
| 21随机扰动回归 | 固定10500～10529；[最终同种子比较](final/comparison.json)，27成功/3安全停车/0失控。 |
| 22Mutation | 下表8类临时变体，检测后还原；正式源码不提交破坏。 |

阈值依据：逻辑镜像与连续边界float容差1e-4，不用于完整plant量化比较；内部PWM slew要求配置值+1e-4，int PWM允许1个整数舍入单位；滤波稳定阶跃≤2周期40 ms；3bit全部8真值且8-bit并行与逐bit参考完全相等。100 ms无脉冲积分上限0.4%PWM来自旧正常Ki0.18×18 RPM×0.1 s=0.324%，并留有限舍入/边界余量，原饱和/斜率冻结仍检查。Trace为200条精确20 ms次序、≤8192 B、越界查询拒绝。物理层保留原35/25 mm成功门槛和实际CPR量化；回归标记误差增加>max(2 mm，基线×25%)为明显恶化，不因此修改原成功门槛。

| 临时故意破坏 | 应失败的测试 | 实际结果 |
|---|---|---|
| 绕过位多数 | 指定单帧噪声 | 检出；reference通过，mutant 1断言失败。 |
| 反转error_rate | 同error不同趋势 | 检出；reference通过，mutant 1断言失败。 |
| 仅一侧加入偏置 | 左右逻辑镜像 | 检出；reference通过，mutant 1断言失败。 |
| 恢复target<10→0 | 5/8/12/18非零目标 | 检出；reference通过，mutant 1断言失败。 |
| 去掉EnterStop首因保护 | STOP连续两故障首因 | 检出；reference通过，mutant 1断言失败。 |
| USER_KEY混用某故障码 | 按键独立停止原因 | 修复脚本后检出；首轮变体头未实际生效而未检出，详见下文。 |
| 去掉重启原因/冻结复位 | STOP重启后新故障 | 检出；reference通过，mutant 1断言失败。 |
| pivot跟随(L+R)/2 | forward base跨pivot/spin交接 | 检出；reference通过，mutant 1断言失败。 |

首轮[mutations.json](mutations.json)是7/8；USER alias未检出的原因是KEY.c提前include原Track.h，include guard使临时变体头没有生效，不是实际USER原因逻辑通过破坏测试。脚本修复为所有C单元预include变体Track.h后，[最终mutation](final/mutations.json)8/8检出，保留首轮失败，不改正式源码来迁就mutation。变体只在临时工作区构建，正式源码hash及运行前后workspace status均不变；完整验证时该轮 `workspace_was_clean=false`。提交后从干净工作区另行[重跑Mutation](final/mutations_clean.json)：8/8检出，`workspace_was_clean=true`、`source_unchanged=true`、`workspace_status_unchanged=true`；临时变体没有残留。两轮产物分别保留，不混淆状态。

| 开关 | 默认值 | 理由 |
|---|---:|---|
| TRACK_TRACE_ENABLE | 1 | 停车首因/黑匣子通过，运动行为不变。 |
| TRACK_SENSOR_FILTER_ENABLE | 0 | 开启候选造成模型正常场景退化，默认运行行为回退。 |
| TRACK_NONLINEAR_FOLLOW_ENABLE | 0 | 独立P保留原D后仍造成标准/随机新增STOP，默认回退。 |
| TRACK_ERROR_TREND_ENABLE | 1 | 4样本有界趋势观测通过，默认不改变轮速。 |
| TRACK_ADAPTIVE_SPEED_ENABLE | 0 | 逻辑交接通过，完整轨迹新增标准/随机失败，默认回退。 |
| TRACK_ERROR_PREDICT_ENABLE | 0 | 开启时明确替代原D，但仍未通过完整路线/实车验证；趋势噪声风险保留。 |
| TRACK_LOW_SPEED_ZONE_ENABLE | 0 | 低速逻辑通过，完整负载/接线回归退化，默认回退。 |
| TRACK_TURN_CONTINUITY_ENABLE | 0 | 两种连续曲线均新增旧成功场景STOP，默认回退。 |
| TRACK_ALIGN_TREND_ENABLE | 1 | 三条逻辑通过，正常/压力/同种子随机无新增回归。 |

## 10. 实车验证顺序 A→G

以下是尚未执行的实车操作顺序，每一步保留最差样本，停止时保存stop_reason及故障前Trace。先用本报告默认配置建立实车基准；FILTER/NONLINEAR/ADAPTIVE/LOW_SPEED/TURN/PRED保持0。若专门验证实验分支，一次只开启相关功能并与同条件默认记录对照，出现旧成功场景新增STOP或明显误差恶化即回退。软件镜像只能验证方向逻辑，四方向摩擦、地面负载与惯性须实测。上一步出现异常先解决，之后再进入下一步。

### A. 架空四方向低速

Left+、Left−、Right+、Right−各逐项测5/8/12/18/25 RPM，另一轮明确0。目的为找出每个方向最低能连续转动的RPM和最低稳定PWM，不要求所有速度都稳定。使用原3 s闭环bench入口；记录目标RPM、实际RPM、PWM、编码器符号、是否一卡一冲、首次起转时间、最低连续RPM及PWM。每次改一个方向，完成后关闭bench再落地。

正常：非零目标保持原数值；轮转方向和编码器符号一致；助推有界，起转后PI/PWM逐步稳定；0目标立即停轮；到3 s或按键立即撤驱动。5 RPM实际停启属于需要记录的机械能力，不允许底层偷偷改目标。

异常：目标正却轮倒转先核对Motor接线/符号；轮正转编码器负先核对Encoder接线/符号；只有某方向停启说明该方向offset/摩擦未标定；长时间无脉冲应有限编码器STOP，不许无限助推。下一步先修符号/机械，再以约0.5%小步调对应 `LEFT/RIGHT_(REVERSE_)DRIVE_OFFSET_PWM`，不能同时改其它方向。

结果回填四方向offset、`SPEED_VERY_LOW_RPM`及`TRACK_MIN_FOLLOW_BASE_RPM`。普通TRACK最小内轮要求、ALIGN/TURN慢转必须结合最差相关方向的最低持续转速：例如某方向最低连续16 RPM，则不应要求它在ALIGN/TURN长期稳定8 RPM。实车最低速度不能通过改CPR/轮径伪造。

### B. 地面直线

看raw、filtered、error、error_rate、base RPM、correction、左右requested/command target、实际RPM和PWM。默认raw=filtered，只有开启FILTER实验才有3帧多数。先中心直线，再小初始左/右偏差，加入能够重复的短暂遮挡噪声；观察至少500控制周期。

正常：两轮target接近，实际RPM接近目标，无错误角点/失线跳转；记录左右实际速度差、最大correction和翻转次数。实验FILTER开启时才检查单帧raw不改变多数值，实验ADAPTIVE开启时才检查稳定后base恢复≥上限95%。异常：raw长期左右不对先核对探头顺序/电平；target对而实际差先回A；位置输入稳定但correction反复翻转才检查原P/D和量化。下一步优先修探头/四方向速度能力，再小步调原中心P/D；实验分支出现退化先恢复默认，不能先整体加速度。

### C. 缓弯

看error逐渐偏离时error_rate、base与correction，对比同error下进弯和回正。默认按原误差幅值减速，error_rate只观测。正常：线保持可见，误差有界，target单周期不超原slew，内轮不长期落入A测出的不能连续运行区。实验ADAPTIVE开启时另检查向外恶化同E下更早降速、回正按配置斜率恢复。异常：内轮停启先回A；偏差扩大却纠偏方向错先查探头/符号；实验回正仍很慢才查趋势惩罚和恢复斜率。下一步默认优先调四方向offset及原跟线增益；实验中只逐一调adaptive权重/恢复率，退化即回退。

### D. 单左直角

观察 CONFIRM→APPROACH→TURN_FAST/SLOW→ALIGN→EXIT→TRACK 的完整链，以及方向、置信度、推进mm/goal、turn angle、requested/command RPM、实际RPM。左dir=−1、yaw左正；旋转为左反/右正。

正常：宽线多帧确认；APPROACH先同步低速、达到几何推进目标；默认TURN requested保持原42→30→18 RPM档位、command经原slew变化，70°后才允许可靠6帧接线进入ALIGN；ALIGN稳定中心后EXIT，EXIT时间+线稳定+位移成立后TRACK。只有TURN实验开启才额外检查40～50°/60～70°requested曲线连续。异常：APPROACH偏航先回A，再看同步补偿；轮轴过/未到顶点调 `CORNER_ADVANCE_TRIM_MM`，不改175 mm；正确角度仍扫过线先核对慢转能力再调FIND_RPM；ALIGN持续恶化或摆动再调ALIGN_KP。下一步按异常所属阶段调一个参数，不扩预算。

### E. 单右直角（镜像跑道）

同尺寸镜像D，看dir=+1、yaw右负、左正/右反的实际RPM与PWM；比较turn requested RPM、state时序、ALIGN误差和停止原因。

正常：方向/目标严格镜像，预算相同；允许真实CPR与电机响应导致小量化差异。异常：可靠右角却选左先核对PF顺序和置信票；dir正确物理转错先回A；代码方向正确但右角明显更差，优先检查A的左正/右反最低持续速度与摩擦。下一步先调对应方向offset/慢转能力，再看ALIGN；不新增“右转特殊预算”。

### F. 连续弯

分别跑左→左、右→右、左→右、右→左。看新角点方向覆盖旧hint/history、重新武装条件、出弯base恢复及Recovery attempts/age是否延续符合稳定条件；只有ADAPTIVE实验开启时才查看它的forward base跨spin/pivot交接。

正常：可靠当前角点优先；历史过期；真正稳定TRACK≥600 ms并前进≥30 mm才清会话，连续假接线沿用预算；实验adaptive的pivot/spin不把前进base污染成0或半轮RPM。异常：第二弯方向沿用前弯查当前证据/hint时效；出弯低速长期不恢复查EXIT稳定条件，实验base异常则回退并查交接；Recovery瞬间归零说明逻辑错误，应保存Trace。下一步先定位状态/交接证据，必要时修代码和回归测试，不通过增加次数让车继续试。

### G. 完整赛道

记录最差一次左转、最差一次右转、最大横向偏差、总完成时间、是否STOP、首个stop_reason和完整Trace；保留失败圈，不只选最好的一圈。

正常：正常赛道持续完成；真正永久失线/编码器/调度故障按原预算停止、PWM0、需要人工重启；假回线不重获无限预算。异常：保护原因指向阶段问题时回对应A～F，先修低速能力/推进/接线，再考虑正常速度。下一步优先调相关方向offset、FIND_RPM、ALIGN_KP或速度趋势，保持SEARCH/Recovery硬预算。通过多次重复记录后才考虑提升速度；FIX5本身不提高60 RPM上限。

## 11. 最终审查、假设与能力边界

逐项源码审查没有发现本轮新增无法退出状态或预算被放宽。关键证据如下，行号对应最终工作源码；完整基线保护出口表见[source audit](source_audit_baseline.md)，最终独立审查见[final source review](final_source_review.md)。

| 审查项 | 结论 / 证据 |
|---|---|
| 状态有限出口、Recovery假清零 | 原状态处理/预算函数保留，仅人工初始化和满足原600 ms+30 mm稳定TRACK时整体清会话；新增趋势/Trace/ALIGN计数不写Recovery预算。 |
| 数值与base | `Track.c:211`实验AdaptiveStep目标/状态有上限与非负夹限；28 RPM低交接不做MIN状态钳高，无实测RPM输入。生产输入为有界整数/有限配置，未发现NaN生成路径；Limit本身不清NaN，不能把它写成万能异常修复。 |
| forward交接 | `Track.c:204`只显式base或mode=FORWARD且双轮请求>0；spin/pivot保留有效base；`test_19a/b/c`验证。 |
| 数组/整数边界 | Trace长度1～250、实际200；3帧sensor历史；trend3～5实际4；ALIGN翻转窗3～16实际6；所有下标模长度。encoder raw先int32提升，原累加饱和保留；`Debug10`先限幅再int16，ASan/UBSan覆盖。 |
| 时间回绕/实时结构 | uint32差值保留，20 ms主任务与轻量ISR不改；Trace只在Track_Task控制末一次，控制周期外停车只冻结；没有delay/动态分配/运行期printf。 |
| PD/低速/anti-windup | 非线性P另滤，PRED关时原D输入保留；PRED开时`Track.c:382`明确Kd=0、`:922`以P趋势替代D；`:580`每轮连续gain与`:602`速度输出原零目标/换向/负载/I冻结保留，功能默认关；测试不能证明实车5 RPM。 |
| 镜像/方向/滤波 | 几何与方向宏不改；raw安全计数不叠多数延迟；majority为整数位运算，实验滤波默认关；4方向反向参数在`CarConfig.h:264`变独立同值定义。 |
| ALIGN两规则 | `Track.c:225`只有整数计数/短翻转窗与gain/base调整；改善不降速；限幅/slew和time/distance/yaw退出未改。 |
| 软件STOP/首因/重启 | `Track.c:497` EnterStop统一开启后的软件停车，state8/明确非NONE/请求命令0/PWM0；STOP重入保持首因；ResetRuntime解除STOP并清原因/解冻；USER_KEY在`:502`清四方向负载以兼容旧API；18种真实来源与key/重启测试覆盖。 |
| 致命异常撤PWM | `user/stm32f10x_it.c:40`最小FaultMotorStop只写TIM2 CCR3/4为0；`:64`HardFault、`:78`MemManage、`:92`BusFault、`:106`UsageFault在原死循环前调用。host执行4个真实handler验证两路归零、其它寄存器不变和不返回；不调用Track/PID/Trace。 |
| 全关与工程 | [最终全关比较](final/all_off/comparison.json)142327条规范golden精确，无float容差；新功能参数用ARMCC5兼容整数`#if/#error`，无`_Static_assert`；未新增固件C单元、原Keil引用存在。最终12配置24检查通过，GCC不等于ARM链接。 |

保守假设：bit0按当前配置为从车顶看左侧；黑低电平；12 mm探头间距仅为既有模型假设；四方向摩擦/起转PWM/电机动态未实测，反向offset初值可与正向相同但应独立调整；完全没有方向证据时维持原有限中性观察后停车策略。

基线四个致命异常处理只有死循环，CPU异常后主循环40 ms保护无法运行，TIM2可能保留旧PWM。本轮已独立修复为死循环前两次volatile寄存器写0，不走Track，不依赖损坏时可能不可用的控制记忆，也不在异常上下文写Trace。软件EnterStop的首因/冻结契约和此异常撤PWM路径分别描述：异常路径不保证生成stop_reason或完整最后样本。host只验证真实函数的寄存器写入与锁止，未验证STM32异常进入、总线故障时外设可访问性或真实电机撤驱动；也没有已启用IWDG/WWDG的证据。基线/最终独立源码审核中记录的旧异常缺口属于修复前状态，应以本段及最终源码为准。

默认host通过只能说明：在当前模型假设和原成功门槛下，未观察到所测默认场景新增控制/数值回归，原失败仍未解决。它不证明STM32 ARMCC编译/链接、20 ms实机耗时、板卡烧录、真实电机5 RPM持续运行或右转已完全解决。没有Keil和真实小车时，不提供虚构HEX/实车结论。
