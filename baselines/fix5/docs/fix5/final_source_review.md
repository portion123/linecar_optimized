# FIX5 最终源码独立审查

本审查直接阅读 `hardware/Track.c`、`Track.h`、`CarConfig.h`、Encoder/Motor/GraySensor、`system/PID.c`、`user/main.c`、异常处理文件及 Keil 工程引用；不依据旧报告推断。审查时默认 Trace=1、Error Trend=1、ALIGN Trend=1，其余 FIX5 行为开关为0。实验开关的逻辑测试通过和 host 编译通过，不能替代完整物理模型防回归，更不能说明实车稳定。

## 默认配置结论和边界

当前默认配置未发现新增的状态无法退出、Recovery 被短暂接线清零、环形数组越界、左右方向混淆或合法输入产生 NaN 的路径。Step8 默认配置的标准几何34/34，原压力9/10，固定种子27/30，新增失控0；这些数量与基线相同。全关逐周期 golden 精确一致，见 `step8/all_off/comparison.json`。

仍保留原有 `pressure_square_-1` 失败及3个随机模型 ALIGN 安全停车。滤波、非线性跟线、自适应速度、低速增益缩放、转速曲线的启用候选曾产生完整模型退化，最终均默认关闭；不能把这些候选称为通过稳定性验证的默认增强。Trend 默认只增加有单位的观察量，adaptive 和 prediction 关闭时不改变普通 TRACK 请求。

## 核心事项逐项检查

| 审查事项 | 源码依据 | 结论 |
|---|---|---|
| 状态退出和有限恢复 | `Track.c:993` BeginApproach；`1055` Approach；`1093` TurnCorner；`1114` Scan；`1157` Align；`1193` ExitLine | CONFIRM/APPROACH/TURN/SEARCH/ALIGN/EXIT 都有原时间、距离或角度出口。TURN 到105°进入有限附近找线，不增加扫描范围；SEARCH 次数和 Recovery 总预算继续覆盖状态切换。CAR_STOP 只由显式启动释放。 |
| Recovery 首次起点和稳定清除 | `Track.c:966` RecoveryBegin；`973` RecoveryStep | 已存在会话立即返回，不重新初始化起点或预算。只有 TRACK、无宽带、raw 线形/误差合格、向前，且同时满足600ms和30mm稳定条件才清除。ALIGN/EXIT 短暂中心不会清除总预算。 |
| 新 Error Trend 窗口 | `Track.c:180`；`CarConfig.h:91` | 4项 float error/秒时间窗；整数 index/count；限幅±100探头单位/秒。索引模4，最多4项，时间间隔求和为正，失线/非相应状态复位观察历史，和 Recovery 清除无关。 |
| ALIGN 两条规则 | `Track.c:226`；`1157`；`CarConfig.h:37` | 连续3次幅值增大才把 base24降至22 RPM、gain提高至115%；幅值改善即解除恶化状态。6帧窗口内3次有效符号翻转把 gain乘70%，忽略±1量化。窗口及累计值有界；lost/reset/BeginAlign 清除记忆。请求仍经过原 correction 限幅、target slew 和 PWM slew。 |
| 时间戳回绕 | `Track.c:1467`；`980`；`948` | milliseconds、时间标记和差值均 uint32_t，使用无符号减法。计时 ISR 仅递增。调度间隔至少20ms才进 Control，超过原40ms上限停车；原回绕测试和 FIX5 对照测试保留。 |
| 合法输入下浮点有限性 | `Track.c:1320`；`741`；`891`；`system/PID.c:32` | GPIO位置±7、编码器int16、标定常数为正，Control 的 elapsed≥20ms；窗口 elapsed 求和非零，PID dt 正，分母正常。base/targets/PWM 有界，未引入 double 常量。`Limit()` 不主动捕获 NaN：结论限定合法输入和当前有效配置，不能声称覆盖 RAM 损坏或任意 NaN 注入。 |
| 普通 TRACK 不暗停轮 | `Track.c:619`；`928`；`934` | 底层只在 target==0 明确停轮，非零低RPM保留其符号和值。默认原 correction ratio0.85 保持双轮前进。实验 adaptive 分支还把内轮请求限到12 RPM，但12 RPM是待实测的模型假设。 |
| 整数中间值和窗口溢出 | `Track.c:703`；`741`；`1313`；`Encoder.c:55` | 编码器先转int32再乘符号，避免-32768反号溢出。当前15项方向窗总绝对计数≤491520，乘100≤49152000；即合法配置32项也低于int32上限。当前5项测速窗和允许16项均安全。total counts 用有界饱和相加；新增计数/索引用 uint8/uint16/uint32，无 host/ARM long 位宽差异。 |
| Trace 数组和 RAM | `Track.c:100`；`124`；`144`；`Track.h:34` | 固定200条，每条32字节，数组6400字节，索引/计数/标志7字节（6407字节显式存储，未把链接器填充声称为实测）。4秒历史，cap≤250，读索引先校验 `<count`；write模cap且count饱和。主循环每个任务末尾最多一次写，ISR不写，无堆/Flash/运行期printf。 |
| STOP 冻结和首因 | `Track.c:497`；`462`；`514`；`1474` | Trace 开时所有软件 Halt/显式 HaltReason/按键都进入 EnterStop。已STOP立即返回；NONE兜底成MODE；requested/command归零，实际撤PWM，state=STOP。控制内故障只允许该任务最后一条记录，再冻结；按键在任务外立即冻结。 |
| 明确 stop_reason 和人工停车 | `Track.h:18`；`Track.c:456`；`467` | NONE=0，USER_KEY=1，与故障2..18分开。编码器/调度故障、各阶段保护、搜索/Recovery预算、无方向、模式、歧义、直行检查和bench超时都有非NONE原因。通用阶段限额保留原阶段原因码，不虚构细分故障。 |
| STOP 后重启 | `Track.c:412`；`449`；`531`；`525`；`551` | ResetRuntime 将 stop_reason=NONE、TraceRestart 清空并解冻；正常启动、forward和speed-test均调用它。新的首故障可再记录。Track_ClearTrace 仅清环，不解除停车；调用它不会放行电机。 |
| 全关兼容 | `Track.c:455`；`464`；`516`；`scripts/fix5_golden.py` | Trace关时保留旧Stop/故障枚举行为（例如编码器9、调度18），避免用统一STOP改变旧规范输出。全9项关，11293单元、70500几何/压力、60534随机记录的规范字段，整数与float hex均零容差一致。新trace索引/观察字段不进入比较。统一新STOP语义适用于默认Trace开配置。 |
| 左右镜像和几何 | `Track.c:944`；`1024`；`1095`；`891` | 左右条件用error符号和corner_dir镜像；非线性为奇函数，趋势按同一时间窗，自适应使用绝对误差及向外幅值，不引入单侧偏置。真实左右count/rev不同，完整模型不能要求左右物理输出逐位相等。251/265.35、44/133/175及SIGN、PF0..7均未改。 |
| 四方向 offset/feedforward | `Track.c:662`；`CarConfig.h:264` | L+/L−/R+/R−各自选择独立offset和feedforward；上一版已经按方向使用offset/feedforward，但负方向定义别名指向正方向参数；本次改成独立数值定义，避免调正向时连带改变反向，初值相等，不改变默认旧输出。整对交换时按physical wheel选择。不得因逻辑测试通过声称真实方向已验证。 |
| 低速缩放连续性 | `Track.c:580` | 实验分支按每轮自己的abs(command)在0/8/24 RPM线性插值。Kp35%→60%→100%、Ki15%→35%→100%，阈值处函数值一致；≥24保持原增益。积分是PWM贡献，改变Ki不重标旧积分。该分支因完整模型退化默认关。 |
| Anti-windup | `system/PID.c:49`；`Track.c:672` | PID候选输出饱和时禁止同向积分；PWM slew进一步冻结受限方向积分。零目标/换向复位瞬态，稳态补偿按方向单独保存并限12% PWM；超速衰减积分，卡轮仍由原无脉冲故障停车，不能无界加助推。 |
| 换向过零和方向窗口 | `Track.c:764`；`626`；`711` | requested异号时command先降0，再切方向；速度输出有原40ms撤驱动及最多220ms惯性等待。反向故障宽容从新方向实际PWM开始而非从上层请求开始，零计数不抹除已有方向证据。原保护阈值未放宽。 |
| adaptive 非TRACK交接 | `Track.c:204`；`786`；`915` | 显式base优先，否则只有forward且L/R都>0才平均请求。异号spin、一轮0的pivot保留最近forward base，不用实际RPM；初始化28。下限只约束goal，状态从低EXIT值按+2RPM/周期恢复。该分支默认关，逻辑交接测试通过不等于路线候选通过。 |
| 转弯速度单调与上限 | `Track.c:1079`；`1093` | 实验curve在原50°/70°阈值前10°线性降低42→30→18 RPM，连续且单调不增。原FAST/SLOW状态、70°可认线、105°转角和搜索/Recovery上限未改变。候选新增随机安全STOP，默认关。 |
| 预测与原PD关系 | `Track.c:922`；`system/PID.c:38` | predicted不送回PID误差输入，启用预测时 ResetPID 把原位置 Kd 设为0，再使用Kp×20ms×error_rate作为一阶趋势贡献；替代原D而非叠加。因未对predicted再次微分，不产生二阶差分。默认0；全关保留原PD。 |
| ARMCC5配置和Keil引用 | `CarConfig.h:45`；`94`；`118`；`236`；`.uvprojx:327` | 新合法性检查均整数#define和#if/#error，无_Static_assert。工程uC99=1，现有C99声明顺序符合该设置；不等于已用ARMCC验证。无新固件.c文件，现有18个C单元/63个引用全部存在；新host .c/.inc无需加入Keil。 |

## raw / filtered 的特别审查

`ReadLine`（Track.c:303）每20ms读取一次真实GPIO；多数滤波有三项uint8历史，并以真实启动读数填满。稳定阶跃到第二个连续新样本时反映，不用delay。默认filter0，raw和filtered相同。

实验filter1时，`sensor_mask`、返回count及`sensor_raw_error`来自raw；TURN/SEARCH取得raw位置，Recovery稳定条件、ALIGN中心计数、EXIT稳定计数、全白和宽带几何边界也使用raw。`ObserveWide`（Track.c:818）已修成raw几何/失线、不让残留filtered宽带清零lost_ms；filtered仅参与置信分数增加。TRACK/ALIGN/EXIT的方向控制error可来自filtered，且仍叠加原浮点位置滤波或阶段确认，这正是候选出现延迟回归的原因之一。

保留的实现还会在第二次 `DecodeLine(sensor_filtered,...)`（319）时覆盖 `side_observed`、`side_error`、`line_ambiguous`。因此filter1的侧块/歧义分类取filtered，而形状/安全count取raw，不能描述为所有传感器字段都来自同一帧。当前默认filter0不会混用；这些分类与多帧确认组合的动态行为未获得完整路线防回归通过，应作为实验局限保留，不能直接用于实车竞速默认配置。

## 异常处理独立审查

上一版 HardFault/MemManage/BusFault/UsageFault 只有死循环，没有显式撤PWM；TIM2可能继续输出最后占空比。这是已确认的既有软件安全缺口。本次在 `user/stm32f10x_it.c:40` 增加独立 `FaultMotorStop`，仅向现有TIM2的CCR3/CCR4写0，四个异常入口先撤PWM再维持原锁定循环，不调用EnterStop/PID/Trace，不改中断实时结构。

`tests/test_fault_stop.py` 在子进程中执行真实四个处理函数，用STM32头文件真实TIM_TypeDef布局的共享寄存器替身检查：4/4通过，只有CCR3/CCR4被清零，其余寄存器字节不变，处理函数持续锁定。host不能验证STM32异常后的总线访问或寄存器影子生效时序；没有ARM链接/烧录/实车异常测试。

本文件给出的是源码审查，host严格警告、编译矩阵、ASan/UBSan、Mutation Check与实车边界以最终验证产物为准。不存在本次 Keil ARMCC固件链接、烧录或5RPM实车持续运行证据。
