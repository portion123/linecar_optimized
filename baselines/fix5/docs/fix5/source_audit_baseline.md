# FIX5 阶段 0：当前交付源码只读审核

本页行号对应 FIX5 修改前、压缩包根目录的当前 `linecar_optimized` 源码，不能用于 `docs/original_source` 的更早 FIX4 文件。审核未修改控制代码、测试门槛、模型、几何或方向参数。测试统计以本轮重新执行的基线结果为准；旧报告中的 94/94 单元、34/34 标准几何和 9/10 压力是待重跑的历史记录。

## 已有功能核实

| 用户要求核实的功能 | 当前源码证据 | 结论与边界 |
|---|---|---|
| Recovery Session | `hardware/Track.c:611`、`:618`、`:654`；预算 `hardware/CarConfig.h:227` | 已有。会话首次启动之后不因 BeginScan 刷新，只累计局部扫描之外的全部 yaw/轮行程，时间、角度、路程和次数分别有限。 |
| 无默认方向 | `hardware/CarConfig.h:138`、`:256`；`hardware/Track.c:592` | `CORNER_DEFAULT_DIR=0` 且编译拒绝非零；可靠当前方向→有效 hint→有效历史→弱近期误差，否则方向 0。 |
| 左右镜像方向逻辑 | `hardware/Track.c:115`、`:500`、`:530`、`:718`、`:732`、`:949` | error 镜像变号；单侧角点票符号相反；左右目标互换；yaw 左正，progress=-dir*yaw，使左右转进度都为正。硬件不对称由各自 CPR 与 offset 表达。 |
| TURN_FAST / TURN_SLOW | `hardware/Track.c:716`；`hardware/CarConfig.h:175` | 已有。50°从 FAST 进入 SLOW，70°后最低找线 18 RPM；当前 requested RPM 是 42→30→18 的阶梯，Step 7 可只平滑此规划。 |
| ALIGN | `hardware/Track.c:686`、`:777`；`hardware/CarConfig.h:182` | 已有。24 RPM 基础速度、最大总差速 20 RPM、连续中心 160 ms；超时 1800 ms/路程 65 mm/追加 yaw 20°保护。 |
| EXIT 分阶段提速 | `hardware/Track.c:804`、`:825`；`hardware/CarConfig.h:189` | 已有。28→40→60 RPM，时间和线稳定同时达标；至少 700 ms、合格线 400 ms、前进 25 mm 后返回 TRACK。requested cap 是档位，实际 command 经原 slew 连续。 |
| target RPM slew | `hardware/Track.c:460`、`:469`；`hardware/CarConfig.h:50`、`:54` | 已有。TRACK 普通每周期 5.4 RPM、边缘 9；CONFIRM/APPROACH 3/6，TURN/SEARCH 4/8，ALIGN/EXIT 2/6，正反换向先过零。 |
| PWM slew | `hardware/Track.c:372`；`hardware/CarConfig.h:103` | 已有。每周期增 3%、减 4%；安全撤驱动、零目标、换向空白和超速滑行允许立即归零。 |
| 低速多周期测速 | `hardware/Track.c:437`；`hardware/CarConfig.h:99` | 已有。5×20 ms计数窗，8 RPM及以下全窗口，8～24 RPM连续混合，之后 60 ms 一阶滤波；不能宣称实车5 RPM稳定。 |
| 左右/正反独立 drive offset | `hardware/Track.c:358`；`hardware/CarConfig.h:84`、`:116` | 运行路径与 `[wheel][direction]` 负载记忆已独立，但反向 offset/feedforward 配置默认是正向宏别名，改正向会同时改反向。Step 6 应把别名改为数值相同的独立定义，保持原默认行为。 |
| 编码器反向窗口 | `hardware/Track.c:399`；`hardware/CarConfig.h:112` | 已有。15帧方向计数，零脉冲不清证据，至少6净反向count且75%反向比；250 ms后判断，换向宽容从新方向实际PWM开始。 |
| 失线整体预算 | `hardware/Track.c:625`、`:657`、`:894`；`hardware/CarConfig.h:227` | 已有。12 s、4次、300°累计绝对yaw、700 mm平均绝对轮路程；短白缝也启动会话，交替假回线2 s后升级搜索。 |
| 状态滞回 | `hardware/Track.c:511`、`:675`、`:793`、`:825`、`:835` | 已有计分、间隙容忍、连续接线/居中、时间+距离再武装。没有发现应以新 Hysteresis 模块替换的来回跳转 bug。 |
| anti-windup | `system/PID.c:40`、`:44`；`hardware/Track.c:371`、`:374` | 已有条件积分、积分限幅、外部PWM slew冻结同向积分、超速时卸瞬态积分；稳态负载等量转移而非重复累加。 |
| 换向过零 | `hardware/Track.c:463`、`:315`、`:322`、`:332`；`hardware/Motor.c:26` | 已有。请求换向先 slew 到0，0目标清该轮PI，换向重新加载对应方向补偿，40 ms空白及最多220 ms惯性等候；底层先CCR=0再切GPIO。 |

## 实时采样、PD 与数值链

`user/main.c:24` 每次主循环先扫描/处理按键，再调用 Track_Task。`hardware/Track.c:1068` 用无符号 `now-last_control` 调度20 ms控制，`:77` 的 TIM3 ISR 仅清标志并递增毫秒；编码器 `hardware/Encoder.c:60` 等 ISR 仅检测另一相电平、饱和计数、清标志。没有 ISR PID、动态内存或运行期 printf。

Gray_Read 在 `hardware/GraySensor.c:16` 只读一次 GPIOF 并行寄存器；控制内 `hardware/Track.c:950` 每周期只调用一次 ReadLine。启动 `:270` 与初始化 `:1102` 还有额外读，不构成周期内高频采样。因此 Step 2 宜采用3帧时间多数，稳定阶跃最迟第二次20 ms样本得到新值；初始化时应以首次读值填满历史，避免人为引入启动白帧。

error 由连续黑块中心计算 `2*first+n-1-7`，实际范围 [-7,+7]，两中心探头 `0x18` 为0，单中心探头为±1。分离块按最近历史选择，等距对称块且异号时保持历史并标含糊，避免按扫描顺序偏左。`hardware/Track.c:573` 已有0.75中心死区，`:577` 再做位置一阶滤波；`:582` 送入PD。原D在 `system/PID.c:34` 是 `(error-last_error)/dt`，单位 error/s，经60 ms低通，Kd=0.18。因此把 `E+K*error_rate` 再送原PD会叠加一阶趋势并产生二阶差分；FIX5预测默认关闭。中心非线性若用于减少摆动，应替代中心死区的P部分并保留非零下限，不能死区以后又给“非零中心增益”却依然零响应。

几何/标定/硬件确认为 `hardware/CarConfig.h:12`～`:15` 的 Encoder(-1,+1)/Motor(+1,-1)，`:20`～`:22` 的44/133/175 mm，`:31`的251.0/265.35 count/rev；`hardware/GraySensor.c:12` PF0～PF7保持不变。`hardware/Track.c:942` 先提升raw int16到int32再乘方向，RPM/距离均按各轮自己的CPR；不能改这些数据换通过率。

## 所有软件停车路径

当前停车不是统一的 CAR_STOP：Halt(`hardware/Track.c:236`)按异常类别进入 CAR_ENCODER_FAULT(9)、CAR_CONTROL_FAULT(18)或CAR_STOP(8)，HaltReason(`:254`)先 Halt(CAR_STOP)再覆盖理由；Track_Stop(`:212`)进入 READY(0)、理由 USER(1)并清全部控制记忆。已有入口关闭PWM和清目标，但无首因保护，无冻结trace；重复 Track_Stop 会丢失现场。Step 1需在开启时统一语义，同时全关仍保持这些旧state/reason逐周期golden输出。

| 触发 | 基线位置 | 基线reason |
|---|---|---|
| 按键KEY0/KEY2、运行中KEY1/KEY5 | `hardware/Track.c:293` | USER=1；进入READY，需FIX5开启后独立 USER_KEY/CAR_STOP。 |
| 无效台架目标/无效运动模式 | `hardware/Track.c:285`、`:606` | MODE=15。 |
| Recovery时间/角度/路程/次数 | `hardware/Track.c:625`、`:626`、`:627`、`:657` | 10/11/12/13。 |
| 推进距离/偏航/超时 | `hardware/Track.c:698` | APPROACH=5。 |
| 旋转净反向>10° | `hardware/Track.c:719` | ENCODER=2；state9。 |
| 旋转超时 | `hardware/Track.c:720` | TURN=6。 |
| 搜索超时 | `hardware/Track.c:741` | SEARCH=9。 |
| 无方向观察超时 | `hardware/Track.c:752` | NO_DIRECTION=14。 |
| edge pivot角度/时间 | `hardware/Track.c:761` | SEARCH=9。 |
| 普通/出弯局部扫描范围/次数耗尽 | `hardware/Track.c:770` | SEARCH=9。 |
| ALIGN时间/路程/附加角度 | `hardware/Track.c:781` | ALIGN=7。 |
| EXIT超时 | `hardware/Track.c:809` | EXIT=8。 |
| CONFIRM时间/距离 | `hardware/Track.c:855` | CONFIRM=4。 |
| 含糊线持续240 ms | `hardware/Track.c:865` | AMBIGUOUS=16。 |
| 调度间隔>40 ms | `hardware/Track.c:952` | PERIOD=3；state18。 |
| 无脉冲/反向窗口 | `hardware/Track.c:954` | ENCODER=2；state9。 |
| 15 s开环/3 s闭环台架期限 | `hardware/Track.c:959`、`:965` | FORWARD_TIME=17/BENCH_TIME=18。 |

`hardware/Track.c:473` 与`:605` 已检查是否running，故BeginScan被会话次数拒绝后调用者继续 SetTargets/DriveTargets 不能重新施加PWM。Control(`:951`)对全部停止/非运行状态每周期保持PWM=0；只有显式Start接口恢复。Start成功 `:279`、`:265`、`:288`清reason；无起始线进入CAR_NO_LINE，不启动电机。FIX5需要分离“重新初始化运动记忆”和“用户停车”，保证重启清NONE并解冻Trace，避免初始化伪造USER停机或首因挡住下一次故障。

所有进入CAR_STOP的基线路径最终均得到非NONE理由；仍应新增逐路径测试覆盖首因、重启和用户原因，而不只静态搜索函数名。

## 状态退出与会话清零

| 状态 | 正常出口 | 有限失败出口 |
|---|---|---|
| TRACK | 角点确认、失线SEARCH、持续边缘pivot | 含糊线240 ms、编码器/调度、已有失线会话预算。 |
| CONFIRM | 候选撤销TRACK；宽线后连续白40 ms→APPROACH | 1800 ms或100 mm停。 |
| APPROACH | 达到基于175 mm探头位置的目标且≥130 mm→TURN或SEARCH | 200 mm、15°偏航、6000 ms停。 |
| TURN_FAST/SLOW | 50°变SLOW；70°后最低找线；六帧接线→ALIGN | 105°后只反扫附近15°，12000 ms、错误方向、会话预算停。 |
| ALIGN | 中心连续160 ms→EXIT | 白120 ms→附近SEARCH；1800 ms、65 mm、附加20°停。 |
| EXIT | ≥700 ms、连续合格400 ms、前进25 mm→TRACK | 白120 ms→附近SEARCH；3000 ms停。 |
| SEARCH | 六帧可靠线→ALIGN | 无方向300 ms；弱方向12°；出弯15°；edge35°/2500 ms；普通两侧60°；总体10000 ms或会话预算停。 |
| FORWARD/SPEED_TEST | 人工停车 | 15 s/3 s、编码器、调度故障停。 |

没有发现合法运动状态永久停留且不受预算约束的路径。所有上述状态共同保留既有编码器、按键与调度保护；不得放宽阈值。TRACK永续是正常运行语义。

Recovery唯一整体清零位置为人工初始化 `hardware/Track.c:229`、首次RecoveryBegin `:614`、严格稳定TRACK `:632`。后者要求无宽线、合格连续窄线、|E|≤5、当帧向前，同时连续600 ms和30 mm。BeginScan只局部清turn_angle/phase，不清会话；假中心/ALIGN/EXIT不清会话。相邻失线预算不能在FIX5滤波或adaptive接交时误清。

## 边界检查与尚未覆盖的风险

- 所有控制/状态超时采用uint32差值，期限远小于半个计数范围；Track_Task和Recovery已有回绕测试。新趋势计时、Trace timestamp必须仍使用uint32，不以有符号时间比较替代。
- Encoder_Add(`hardware/Encoder.c:55`)饱和int16；AddTotal(`hardware/Track.c:921`)饱和int32。当前direction窗口最大15，允许编译最大32，32768×32×100=104857600未超过int32；RPM窗口最大16，sum约52万。新增环形下标须有长度>0/上限检查，缩放物理量先限幅再转int16。
- 当前所有应用浮点常量带f，PID/物理链为float；新增简单位历史/计数必须用stdint整数。Limit对NaN不会修复NaN；当前量来自固定有限配置和有界整数，所以未找到合法输入制造NaN路径，新规划应保证参数分母>0、base有限且非负。
- raw始终是安全判定的唯一现有输入。轻滤波只接管控制位置/角点置信；TURN找线、lost/all-white、全黑保护、ALIGN/EXIT稳定计数、Recovery清零证据保持raw避免多层延迟。
- `user/stm32f10x_it.c:56`、`:69`、`:82`、`:95` 的HardFault/MemManage/BusFault/UsageFault仅死循环，未撤PWM。工程无IWDG/WWDG使用。CPU异常时主循环调度保护无机会执行，TIM2可能保持旧PWM；此为原有边界，不能把40 ms调度保护写成CPU异常也已安全停车。异常处理应独立审查/最小硬件撤驱动，不能调用复杂EnterStop或在异常ISR写Trace。

## 与旧报告的关系

`docs/阶段0_扫描与核实.md` 是更早原源码报告，它的 CORNER_DEFAULT_DIR=-1、无ALIGN、低于10RPM停轮等描述不适用于压缩包当前根目录。`docs/阶段4_源码审查.md` 和`docs/最终报告.md`描述的重构基本与当前源码一致，当前错误不是要重新做Recovery/ALIGN/低速窗口。需补充：反向参数仍为正向别名；安全停车仍分state8/9/18和人工READY，首因与Trace尚不存在；异常处理不在当前安全保证内。旧报告的压力右矩形失败应保留并本轮重跑核实，不能解释成已解决。

## Step 1～8 的最小增量建议

1. 复用Halt/HaltReason改为单停车入口；TRACK_TRACE_ENABLE开启时固定环形RAM，主循环一次/20 ms写，冻结前保留故障最后样本，停止首因不覆盖；重启单独ResetRunState清reason并解冻。全关保持旧state/reason，golden不得改。
2. Gray_Read不变，在Track中3帧多数；原ReadLine拆为“读取/解读已有mask”，同周期raw与filtered各解读但只有原应有的一次GPIO读。多帧安全证据全用raw，计算error/置信可用filtered。
3. 保留原PD D链，P以三段连续奇函数替换；中心斜率非零，累积折线截距保证区域边界连续，不整体提高Kp。现中心deadband只在功能关闭保留，以满足全关golden。
4. 用3～5个filtered error计算有符号rate并限幅，单位明确error/s；趋势减速可看|E|的增大趋势，避免回正与进弯同样惩罚。预测保持0，若单独编译实现不得再叠两份D。
5. adaptive只接管TRACK base；上限仍60，目标下限合理且普通内轮不被逼入近零；非TRACK显式base优先，合法双轮同向请求平均次之，pivot/spin保持最近有效前进base，启动用配置值。返回TRACK状态量可低于目标min，由恢复slew慢爬，不做状态量下方钳高。
6. speed PI结构不改，仅把每轮Kp/Ki依据自己的|target|连续缩放；原antiwindup、助推、测速/PWM slew、过零仍保留。把反向四个宏从别名改成当前数值独立定义即可修配置交叉影响，无默认输出差异。
7. TURN使用现有42/30/18值及50°/70°锚点构成随progress单调不增的折线，保留state门槛、最低找线角、105°/15°限制，仍经target slew。无新状态机。
8. ALIGN仅记录短整数恶化/翻转证据；恶化达N降base、适量加强P，翻转达门槛降低P；改善保持原行为。中心稳定计数与时间/距离/yaw保护不变，默认先谨慎，由独立逻辑测试与模型回归决定是否开启。

本审核只确认源码行为与最小修改位置；测试通过率、误差、完成时间及STM32编译能力不得从本页静态审核推断。
