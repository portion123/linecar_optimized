#ifndef CAR_CONFIG_H
#define CAR_CONFIG_H
/* 所有调参入口集中在此。先核对方向与脉冲数，再调尺寸与速度。 */

/* 1. 延用原接线：黑线为低电平，输出 1/PF0 在车身左侧。
 * 翻过底盘看时左右会颠倒，应从车顶看、车头朝前核对。
 */
#define TRACK_BLACK_IS_LOW              1
#define TRACK_SENSOR_REVERSED           0 /* 左端遮黑却显示在右端时改为 1 */
#define DRIVE_PAIRS_SWAPPED             0 /* 电机及其编码器整对左右接反才改为 1 */
#define TRACK_ENABLE_CORNERS            1 /* 1：包含直角；0：只做连续弯道纠偏 */
#define LEFT_ENCODER_SIGN              -1 /* 原工程设置，需实车核对 */
#define RIGHT_ENCODER_SIGN              1
#define LEFT_MOTOR_SIGN                 1
#define RIGHT_MOTOR_SIGN               -1

/* 2. 几何尺寸，单位 mm。2026-10-01 用户按尺寸照片文字确认：44 / 133 / 175。
 * 前置距离基准为驱动轮轴心连线到红外探头实际检测点。
 */
#define WHEEL_DIAMETER_MM              44.0f /* 轮胎外径，用户确认 */
#define AXLE_TRACK_MM                 133.0f /* 两驱动轮接地点中心间距，用户确认 */
#define SENSOR_FRONT_OFFSET_MM        175.0f /* 探头到驱动轴线，用户确认 */
#define LINE_HALF_WIDTH_MM             10.0f /* 黑线约 20 mm 总宽的初值 */
#define CORNER_ADVANCE_TRIM_MM          0.0f /* 正数多前进，负数少前进 */

/* 3. 编码器计数：填“本程序中驱动轮实际转一圈”的脉冲数。
 * 当前下降沿条件解码不是四倍频，不能直接填四倍频标称值。
 * 2026-10-01 手动前进10圈两次取平均：左2516/2504，右2656/2651。
 * 每圈 = 两次总数 / 20；测速、距离和转角均分别使用左右轮实测值。
 */
#define LEFT_COUNTS_PER_REV          251.0f
#define RIGHT_COUNTS_PER_REV         265.35f

/* 4. 普通循迹：每 20 ms 更新，所有轮速单位为 RPM。
 * 前置探头像长探路杆，先看到弯道很正常；过强、突然的纠偏容易甩开线。
 */
#define TRACK_PERIOD_MS               20U
#define TRACK_BASE_RPM                60.0f /* 真实RPM，44mm轮约138mm/s；旧计数下20约等于75~80 */
#define TRACK_BEND_MIN_RPM            36.0f /* 偏差大时的最低基础轮速 */
#define TRACK_BEND_SLOWDOWN            4.5f /* 随真实RPM量纲同步调整 */
#define TRACK_LINE_KP                  5.4f /* 中间区域柔和纠偏，最外侧仍有救线差速 */
#define TRACK_LINE_KI                  0.0f /* 先关位置积分，减少转弯后的残留 */
#define TRACK_LINE_KD                  0.18f /* 微分：抑制左右摆动 */
#define TRACK_LINE_I_LIMIT             2.0f
#define TRACK_LINE_D_TAU               0.06f /* 微分低通，防止探头跳变被放大 */
#define TRACK_LINE_FILTER_TAU          0.040f /* 中间位置平滑，边缘另用快速响应 */
#define TRACK_CENTER_DEADBAND          0.75f /* 中间单探头的±1跳变只作轻微纠偏 */
#define TRACK_WIDTH_LEARN_MS          400U /* 正常线宽连续确认，角点中间几帧不能污染基准 */
#define TRACK_MAX_CORRECTION_RATIO     0.85f /* 普通纠偏内轮保持向前 */
#define TRACK_TARGET_SLEW_RPM_PER_S   270.0f /* 真实轮速命令平滑变化 */
#define TRACK_EDGE_ERROR               5.0f /* 到倒数第二/最外探头时优先救线 */
#define TRACK_EDGE_FILTER_TAU          0.010f /* 边缘位置快速响应，中心仍用原滤波 */
#define TRACK_EDGE_SLEW_RPM_PER_S     450.0f /* 边缘及时响应，同时限制阶跃 */
#define TRACK_EDGE_CORRECTION_RATIO    0.85f
#define TRACK_EDGE_MIN_CORRECTION      0.65f /* 到边缘时保证足够差速，避免紧弯纠正迟缓 */
#define TRACK_SIDE_CONFIRM_MS          60U /* 中线+单侧黑区连续三帧后跟随侧线 */
#define TRACK_TURN_HINT_MS            600U /* 一次短暂居中不抹去最近的转向方向 */
#define TRACK_EDGE_ALIGN_MS           600U /* 只在最外侧持续无改善时接管，不打断正常弧弯 */
#define TRACK_EDGE_ALIGN_ERROR         6.0f
#define TRACK_EDGE_PIVOT_RPM          50.0f /* 有线贴边：内轮停、外轮低速前进，不反扫 */
#define TRACK_EDGE_ALIGN_LIMIT_RAD     0.6108652f /* 35°内仍未回线则停止本次恢复 */
#define TRACK_EDGE_ALIGN_MAX_MS     2500U
#define TRACK_RECOVERY_SOFT_MS        400U /* 普通找线后柔和接回前进，不停车重启 */
#define TRACK_RECOVERY_RPM            42.0f

/* 5. 左右轮分别执行速度 PI：转慢就加 PWM，转快就减 PWM。
 * 起动助推逐帧增加，读到转动即撤回；封顶仍无脉冲则故障停车。PWM 单位为百分比。
 */
#define TRACK_SPEED_KP                 0.06f /* 测得电机每1%PWM对应约20RPM，减小旧比例增益 */
#define TRACK_SPEED_KI                 0.18f /* 低增益积分补足负载，避免速度环追着单计数跳变 */
#define TRACK_SPEED_KD                 0.0f
#define TRACK_SPEED_I_LIMIT           20.0f
#define TRACK_LOAD_COMP_MAX_PWM       12.0f /* 阶段切换保留已学到的有界负载补偿 */
#define TRACK_SPEED_D_TAU              0.04f
#define TRACK_RPM_FILTER_TAU           0.06f /* 约251计数/圈，20ms采样一计数约12RPM，需适度滤波 */
/* 30/40%空转两点线性拟合的初值；低速及落地负载由PI补偿，不当作实测低速曲线。 */
#define LEFT_SPEED_FEEDFORWARD         0.04892885f
#define RIGHT_SPEED_FEEDFORWARD        0.04991535f
#define LEFT_DRIVE_OFFSET_PWM          6.478873f
#define RIGHT_DRIVE_OFFSET_PWM         4.672686f
#define TRACK_BOOST_PWM               18.0f /* 反馈式起转助推的初始 PWM 下限 */
#define TRACK_STALL_KICK_MS            60U  /* 原始轮速在(-6,6) RPM内持续多久开始助推 */
#define TRACK_KICK_STEP_PWM             2.0f /* 每控制周期增加2个百分点 */
#define TRACK_KICK_MAX_PWM             40.0f /* 助推封顶；正常速度环上限仍为30% */
#define TRACK_MAX_PWM                 30 /* 闭环最高30%；KEY5人工检查仍固定30% */
#define TRACK_MIN_DRIVE_RPM           10.0f /* 保留紧弯内轮约10~15RPM的连续前进 */
#define TRACK_SPEED_COAST_MARGIN      12.0f /* 超速时松开驱动并卸掉旧积分 */
#define ENCODER_FAULT_MIN_PWM           8 /* 新版低速PWM较小，不能仍以18%作为故障检测门槛 */

/* 6. 直角：连续横带后确认全白，先推进轮轴，再低速转弯。
 * 六路以上含中心触边/全黑为强横带；四五路弱线索仍做减速 PD。
 * 全黑不能分清左/右；先采用单侧线索、上次方向或默认方向，再试另一侧。
 */
#define CORNER_APPROACH_RPM           50.0f
#define TRACK_APPROACH_YAW_KP          40.0f /* 每弧度推进偏航对应的单轮补偿，RPM/rad */
#define TRACK_APPROACH_YAW_MAX_RPM     10.0f /* 单轮补偿上限，RPM */
#define CORNER_RPM                    42.0f
#define CORNER_ALIGN_RPM              30.0f
#define CORNER_CONFIRM_MS             40U /* 至少两帧宽线 */
#define CORNER_CONFIRM_GAP_MS         40U /* 两帧宽带之间容许一两帧漏读，单帧仍不推进 */
#define CORNER_MIN_ADVANCE_PROBES       5U /* 四路侧线消失仅找线，不盲目按前置距推进 */
#define CORNER_LOST_CONFIRM_MS        40U /* 宽线后至少两帧全白 */
#define CORNER_MEMORY_MS             200U /* 宽线线索最多保留 200 ms */
#define CORNER_CLEAR_CENTER_MS       100U /* 恢复中线需连续确认，不能一帧撤销角点 */
#define CORNER_CLEAR_TRAVEL_MM          8.0f
#define CORNER_SETTLE_MS              80U /* 刹停后等待惯性稍减 */
#define CORNER_MIN_ANGLE_RAD            0.9599311f /* 55°，避免认回原直线 */
#define CORNER_SCAN_LIMIT_RAD          1.8325957f /* 每侧相对原方向最多 105° */
#define CORNER_DEFAULT_DIR            -1 /* -1 先试左；1 先试右 */
#define CORNER_APPROACH_MAX_MS      6000U /* 推进异常超时停车，不能提前旋转掩盖 */
#define CORNER_TURN_MAX_MS         12000U /* 两侧都找不到线就停车 */
#define CORNER_CENTER_SAMPLES          2U /* 首帧严格居中，后续近中心确认；容许一帧漏读 */
#define CORNER_REARM_CENTER_MS       160U
#define CORNER_REARM_TRAVEL_MM        50.0f /* 离开上一角点后才重新识别 */

/* 7. 普通丢线不套用前置距离。短暂全白先缓冲，确认后有限角度扫描两侧。 */
#define TRACK_LOST_GRACE_MS           80U /* 多留一帧给窄线探头间隙，仍低速保留差速 */
#define TRACK_SEARCH_RPM              36.0f
#define TRACK_SEARCH_SCAN_RAD          1.0471976f /* 每侧相对原方向最多 60° */
#define TRACK_SEARCH_MAX_MS        10000U /* 44mm轮/133mm轮距，覆盖两侧扫描及起转时间 */
#define ENCODER_NO_PULSE_MS          700U
#define ENCODER_REVERSE_MS           250U
#define TRACK_OLED_ENABLED             1
#define FORWARD_PWM                   30 /* KEY5 人工直行检查 */
#define KEY_DEBOUNCE_MS               20U

#if DRIVE_PAIRS_SWAPPED != 0 && DRIVE_PAIRS_SWAPPED != 1
#error DRIVE_PAIRS_SWAPPED_must_be_0_or_1
#endif
#if TRACK_SENSOR_REVERSED != 0 && TRACK_SENSOR_REVERSED != 1
#error TRACK_SENSOR_REVERSED_must_be_0_or_1
#endif
#if CORNER_DEFAULT_DIR != -1 && CORNER_DEFAULT_DIR != 1
#error CORNER_DEFAULT_DIR_must_be_minus_1_or_plus_1
#endif
#endif
