#ifndef CAR_CONFIG_H
#define CAR_CONFIG_H
/* 所有调参入口集中在此。先核对方向与脉冲数，再调尺寸与速度。 */

/* FIX5 switches: all 0 preserves the exact FIX4 host control contract. */
#ifndef TRACK_TRACE_ENABLE
#define TRACK_TRACE_ENABLE 0
#endif
#ifndef TRACK_SENSOR_FILTER_ENABLE
#define TRACK_SENSOR_FILTER_ENABLE 0 /* experimental: time majority regressed physical baseline */
#endif
#ifndef TRACK_NONLINEAR_FOLLOW_ENABLE
#define TRACK_NONLINEAR_FOLLOW_ENABLE 0 /* experimental: full model regression remains */
#endif
#ifndef TRACK_ERROR_TREND_ENABLE
#define TRACK_ERROR_TREND_ENABLE 0
#endif
#ifndef TRACK_ADAPTIVE_SPEED_ENABLE
#define TRACK_ADAPTIVE_SPEED_ENABLE 0
#endif
#ifndef TRACK_ERROR_PREDICT_ENABLE
#define TRACK_ERROR_PREDICT_ENABLE 0
#endif
#ifndef TRACK_LOW_SPEED_ZONE_ENABLE
#define TRACK_LOW_SPEED_ZONE_ENABLE 0
#endif
#ifndef TRACK_TURN_CONTINUITY_ENABLE
#define TRACK_TURN_CONTINUITY_ENABLE 0
#endif
#ifndef TRACK_ALIGN_TREND_ENABLE
#define TRACK_ALIGN_TREND_ENABLE 0
#endif
/* Adaptive Speed (RPM, per scheduled 20 ms task): never raises the old 60 RPM ceiling.
 * Integer configuration permits ARMCC5 #if checks; floating control is retained.
 * Error penalty 4.5 RPM/unit; outward-rate penalty .08 RPM/(unit/s).
 * Fast fall 4 RPM/tick, slow recovery 2; greater values react faster.
 * Start 28 RPM preserves low EXIT handoff; MIN limits desired value only.
 * Continuous inner target 12 RPM is an assumption to validate on the car. */
#define TRACK_MAX_BASE_RPM 60
#define TRACK_MIN_FOLLOW_BASE_RPM 36
#define TRACK_ADAPTIVE_START_RPM 28
#define TRACK_BASE_RISE_RPM_PER_TICK 2
#define TRACK_BASE_FALL_RPM_PER_TICK 4
#define TRACK_ADAPTIVE_K_ERR_PERCENT 450
#define TRACK_ADAPTIVE_K_RATE_PERCENT 8
#define TRACK_MIN_CONTINUOUS_RPM 12
#if TRACK_MIN_FOLLOW_BASE_RPM < 1 || TRACK_MIN_FOLLOW_BASE_RPM > TRACK_MAX_BASE_RPM || TRACK_ADAPTIVE_START_RPM < 1 || TRACK_ADAPTIVE_START_RPM > TRACK_MAX_BASE_RPM
#error TRACK_adaptive_base_limits_invalid
#endif
#if TRACK_BASE_RISE_RPM_PER_TICK < 1 || TRACK_BASE_FALL_RPM_PER_TICK < TRACK_BASE_RISE_RPM_PER_TICK || TRACK_ADAPTIVE_K_ERR_PERCENT < 0 || TRACK_ADAPTIVE_K_RATE_PERCENT < 0
#error TRACK_adaptive_slopes_invalid
#endif
#if TRACK_MIN_CONTINUOUS_RPM < 1 || TRACK_MIN_CONTINUOUS_RPM*2 > TRACK_MIN_FOLLOW_BASE_RPM
#error TRACK_inner_forward_target_invalid
#endif
/* Error Trend: rate in sensor-position units/second, averaging 4 samples.
 * 3..5 samples recommended; more smooths noise but delays slowdown.
 * Rate limit 60..150 units/s; larger reacts harder to rapid outward motion. */
#define TRACK_ERROR_TREND_SAMPLES 4U
#define TRACK_ERROR_RATE_LIMIT 100
#define TRACK_ERROR_PREDICT_MS 20U /* P-only lookahead; 0..40 ms, experimental OFF */
#if TRACK_ERROR_TREND_SAMPLES < 3 || TRACK_ERROR_TREND_SAMPLES > 5 || TRACK_ERROR_RATE_LIMIT < 1 || TRACK_ERROR_RATE_LIMIT > 500
#error TRACK_error_trend_parameters_out_of_range
#endif
#if TRACK_ERROR_PREDICT_MS > 40
#error TRACK_error_prediction_horizon_too_large
#endif
/* Nonlinear Follow: position range -7..+7 sensor units. P only; D unchanged.
 * Percent slopes: centre stays nonzero; mid catches up at error=4;
 * outer strengthens rescue. 20..50 / 100..130 / 100..130 recommended.
 * Larger centre means more small-error steering; larger outer rescues harder. */
#define TRACK_CENTER_ZONE 1
#define TRACK_MEDIUM_ZONE 4
#define TRACK_CENTER_GAIN_PERCENT 25
#define TRACK_MEDIUM_GAIN_PERCENT 125
#define TRACK_OUTER_GAIN_PERCENT 110
#if TRACK_CENTER_ZONE < 1 || TRACK_MEDIUM_ZONE <= TRACK_CENTER_ZONE || TRACK_MEDIUM_ZONE > 7
#error TRACK_nonlinear_zones_out_of_range
#endif
#if TRACK_CENTER_GAIN_PERCENT < 10 || TRACK_CENTER_GAIN_PERCENT > 100 || TRACK_MEDIUM_GAIN_PERCENT < 50 || TRACK_MEDIUM_GAIN_PERCENT > 150 || TRACK_OUTER_GAIN_PERCENT < 100 || TRACK_OUTER_GAIN_PERCENT > 150
#error TRACK_nonlinear_slopes_out_of_range
#endif
/* Sensor Filter: three 50 Hz frames, majority step response <=40 ms.
 * Raw safety/confirm counters never wait for this extra frame. */
#define TRACK_SENSOR_FILTER_FRAMES 3U
#if TRACK_SENSOR_FILTER_FRAMES != 3
#error TRACK_SENSOR_FILTER_requires_three_frames
#endif
/* Trace: records at 20 ms, 200 = 4 s, fixed RAM only (32 bytes/record).
 * 150..250 recommended; larger retains more history and consumes more RAM. */
#define TRACK_TRACE_CAPACITY 200U
#if TRACK_TRACE_CAPACITY < 1 || TRACK_TRACE_CAPACITY > 250
#error TRACK_TRACE_CAPACITY_must_fit_8KB
#endif

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
#define TRACK_BASE_RPM                (1.0f*TRACK_MAX_BASE_RPM) /* 真实RPM，44mm轮约138mm/s；旧计数下20约等于75~80 */
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
/* 每20ms的加/减速界限。换向先降到0，至少一周期后再进入另一方向。 */
#define TRACK_APPROACH_ACCEL_RPM       3.0f /* 2~5 RPM/周期 */
#define TRACK_APPROACH_DECEL_RPM       6.0f /* 4~10 */
#define TRACK_TURN_ACCEL_RPM           4.0f /* 2~6 */
#define TRACK_TURN_DECEL_RPM           8.0f /* 5~12 */
#define TRACK_EXIT_ACCEL_RPM           2.0f /* 1~3，出弯恢复平滑 */
#define TRACK_EXIT_DECEL_RPM           6.0f
#define TRACK_EDGE_CORRECTION_RATIO    0.85f
#define TRACK_EDGE_MIN_CORRECTION      0.65f /* 到边缘时保证足够差速，避免紧弯纠正迟缓 */
#define TRACK_SIDE_CONFIRM_MS          60U /* 中线+单侧黑区连续三帧后跟随侧线 */
#define TRACK_TURN_HINT_MS            600U /* 一次短暂居中不抹去最近的转向方向 */
#define TRACK_EDGE_ALIGN_MS           600U /* 只在最外侧持续无改善时接管，不打断正常弧弯 */
#define TRACK_EDGE_ALIGN_ERROR         6.0f
#define TRACK_EDGE_PIVOT_RPM          50.0f /* 有线贴边：内轮停、外轮低速前进，不反扫 */
#define TRACK_EDGE_ALIGN_LIMIT_RAD     0.6108652f /* 35°内仍未回线则停止本次恢复 */
#define TRACK_EDGE_ALIGN_MAX_MS     2500U
#define TRACK_RECOVERY_SOFT_MS        400U /* 旧参数兼容：现由ALIGN/EXIT接管，不再用于TRACK降速 */
#define TRACK_RECOVERY_RPM            42.0f /* 旧参数兼容，当前恢复速度使用ALIGN/EXIT参数 */

/* 5. 左右轮分别执行速度 PI：转慢就加 PWM，转快就减 PWM。
 * 起动助推只短暂施加，不会在卡轮后无限维持；PWM 单位为百分比。
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
#define TRACK_BOOST_PWM               18.0f /* 缩短、降低原40%助推，防止高速电机甩头 */
#define TRACK_BOOST_EXTRA_PWM          4.0f /* 第一帧仍完全无计数，最后一帧最多加到22% */
#define TRACK_BOOST_WAIT_MS          120U
#define TRACK_BOOST_DURATION_MS       40U
#define TRACK_BOOST_REARM_MS         100U
#define TRACK_MAX_PWM                 30 /* 闭环最高30%；KEY5人工检查仍固定30% */
#define TRACK_MIN_DRIVE_RPM            0.0f /* 仅零目标停转；非零小目标仍按真实 RPM 闭环，不抬速 */
#define TRACK_SPEED_COAST_MARGIN      12.0f /* 超速时松开驱动并卸掉旧积分 */
#define ENCODER_FAULT_MIN_PWM           8 /* 新版低速PWM较小，不能仍以18%作为故障检测门槛 */

/* 低速结构参数：20ms单计数约12RPM，100ms窗口约2.4RPM，不改计数倍率/PI。 */
#define TRACK_RPM_WINDOW_SAMPLES        5U /* 3~6，增大更平滑但响应变慢 */
#define TRACK_LOW_SPEED_RPM           24.0f /* 18~30，仅低速用窗口，高速沿用原60ms滤波响应 */
#define TRACK_LOW_SPEED_FULL_RPM       8.0f /* 6~12，8~24RPM之间线性混合窗口/原测速 */
#define TRACK_BOOST_RAMP_MAX_MS       120U /* 80~160，按3%/周期从低输出爬至22%的有界时间 */
#define TRACK_PWM_RISE_PER_TICK         3.0f /* 1~3 %/20ms，限定有限助推的输出斜率 */
#define TRACK_PWM_FALL_PER_TICK         4.0f /* 2~6 %/20ms；故障/零目标立即撤驱动 */
#define TRACK_LOAD_LEARN_MIN_RPM        8.0f
#define TRACK_LOAD_LEARN_ERROR_RPM      8.0f /* 5~10，接近稳态且未饱和时才保存本方向补偿 */
#define TRACK_LOAD_LEARN_PWM_MARGIN     1.0f
#define TRACK_BOOST_MOVING_RPM          6.0f
#define TRACK_REVERSE_COAST_RPM         6.0f
#define TRACK_REVERSE_BLANK_MS         40U /* 20~80，换向先撤驱动 */
#define TRACK_REVERSE_COAST_MAX_MS    220U /* 120~300，换向惯性宽容，总故障宽容另设 */
#define ENCODER_DIRECTION_WINDOW       15U /* 300ms滑动窗口，不因零计数清空反向证据 */
#define ENCODER_DIRECTION_MIN_COUNTS     6U /* 4~10，至少这么多有符号反向净计数 */
#define ENCODER_REVERSE_PERCENT        75U /* 65~90，反向脉冲占比 */
#define ENCODER_REVERSAL_GRACE_MS      260U /* 200~400，正常换向惯性不作为故障证据 */
#define LEFT_REVERSE_DRIVE_OFFSET_PWM  LEFT_DRIVE_OFFSET_PWM /* 未测反转，初值对称 */
#define RIGHT_REVERSE_DRIVE_OFFSET_PWM RIGHT_DRIVE_OFFSET_PWM
#define LEFT_REVERSE_FEEDFORWARD       LEFT_SPEED_FEEDFORWARD
#define RIGHT_REVERSE_FEEDFORWARD      RIGHT_SPEED_FEEDFORWARD

/* 6. 直角：形态+整数计分确认宽带，全白后按175mm几何推进轴线。
 * 可靠单侧角点→有效hint→可靠历史；无证据不默认左/右。
 * 分段旋转→120ms接线确认→低速ALIGN→700ms平滑EXIT。
 */
#define CORNER_APPROACH_RPM           50.0f
#define CORNER_RPM                    42.0f
#define CORNER_ALIGN_RPM              30.0f
#define CORNER_CONFIRM_MS             40U /* 至少两帧宽线 */
#define CORNER_CONFIRM_GAP_MS         40U /* 两帧宽带之间容许一两帧漏读，单帧仍不推进 */
#define CORNER_MIN_ADVANCE_PROBES       5U /* 旧参数兼容：现以形态与置信分数决定推进 */
#define CORNER_LOST_CONFIRM_MS        40U /* 宽线后至少两帧全白 */
#define CORNER_MEMORY_MS             200U /* 宽线线索最多保留 200 ms */
#define CORNER_CLEAR_CENTER_MS       100U /* 恢复中线需连续确认，不能一帧撤销角点 */
#define CORNER_CLEAR_TRAVEL_MM          8.0f
#define CORNER_SETTLE_MS              80U /* 旧参数兼容：现以目标斜率与每轮换向宽容替代整车刹停 */
#define CORNER_MIN_ANGLE_RAD            1.2217305f /* 70°，60~75°；前置175mm避免认回原线 */
#define CORNER_SCAN_LIMIT_RAD          1.8325957f /* 每侧相对原方向最多 105° */
#define CORNER_DEFAULT_DIR             0 /* 兼容旧参数名，停用固定左右默认方向 */
#define CORNER_APPROACH_MAX_MS      6000U /* 推进异常超时停车，不能提前旋转掩盖 */
#define CORNER_TURN_MAX_MS         12000U /* 两侧都找不到线就停车 */
#define CORNER_CENTER_SAMPLES          6U /* 120ms，排除1~2帧偶然中心读数；4~8 */
#define CORNER_REARM_CENTER_MS       160U
#define CORNER_REARM_TRAVEL_MM        50.0f /* 离开上一角点后才重新识别 */

/* 角点计分和方向：加分、衰减均为整数，门限依正常线宽相对变化。 */
#define TRACK_HINT_ERROR               3.0f
#define TRACK_CENTER_ERROR             1.5f
#define TRACK_RECENT_LINE_MS          300U
#define TRACK_TURN_HISTORY_MS        2000U /* 1000~3000，可靠出弯历史也会过期 */
#define CORNER_WIDTH_MIN_PROBES         4U
#define CORNER_WIDTH_EXTRA_PROBES       2U
#define CORNER_STRONG_WIDTH_EXTRA       4U
#define CORNER_CONFIDENCE_REQUIRED      4U /* 3~6，强宽带2帧，弱宽带4帧 */
#define CORNER_CONFIDENCE_MAX           8U
#define CORNER_CONFIDENCE_STRONG        2U
#define CORNER_CONFIDENCE_WEAK          1U
#define CORNER_CONFIDENCE_DECAY         1U
#define CORNER_DIRECTION_VOTES         2  /* 2~3，单帧侧线不直接改变转向 */
#define CORNER_DIRECTION_VOTE_MAX       4
#define CORNER_CONFIRM_MAX_MS       1800U /* 1000~2500，宽线卡住不能无限直行 */
#define CORNER_CONFIRM_MAX_TRAVEL_MM  100.0f /* 60~150 */
#define CORNER_WIDE_HALF_MIN_MM         2.0f
#define CORNER_WIDE_HALF_MAX_MM        45.0f
#define CORNER_APPROACH_MIN_MM        130.0f /* 110~145，不改175mm探头几何 */
#define CORNER_APPROACH_TARGET_MAX_MM 190.0f
#define CORNER_APPROACH_MAX_MM        200.0f /* 190~215，超过则停车 */
#define CORNER_APPROACH_SYNC_KP         0.25f /* 0.1~0.4，用窗口RPM限制两轮推进偏航 */
#define CORNER_APPROACH_YAW_KP         24.0f /* 12~32 RPM/rad，纠正累计偏航，输出仍限±6RPM */
#define CORNER_APPROACH_SYNC_RPM        8.0f /* 5~12，先低速把两轮同步，再推进50RPM */
#define CORNER_APPROACH_SYNC_MS       100U /* 60~160 */
#define CORNER_APPROACH_MAX_DIFF_RPM   12.0f /* 6~16 */
#define CORNER_APPROACH_END_RPM        28.0f /* 24~34，推进末段提前降速，减少轴线过冲 */
#define CORNER_APPROACH_SLOW_MM        35.0f /* 25~50，距离目标这么远时开始减速 */
#define CORNER_APPROACH_MAX_YAW_RAD     0.2617994f /* 15°，10~20° */
#define CORNER_FAST_END_RAD             0.8726646f /* 50°，40~55° */
#define CORNER_SLOW_END_RAD             1.2217305f /* 70°，60~75° */
#define CORNER_MID_RPM                 30.0f /* 24~34 */
#define CORNER_FIND_RPM                18.0f /* 12~24；真实非零低速闭环 */
#define TRACK_ACQUIRE_ERROR             2.0f /* 1.5~2.5，含中心探头的窄线 */
#define TRACK_ACQUIRE_TREND_TOL         1.0f /* 相邻周期朝中央变化时允许数字探头噪声 */
#define TRACK_ACQUIRE_CROSS_ERROR       1.5f
#define TRACK_ALIGN_BASE_RPM           24.0f /* 18~28 */
#define TRACK_ALIGN_KP                  4.0f /* 2~5 */
#define TRACK_ALIGN_MAX_DIFF_RPM       20.0f /* 12~24，总左右速度差 */
#define TRACK_ALIGN_CENTER_MS         160U /* 120~240 */
#define TRACK_ALIGN_MAX_MS           1800U /* 1200~2500 */
#define TRACK_ALIGN_MAX_TRAVEL_MM      65.0f
#define TRACK_ALIGN_MAX_YAW_RAD         0.3490659f /* 找线后最多附加20° */
#define TRACK_EXIT_START_RPM           28.0f /* 20~30 */
#define TRACK_EXIT_MID_RPM             40.0f
#define TRACK_EXIT_MID_MS             200U
#define TRACK_EXIT_FULL_MS            500U
#define TRACK_EXIT_MIN_MS             700U /* 500~1000，稳定后才交回TRACK */
#define TRACK_EXIT_CENTER_MS          200U
#define TRACK_EXIT_CENTER_ERROR         2.0f /* 1.5~2.0，量化边界允许约12mm偏差，仍须中心探头 */
#define TRACK_EXIT_MID_CENTER_MS       80U
#define TRACK_EXIT_MIN_TRAVEL_MM       25.0f /* 15~40，静止假中心不能通过 */
#define TRACK_EXIT_MAX_MS            3000U /* 2200~3500，高负载回正留时间，会话总预算仍12s */
#define TRACK_RECOVERY_EXIT_ERROR      5.0f /* 普通弧弯恢复允许稳定侧线；直角仍要求中心 */
#define TRACK_EXIT_WHITE_GRACE_MS     120U /* 100~160，20~100ms白缝不反扫 */
#define TRACK_EXIT_PREDICT_RPM         16.0f
#define TRACK_EXIT_PREDICT_DIFF_RPM     6.0f
#define TRACK_NEAR_LINE_SCAN_RAD        0.2617994f /* 15°，出弯附近有限修正 */
#define TRACK_UNKNOWN_PROBE_MS        300U /* 无方向且无新线索，短时后停车 */
#define TRACK_UNKNOWN_PROBE_RAD         0.2094395f /* 弱方向线索最多试12° */
#define TRACK_AMBIGUOUS_MAX_MS        240U /* 对称分离双线无中心/无历史时，不任意拉向左侧 */
#define TRACK_WRONG_TURN_RAD            0.1745329f /* 10°反向位移，编码器/接线检查 */
#define TRACK_CONTROL_MAX_GAP_MS       40U /* >2周期未调度，立即停车，不用大dt补算 */
#define TRACK_FORWARD_TEST_MAX_MS   15000U /* 人工开环测试也有限时 */
/* 台架测试默认关闭；打开前架空驱动轮，改这里的±5/±12/±18逐轮测。 */
#ifndef TRACK_BENCH_TEST
#define TRACK_BENCH_TEST                0 /* 1：上电只执行一次下面的3s速度测试 */
#endif
#define TRACK_BENCH_LEFT_RPM            5 /* 单轮测试另一轮填0；双轮反向可用-18/+18 */
#define TRACK_BENCH_RIGHT_RPM           0
#define TRACK_BENCH_MAX_RPM            60
#define TRACK_BENCH_MAX_MS           3000U

/* 7. 普通丢线不套用前置距离。短暂全白先缓冲，确认后有限角度扫描两侧。 */
#define TRACK_LOST_GRACE_MS           80U /* 多留一帧给窄线探头间隙，仍低速保留差速 */
#define TRACK_SEARCH_RPM              36.0f
#define TRACK_SEARCH_SMALL_RPM        24.0f /* 18~28，小角阶段慢找，扩大时才用36RPM */
#define TRACK_SEARCH_SMALL_RAD         0.2617994f /* 15°，10~20° */
#define TRACK_SEARCH_SCAN_RAD          1.0471976f /* 每侧相对原方向最多 60° */
#define TRACK_SEARCH_MAX_MS        10000U /* 44mm轮/133mm轮距，覆盖两侧扫描及起转时间 */
/* 会话覆盖 APPROACH/TURN/ALIGN/EXIT/SEARCH 和假恢复期间的 TRACK。 */
#define TRACK_RECOVERY_MAX_MS       12000U /* 8000~15000，从首次真正失线起计时 */
#define TRACK_RECOVERY_MAX_ATTEMPTS     4U /* 3~5，仅人工启动或稳定TRACK后归零 */
#define TRACK_RECOVERY_MAX_ANGLE_RAD    5.2359878f /* 300°，220~360°累计绝对转角 */
#define TRACK_RECOVERY_MAX_DISTANCE_MM 700.0f /* 500~900，两轮绝对路程的平均，含旋转 */
#define TRACK_RECOVERY_STABLE_MS      600U /* 500~1000，正常循迹连续稳定 */
#define TRACK_RECOVERY_STABLE_MM       30.0f /* 20~60，与稳定时间同时成立才清会话 */
#define TRACK_RECOVERY_STABLE_ERROR     5.0f /* 3~5，正常小半径弯道的稳定偏侧线也有效 */
#define TRACK_RECOVERY_PREDICT_MAX_MS 2000U /* 1000~3000，交替短白缝也不能刷新预测预算 */
#define ENCODER_NO_PULSE_MS          700U
#define ENCODER_REVERSE_MS           250U
#define TRACK_OLED_ENABLED             1
#ifndef TRACK_DEBUG
#define TRACK_DEBUG                    0 /* 1：每100ms更新track_debug，Keil Watch可直接看 */
#endif
#define TRACK_DEBUG_PERIOD_MS        100U
#define TRACK_OLED_ROW_MS             60U
#define TRACK_OLED_PAGE_MS          1200U /* 800~2000，T/V、PWM、角度/距离三个页面 */
#define TRACK_OLED_SERVICE_BYTES       8U /* 每空闲片只发8个像素字节 */
#define TRACK_OLED_IDLE_MARGIN_MS      3U /* 控制到期前至少3ms，不启动OLED传输 */
#define TRACK_OLED_SERVICE_MS          1U /* 同一毫秒不重复刷，控制优先 */
#define FORWARD_PWM                   30 /* KEY5 人工直行检查 */
#define KEY_DEBOUNCE_MS               20U

#if DRIVE_PAIRS_SWAPPED != 0 && DRIVE_PAIRS_SWAPPED != 1
#error DRIVE_PAIRS_SWAPPED_must_be_0_or_1
#endif
#if TRACK_SENSOR_REVERSED != 0 && TRACK_SENSOR_REVERSED != 1
#error TRACK_SENSOR_REVERSED_must_be_0_or_1
#endif
#if CORNER_DEFAULT_DIR != 0
#error CORNER_DEFAULT_DIR_is_disabled_use_zero
#endif
#if TRACK_RPM_WINDOW_SAMPLES < 1 || TRACK_RPM_WINDOW_SAMPLES > 16
#error TRACK_RPM_WINDOW_SAMPLES_out_of_range
#endif
#if ENCODER_DIRECTION_WINDOW < 1 || ENCODER_DIRECTION_WINDOW > 32
#error ENCODER_DIRECTION_WINDOW_out_of_range
#endif
#if TRACK_OLED_SERVICE_BYTES != 8
#error OLED_service_uses_8_pixel_byte_tiles
#endif
#endif
