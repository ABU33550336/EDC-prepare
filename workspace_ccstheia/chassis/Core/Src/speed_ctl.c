//底盘轮速PID控制器,编码器反馈速度闭环,50ms周期更新
//适用于JGB37-520直流减速电机,配合hw_adapter层TB6612驱动
//输出范围[-1000,1000]直接映射为HW_SetMotorPWM的PWM占空比
#include "speed_ctl.h"
#include "hw_adapter.h"

/**
 * @brief 初始化速度控制器参数为默认值
 * @param sc 控制器实例指针
 * @note kp=0.5,ki=0.05,kd=0.0(D默认关闭),积分限幅200,输出限幅1000
 *       左右轮使用相同参数;记录当前编码器值作为首次差分的基准
 *       main
 */
void SpeedCtl_Init(SpeedCtl_t *sc)
{
    //kp=0.5:误差1RPM产生0.5 PWM输出,对330RPM满量程≈0.15%PWM/%
    //ki=0.05:每50ms叠加0.05×误差,等效积分时间常数=1/0.05=20个周期=1s
    //kd=0.0:默认关闭,待实际跑车确认是否需要阻尼再开启
    sc->left.kp  = 0.5f;
    sc->left.ki  = 0.05f;
    sc->left.kd  = 0.0f;
    sc->left.integral        = 0.0f;
    //积分限幅200:允许积分最大贡献20%PWM,防止启动时大幅超调
    sc->left.integral_limit  = 200.0f;
    //输出限幅1000:与HW_SetMotorPWM的范围[-1000,1000]完全对应
    sc->left.output_limit    = 1000.0f;
    sc->left.prev_err        = 0.0f;

    sc->right.kp  = 0.5f;
    sc->right.ki  = 0.05f;
    sc->right.kd  = 0.0f;
    sc->right.integral        = 0.0f;
    sc->right.integral_limit  = 200.0f;
    sc->right.output_limit    = 1000.0f;
    sc->right.prev_err        = 0.0f;

    sc->target_rpm_l = 0.0f;
    sc->target_rpm_r = 0.0f;
    sc->meas_rpm_l   = 0.0f;
    sc->meas_rpm_r   = 0.0f;

    //取当前编码器值作为首次差分的last,这样首次运行时不会产生大脉冲
    sc->last_enc_l = HW_GetEncoderCnt(HW_ENCODER_CH_LEFT);
    sc->last_enc_r = HW_GetEncoderCnt(HW_ENCODER_CH_RIGHT);
    sc->acc_enc_l  = 0;
    sc->acc_enc_r  = 0;
    sc->last_run   = HW_GetTick();
    sc->enabled    = false;
}

/**
 * @brief 使能速度控制,清零积分和上次误差防启动冲击
 * @param sc 控制器实例指针
 * @note 每次使能时清零integral和prev_err,防止上次Disable残留的累积值
 *       导致使能瞬间电机全速冲出(常见于积分饱和恢复场景)
 *       调用者:
 *       main按钮(empty_cpp.cpp)
 */
void SpeedCtl_Enable(SpeedCtl_t *sc)
{
    sc->enabled = true;
    //积分不清零的话,之前累加的积分会在使能瞬间释放,导致电机突然加速
    sc->left.integral  = 0.0f;
    sc->right.integral = 0.0f;
    //prev_err不清零的话,首次err计算出的deriv会异常大(prev_err=0→实际≠0)
    sc->left.prev_err  = 0.0f;
    sc->right.prev_err = 0.0f;
}

/**
 * @brief 禁能速度控制,停机并清零
 * @param sc 控制器实例指针
 * @note 清零目标/积分后直接HW_SetMotorPWM(0,0),而不是依靠PID输出趋于0
 *       调用者:
 *       main按钮(empty_cpp.cpp)
 */
void SpeedCtl_Disable(SpeedCtl_t *sc)
{
    sc->enabled = false;
    //目标清零:防止下次Enable时读到上次的旧目标导致突然加速
    sc->target_rpm_l = 0;
    sc->target_rpm_r = 0;
    sc->left.integral  = 0.0f;
    sc->right.integral = 0.0f;
    sc->left.prev_err  = 0.0f;
    sc->right.prev_err = 0.0f;
    //直接强制停机,不依赖PID计算,确保安全
    HW_SetMotorPWM(0, 0);
}

/**
 * @brief 设置目标转速
 * @param sc 控制器实例指针
 * @param left_rpm 左轮目标,rpm,±SPEED_CTL_MAX_RPM限幅
 * @param right_rpm 右轮目标,rpm,±SPEED_CTL_MAX_RPM限幅
 * @note 先赋值再限幅,限幅到±330RPM(JGB37-520额定),超限虽不会烧电机,
 *       但编码器采样精度会因脉冲间隔过小(>330RPM时<2ms)而无法可靠测速
 *       调用者:
 *       main按钮(empty_cpp.cpp)
 */
void SpeedCtl_SetTarget(SpeedCtl_t *sc, float left_rpm, float right_rpm)
{
    sc->target_rpm_l = left_rpm;
    sc->target_rpm_r = right_rpm;
    //限幅:超过额定转速时力矩急剧下降,且11PPR编码器无法辨别
    if (left_rpm > SPEED_CTL_MAX_RPM)  sc->target_rpm_l = SPEED_CTL_MAX_RPM;
    if (left_rpm < -SPEED_CTL_MAX_RPM) sc->target_rpm_l = -SPEED_CTL_MAX_RPM;
    if (right_rpm > SPEED_CTL_MAX_RPM)  sc->target_rpm_r = SPEED_CTL_MAX_RPM;
    if (right_rpm < -SPEED_CTL_MAX_RPM) sc->target_rpm_r = -SPEED_CTL_MAX_RPM;
}

/**
 * @brief 更新PID参数(左右轮共用)
 * @param sc 控制器实例指针
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 * @note 左右轮使用相同PID参数,若需左右差异化(如编码器分辨率不同导致增益差异)
 *       需分别维护kp/ki/kd并修改此函数签名或改用独立setter
 *       调用者:
 *       exec_cmd(debug_cli.c,当前CLI已省略)
 */
void SpeedCtl_SetParam(SpeedCtl_t *sc, float kp, float ki, float kd)
{
    sc->left.kp  = kp;
    sc->left.ki  = ki;
    sc->left.kd  = kd;
    sc->right.kp = kp;
    sc->right.ki = ki;
    sc->right.kd = kd;
}

/**
 * @brief PID速度闭环更新主函数,每50ms执行一次
 * @param sc 控制器实例指针
 * @note 测速公式: meas = acc_pulses × 60000 / (CPR × elapsed)
 *       ↑ per-second脉冲 ÷ per-rev脉冲 × 60秒 = RPM
 *       输出公式: out = kp×err + integral + kd×(err - prev_err)
 *       限幅顺序: 先限幅积分再计算输出,最后限幅输出(防积分饱和的关键)
 *       调用者:
 *       main(empty_cpp.cpp每loop调用)
 */
void SpeedCtl_Update(SpeedCtl_t *sc)
{
    //--- 阶段一:编码器差分累积(每帧执行,不依赖周期) ---
    //每帧都读取并差分,确保不丢失脉冲(丢失一个脉冲=转速计算偏差)
    int32_t now_l = HW_GetEncoderCnt(HW_ENCODER_CH_LEFT);
    int32_t now_r = HW_GetEncoderCnt(HW_ENCODER_CH_RIGHT);
    sc->acc_enc_l += now_l - sc->last_enc_l;
    sc->acc_enc_r += now_r - sc->last_enc_r;
    sc->last_enc_l = now_l;
    sc->last_enc_r = now_r;

    //--- 阶段二:判断是否到执行周期 ---
    uint32_t now = HW_GetTick();
    uint32_t elapsed = now - sc->last_run;
    if (elapsed < SPEED_CTL_PERIOD_MS) return;
    sc->last_run = now;

    //主循环卡顿保护:如果超过3个周期(150ms)没跑进来,说明主循环被阻塞
    //放弃本次计算并清零prev_err,防止恢复后微分项产生异常大的跳变输出
    if (elapsed > SPEED_CTL_PERIOD_MS * 3) {
        sc->acc_enc_l = 0;
        sc->acc_enc_r = 0;
        sc->left.prev_err  = 0.0f;
        sc->right.prev_err = 0.0f;
        return;
    }

    //--- 阶段三:计算实测RPM ---
    //测速原理: 编码器Δ脉冲 ÷ (CPR × Δ秒) = 转/秒 → ×60 = 转/分
    //这里Δ用ms: Δ脉冲/CPR = 圈数, ÷Δms×1000 = 圈/秒, ×60 = 圈/分
    //合并: Δ脉冲 × 60000 / (CPR × Δms)
    sc->meas_rpm_l = (float)sc->acc_enc_l * 60000.0f
                   / (SPEED_CTL_LEFT_CPR * (float)elapsed);
    sc->meas_rpm_r = (float)sc->acc_enc_r * 60000.0f
                   / (SPEED_CTL_RIGHT_CPR * (float)elapsed);
    sc->acc_enc_l = 0;
    sc->acc_enc_r = 0;

    if (!sc->enabled) return;

    //--- 阶段四:PID计算 ---
    //关键限幅顺序:先限幅积分→再组合输出→最后限幅输出
    //如果先组合再整体限幅,积分项得不到抑制,下次还会继续增长(积分饱和)

    //左轮PID
    float err_l = sc->target_rpm_l - sc->meas_rpm_l;
    //微分项:kd×(Δerr/Δt),这里Δt隐含在kd的标定中(周期固定50ms)
    //即kd的含义是"每变化1RPM误差,输出变化kd个PWM单位(每50ms)"
    float deriv_l = (err_l - sc->left.prev_err) * sc->left.kd;
    sc->left.prev_err = err_l;
    //积分累积:每周期加一次ki×err,限幅200防止过饱和
    sc->left.integral += sc->left.ki * err_l;
    if (sc->left.integral > sc->left.integral_limit)
        sc->left.integral = sc->left.integral_limit;
    if (sc->left.integral < -sc->left.integral_limit)
        sc->left.integral = -sc->left.integral_limit;
    float out_l = sc->left.kp * err_l + sc->left.integral + deriv_l;
    if (out_l > sc->left.output_limit) out_l = sc->left.output_limit;
    if (out_l < -sc->left.output_limit) out_l = -sc->left.output_limit;

    //右轮PID(同上)
    float err_r = sc->target_rpm_r - sc->meas_rpm_r;
    float deriv_r = (err_r - sc->right.prev_err) * sc->right.kd;
    sc->right.prev_err = err_r;
    sc->right.integral += sc->right.ki * err_r;
    if (sc->right.integral > sc->right.integral_limit)
        sc->right.integral = sc->right.integral_limit;
    if (sc->right.integral < -sc->right.integral_limit)
        sc->right.integral = -sc->right.integral_limit;
    float out_r = sc->right.kp * err_r + sc->right.integral + deriv_r;
    if (out_r > sc->right.output_limit) out_r = sc->right.output_limit;
    if (out_r < -sc->right.output_limit) out_r = -sc->right.output_limit;

    HW_SetMotorPWM((int16_t)out_l, (int16_t)out_r);
}
