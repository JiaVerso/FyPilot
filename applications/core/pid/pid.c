/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "pid.h"
#include "common/filter_math.h"

typedef enum {
    LEVEL_MODE_OFF = 0,
    LEVEL_MODE_R,
    LEVEL_MODE_RP,
} levelMode_e;

const char pidNames[] =
    "ROLL;"
    "PITCH;"
    "YAW;"
    "LEVEL;"
    "MAG;";

uint32_t targetPidLooptime;
pidAxisData_t pidData[XYZ_AXIS_COUNT];
pidRuntime_t pidRuntime;

static pidProfile_t profiles[PID_PROFILE_COUNT];
static pidConfig_t config = { .pid_process_denom = 1 };

static struct {
    float previousSetpoint[3];
    float previousGyroRateDterm[3];
    float previousThrottle;
    bool isTpaLowFaded;
    bool dtermPrimed;
} history;

const pidProfile_t *pidProfiles(unsigned index)
{
    return index < PID_PROFILE_COUNT ? &profiles[index] : NULL;
}

 /* 返回PID参数组的地址 */
pidProfile_t *pidProfilesMutable(unsigned index)
{
    return index < PID_PROFILE_COUNT ? &profiles[index] : NULL;
}
const pidConfig_t *pidConfig(void) { return &config; }
pidConfig_t *pidConfigMutable(void) { return &config; }

static void resetLowpassState(dtermLowpass_t *filter, uint8_t type)
{
    switch (type) {
    case FILTER_PT1: pt1FilterInit(&filter->pt1Filter, filter->pt1Filter.k); break;
    case FILTER_PT2: pt2FilterInit(&filter->pt2Filter, filter->pt2Filter.k); break;
    case FILTER_PT3: pt3FilterInit(&filter->pt3Filter, filter->pt3Filter.k); break;
    }
}

void pidResetState(void)
{
    memset(pidData, 0, sizeof(pidData));
    memset(&history, 0, sizeof(history));
    memset(pidRuntime.previousPidSetpoint, 0, sizeof(pidRuntime.previousPidSetpoint));
    pidRuntime.antiGravityThrottleD = 0.0f;
    pidRuntime.itermAccelerator = 0.0f;
    pidRuntime.angleYawSetpoint = 0.0f;
    memset(pidRuntime.angleTarget, 0, sizeof(pidRuntime.angleTarget));
    memset(pidRuntime.axisInAngleMode, 0, sizeof(pidRuntime.axisInAngleMode));
    pt1FilterInit(&pidRuntime.ptermYawLowpass, pidRuntime.ptermYawLowpass.k);
    pt2FilterInit(&pidRuntime.antiGravityLpf, pidRuntime.antiGravityLpf.k);
    pt1FilterInit(&pidRuntime.horizonSmoothingPt1, pidRuntime.horizonSmoothingPt1.k);
    for (int i = 0; i < 3; ++i) {
        resetLowpassState(&pidRuntime.dtermLowpass[i], pidRuntime.dtermLowpassType);
        pt1FilterInit(&pidRuntime.windupLpf[i], pidRuntime.windupLpf[i].k);
        pt2FilterInit(&pidRuntime.dMinRange[i], pidRuntime.dMinRange[i].k);
        pt2FilterInit(&pidRuntime.dMinLowpass[i], pidRuntime.dMinLowpass[i].k);
        pt3FilterInit(&pidRuntime.angleFeedforwardPt3[i], pidRuntime.angleFeedforwardPt3[i].k);
        if (i < 2)
            pt3FilterInit(&pidRuntime.attitudeFilter[i], pidRuntime.attitudeFilter[i].k);
    }
}

void resetPidProfile(pidProfile_t *pidProfile)
{
    if (pidProfile == NULL) 
        return;

    /* 初始化默认参数 */
    pidProfile_t profile = {
        .pid = {
            [PID_ROLL] =  PID_ROLL_DEFAULT,
            [PID_PITCH] = PID_PITCH_DEFAULT,
            [PID_YAW] =   PID_YAW_DEFAULT,
            [PID_LEVEL] = { 50, 75, 75, 50 },
            [PID_MAG] =   { 40, 0, 0, 0 },
        },
        .pidSumLimit = PIDSUM_LIMIT,
        .pidSumLimitYaw = PIDSUM_LIMIT_YAW,
        .yaw_lowpass_hz = 100,
        .itermWindupPointPercent = 85,
        .pidAtMinThrottle = PID_STABILISATION_ON,
        .angle_limit = 60,
        .feedforward_transition = 0,
        .yawRateAccelLimit = 0,
        .rateAccelLimit = 0,
        .anti_gravity_gain = 80,

        .horizon_limit_degrees = 135,
        .horizon_ignore_sticks = false,

        .itermLimit = 400,

        .iterm_rotation = false,
        .iterm_relax = ITERM_RELAX_RP,
        .iterm_relax_cutoff = ITERM_RELAX_CUTOFF_DEFAULT,
        .iterm_relax_type = ITERM_RELAX_SETPOINT,

        .dterm_lpf1_static_hz = DTERM_LPF1_DYN_MIN_HZ_DEFAULT,

        .dterm_lpf1_type = FILTER_PT1,
        .dterm_lpf1_dyn_min_hz = DTERM_LPF1_DYN_MIN_HZ_DEFAULT,
        .dterm_lpf1_dyn_max_hz = DTERM_LPF1_DYN_MAX_HZ_DEFAULT,

        .d_min = D_MIN_DEFAULT,
        .d_min_gain = 37,
        .d_min_advance = 20,
        .motor_output_limit = 100,
        .auto_profile_cell_count = 0,

        .profileName = { 0 },
        .dyn_idle_min_rpm = 0,
        .dyn_idle_p_gain = 50,
        .dyn_idle_i_gain = 50,
        .dyn_idle_d_gain = 50,
        .dyn_idle_max_increase = 150,
        .dyn_idle_start_increase = 50,
        .feedforward_averaging = FEEDFORWARD_AVERAGING_OFF,
        .feedforward_max_rate_limit = 90,
        .feedforward_smooth_factor = 25,
        .feedforward_jitter_factor = 7,
        .feedforward_boost = 15,
        .dterm_lpf1_dyn_expo = 5,
        .level_race_mode = false,
        .vbat_sag_compensation = 0,
        .simplified_pids_mode = 2,
        .simplified_master_multiplier = 100,
        .simplified_roll_pitch_ratio = 100,
        .simplified_i_gain = 100,
        .simplified_d_gain = 100,
        .simplified_pi_gain = 100,
        .simplified_dmin_ratio = 100,
        .simplified_feedforward_gain = 100,
        .simplified_pitch_pi_gain = 100,
        .simplified_dterm_filter = true,
        .simplified_dterm_filter_multiplier = 100,
        .anti_gravity_cutoff_hz = 5,
        .anti_gravity_p_gain = 100,
        .tpa_mode = TPA_MODE_D,
        .tpa_rate = 65,
        .tpa_breakpoint = 1350,
        .angle_feedforward_smoothing_ms = 80,
        .angle_earth_ref = 100,
        .horizon_delay_ms = 500, // 500ms time constant on any increase in horizon strength
        .tpa_low_rate = 20,
        .tpa_low_breakpoint = 1050,
        .tpa_low_always = 0,
        .ez_landing_threshold = 25,
        .ez_landing_limit = 15,
        .ez_landing_speed = 50,
    };
    *pidProfile = profile;
}

void pidResetProfiles(void)
{
    for (int i = 0; i < PID_PROFILE_COUNT; i++) {
        resetPidProfile(&profiles[i]);
    }
}

void pidSetItermAccelerator(float newItermAccelerator)
{
    pidRuntime.itermAccelerator = newItermAccelerator;
}

bool pidOsdAntiGravityActive(void)
{
    return (pidRuntime.itermAccelerator > pidRuntime.antiGravityOsdCutoff);
}

void pidStabilisationState(pidStabilisationState_e pidControllerState)
{
    pidRuntime.pidStabilisationEnabled = (pidControllerState == PID_STABILISATION_ON) ? true : false;
}


void pidResetIterm(void)
{
    for (int axis = 0; axis < 3; axis++) {
        pidData[axis].I = 0.0f;
    }
}

void pidUpdateTpaFactor(float throttle)
{
    float tpaRate = 0.0f;

    /* 约束油门 */
    throttle = constrainf(throttle, 0.0f, 1.0f);

    /* 油门是否越过最低油门点，未越过: False */
    bool isThrottlePastTpaLowBreakpoint = (throttle < pidRuntime.tpaLowBreakpoint && pidRuntime.tpaLowBreakpoint > 0.01f) ? false : true;
    
    /* 正常油门段以及高油门段 */
    if (isThrottlePastTpaLowBreakpoint) {
        tpaRate = pidRuntime.tpaMultiplier * fmaxf(throttle - pidRuntime.tpaBreakpoint, 0.0f);
    } else {
    /* 低油门段 */
        tpaRate = pidRuntime.tpaLowMultiplier * (pidRuntime.tpaLowBreakpoint - throttle);
    }

    /* PID增益系数 */
    pidRuntime.tpaFactor = 1.0f - tpaRate;
}

 /* 油门快速变化时补偿 I 项，防止掉高或过冲 */
void pidUpdateAntiGravityThrottleFilter(float throttle)
{
    const float throttleInv = 1.0f - throttle;
    float throttleDerivative = fabsf(throttle - history.previousThrottle) * pidRuntime.pidFrequency;
    throttleDerivative *= throttleInv * throttleInv;

    if (throttle > history.previousThrottle) {
        throttleDerivative *= throttleInv * 0.5f;

    }
    history.previousThrottle = throttle;
    throttleDerivative = pt2FilterApply(&pidRuntime.antiGravityLpf, throttleDerivative);

    pidRuntime.antiGravityThrottleD = throttleDerivative;
}

static float calcHorizonLevelStrength(const pidInput_t *input)
{
    const float currentInclination = MAX(fabsf(input->attitude_deg[FD_ROLL]), fabsf(input->attitude_deg[FD_PITCH]));

    float absMaxStickDeflection = MAX(fabsf(input->rc_deflection[FD_ROLL]), fabsf(input->rc_deflection[FD_PITCH]));

    float horizonLevelStrength = MAX((pidRuntime.horizonLimitDegrees - currentInclination) * pidRuntime.horizonLimitDegreesInv, 0.0f);

    horizonLevelStrength *= MAX((pidRuntime.horizonLimitSticks - absMaxStickDeflection) * pidRuntime.horizonLimitSticksInv, pidRuntime.horizonIgnoreSticks);

    horizonLevelStrength *= pidRuntime.horizonGain;

    if (pidRuntime.horizonDelayMs) {
        const float horizonLevelStrengthSmoothed = pt1FilterApply(&pidRuntime.horizonSmoothingPt1, horizonLevelStrength);
        horizonLevelStrength = MIN(horizonLevelStrength, horizonLevelStrengthSmoothed);
    }
    return horizonLevelStrength;

}

/* 
 * currentPidSetpoint: 摇杆的输入
 * angleTrim: 角度微调值，单位为 deg
 * horizonLevelStrength: Horizon 模式下的自稳混合权重，范围为 0.0 ~ 1.0
 */
static float pidLevel(int axis, const pidProfile_t *pidProfile, const pidInput_t *input,
                                                        float currentPidSetpoint, float horizonLevelStrength)
{

    const float angleLimit = pidProfile->angle_limit;
    float angleFeedforward = 0.0f;

    const float maxSetpointRateInv = 1.0f / input->max_rc_rate_deg_s[axis];

    angleFeedforward = angleLimit * input->feedforward[axis] * pidRuntime.angleFeedforwardGain * maxSetpointRateInv;

    angleFeedforward = pt3FilterApply(&pidRuntime.angleFeedforwardPt3[axis], angleFeedforward);

 	/* 归一化角度 */
    float angleTarget = angleLimit * currentPidSetpoint * maxSetpointRateInv;

	/* 当前欧拉角 */
    const float currentAngle = (input->attitude_deg[axis] - input->angle_trim_deg[axis]); // stepped at 500hz with some 4ms flat spots
    const float errorAngle = angleTarget - currentAngle;
	/* 目标角速度 */
    float angleRate = errorAngle * pidRuntime.angleGain + angleFeedforward;

    float sinAngle = sinf(DEGREES_TO_RADIANS(pidRuntime.angleTarget[axis == FD_ROLL ? FD_PITCH : FD_ROLL]));
    sinAngle *= (axis == FD_ROLL) ? -1.0f : 1.0f; // must be negative for Roll
    const float earthRefGain = pidRuntime.angleEarthRef;
    angleRate += pidRuntime.angleYawSetpoint * sinAngle * earthRefGain;
    pidRuntime.angleTarget[axis] = angleTarget;  // set target for alternate axis to current axis, for use in preceding calculation

    angleRate = pt3FilterApply(&pidRuntime.attitudeFilter[axis], angleRate);

    if ((input->mode == PID_MODE_ANGLE)) {
        currentPidSetpoint = angleRate;
    } else {

        currentPidSetpoint = currentPidSetpoint * (1.0f - horizonLevelStrength) + angleRate * horizonLevelStrength;
    }

    return currentPidSetpoint;
}

static float accelerationLimit(int axis, float currentPidSetpoint)
{
    const float currentVelocity = currentPidSetpoint - history.previousSetpoint[axis];

    if (fabsf(currentVelocity) > pidRuntime.maxVelocity[axis]) {
        currentPidSetpoint = (currentVelocity > 0) ? history.previousSetpoint[axis] + pidRuntime.maxVelocity[axis] : history.previousSetpoint[axis] - pidRuntime.maxVelocity[axis];
    }

    history.previousSetpoint[axis] = currentPidSetpoint;
    return currentPidSetpoint;
}

static void rotateVector(float v[XYZ_AXIS_COUNT], float rotation[XYZ_AXIS_COUNT])
{

    for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
        int i_1 = (i + 1) % 3;
        int i_2 = (i + 2) % 3;
        float newV = v[i_1] + v[i_2] * rotation[i];
        v[i_2] -= v[i_1] * rotation[i];
        v[i_1] = newV;
    }
}

static void rotateItermAndAxisError(const pidInput_t *input)
{
    if (pidRuntime.itermRotation
        ) {
        const float gyroToAngle = pidRuntime.dT * RAD;
        float rotationRads[XYZ_AXIS_COUNT];
        for (int i = FD_ROLL; i <= FD_YAW; i++) {
            rotationRads[i] = input->gyro_deg_s[i] * gyroToAngle;
        }
        if (pidRuntime.itermRotation) {
            float v[XYZ_AXIS_COUNT];
            for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
                v[i] = pidData[i].I;
            }
            rotateVector(v, rotationRads );
            for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
                pidData[i].I = v[i];
            }
        }
    }
}


static void applyItermRelax(const pidInput_t *input, const int axis, const float iterm,
    const float gyroRate, float *itermErrorRate, float *currentPidSetpoint)
{
    const float setpointLpf = pt1FilterApply(&pidRuntime.windupLpf[axis], *currentPidSetpoint);
    const float setpointHpf = fabsf(*currentPidSetpoint - setpointLpf);

    if (pidRuntime.itermRelax) {
        if (axis < FD_YAW || pidRuntime.itermRelax == ITERM_RELAX_RPY || pidRuntime.itermRelax == ITERM_RELAX_RPY_INC) {
            float itermRelaxThreshold = ITERM_RELAX_SETPOINT_THRESHOLD;
            if ((input->mode == PID_MODE_ANGLE)) {
                itermRelaxThreshold *= 0.2f;
            }
            const float itermRelaxFactor = MAX(0, 1 - setpointHpf / itermRelaxThreshold);
            const bool isDecreasingI =
                ((iterm > 0) && (*itermErrorRate < 0)) || ((iterm < 0) && (*itermErrorRate > 0));
            if ((pidRuntime.itermRelax >= ITERM_RELAX_RP_INC) && isDecreasingI) {

            } else if (pidRuntime.itermRelaxType == ITERM_RELAX_SETPOINT) {
                *itermErrorRate *= itermRelaxFactor;
            } else if (pidRuntime.itermRelaxType == ITERM_RELAX_GYRO ) {
                *itermErrorRate = fapplyDeadband(setpointLpf - gyroRate, setpointHpf);
            } else {
                *itermErrorRate = 0.0f;
            }

        }

    }
}

 /* 控制线程中调用的PID计算函数 */
void pidController(const pidProfile_t *pidProfile, const pidInput_t *input)
{
	/*  */
    if (!pidRuntime.initialized || pidProfile == NULL || input == NULL) {
        pidResetState();
        return;
    }

    /* isfinite 判断有限浮点数 */
    bool valid = isfinite(input->throttle) && isfinite(input->motor_mix_range) &&
        input->motor_mix_range >= 0.0f && (uint8_t)input->mode <= PID_MODE_HORIZON;

    /* 三轴角速度/姿态角/目标角速度 */
    for (uint8_t axis = 0; axis < 3; ++axis) {
        valid = valid && isfinite(input->gyro_deg_s[axis]) &&
            isfinite(input->attitude_deg[axis]) && isfinite(input->rate_setpoint_deg_s[axis]) &&
            input->max_rc_rate_deg_s[axis] > 0.0f;
    }
    if (!valid) 
    {
        pidResetState();
        return;
    }

    /* 油门PID增益 */
    pidUpdateTpaFactor(input->throttle);

    pidUpdateAntiGravityThrottleFilter(constrainf(input->throttle, 0.0f, 1.0f));
    dynLpfDTermUpdate(constrainf(input->throttle, 0.0f, 1.0f));
    /* 原移植丢失的积分抗饱和因子，输入来自上一拍 mixer。 */
    const float dynCi = constrainf((1.0f - input->motor_mix_range) *
                                  pidRuntime.itermWindupPointInv, 0.0f, 1.0f);

    const float tpaFactorKp = (pidProfile->tpa_mode == TPA_MODE_PD) ? pidRuntime.tpaFactor : 1.0f;

/* 加速度计相关 */
    float horizonLevelStrength = 0.0f;

    levelMode_e levelMode;
    if ((input->mode == PID_MODE_ANGLE) || (input->mode == PID_MODE_HORIZON)) {
        if (pidRuntime.levelRaceMode) {
            levelMode = LEVEL_MODE_R;
        } else {
            levelMode = LEVEL_MODE_RP;
        }

        if ((input->mode == PID_MODE_HORIZON)) {
            horizonLevelStrength = calcHorizonLevelStrength(input);
        }
    } else {
        levelMode = LEVEL_MODE_OFF;
    }

    if (pidRuntime.antiGravityEnabled) {
        pidRuntime.antiGravityThrottleD *= pidRuntime.antiGravityGain;

        pidRuntime.itermAccelerator = pidRuntime.antiGravityThrottleD * ANTIGRAVITY_KI;
    } else {
        pidRuntime.antiGravityThrottleD = 0.0f;
        pidRuntime.itermAccelerator = 0.0f;
    }

    float gyroRateDterm[XYZ_AXIS_COUNT];
    for (int axis = FD_ROLL; axis <= FD_YAW; ++axis) {
        gyroRateDterm[axis] = input->gyro_deg_s[axis];

        gyroRateDterm[axis] = pidRuntime.dtermLowpassApplyFn((filter_t *) &pidRuntime.dtermLowpass[axis], gyroRateDterm[axis]);
    }

    if (!history.dtermPrimed) {
        memcpy(history.previousGyroRateDterm, gyroRateDterm, sizeof(gyroRateDterm));
        history.dtermPrimed = true;
    }
    rotateItermAndAxisError(input);

    for (int axis = FD_ROLL; axis <= FD_YAW; ++axis) {

        float currentPidSetpoint = input->rate_setpoint_deg_s[axis];
        if (pidRuntime.maxVelocity[axis]) {
            currentPidSetpoint = accelerationLimit(axis, currentPidSetpoint);
        }

        pidRuntime.axisInAngleMode[axis] = false;
        if (axis < FD_YAW) {
            if (levelMode == LEVEL_MODE_RP || (levelMode == LEVEL_MODE_R && axis == FD_ROLL)) {
                pidRuntime.axisInAngleMode[axis] = true;
                currentPidSetpoint = pidLevel(axis, pidProfile, input, currentPidSetpoint, horizonLevelStrength);
            }
        } else { // yaw axis only
            if (levelMode == LEVEL_MODE_RP) {

                const float earthRefGain = pidRuntime.angleEarthRef;
                if (earthRefGain) {
                    pidRuntime.angleYawSetpoint = currentPidSetpoint;
                    float maxAngleTargetAbs = earthRefGain * fmaxf( fabsf(pidRuntime.angleTarget[FD_ROLL]), fabsf(pidRuntime.angleTarget[FD_PITCH]) );
                    maxAngleTargetAbs *= ((input->mode == PID_MODE_HORIZON)) ? horizonLevelStrength : 1.0f;

                    currentPidSetpoint *= cosf(DEGREES_TO_RADIANS(maxAngleTargetAbs));
                }
            }
        }

        const float gyroRate = input->gyro_deg_s[axis]; // Process variable from gyro output in deg/sec
        float errorRate = currentPidSetpoint - gyroRate; // r - y

        const float previousIterm = pidData[axis].I;
        float itermErrorRate = errorRate;

        {
            applyItermRelax(input, axis, previousIterm, gyroRate, &itermErrorRate, &currentPidSetpoint);
            errorRate = currentPidSetpoint - gyroRate;
        }

        pidData[axis].P = pidRuntime.pidCoefficient[axis].Kp * errorRate * tpaFactorKp;
        if (axis == FD_YAW) {
            pidData[axis].P = pidRuntime.ptermYawLowpassApplyFn((filter_t *) &pidRuntime.ptermYawLowpass, pidData[axis].P);
        }

        float Ki = pidRuntime.pidCoefficient[axis].Ki;
        {
            if (axis == FD_YAW) {
                pidRuntime.itermAccelerator = 0.0f; // no antigravity on yaw iTerm
            }
        }
        const float iTermChange = (Ki + pidRuntime.itermAccelerator) * dynCi * pidRuntime.dT * itermErrorRate;
        pidData[axis].I = constrainf(previousIterm + iTermChange, -pidRuntime.itermLimit, pidRuntime.itermLimit);

        float pidSetpointDelta = 0;

        if ((input->mode == PID_MODE_ANGLE) && pidRuntime.axisInAngleMode[axis]) {

            pidSetpointDelta = 0.0f;
        } else {

            pidSetpointDelta = input->feedforward[axis];
        }
        pidRuntime.previousPidSetpoint[axis] = currentPidSetpoint; // this is the value sent to blackbox, and used for Dmin setpoint

        if ((pidRuntime.pidCoefficient[axis].Kd > 0)) {

            const float delta = - (gyroRateDterm[axis] - history.previousGyroRateDterm[axis]) * pidRuntime.pidFrequency;
            float preTpaD = pidRuntime.pidCoefficient[axis].Kd * delta;

            float dMinFactor = 1.0f;
            if (pidRuntime.dMinPercent[axis] > 0) {
                float dMinGyroFactor = pt2FilterApply(&pidRuntime.dMinRange[axis], delta);
                dMinGyroFactor = fabsf(dMinGyroFactor) * pidRuntime.dMinGyroGain;
                const float dMinSetpointFactor = (fabsf(pidSetpointDelta)) * pidRuntime.dMinSetpointGain;
                dMinFactor = MAX(dMinGyroFactor, dMinSetpointFactor);
                dMinFactor = pidRuntime.dMinPercent[axis] + (1.0f - pidRuntime.dMinPercent[axis]) * dMinFactor;
                dMinFactor = pt2FilterApply(&pidRuntime.dMinLowpass[axis], dMinFactor);
                dMinFactor = MIN(dMinFactor, 1.0f);
            }

            preTpaD *= dMinFactor;
            pidData[axis].D = preTpaD * pidRuntime.tpaFactor;

        } else {
            pidData[axis].D = 0;
        }

        history.previousGyroRateDterm[axis] = gyroRateDterm[axis];

        const float feedforwardGain = pidRuntime.pidCoefficient[axis].Kf;
        pidData[axis].F = feedforwardGain * pidSetpointDelta;

        if (axis != FD_YAW) {
            float agSetpointAttenuator = fabsf(currentPidSetpoint) / 50.0f;
            agSetpointAttenuator = MAX(agSetpointAttenuator, 1.0f);

            const float antiGravityPBoost = 1.0f + (pidRuntime.antiGravityThrottleD / agSetpointAttenuator) * pidRuntime.antiGravityPGain;
            pidData[axis].P *= antiGravityPBoost;
            if (axis == FD_PITCH) {
            }
        }

        const float pidSum = pidData[axis].P + pidData[axis].I + pidData[axis].D + pidData[axis].F;
        {
            pidData[axis].Sum = pidSum;
        }
    }

    if (!pidRuntime.pidStabilisationEnabled || input->gyro_overflow) {
        for (int axis = FD_ROLL; axis <= FD_YAW; ++axis) {
            pidData[axis].P = 0;
            pidData[axis].I = 0;
            pidData[axis].D = 0;
            pidData[axis].F = 0;

            pidData[axis].Sum = 0;
        }
    } else if (pidRuntime.zeroThrottleItermReset) {
        pidResetIterm();
    }
}

void pidSetAntiGravityState(bool newState)
{
    if (newState != pidRuntime.antiGravityEnabled) {

        pidRuntime.itermAccelerator = 0.0f;
    }
    pidRuntime.antiGravityEnabled = newState;
}

bool pidAntiGravityEnabled(void)
{
    return pidRuntime.antiGravityEnabled;
}

void dynLpfDTermUpdate(float throttle)
{
    if (pidRuntime.dynLpfFilter != DYN_LPF_NONE) {
        float cutoffFreq;
        if (pidRuntime.dynLpfCurveExpo > 0) {
            cutoffFreq = dynLpfCutoffFreq(throttle, pidRuntime.dynLpfMin, pidRuntime.dynLpfMax, pidRuntime.dynLpfCurveExpo);
        } else {
            cutoffFreq = fmaxf(throttle * pidRuntime.dynLpfMax, pidRuntime.dynLpfMin);
        }

        switch (pidRuntime.dynLpfFilter) {
        case DYN_LPF_PT1:
            for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
                pt1FilterUpdateCutoff(&pidRuntime.dtermLowpass[axis].pt1Filter, pt1FilterGain(cutoffFreq, pidRuntime.dT));
            }
            break;
        case DYN_LPF_PT2:
            for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
                pt2FilterUpdateCutoff(&pidRuntime.dtermLowpass[axis].pt2Filter, pt2FilterGain(cutoffFreq, pidRuntime.dT));
            }
            break;
        case DYN_LPF_PT3:
            for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
                pt3FilterUpdateCutoff(&pidRuntime.dtermLowpass[axis].pt3Filter, pt3FilterGain(cutoffFreq, pidRuntime.dT));
            }
            break;
        }
    }
}

float dynLpfCutoffFreq(float throttle, uint16_t dynLpfMin, uint16_t dynLpfMax, uint8_t expo)
{
    const float expof = expo / 10.0f;
    const float curve = throttle * (1 - throttle) * expof + throttle;
    return (dynLpfMax - dynLpfMin) * curve + dynLpfMin;
}

void pidSetItermReset(bool enabled)
{
    pidRuntime.zeroThrottleItermReset = enabled;
}

float pidGetPreviousSetpoint(int axis)
{
    return pidRuntime.previousPidSetpoint[axis];
}

float pidGetDT(void)
{
    return pidRuntime.dT;
}

float pidGetPidFrequency(void)
{
    return pidRuntime.pidFrequency;
}
