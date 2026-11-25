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

#include <string.h>
#include <math.h>
#include "pid_init.h"
#include "common/filter_math.h"

static float pt1FilterApplyAdapter(filter_t *filter, float input)
{
    return pt1FilterApply((pt1Filter_t *)filter, input);
}

static float pt2FilterApplyAdapter(filter_t *filter, float input)
{
    return pt2FilterApply((pt2Filter_t *)filter, input);
}

static float pt3FilterApplyAdapter(filter_t *filter, float input)
{
    return pt3FilterApply((pt3Filter_t *)filter, input);
}

#define D_MIN_RANGE_HZ 85    // PT2 lowpass input cutoff to peak D around propwash frequencies
#define D_MIN_LOWPASS_HZ 35  // PT2 lowpass cutoff to smooth the boost effect
#define D_MIN_GAIN_FACTOR 0.00008f
#define D_MIN_SETPOINT_GAIN_FACTOR 0.00008f

#define ATTITUDE_CUTOFF_HZ 50

static void pidSetTargetLooptime(uint32_t pidLooptime)
{
    targetPidLooptime = pidLooptime;
    pidRuntime.dT = targetPidLooptime * 1e-6f;
    pidRuntime.pidFrequency = 1.0f / pidRuntime.dT;
}

/* D 项低通；三轴分别保存状态，类型可选 PT1/PT2/PT3。 */
static void pidInitDtermLowpass(const pidProfile_t *pidProfile)
{
    pidRuntime.dtermLowpassApplyFn = nullFilterApply;
    uint16_t dterm_lpf1_init_hz = pidProfile->dterm_lpf1_static_hz;
    
    if (dterm_lpf1_init_hz > 0) 
    {
        switch (pidProfile->dterm_lpf1_type) 
        {
        case FILTER_PT1:
            pidRuntime.dtermLowpassApplyFn = pt1FilterApplyAdapter;
            for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
                pt1FilterInit(&pidRuntime.dtermLowpass[axis].pt1Filter, pt1FilterGain(dterm_lpf1_init_hz, pidRuntime.dT));
            }
            break;
        case FILTER_PT2:
            pidRuntime.dtermLowpassApplyFn = pt2FilterApplyAdapter;
            for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
                pt2FilterInit(&pidRuntime.dtermLowpass[axis].pt2Filter, pt2FilterGain(dterm_lpf1_init_hz, pidRuntime.dT));
            }
            break;
        case FILTER_PT3:
            pidRuntime.dtermLowpassApplyFn = pt3FilterApplyAdapter;
            for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
                pt3FilterInit(&pidRuntime.dtermLowpass[axis].pt3Filter, pt3FilterGain(dterm_lpf1_init_hz, pidRuntime.dT));
            }
            break;
        default:
            pidRuntime.dtermLowpassApplyFn = nullFilterApply;
            break;
        }
    } else {
        pidRuntime.dtermLowpassApplyFn = nullFilterApply;
    }

}

void pidInitFilters(const pidProfile_t *pidProfile)
{
    pidRuntime.dtermLowpassApplyFn = nullFilterApply;
    pidRuntime.ptermYawLowpassApplyFn = nullFilterApply;
    if (pidProfile == NULL || targetPidLooptime == 0 || pidRuntime.dT <= 0.0f) {
        return;
    }
    pidInitDtermLowpass(pidProfile);

    if (pidProfile->yaw_lowpass_hz == 0) {
        pidRuntime.ptermYawLowpassApplyFn = nullFilterApply;
    } else {
        pidRuntime.ptermYawLowpassApplyFn = pt1FilterApplyAdapter;
        pt1FilterInit(&pidRuntime.ptermYawLowpass, pt1FilterGain(pidProfile->yaw_lowpass_hz, pidRuntime.dT));
    }

    if (pidRuntime.itermRelax) {
        for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
            pt1FilterInit(&pidRuntime.windupLpf[i], pt1FilterGain(pidRuntime.itermRelaxCutoff, pidRuntime.dT));
        }
    }

    // Initialize the filters for all axis even if the d_min[axis] value is 0
    // Otherwise if the pidProfile->d_min_xxx parameters are ever added to
    // in-flight adjustments and transition from 0 to > 0 in flight the feature
    // won't work because the filter wasn't initialized.
    for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
        pt2FilterInit(&pidRuntime.dMinRange[axis], pt2FilterGain(D_MIN_RANGE_HZ, pidRuntime.dT));
        pt2FilterInit(&pidRuntime.dMinLowpass[axis], pt2FilterGain(D_MIN_LOWPASS_HZ, pidRuntime.dT));
     }

    const float k = pt3FilterGain(ATTITUDE_CUTOFF_HZ, pidRuntime.dT);
    const float angleCutoffHz = 1000.0f / (2.0f * M_PIf * MAX(pidProfile->angle_feedforward_smoothing_ms, 1)); // default of 80ms -> 2.0Hz, 160ms -> 1.0Hz, approximately
    const float k2 = pt3FilterGain(angleCutoffHz, pidRuntime.dT);
    pidRuntime.horizonDelayMs = pidProfile->horizon_delay_ms;
    if (pidRuntime.horizonDelayMs) {
        const float horizonSmoothingHz = 1e3f / (2.0f * M_PIf * pidProfile->horizon_delay_ms); // default of 500ms means 0.318Hz
        const float kHorizon = pt1FilterGain(horizonSmoothingHz, pidRuntime.dT);
        pt1FilterInit(&pidRuntime.horizonSmoothingPt1, kHorizon);
    }

    for (int axis = 0; axis < 2; axis++) {  // ROLL and PITCH only
        pt3FilterInit(&pidRuntime.attitudeFilter[axis], k);
        pt3FilterInit(&pidRuntime.angleFeedforwardPt3[axis], k2);
    }
    pidRuntime.angleYawSetpoint = 0.0f;

    pt2FilterInit(&pidRuntime.antiGravityLpf, pt2FilterGain(pidProfile->anti_gravity_cutoff_hz, pidRuntime.dT));
}

bool pidInit(const pidProfile_t *pidProfile, uint32_t looptime_us)
{
    if (pidProfile == NULL || looptime_us == 0 || looptime_us > 1000000U) 
    {
        pidRuntime.initialized = false;
        pidResetState();
        return false;
    }

    memset(&pidRuntime, 0, sizeof(pidRuntime));

    /* 清除 PID 的历史数据 */
    pidResetState();

    /* 设置计算周期/频率 */
    pidSetTargetLooptime(looptime_us);

    /* 初始化配置转换为运行时配置 */
    pidInitConfig(pidProfile);  

    /* 初始化滤波器系数 */
    pidInitFilters(pidProfile);

    /* 初始化油门衰减系数 */
    pidRuntime.tpaFactor = 1.0f;
    pidRuntime.initialized = true;
    return true;
}

void pidInitConfig(const pidProfile_t *pidProfile)
{
    pidRuntime.dtermLowpassType = pidProfile->dterm_lpf1_type;

    for (int axis = FD_ROLL; axis <= FD_YAW; axis++) 
    {
        pidRuntime.pidCoefficient[axis].Kp = PTERM_SCALE * pidProfile->pid[axis].P;
        pidRuntime.pidCoefficient[axis].Ki = ITERM_SCALE * pidProfile->pid[axis].I;
        pidRuntime.pidCoefficient[axis].Kd = DTERM_SCALE * pidProfile->pid[axis].D;
        pidRuntime.pidCoefficient[axis].Kf = FEEDFORWARD_SCALE * (pidProfile->pid[axis].F * 0.01f);
    }

    pidRuntime.angleGain = pidProfile->pid[PID_LEVEL].P / 10.0f;
    pidRuntime.angleFeedforwardGain = pidProfile->pid[PID_LEVEL].F / 100.0f;
    pidRuntime.angleEarthRef = pidProfile->angle_earth_ref / 100.0f;

    pidRuntime.horizonGain = MIN(pidProfile->pid[PID_LEVEL].I / 100.0f, 1.0f);
    pidRuntime.horizonIgnoreSticks = (pidProfile->horizon_ignore_sticks) ? 1.0f : 0.0f;

    pidRuntime.horizonLimitSticks = pidProfile->pid[PID_LEVEL].D / 100.0f;
    pidRuntime.horizonLimitSticksInv = (pidProfile->pid[PID_LEVEL].D) ? 1.0f / pidRuntime.horizonLimitSticks : 1.0f;
    pidRuntime.horizonLimitDegrees = (float)pidProfile->horizon_limit_degrees;
    pidRuntime.horizonLimitDegreesInv = (pidProfile->horizon_limit_degrees) ? 1.0f / pidRuntime.horizonLimitDegrees : 1.0f;
    pidRuntime.horizonDelayMs = pidProfile->horizon_delay_ms;

    pidRuntime.maxVelocity[FD_ROLL] = pidRuntime.maxVelocity[FD_PITCH] = pidProfile->rateAccelLimit * 100 * pidRuntime.dT;
    pidRuntime.maxVelocity[FD_YAW] = pidProfile->yawRateAccelLimit * 100 * pidRuntime.dT;
    pidRuntime.itermWindupPointInv = 1.0f;
    if (pidProfile->itermWindupPointPercent < 100) {
        const float itermWindupPoint = pidProfile->itermWindupPointPercent / 100.0f;
        pidRuntime.itermWindupPointInv = 1.0f / (1.0f - itermWindupPoint);
    }
    pidRuntime.antiGravityGain = pidProfile->anti_gravity_gain;

    pidRuntime.itermLimit = pidProfile->itermLimit;
    pidRuntime.itermRotation = pidProfile->iterm_rotation;

    // Calculate the anti-gravity value that will trigger the OSD display when its strength exceeds 25% of max.
    // This gives a useful indication of AG activity without excessive display.
    pidRuntime.antiGravityOsdCutoff = (pidRuntime.antiGravityGain / 10.0f) * 0.25f;
    pidRuntime.antiGravityPGain = ((float)(pidProfile->anti_gravity_p_gain) / 100.0f) * ANTIGRAVITY_KP;

    pidRuntime.itermRelax = pidProfile->iterm_relax;
    pidRuntime.itermRelaxType = pidProfile->iterm_relax_type;
    pidRuntime.itermRelaxCutoff = pidProfile->iterm_relax_cutoff;

    if (pidProfile->dterm_lpf1_dyn_min_hz > 0) {
        switch (pidProfile->dterm_lpf1_type) {
        case FILTER_PT1:
            pidRuntime.dynLpfFilter = DYN_LPF_PT1;
            break;
        case FILTER_PT2:
            pidRuntime.dynLpfFilter = DYN_LPF_PT2;
            break;
        case FILTER_PT3:
            pidRuntime.dynLpfFilter = DYN_LPF_PT3;
            break;
        default:
            pidRuntime.dynLpfFilter = DYN_LPF_NONE;
            break;
        }
    } else {
        pidRuntime.dynLpfFilter = DYN_LPF_NONE;
    }
    pidRuntime.dynLpfMin = pidProfile->dterm_lpf1_dyn_min_hz;
    pidRuntime.dynLpfMax = pidProfile->dterm_lpf1_dyn_max_hz;
    pidRuntime.dynLpfCurveExpo = pidProfile->dterm_lpf1_dyn_expo;

    for (int axis = FD_ROLL; axis <= FD_YAW; ++axis) {
        const uint8_t dMin = pidProfile->d_min[axis];
        if ((dMin > 0) && (dMin < pidProfile->pid[axis].D)) {
            pidRuntime.dMinPercent[axis] = dMin / (float)(pidProfile->pid[axis].D);
        } else {
            pidRuntime.dMinPercent[axis] = 0;
        }
    }
    pidRuntime.dMinGyroGain = D_MIN_GAIN_FACTOR * pidProfile->d_min_gain / D_MIN_LOWPASS_HZ;
    pidRuntime.dMinSetpointGain = D_MIN_SETPOINT_GAIN_FACTOR * pidProfile->d_min_gain * pidProfile->d_min_advance / 100.0f / D_MIN_LOWPASS_HZ;
    // lowpass included inversely in gain since stronger lowpass decreases peak effect

    pidRuntime.feedforwardTransition = pidProfile->feedforward_transition / 100.0f;
    pidRuntime.feedforwardTransitionInv = (pidProfile->feedforward_transition == 0) ? 0.0f : 100.0f / pidProfile->feedforward_transition;
    pidRuntime.feedforwardAveraging = pidProfile->feedforward_averaging;
    pidRuntime.feedforwardSmoothFactor = 1.0f - (0.01f * pidProfile->feedforward_smooth_factor);
    pidRuntime.feedforwardJitterFactor = pidProfile->feedforward_jitter_factor;
    pidRuntime.feedforwardJitterFactorInv = 1.0f / (2.0f * MAX(pidProfile->feedforward_jitter_factor, 1));
    // the extra division by 2 is to average the sum of the two previous rcCommandAbs values
    pidRuntime.feedforwardBoostFactor = 0.1f * pidProfile->feedforward_boost;
    pidRuntime.feedforwardMaxRateLimit = pidProfile->feedforward_max_rate_limit;

    pidRuntime.levelRaceMode = pidProfile->level_race_mode;
    pidRuntime.tpaBreakpoint = constrainf((pidProfile->tpa_breakpoint - 1000.0f) / 1000.0f, 0.0f, 0.99f);
    // default of 1350 returns 0.35. range limited to 0 to 0.99
    pidRuntime.tpaMultiplier = (pidProfile->tpa_rate / 100.0f) / (1.0f - pidRuntime.tpaBreakpoint);
    // it is assumed that tpaLowBreakpoint is always less than or equal to tpaBreakpoint
    pidRuntime.tpaLowBreakpoint = constrainf((pidProfile->tpa_low_breakpoint - 1000.0f) / 1000.0f, 0.01f, 1.0f);
    pidRuntime.tpaLowBreakpoint = MIN(pidRuntime.tpaLowBreakpoint, pidRuntime.tpaBreakpoint);
    pidRuntime.tpaLowMultiplier = pidProfile->tpa_low_rate / (100.0f * pidRuntime.tpaLowBreakpoint);
    pidRuntime.tpaLowAlways = pidProfile->tpa_low_always;
}

void pidCopyProfile(uint8_t dstPidProfileIndex, uint8_t srcPidProfileIndex)
{
    if (dstPidProfileIndex < PID_PROFILE_COUNT && srcPidProfileIndex < PID_PROFILE_COUNT
        && dstPidProfileIndex != srcPidProfileIndex) {
        memcpy(pidProfilesMutable(dstPidProfileIndex), pidProfilesMutable(srcPidProfileIndex), sizeof(pidProfile_t));
    }
}
