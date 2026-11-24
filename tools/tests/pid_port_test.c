/* Host regression tests for the standalone BF-derived PID port. */
#include "pid_init.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static pidProfile_t profile;
static pidInput_t input;

static void near_value(float actual, float expected, float tolerance)
{
    assert(isfinite(actual));
    if (fabsf(actual - expected) > tolerance) {
        fprintf(stderr, "actual=%g expected=%g tolerance=%g\n", actual, expected, tolerance);
        assert(0);
    }
}

static void setup(void)
{
    resetPidProfile(&profile);
    memset(&input, 0, sizeof(input));
    for (int i = 0; i < 3; ++i) {
        memset(&profile.pid[i], 0, sizeof(profile.pid[i]));
        profile.d_min[i] = 0;
        input.max_rc_rate_deg_s[i] = 600.0f;
    }
    profile.dterm_lpf1_static_hz = profile.dterm_lpf1_dyn_min_hz = 0;
    profile.dterm_lpf2_static_hz = profile.yaw_lowpass_hz = 0;
    profile.iterm_relax = ITERM_RELAX_OFF;
    profile.tpa_rate = profile.tpa_low_rate = 0;
    profile.angle_earth_ref = 0;
}

static void start(void)
{
    assert(pidInit(&profile, 4000));
    pidStabilisationState(PID_STABILISATION_ON);
}

static void terms_and_saturation(void)
{
    setup();
    profile.pid[PID_ROLL].P = 10;
    start();
    input.rate_setpoint_deg_s[0] = 30;
    input.gyro_deg_s[0] = 10;
    pidController(&profile, &input);
    near_value(pidData[0].Sum, 6.4058f, 0.0001f);
    pidStabilisationState(PID_STABILISATION_OFF);
    pidController(&profile, &input);
    near_value(pidData[0].Sum, 0, 0);

    setup();
    profile.pid[PID_ROLL].I = 2;
    profile.itermLimit = 1;
    start();
    input.rate_setpoint_deg_s[0] = 10;
    pidController(&profile, &input);
    near_value(pidData[0].I, 0.01955048f, 0.000001f);
    input.motor_mix_range = 1.1f;
    for (int i = 0; i < 20; ++i) pidController(&profile, &input);
    near_value(pidData[0].I, 0.01955048f, 0.000001f);
    input.motor_mix_range = 0;
    for (int i = 0; i < 200; ++i) pidController(&profile, &input);
    near_value(pidData[0].I, 1, 0.000001f);
    pidResetIterm();
    near_value(pidData[0].I, 0, 0);

    setup();
    profile.pid[PID_ROLL].D = 1;
    start();
    input.gyro_deg_s[0] = 5;
    pidController(&profile, &input);
    near_value(pidData[0].D, 0, 0); /* Prime D history, no startup impulse. */
    input.gyro_deg_s[0] = 15;
    pidController(&profile, &input);
    near_value(pidData[0].D, -1.3225f, 0.00001f);

    setup();
    profile.pid[PID_ROLL].F = 100;
    start();
    input.feedforward[0] = 20;
    pidController(&profile, &input);
    near_value(pidData[0].F, 0.27508f, 0.00001f);
}

static void level_and_filters(void)
{
    setup();
    profile.pid[PID_ROLL].P = 10;
    start();
    input.mode = PID_MODE_ANGLE;
    input.rate_setpoint_deg_s[0] = 300; /* half stick -> 30 degrees */
    input.attitude_deg[0] = 10;
    for (int i = 0; i < 1000; ++i) pidController(&profile, &input);
    near_value(pidGetPreviousSetpoint(0), 100, 0.001f); /* (30-10)*5 */

    input.mode = PID_MODE_HORIZON;
    input.rate_setpoint_deg_s[0] = input.attitude_deg[0] = 0;
    for (int i = 0; i < 1000; ++i) pidController(&profile, &input);
    near_value(pidGetPreviousSetpoint(0), 0, 0.001f);

    for (int type = FILTER_PT1; type <= FILTER_PT3; ++type) {
        resetPidProfile(&profile);
        profile.dterm_lpf1_type = profile.dterm_lpf2_type = (uint8_t)type;
        profile.dterm_lpf1_dyn_min_hz = 30;
        profile.dterm_lpf1_dyn_max_hz = 80;
        profile.dterm_lpf2_static_hz = 60;
        profile.dterm_notch_hz = 90;
        profile.dterm_notch_cutoff = 70;
        profile.iterm_rotation = 1;
        start();
        pidSetAntiGravityState(true);
        for (int j = 0; j < 2000; ++j) {
            input.mode = (pidMode_t)(j % 3);
            input.throttle = (j % 100) / 100.0f;
            for (int a = 0; a < 3; ++a) {
                input.gyro_deg_s[a] = 30 * sinf((float)j * 0.04f + a);
                input.rate_setpoint_deg_s[a] = 50 * sinf((float)j * 0.02f + a);
            }
            pidController(&profile, &input);
            for (int a = 0; a < 3; ++a) assert(isfinite(pidData[a].Sum));
        }
        /* Reset history + filter states must reproduce cold-start behavior. */
        pidResetState();
        pidController(&profile, &input);
        float first = pidData[0].Sum;
        start();
        pidSetAntiGravityState(true);
        pidController(&profile, &input);
        near_value(pidData[0].Sum, first, 0.0001f);
    }
    /* Invalid LPF2 biquad frequency must fall back to a callable passthrough. */
    setup();
    profile.dterm_lpf2_type = FILTER_BIQUAD;
    profile.dterm_lpf2_static_hz = 1000;
    profile.dterm_notch_hz = profile.dterm_notch_cutoff = 80;
    start();
    pidController(&profile, &input);
    assert(isfinite(pidData[0].Sum));
}

static void initialization_and_invalid_input(void)
{
    pidResetProfiles();
    assert(pidProfilesMutable(PID_PROFILE_COUNT) == NULL);
    assert(pidProfiles(PID_PROFILE_COUNT) == NULL);
    assert(pidProfiles(0)->pid[PID_ROLL].P == 45);
    pidProfilesMutable(0)->pid[PID_ROLL].P = 12;
    pidCopyProfile(1, 0);
    assert(pidProfiles(1)->pid[PID_ROLL].P == 12);
    setup();
    profile.angle_feedforward_smoothing_ms = profile.feedforward_jitter_factor = 0;
    start();
    near_value(pidGetDT(), 0.004f, 0.0000001f);
    near_value(pidGetPidFrequency(), 250, 0.001f);
    input.gyro_deg_s[0] = NAN;
    pidController(&profile, &input);
    near_value(pidData[0].Sum, 0, 0);
    input.gyro_deg_s[0] = 0;
    input.max_rc_rate_deg_s[0] = 0;
    pidController(&profile, &input);
    near_value(pidData[0].Sum, 0, 0);
    assert(!pidInit(&profile, 0));
    assert(!pidRuntime.initialized);
    assert(!pidInit(NULL, 4000));
}

int main(void)
{
    initialization_and_invalid_input();
    terms_and_saturation();
    level_and_filters();
    puts("PASS: PID terms, saturation, angle/horizon, filters, reset, profiles, invalid inputs");
    return 0;
}
