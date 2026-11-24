#ifndef FYPILOT_FILTER_MATH_H
#define FYPILOT_FILTER_MATH_H
#include <math.h>
#define M_PIf 3.14159265358979323846f
#define RAD (M_PIf / 180.0f)
#define DEGREES_TO_RADIANS(x) ((x) * RAD)
#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif
#ifndef MAX
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#endif
static inline float constrainf(float value, float low, float high)
{
    return fminf(fmaxf(value, low), high);
}
static inline float fapplyDeadband(float value, float deadband)
{
    return fabsf(value) <= deadband ? 0.0f :
        (value > 0.0f ? value - deadband : value + deadband);
}
static inline float sq(float value) { return value * value; }
#endif
