# FyPilot PID 适配

`pid.c` / `pid_init.c` 保留原有的角度环、角速度 P/I/D/F、TPA、Anti-Gravity、
I-term Relax、积分旋转、D-min 及 PT1/PT2/PT3/biquad 滤波计算。
这些算法均在本模块编译，通过配置字段及输入模式控制，不再依赖 Betaflight 的全局功能宏。

## 依赖边界

- `PG_*` 参数注册替换为静态 RAM 配置；`pidResetProfiles()` 填默认值，
  `pidProfilesMutable()` 修改，`pidCopyProfile()` 复制。配置保存由应用实现。
- `gyro`、`attitude`、RC、飞行模式和混控状态统一由 `pidInput_t` 传入。
- 控制周期由 `pidInit(profile, looptime_us)` 显式指定，不能与线程实际周期不一致。
- DShot、设备驱动和消息总线不属于 PID 模块。PID 不直接读设备或发布电机命令。
- `common/filter.c` 继续使用原有滤波公式；缺失的 BF 数学辅助依赖改为
  `common/filter_math.h` 和标准 C `sinf/cosf`。

## 最小使用示例

```c
#include "pid_init.h"

/* 初始化阶段：仅单个控制线程拥有 PID 实例。 */
pidResetProfiles();
pidProfile_t *profile = pidProfilesMutable(0);
if (!pidInit(profile, 4000)) {
    /* 初始化失败处理 */
}

/* 每 4000 us 运行一次，input 必须由应用填入真实数据。 */
pidInput_t input = {0};
input.mode = PID_MODE_RATE;
for (int i = 0; i < 3; ++i) {
    input.max_rc_rate_deg_s[i] = 600.0f; /* 示例值，按遥控目标映射配置 */
    input.gyro_deg_s[i] = measured_gyro_rad_s[i] * 57.29577951f;
    input.rate_setpoint_deg_s[i] = desired_rate_rad_s[i] * 57.29577951f;
    input.attitude_deg[i] = measured_angle_rad[i] * 57.29577951f;
}
input.throttle = throttle;                     /* [0,1] */
input.motor_mix_range = previous_motor_range;  /* 来自上一拍混控，不能一直伪造为 0 */
pidStabilisationState(armed_and_inputs_valid ?
                     PID_STABILISATION_ON : PID_STABILISATION_OFF);
pidController(profile, &input);

/* 三轴输出；是否归一化/限幅，由下一层混控接口约定决定。 */
float roll_command = pidData[FD_ROLL].Sum;
```

## 单位与模式

- PID 内部延续 BF 的 **deg / deg/s**。uMCN 若使用 rad / rad/s，调用前必须转换。
- `PID_MODE_RATE`：`rate_setpoint_deg_s[]` 直接为三轴角速度目标。
- `PID_MODE_ANGLE`：roll/pitch 目标角度为
  `angle_limit * rate_setpoint_deg_s[axis] / max_rc_rate_deg_s[axis]`，yaw 仍是角速度。
- `PID_MODE_HORIZON`：保留原自稳与角速度混合逻辑，需提供真实 `rc_deflection[2]`。
- `angle_trim_deg[2]` 为角度微调，未配置时为 0。
- `feedforward[3]` 是外部 RC 处理后的 BF 风格前馈量，并非直接把摇杆值放进去。
  前馈平滑/插值模块没有随本次移植；没有有效前馈输入时填 0。
- `motor_mix_range` 是混控姿态控制分量的占用范围，和单纯的油门大小不同。
  通过 `clamp((1-range)*itermWindupPointInv, 0, 1)` 计算原代码缺失的 `dynCi`。
- 输出仍是 BF 缩放的控制量，不能直接视为 N·m 或 DShot 油门。

`pidInit()` 默认关闭稳定控制，需由应用根据状态显式启用。
`pidSetAntiGravityState()` 控制 Anti-Gravity；每拍 TPA、油门变化滤波与动态 D 低通
更新已由 `pidController()` 调用，无需再重复执行。

`pidResetIterm()` 仅清积分；`pidResetState()` 清输出、历史和滤波器状态，保留配置及使能状态。
更改周期或配置后应重新 `pidInit()`，同时重新设置需要的使能状态。
原文件中未实现的 RPM、自动电池选配置和简化调参相关字段仅保留为配置元数据。

## 验证

```sh
bash tools/tests/run_pid_port_test.sh
cd bsp/stm32h743
scons --cdb -j8
```

主机测试覆盖 P/I/D/F 数值、积分限幅与混控饱和、角度/Horizon 模式、各类滤波器、
状态复位、配置复制和无效输入，并启用 AddressSanitizer/UndefinedBehaviorSanitizer。
主机测试不替代目标板上的执行耗时、传感器方向及实机调参验证。

当前控制线程中的算法调用仍是模板；本次只完成 PID 依赖适配与编译接入。
