# FyPilot

**An open-source flight controller project built on STM32H7 and RT-Thread.**

基于 STM32H7 与 RT-Thread 的飞控开发工程，面向自主飞控算法、传感器驱动和实时控制系统的学习与开发。当前硬件目标为 **AOPAH743 / STM32H743VIH6**。

[项目源码](https://github.com/JiaVerso/FyPilot) · [问题反馈](https://github.com/JiaVerso/FyPilot/issues) · [开发分支](https://github.com/JiaVerso/FyPilot/tree/develop) · [源码许可](#许可与致谢)

## 项目状态

FyPilot 目前处于板级启动、接收机通信和传感器适配阶段。默认固件运行 **Hello World + CRSF 接收 + FinSH 控制台**；另一配置用于 BMI270 采样和数学接口自检。

| 模块 | 当前状态 |
| --- | --- |
| STM32H743 启动、时钟、RT-Thread 调度 | 已实现，可交叉编译 |
| USART3 控制台与 Hello World | 默认启用，115200 8N1 |
| USART6 / ELRS-CRSF 接收 | 默认启用，420000 8N1，16 路通道与链路统计 |
| BMI270 / SPI4 / 数据就绪中断 | IMU 调试配置启用，使用 Bosch API 和 RT-Thread SPI 驱动 |
| 数学、矩阵、环形缓冲区 | 已接入调试配置，提供宿主机检查 |
| 四元数 EKF、PID、滤波、任务与消息模块 | 源码已归位，完整控制链尚未接通 |
| DPS310、IST8310、MAX7456、SD 卡、ADC、USB | 已记录硬件引脚，尚未完成硬件接入 |
| 电机 PWM / DShot / mixer | 尚未启用；10 路电机引脚保持下拉输入 |

目前完成的是编译及软件检查，**尚未在目标板验证启动、接收机通信或飞行**。源码存在不代表对应功能已经启用；本阶段用于桌面调试，上电检查时卸下螺旋桨。

## 项目组成

应用与自定义驱动按职责组织；RT-Thread 内核、组件、抽象驱动、CPU 移植层和构建工具完整保留在 SDK 中。构建仅选择配置启用的功能。

~~~text
FyPilot/
├── app/                        应用入口、飞控算法和任务
│   ├── main.c                  fc_init / Hello World
│   ├── fc/                     初始化、状态、控制台与自检
│   ├── rx/                     CRSF 解析和 RT-Thread 串口接入
│   ├── flight/                 ekf、pid、attitude
│   ├── sensors/                IMU 采样与传感器管理
│   ├── tasks/                  姿态、控制、位置、通信、日志任务
│   ├── common/                 数学、矩阵、滤波、缓冲区和数据类型
│   ├── middleware/             IPC 与 uMCN 消息通信
│   ├── platform/               RT-Thread OS、时间、内存、调度接口
│   └── simulation/             虚拟传感器、电机、遥控器与回放
├── drivers/                    自定义传感器驱动和适配
│   ├── bmi270/                 BMI270 SPI 适配
│   └── barometer/              DPS368 驱动源码，尚未启用
├── bsp/stm32h743/               AOPAH743 板级支持与 SCons 入口
│   ├── board/                  时钟、引脚、HAL MSP、启动与链接布局
│   └── .config                 默认功能配置
├── sdk/                        随工程携带的 SDK
│   ├── rt-thread/              完整内核、组件、驱动、libcpu 和 tools
│   ├── stm32/                  STM32H7 HAL/LL、CMSIS、RT-Thread 公共驱动
│   ├── bmi270/                 Bosch BMI270 Sensor API
│   ├── cmsis-dsp/              当前矩阵接口使用的 CMSIS-DSP 源码
│   └── VERSIONS                上游版本及本地适配记录
├── tools/                      烧录、固件验证、宿主机检查和迁移清单
├── documentation/bringup.txt    详细启动、接口及迁移说明
├── build.sh                    SCons 编译包装
└── flash.sh                    J-Link / OpenOCD 烧录入口
~~~

[应用构建清单](app/SConscript)与[驱动构建清单](drivers/SConscript)使用明确的源文件列表，未适配模块不会因为放入目录就自动编译或启动。app/Kconfig 中的原任务配置尚未加入当前 BSP 菜单。

CRSF 的 app/rx/crsf.c/.h 保留上游实现及版权；当前固件编译 crsf_parser.c/.h 和 receiver_crsf.c/.h，两套实现不同时编译。PID 等原有模块仍依赖部分未移植的 Betaflight 接口；drivers/barometer 中的 DPS368 驱动不能直接视为板载 DPS310 驱动。

## 硬件目标

| 项目 | 配置 |
| --- | --- |
| 板卡 | AOPAH743 |
| MCU | STM32H743VIH6，Cortex-M7，硬件浮点 |
| 外部晶振 | 8 MHz HSE |
| 当前时钟 | CPU 400 MHz，HCLK 200 MHz，APB 100 MHz |
| 供电模式 | LDO |
| Flash | 2 MiB，镜像从 0x08000000 启动，无 bootloader 偏移 |
| 当前链接 RAM | AXI SRAM，0x24000000 起，512 KiB |
| Cache / DMA | I-Cache 开启，D-Cache 关闭，UART/SPI DMA 未启用 |
| 下载方式 | J-Link，SWD，默认 1000 kHz |

### 串口

| 外设 | TX | RX | 用途 / 状态 |
| --- | --- | --- | --- |
| USART1 | PA9 | PA10 | 引脚已记录，尚未启用 |
| USART2 | PD5 | PD6 | 引脚已记录，尚未启用 |
| USART3 | PD8 | PD9 | 调试控制台，115200 8N1 |
| UART4 | PB9 | PB8 | 引脚已记录，尚未启用 |
| USART6 | PC6 | PC7 | ELRS / CRSF，420000 8N1，当前仅接收 |
| UART7 | PE8 | PE7 | 引脚已记录，尚未启用 |
| UART8 | PE1 | PE0 | 引脚已记录，尚未启用 |

### SPI 与 I²C

SDI / SDO 按主控方向分别记为 MISO / MOSI。除已适配的 SPI4 外，表中总线只是用户提供的板卡分配，需配置并实测后启用。

| 外设 | SCK | MISO / SDI | MOSI / SDO | 设备 / 状态 |
| --- | --- | --- | --- | --- |
| SPI1 | PA5 | PA6 | PD7 | 尚未启用；其他传感器焊盘未作为主 IMU 配置 |
| SPI2 | PB13 | PB14 | PB15 | MAX7456，CS = PB12；尚未启用 |
| SPI3 | PB3 | PB4 | PB5 | 预留 Flash 总线，片选未提供；尚未启用 |
| SPI4 | PE12 | PE13 | PE14 | BMI270，CS = PE11，INT1 = PE15；IMU 配置启用 |

| 外设 | SCL | SDA | 设备 / 状态 |
| --- | --- | --- | --- |
| I²C1 | PB6 | PB7 | 引脚已记录，尚未启用 |
| I²C2 | PB10 | PB11 | DPS310 / IST8310，尚未启用 |

BMI270 为 U12，初始安装方向按 CW0_DEG / identity 记录，需按板上实物确认传感器坐标轴和正负方向。

### 电机与扩展引脚

| 输出 | 引脚 | 输出 | 引脚 |
| --- | --- | --- | --- |
| MOTOR1 | PB0 | MOTOR6 | PA3 |
| MOTOR2 | PB1 | MOTOR7 | PD12 |
| MOTOR3 | PA0 | MOTOR8 | PD13 |
| MOTOR4 | PA1 | MOTOR9 | PD14 |
| MOTOR5 | PA2 | MOTOR10 | PD15 |

这些引脚当前均为下拉输入。PWM / DShot 的定时器、通道、复用功能和 DMA 映射尚未实现和验证。

| 功能 | 引脚 / 配置 |
| --- | --- |
| LED0 / LED1 | PE3 / PE4 |
| LED strip | PA8 |
| 蜂鸣器 | PA15，原板卡配置为反相驱动 |
| 电池电压 / 电流 | PC0 / PC1 |
| RSSI / 外部 ADC1 / 第二电流 | PC5 / PC4 / PA7 |
| SDMMC1 CLK / CMD | PC12 / PD2 |
| SDMMC1 D0 / D1 / D2 / D3 | PC8 / PC9 / PC10 / PC11，目标为 4 bit，尚未启用 |
| SD 卡检测 | 未配置独立检测引脚 |
| PINIO1 / PINIO2 | PD10 / PD11 |
| SWDIO / SWCLK | PA13 / PA14 |

完整引脚记录见 [pinmap.csv](bsp/stm32h743/board/pinmap.csv)，当前生效的板级定义见 [target_config.h](bsp/stm32h743/board/target_config.h)。USB、外置 Flash 片选及其他未提供引脚不作默认假设。

## 获取与编译

需要 Python 3、SCons 4.x、kconfiglib，以及带 newlib 的 arm-none-eabi-gcc 工具链。Linux 下宿主机检查还需要 GCC 或 Clang。SDK 已包含在仓库中，不依赖旁边的 RT-Thread 仓库，也不需要初始化子模块。

~~~sh
git clone https://github.com/JiaVerso/FyPilot.git
cd FyPilot
python3 -m venv .venv
. .venv/bin/activate
python3 -m pip install -r requirements.txt

# 工具链已在 PATH 中时可省略此行
export RTT_EXEC_PATH=/path/to/arm-none-eabi/bin
./build.sh -j8
python3 tools/verify_firmware.py --profile hello --require-crsf
~~~

工具链查找优先级为 RTT_EXEC_PATH、PATH，最后尝试本机 STM32CubeCLT 安装目录。build.sh 依次运行配置生成和 SCons 编译。也可以直接运行：

~~~sh
cd bsp/stm32h743
scons --pyconfig-silent
scons -j8
~~~

输出位于 **bsp/stm32h743/build/**：

- **fypilot.elf**：带符号信息，可用于下载和调试。
- **fypilot.bin**：原始 Flash 镜像，下载起始地址 0x08000000。
- **fypilot.hex**：Intel HEX 镜像。
- **fypilot.map**：链接和内存占用信息。

生成物不提交到源码分支，克隆后本地编译获得。SDK 中上游自带的预编译库属于依赖输入，保留原文件与许可。

### 选择固件配置

~~~sh
cd bsp/stm32h743
scons --menuconfig
~~~

在 **AOPAH743 hardware** 中选择：

| 选项 | 默认值 | 说明 |
| --- | --- | --- |
| BSP_USING_HELLO_WORLD | y | 每秒输出 Hello World，BMI270 暂不启动 |
| BSP_USING_CRSF | y | 启用 USART6 CRSF 接收 |
| BSP_CRSF_RX_BUFFER_SIZE | 1024 | 中断接收 FIFO 字节数 |
| BSP_CRSF_SIGNAL_TIMEOUT_MS | 100 | 通道数据新鲜度超时，ms |

关闭 BSP_USING_HELLO_WORLD 后，SPI4 与 BMI270 的默认配置会启用。确认配置后重新编译，并从工程根目录检查：

~~~sh
./build.sh -j8
python3 tools/verify_firmware.py --profile imu --require-crsf
~~~

修改 .config 后必须通过 scons --pyconfig-silent 重新生成 rtconfig.h，不要手工编辑该头文件。测试结束后可重新启用 Hello World 并编译，恢复默认配置。

## 下载与首次启动

J-Link 接线：**SWDIO → PA13，SWCLK → PA14**，连接 GND 和目标板 VTref，保证板卡正常供电。调试目标为 STM32H743VI。当前脚本使用系统复位，不要求额外连接 NRST。

~~~sh
./flash.sh
# 或明确指定镜像
./tools/flash_jlink.sh bsp/stm32h743/build/fypilot.elf
~~~

脚本先验证固件，再通过 SEGGER JLinkExe 下载；flash.sh 在没有该命令时尝试 OpenOCD 的 J-Link 驱动。需要预先安装对应下载工具。

控制台接线：板上 **PD8 → 3.3 V USB-TTL RX，PD9 → USB-TTL TX**，共地，串口参数 **115200 8N1**。当前没有 USB 虚拟串口。

默认上电预期输出 FyPilot 板型、时钟与 CRSF 初始化状态，随后每秒输出 Hello World。可在 FinSH 中输入：

~~~text
board_status       查看芯片、时钟和堆内存
crsf               查看 16 路通道、链路统计和接收错误
list device        查看注册设备
list thread        查看任务
imu                IMU 配置下查看 BMI270 数据与中断统计
mp_selftest        IMU 配置下运行数学、CRC 和缓冲区自检
~~~

## ELRS / CRSF 接入

接收机 TX → **PC7 / USART6 RX**，共地并按接收机规格供电。PC6 是 USART6 TX，当前固件不发送遥测，可不接接收机 RX。协议为 **420000 baud、8N1、无反相**。

接收路径为：USART6 中断 → RT-Thread 串口 FIFO → CRSF 工作线程 → CRC 校验与解包 → 通道快照。支持完整 0x16 的 16 路 11 bit 通道，以及 0x14 链路统计；尚未实现遥测发送、MSP、0x17 子集通道或 V3 波特率协商。

~~~c
#include "rx/receiver_crsf.h"

crsf_state_t rc;
if (crsf_receiver_get(&rc)) {
    /* rc.raw[0]：通道 1 的 11 bit 原始值
     * rc.channel_us[0]：等效脉宽值，用于后续控制映射 */
}
~~~

通道顺序保持接收机原顺序，未预设 AETR / TAER。返回 true 表示已收到且未超时的通道；默认 100 ms 无有效通道帧即失效。这是数据新鲜度判断，尚未连接解锁、失联处置和电机状态机。详细接口与上游来源见 [bringup.txt](documentation/bringup.txt) 和 [ORIGIN.txt](app/rx/ORIGIN.txt)。

## 软件验证

在工程根目录运行：

~~~sh
python3 tools/check_crsf.py
python3 tools/check_math.py
python3 tools/check_buffer.py
python3 tools/verify_firmware.py --profile hello --require-crsf
~~~

- CRSF：上游抓包样例、16 路通道、分包/粘包、CRC/长度异常、噪声重同步、超时与时间回绕、链路统计，以及 5000 组随机通道和 250000 个噪声字节；启用 ASan / UBSan。
- 数学：矩阵逆与乘法、奇异矩阵、尺寸错误、空参数、CRC16。
- 缓冲区：覆盖、回绕、读取边界、复位与生命周期。
- 固件：ARM hard-float ABI、Flash/RAM 范围、启动向量、USART6 中断向量及 ELF/BIN/HEX 一致性。

这些检查验证软件逻辑和镜像结构。板卡启动、时序、信号电平、坐标轴及接收机通信仍需硬件测试。

## 开发与贡献

- **main**：经过当前编译和软件检查的集成基线，便于复现；不代表已完成飞行验证。
- **develop**：开发和测试分支，用于驱动、传感器、控制链等功能迭代。
- 功能修改从 develop 创建分支，完成验证后提交 Pull Request。可使用 feature/name 或 fix/name。

~~~sh
git switch develop
git pull --ff-only
git switch -c feature/bmi270-validation
# 修改后编译并运行相关检查，再提交和推送功能分支
~~~

报告问题时请附板型、分支/提交、构建配置、工具链版本和串口日志。欢迎通过 [Issues](https://github.com/JiaVerso/FyPilot/issues) 提交问题，通过 [Pull Requests](https://github.com/JiaVerso/FyPilot/pulls) 贡献代码。硬件适配请同时更新引脚说明和验证状态。

当前开发方向包括 BMI270 实板验证、EKF/PID 接口整合、Acro/Angle/Horizon 控制、mixer 与电机输出，以及气压计、磁力计、ADC、SD 卡等板载外设。它们是后续任务，不是当前已经可用的功能。

## 许可与致谢

本仓库包含多来源源码，各目录及文件中的原许可和版权声明保留。主要来源如下：

| 部分 | 来源与许可 |
| --- | --- |
| PID、部分公共算法、CRSF 原实现与移植代码 | Cleanflight / Betaflight / HPMicro Betaflight，GPL-3.0-or-later |
| RT-Thread | Apache-2.0；各组件仍以随附的许可声明为准 |
| STM32H7 HAL / CMSIS Device | STMicroelectronics 随附许可 |
| BMI270 Sensor API | Bosch Sensortec，BSD-3-Clause |
| CMSIS-DSP 矩阵接口 | Arm，Apache-2.0 |

GPL 全文见 [COPYING.GPL-3.0](COPYING.GPL-3.0)，上游版本和本地适配见 [sdk/VERSIONS](sdk/VERSIONS)。GPL 部分允许依照许可证进行商业使用；本仓库未额外添加禁止商用条款。发布和再分发时须遵守所用代码的许可。

感谢 RT-Thread、STMicroelectronics、Bosch Sensortec、Arm、Cleanflight、Betaflight、HPMicro 以及 StarryPilot 的相关工作。README 的项目介绍、开发入口、参与方式和许可章节组织参考 [ArduPilot](https://github.com/ArduPilot/ardupilot)，该参考不表示 FyPilot 与 ArduPilot 存在隶属关系。

项目维护：[JiaVerso](https://github.com/JiaVerso)。
