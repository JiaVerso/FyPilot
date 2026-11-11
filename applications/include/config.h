/*
 * @Author: Jack Shaw 2870998241@qq.com
 * @Date: 2026-11-05 20:45:56
 * @LastEditors: Jack Shaw 2870998241@qq.com
 * @LastEditTime: 2026-11-11 17:17:43
 * @FilePath: /FyPilot/applications/include/config.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef FYPILOT_FLIGHT_CONFIG_H
#define FYPILOT_FLIGHT_CONFIG_H

#include <rtconfig.h>

#ifdef FYPILOT_LOOP_HZ
#define FLIGHT_LOOP_HZ FYPILOT_LOOP_HZ
#else
#define FLIGHT_LOOP_HZ 250
#endif

#define FLIGHT_THREAD_NAME       "flight"
#define FLIGHT_THREAD_STACK_SIZE 4096
#define TEST_THREAD_PRIORITY   12
#define FLIGHT_THREAD_TIMESLICE  5

#endif
