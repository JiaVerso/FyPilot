/*
 * @Author: Jack Shaw 2870998241@qq.com
 * @Date: 2026-11-05 23:32:38
 * @LastEditors: Jack Shaw 2870998241@qq.com
 * @LastEditTime: 2026-11-24 11:03:05
 * @FilePath: /FyPilot/applications/elrs/receiver_crsf.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */

#ifndef RECEIVER_CRSF_H
#define RECEIVER_CRSF_H
#include <rtthread.h>
#include "crsf_parser.h"
#include "crsf_protocol.h"

rt_err_t crsf_receiver_start(void);

#endif
