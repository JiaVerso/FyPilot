/* SPDX-License-Identifier: GPL-3.0-or-later */
#ifndef FYPILOT_RECEIVER_CRSF_H
#define FYPILOT_RECEIVER_CRSF_H
#include <rtthread.h>
#include "crsf_parser.h"
rt_err_t crsf_receiver_start(void);
/* False means no fresh RC frame. Channel values remain last-known, never invented. */
bool crsf_receiver_get(crsf_state_t *state);
#endif
