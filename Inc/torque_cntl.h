/*
 * torque_cntl.h
 *
 *  Created on: 2016/12/12
 *      Author: strv
 */

#ifndef TORQUE_CNTL_H_
#define TORQUE_CNTL_H_

#include <stdint.h>
#include "motor_driver.h"

void tc_init(void);
void tc_enable(MD_CH ch);
void tc_disable(MD_CH ch);
void tc_set_kt(MD_CH ch, int32_t kt);
void tc_set_motor_param(MD_CH ch, int32_t l, int32_t r);
void tc_set_ramp(MD_CH ch, int32_t ramp);
void tc_set_gain_by_lsm(MD_CH ch, int32_t kc, int32_t fc);
void tc_set_gain(MD_CH ch, int32_t kp, int32_t ki, int32_t kd);
void tc_get_gain(MD_CH ch, int32_t* pkp, int32_t* pki, int32_t* pkd);
void tc_set_trq(MD_CH ch, int32_t torque);
void tc_set_ma(MD_CH ch, int32_t ma);
int32_t tc_bemf_est(MD_CH ch);
void tc_proc(void);

#endif /* TORQUE_CNTL_H_ */
