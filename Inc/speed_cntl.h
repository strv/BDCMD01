/*
 * speed_cntl.h
 *
 *  Created on: 2016/12/17
 *      Author: strv
 */

#ifndef SPEED_CNTL_H_
#define SPEED_CNTL_H_

#include <stdint.h>
#include <stdbool.h>
#include "motor_driver.h"

#define	SC_PID	1
#define	SC_I_PD	2
#define	SC_METHOD	SC_PID

void sc_init(void);
void sc_enable(MD_CH ch);
void sc_disable(MD_CH ch);
void sc_set_kw(MD_CH ch, int32_t kw);
void sc_set_motor_param(MD_CH ch, int32_t j, int32_t d);
void sc_set_profile(MD_CH ch, MotorProfile mt_prof, SpeedCTRLProfile sc_prof);
void sc_set_ramp(MD_CH ch, int32_t ramp);
void sc_set_gain_by_lsm(MD_CH ch, int32_t kc, int32_t fc);
void sc_set_gain(MD_CH ch, int32_t kp, int32_t ki, int32_t kd);
void sc_get_gain(MD_CH ch, int32_t* pkp, int32_t* pki, int32_t* pkd);
void sc_set_speed(MD_CH ch, int32_t speed);
int32_t sc_get_speed_bemf(MD_CH ch);
int32_t sc_get_speed_enc(MD_CH ch);
void sc_proc(void);

#endif /* SPEED_CNTL_H_ */
