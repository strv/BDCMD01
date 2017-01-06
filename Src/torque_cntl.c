 /*
 * torque_cntl.c
 *
 *  Created on: 2016/12/12
 *      Author: strv
 */

#include "torque_cntl.h"
#include <stdbool.h>
#include "main.h"
#include "adc.h"
#include "pwm.h"
#include "tim.h"
#include "dac.h"

static const int32_t Isum_max = 100000;
static int32_t tc_freq;
static int32_t gkp[2], gki[2], gkd[2];
static const int32_t gainQ = 8;
static int32_t gkt[2];				//uNm/A
static int32_t gl[2], gr[2];		//uH, mOhm
static int32_t gramp[2];
static int32_t isum[2] = {};
static int32_t cur_target[2] = {};	//mA
static int32_t cur_target_tmp[2] = {};	//uA
static int32_t vol_out[2] = {};		//mV
static int32_t vol_brush[2] = {};
static int32_t bemf[2] = {};		//mV
static bool do_tc[2] = {false, false};

void tc_init(void){
	tc_freq = HAL_RCC_GetSysClockFreq() / (TC_Period);

	tc_set_motor_param(MD_CH1, 24310, 20260);	//371 motor
	tc_set_kt(MD_CH1, 12200);
	tc_set_gain_by_lsm(MD_CH1, 50000, 1800);
	vol_brush[0] = 300;
/*
	tc_set_motor_param(MD_CH12, 420, 5000);		//TG-21R
	tc_set_kt(MD_CH12, 16333);
	tc_set_gain_by_lsm(MD_CH1, 1200, 1350);
*/

	tc_set_motor_param(MD_CH2, 75, 110);		//GT tune
	tc_set_kt(MD_CH2, 0);
	tc_set_gain_by_lsm(MD_CH2, 100, 1200);

	tc_set_ramp(MD_CH12, 1000);
	control_tim_start();
}

void tc_enable(MD_CH ch){
	pwm_set_mode(ch, PWM_VCMD);
	if(ch & MD_CH1){
		isum[0] = 0;
		cur_target[0] = 0;
		do_tc[0] = true;
	}
	if(ch & MD_CH2){
		isum[1] = 0;
		cur_target[1] = 0;
		do_tc[1] = true;
	}
}

void tc_disable(MD_CH ch){
	if(ch & MD_CH1){
		do_tc[0] = false;
	}
	if(ch & MD_CH2){
		do_tc[1] = false;
	}
}

/**
 * @param[in] kt : uNm/A
 */
void tc_set_kt(MD_CH ch, int32_t kt){
	if(ch & MD_CH1){
		gkt[0] = kt;
	}
	if(ch & MD_CH2){
		gkt[1] = kt;
	}
}

void tc_set_motor_param(MD_CH ch, int32_t l, int32_t r){
	if(ch & MD_CH1){
		gl[0] = l;
		gr[0] = r;
	}
	if(ch & MD_CH2){
		gl[1] = l;
		gr[1] = r;
	}
}

/**
 * @param[in] ramp : [A/sec]
 */
void tc_set_ramp(MD_CH ch, int32_t ramp){
	if(ch & MD_CH1){
		if(gl[0] == 0){
			gramp[0] = ramp * 1000 / tc_freq; // A/sec to mA/tick
		}
	}
	if(ch & MD_CH2){
		if(gl[1] == 0){
			gramp[1] = ramp * 1000 / tc_freq; // A/sec to mA/tick
		}
	}
}

void tc_set_gain_by_lsm(MD_CH ch, int32_t kc, int32_t fc){
	int32_t kp, ki, kd;
#if 1
	kp = kc * 45 / 100;
	ki = kp * fc * 1000 / 833;
	kd = 0;
#else
#endif
	tc_set_gain(ch, kp, ki, kd);
}

void tc_set_gain(MD_CH ch, int32_t kp, int32_t ki, int32_t kd){
	if(ch & MD_CH1){
		gkp[0] = kp;
		gki[0] = ki;
		gkd[0] = kd;
	}
	if(ch & MD_CH2){
		gkp[1] = kp;
		gki[1] = ki;
		gkd[1] = kd;
	}
}

void tc_get_gain(MD_CH ch, int32_t* pkp, int32_t* pki, int32_t* pkd){
	switch(ch){
	case MD_CH1:
		*pkp = gkp[0];
		*pki = gki[0];
		*pkd = gkd[0];
		break;

	case MD_CH2:
		*pkp = gkp[1];
		*pki = gki[1];
		*pkd = gkd[1];
		break;

	default:
		break;
	}
}

/**
 *
 * @param	torque[in] : Target torque value in uNm
**/
void tc_set_trq(MD_CH ch, int32_t torque){
	if(ch & MD_CH1){
		if(gkt[0] != 0){
			cur_target[0] = torque * 1000 / gkt[0];	//mA = uNm / (uNm / A)
		}else{
			cur_target[0] = torque;
		}
	}
	if(ch & MD_CH2){
		if(gkt[1] != 0){
			cur_target[1] = torque * 1000 / gkt[1];	//mA = uNm / (uNm/A)
		}else{
			cur_target[1] = torque;
		}
	}
}

void tc_set_ma(MD_CH ch, int32_t ma){
	if(ch & MD_CH1){
		cur_target[0] = ma;
	}
	if(ch & MD_CH2){
		cur_target[1] = ma;
	}
}

/**
 * @return BEMF value in [mV]
 */
int32_t tc_bemf_est(MD_CH ch){
	int32_t i;
	if(ch == MD_CH1){
		i = 0;
	}else if(ch == MD_CH2){
		i = 1;
	}else{
		return 0;
	}
	return bemf[i];
}

void tc_proc(void){
	static int64_t cur_prev[2] = {};
	static int32_t cur_target_prev[2] = {};
	int64_t ff;
	int32_t cur[2];
	int32_t target;
	int64_t cur_diff[2];
	int32_t _p,_i,_d;
	cur[0] = adc_cur1();
	cur[1] = adc_cur2();
	int32_t pres_vb = adc_vbatt() * 95 / 100;

	for(int32_t i = 0; i < 2; i++){
		if(!do_tc[i]){
			continue;
		}

		target = cur_target[i];
		if(gramp[i] > 0){
			if(target > cur_target_tmp[i]){
				cur_target_tmp[i] += gramp[i];
				if(cur_target_tmp[i] > target){
					cur_target_tmp[i] = target;
				}
			}else if(target < cur_target_tmp[i]){
				cur_target_tmp[i] -= gramp[i];
				if(cur_target_tmp[i] < target){
					cur_target_tmp[i] = target;
				}
			}
			cur_diff[i] = cur_target_tmp[i] - cur[i];
		}else{
			cur_diff[i] = target - cur[i];
		}

		isum[i] += cur_diff[i];
		if(isum[i] > Isum_max){
			isum[i] = Isum_max;
		}else if(isum[i] < -Isum_max){
			isum[i] = -Isum_max;
		}

		ff = target * gr[i] / 1000
				+ (target - cur_target_prev[i]) * gl[i] / 1000000;
		if(target > 0){
			ff += vol_brush[i];
		}else if(target < 0){
			ff -= vol_brush[i];
		}
		//mV = mA * mOhm / 1000 + d(mA)/dt * uH / 1000000

		_p = gkp[i] * cur_diff[i] / (1 << gainQ);
		_i = (int64_t)gki[i] * (int64_t)isum[i] / tc_freq / (1 << gainQ);
		_d = (int64_t)(gkd[i] * (cur_diff[i] - cur_prev[i])) * (int64_t)tc_freq / (1 << gainQ);
		vol_out[i] = ff + _p + _i + _d;

#if DAC_OUT == DAC_TC
		if(i == 0){
			dac_set_mv(0, cur[0] + 3300 / 2);
			dac_set_mv(1, vol_out[i] / 10 + 3300 / 2);
		}
#endif
		cur_prev[i] = cur_diff[i];
		cur_target_prev[i] = target;

		if(vol_out[i] > pres_vb){
			vol_out[i] = pres_vb;
		}else if(vol_out[i] < -pres_vb){
			vol_out[i] = -pres_vb;
		}

		bemf[i] = ((vol_out[i] - ff) * 1 + bemf[i] * 7) / 8;

		pwm_set_mv(i == 0 ? MD_CH1: MD_CH2, vol_out[i]);
	}

}
