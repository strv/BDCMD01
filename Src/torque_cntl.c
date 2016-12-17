 /*
 * torque_cntl.c
 *
 *  Created on: 2016/12/12
 *      Author: strv
 */

#include "torque_cntl.h"
#include <stdbool.h>
#include "adc.h"
#include "pwm.h"
#include "tim.h"
#include "dac.h"

static const int64_t Isum_max = 100000;
static const int64_t tc_freq[2] = {10*1000, 10*1000};
static int64_t gkp[2], gki[2], gkd[2];
static int64_t gkp_div[2], gki_div[2], gkd_div[2];
static int32_t gkt[2];
static int32_t gl[2], gr[2];		//uH, mOhm
static int32_t gramp[2];
static int64_t isum[2] = {};
static int32_t cur_target[2] = {};	//mA
static int32_t cur_target_tmp[2] = {};	//uA
static int64_t vol_out[2] = {};		//mV
static bool do_tc[2] = {false, false};

void tc_init(void){
	gkp_div[0] = 0xFF;
	gki_div[0] = 0xFF;
	gkd_div[0] = 0xFF;
	gr[0] = 1000;
	gl[0] = 500;
	gkp_div[1] = 0xFF;
	gki_div[1] = 0xFF;
	gkd_div[1] = 0xFF;
	gr[1] = 1000;
	gl[1] = 500;
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
			gramp[0] = ramp * 1000 / tc_freq[0]; // A/sec to mA/tick
		}
	}
	if(ch & MD_CH2){
		if(gl[1] == 0){
			gramp[1] = ramp * 1000 / tc_freq[1]; // A/sec to mA/tick
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
 * @param	torque[in] : Target torque value in mNm/mA
**/
void tc_set_trq(MD_CH ch, int32_t torque){
	if(ch & MD_CH1){
		if(gkt[0] != 0){
			cur_target[0] = torque / gkt[0];
		}else{
			cur_target[0] = torque;
		}
	}
	if(ch & MD_CH2){
		if(gkt[1] != 0){
			cur_target[1] = torque / gkt[1];
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

int32_t tc_bemf_est(MD_CH ch){
	int32_t bemf = 0, i;
	if(ch == MD_CH1){
		i = 0;
	}else if(ch == MD_CH2){
		i = 1;
	}else{
		return 0;
	}
	bemf = vol_out[i] - cur_target[i] * gr[i] / 1000;
	return bemf;
}

void tc_proc(void){
	static int64_t cur_prev[2] = {};
	static int32_t cur_target_prev[2] = {};
	int64_t ff;
	int32_t cur[2];
	int32_t target;
	int64_t cur_diff[2];
	cur[0] = adc_cur1();
	cur[1] = adc_cur2();

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
		//mV = mA * mOhm / 1000 + d(mA)/dt * uH / 1000000

		vol_out[i] = ff
				+ gkp[i] * cur_diff[i] / gkp_div[i]
				+ gkd[i] * (cur_diff[i] - cur_prev[i]) * tc_freq[i] / gkd_div[i]
				+ gki[i] * isum[i] / tc_freq[i] / gki_div[i];

		if(i == 0){
			dac_set_mv(0, cur[0] + 3300 / 2);
			dac_set_mv(1, vol_out[i] / 10 + 3300 / 2);
		}

		cur_prev[i] = cur_diff[i];
		cur_target_prev[i] = target;

		pwm_set_mv(i == 0 ? MD_CH1: MD_CH2, vol_out[i]);
	}

}
