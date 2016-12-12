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

static const int64_t Isum_max = 100000;
static int64_t gkp[2], gki[2], gkd[2];
static int32_t gkt[2];
static int32_t gl[2], gr[2];		//mH, mOhm
static int64_t isum[2] = {};
static int64_t cur_target[2] = {};	//mA
static int64_t vol_out[2] = {};		//mV
static bool do_tc[2] = {false, false};

void tc_init(void){

}

void tc_enable(MD_CH ch){
	if(ch & MD_CH1){
		do_tc[0] = true;
	}
	if(ch & MD_CH2){
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

void tc_proc(void){
	static int64_t cur_prev[2] = {};
	int64_t ff;
	int64_t cur[2], cur_diff[2];
	cur[0] = adc_cur1();
	cur[1] = adc_cur2();

	for(int32_t i = 0; i < 2; i++){
		if(!do_tc[i]){
			continue;
		}

		cur_diff[i] = cur[i] - cur_target[i];
		isum[i] += cur_diff[i];
		if(isum[i] > Isum_max){
			isum[i] = Isum_max;
		}else if(isum[i] < -Isum_max){
			isum[i] = -Isum_max;
		}
		ff = cur_target[i] * 1000 / gr[i];	//mV = mA * 1000 / mOhm
		vol_out[i] = ff + gkp[i] * cur_diff[i] + gkd[i] * (cur_diff[i] - cur_prev[i]) + gki[i] * isum[i];
		pwm_set_mv(i == 0 ? MD_CH1: MD_CH2, vol_out[i]);
	}

}
