/*
 * speed_cntl.c
 *
 *  Created on: 2016/12/17
 *      Author: strv
 */

#include "speed_cntl.h"
#include "main.h"
#include "torque_cntl.h"
#include "dac.h"
#include "pwm.h"
#include "encoder.h"

static const int64_t Isum_max = 1000000000;
static int64_t sc_freq;
static int64_t gkp[2], gki[2], gkd[2];
static const int32_t gainQ = 10;
static int32_t gkw[2];					// rpm / V
static int32_t gramp[2];
static int64_t isum[2] = {};
static int32_t rpm_target[2] = {};		// rpm
static int32_t rpm_target_tmp[2] = {};	// rpm
static int64_t trq_out[2] = {};			// uNm
static bool do_sc[2] = {false, false};
static int32_t enc_diff[2] = {};
static int32_t enc_cpr[2] = {1, 1};
static const int32_t encQ = 20;
static const int32_t enc_lpfQ = 3;

void sc_init(void){
	sc_freq =  HAL_RCC_GetSysClockFreq() / (SC_Period);
/*
	sc_set_kw(MD_CH12, 6250 * 19 / 12 / 20);	//TG-21R
	sc_set_gain_by_lsm(MD_CH1, 13000, 110);
*/

	sc_set_kw(MD_CH2, 2277);		//GT tune
	sc_set_gain(MD_CH2, 3000, 6000000, 0);

	sc_set_kw(MD_CH1, 356);					//371
	sc_set_gain(MD_CH1, 800, 600000, 0);
	enc_cpr[0] = 13;

	sc_set_ramp(MD_CH12, 1000);
}

void sc_enable(MD_CH ch){
	pwm_set_mode(ch, PWM_VCMD);
	tc_enable(ch);
	if(ch & MD_CH1){
		isum[0] = 0;
		rpm_target[0] = 0;
		do_sc[0] = true;
	}
	if(ch & MD_CH2){
		isum[1] = 0;
		rpm_target[1] = 0;
		do_sc[1] = true;
	}
}

void sc_disable(MD_CH ch){
	if(ch & MD_CH1){
		do_sc[0] = false;
	}
	if(ch & MD_CH2){
		do_sc[1] = false;
	}
}

/**
 * @param[in] kw : revolution constant [rpm / V]
 */
void sc_set_kw(MD_CH ch, int32_t kw){
	if(ch & MD_CH1){
		gkw[0] = kw;
	}
	if(ch & MD_CH2){
		gkw[1] = kw;
	}
}

void sc_set_motor_param(MD_CH ch, int32_t j, int32_t d){

}

void sc_set_profile(MD_CH ch, MotorProfile mt_prof, SpeedCTRLProfile sc_prof){

}

/**
 * param[in] ramp : [rpm / s]
 */
void sc_set_ramp(MD_CH ch, int32_t ramp){
	if(ch & MD_CH1){
		gramp[0] = ramp / sc_freq; // rpm/sec to rpm/tick
	}
	if(ch & MD_CH2){
		gramp[1] = ramp / sc_freq; // rpm/sec to rpm/tick
	}
}

void sc_set_gain_by_lsm(MD_CH ch, int32_t kc, int32_t fc){
	int32_t kp, ki, kd;
#if 1
	kp = kc * 45 / 100;
	ki = kp * fc * 1000 / 833;
	kd = 0;
#else
#endif
	sc_set_gain(ch, kp, ki, kd);
}

void sc_set_gain(MD_CH ch, int32_t kp, int32_t ki, int32_t kd){
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

void sc_get_gain(MD_CH ch, int32_t* pkp, int32_t* pki, int32_t* pkd){
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

void sc_set_speed(MD_CH ch, int32_t speed){
	if(ch & MD_CH1){
		rpm_target[0] = speed;
	}
	if(ch & MD_CH2){
		rpm_target[1] = speed;
	}
}

int32_t sc_get_speed_bemf(MD_CH ch){
	switch(ch){
	case MD_CH1:
		return tc_bemf_est(MD_CH1) * gkw[0] / 1000;
		break;

	case MD_CH2:
		return tc_bemf_est(MD_CH2) * gkw[1] / 1000;
		break;

	default:
		break;
	}
	return 0;
}

int32_t sc_get_speed_enc(MD_CH ch){
	switch(ch){
	case MD_CH1:
		return enc_diff[0] * sc_freq * 60 / (enc_cpr[0] * 4) / (1 << encQ);
		break;

	case MD_CH2:
		return enc_diff[1] * sc_freq * 60 / (enc_cpr[1] * 4) / (1 << encQ);
		break;

	default:
		break;
	}
	return 0;
}

void sc_proc(void){
	static int64_t rpm_prev[2] = {};
	static int32_t rpm_target_prev[2] = {};
	static int64_t enc_prev[2] = {};
	int64_t enc[2];
	int64_t ff;
	int32_t rpm[2];
	int32_t target;
	int64_t rpm_diff[2];
	rpm[0] = tc_bemf_est(MD_CH1) * gkw[0] / 1000;	// rpm = mV * rpm / V / 1000
	rpm[1] = tc_bemf_est(MD_CH2) * gkw[1] / 1000;

	for(int32_t i = 0; i < 2; i++){
		enc[i] = encoder_get(i == 0 ? MD_CH1: MD_CH2);
		enc_diff[i] = ( (int32_t)(enc[i] - enc_prev[i]) * (1 << encQ) + enc_diff[i] * ( (1 << enc_lpfQ) - 1 ) ) / (1 << enc_lpfQ);
		enc_prev[i] = enc[i];

		if(!do_sc[i]){
			continue;
		}

		target = rpm_target[i];
		if(gramp[i] > 0){
			if(target > rpm_target_tmp[i]){
				rpm_target_tmp[i] += gramp[i];
				if(rpm_target_tmp[i] > target){
					rpm_target_tmp[i] = target;
				}
			}else if(target < rpm_target_tmp[i]){
				rpm_target_tmp[i] -= gramp[i];
				if(rpm_target_tmp[i] < target){
					rpm_target_tmp[i] = target;
				}
			}
			rpm_diff[i] = rpm_target_tmp[i] - rpm[i];
		}else{
			rpm_diff[i] = target - rpm[i];
		}

		isum[i] += rpm_diff[i];
		if(isum[i] > Isum_max){
			isum[i] = Isum_max;
		}else if(isum[i] < -Isum_max){
			isum[i] = -Isum_max;
		}

		ff = 0;
		trq_out[i] = ff
				+ gkp[i] * rpm_diff[i] / (1 << gainQ)
				+ gkd[i] * (rpm_diff[i] - rpm_prev[i]) * sc_freq / (1 << gainQ)
				+ gki[i] * isum[i] / sc_freq / (1 << gainQ);
#if DAC_OUT == DAC_SC
		if(i == 0){
			dac_set_mv(0, rpm[0] + 3300 / 2);
			dac_set_mv(1, trq_out[i] / 20 + 3300 / 2);
		}
#endif
		rpm_prev[i] = rpm_diff[i];
		rpm_target_prev[i] = target;

		tc_set_trq(i == 0 ? MD_CH1: MD_CH2, trq_out[i]);
	}
}
