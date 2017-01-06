/*
 * motor_driver.h
 *
 *  Created on: 2016/12/12
 *      Author: strv
 */

#ifndef MOTOR_DRIVER_H_
#define MOTOR_DRIVER_H_

#include <stdint.h>
#include <stdbool.h>

#define	DAC_TC	(1)
#define	DAC_SC	(2)
#define	DAC_PC	(3)
#define	DAC_OUT	DAC_SC

typedef enum{
	MD_CH1 = 1 << 0,
	MD_CH2 = 1 << 1,
	MD_CH12 = MD_CH1 | MD_CH2,
	MD_CH_MAX
}MD_CH;

typedef enum{
	DIR_FWD,
	DIR_REV
}DIR;

typedef enum{
	SM_Encoder,
	SM_BEMF,
	SM_MAX
}SpeedMeasure;

typedef struct{
	int32_t id;					// This value is automatically set by software.
	const char* name;
	int32_t resistance;			// mOhm
	int32_t inductance;			// uH
	int32_t kw;					// rpm/V
	int32_t kt;					// uNm/A
	int32_t	stall_current;		// mA
	int32_t continuous_current;	// mA
	int32_t free_current;		// mA
	int32_t watt;				// mWatts
	int32_t tc_gain_kp;			// PID control gain. mV/mA * 255
	int32_t tc_gain_ki;			// PID control gain. mV/mA-s * 255
	int32_t tc_gain_kd;			// PID control gain. mV/mA/s * 255
}MotorProfile;

typedef struct{
	int32_t encoder_cpr;		// Original count per rotation value. Before multiplying.
	bool	encoder_reverse;	// Set encoder direction.
	int32_t output_ratio_nmr;	// numerator of motor axle to output axle ratio
	int32_t output_ratio_dnm;	// denominator of motor axle to output axle ratio
	int32_t sc_gain_ki;			// I-P control gain
	int32_t sc_gain_kp;			// I-P control gain
}SpeedCTRLProfile;

#endif /* MOTOR_DRIVER_H_ */
