//=============================================================================================
// MadgwickAHRS.h
//=============================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
//
// From the x-io website "Open-source resources available on this website are
// provided under the GNU General Public Licence unless an alternative licence
// is provided in source."
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
// 21/07/2016   strv            Port to C and add some functions
//=============================================================================================
#ifndef MadgwickAHRS_h
#define MadgwickAHRS_h
#include <math.h>
#include <stdbool.h>

//-------------------------------------------------------------------------------------------
// Function declarations

void Madgwick_init(void);
void Madgwick_begin(float sampleFrequency, float beta);
void Madgwick_update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void Madgwick_updateIMU(float gx, float gy, float gz, float ax, float ay, float az);
bool Madgwick_is_init(void);
float Madgwick_getRoll();
float Madgwick_getPitch();
float Madgwick_getYaw();
float Madgwick_getRollRadians();
float Madgwick_getPitchRadians();
float Madgwick_getYawRadians();
void Madgwick_getPose(float* pr, float* pp, float* py);
void Madgwick_getPoseRadians(float* pr, float* pp, float* py);

#endif

