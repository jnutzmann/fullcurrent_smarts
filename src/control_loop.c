/********************************************************************
 control_loop.c

Copyright (c) 2014, Jonathan Nutzmann

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
 ********************************************************************/

/****************************************************************************
 * Includes
 ***************************************************************************/

#include "control_loop.h"
#include "system_cfg.h"
#include "adc.h"
#include "fir_filters.h"
#include "math_limits.h"
#include "halls.h"
#include "pwm.h"
#include "d1k_led.h"
#include "smarts_leds.h"
#include "foc.h"
#include "pi.h"
#include "d1k_can.h"
#include "mCAN.h"
#include "FreeRTOS.h"
#include "task.h"
#include "d1k_stdio_can.h"
#include "d1k_portal.h"
#include "fullCAN.h"
#include "diagnostics.h"
#include "motor.h"
#include "motor_position.h"
#include "stdlib.h"
//#include "ymath.h"
#include "fram_map.h"
#include "d1k_fram.h"
#include "pole_position.h"
#include "stm32f4xx_gpio.h"

/****************************************************************************
 * Definitions
 ***************************************************************************/

#define DEBUG_PERIOD_MS				( 2 )

#define DTI  		( 1.0f / ((float)CONTROL_LOOP_FREQUENCY_HZ) )

#define VDQ_LIMIT_PERCENT	( 0.54f ) // this limits to 0.957 duty cycle
#define VQ_LIMIT_PERCENT    ( VDQ_LIMIT_PERCENT * 0.85f )	// This limits vector to 1
#define VD_LIMIT_PERCENT    ( VDQ_LIMIT_PERCENT * 0.5268f )

#define IQ_LIMIT_A			( 140.0f )
#define ID_LIMIT_A			( 0.0f  )

#define SOFTWARE_CURRENT_FAULT_SOFT	( 160.0f )
#define SOFTWARE_CURRENT_FAULT_HARD ( 185.0f )

/****************************************************************************
 * Typedefs
 ***************************************************************************/

typedef struct
{
	drivemode_t drivemode;
	drivemode_t pendingDrivemode;

	float vqMax;
	float vdMax;
	float vqMin;
	float vdMin;

	float idRef;
	float iqRef;

} controlParameters_t;

/****************************************************************************
 * Global Variables
 ***************************************************************************/

q15_t filter_hamming_4khz[FILTER_HAMMING_4KHZ_LENGTH] = { 26, 28, 29, 30, 29,
		25, 16, 0, -23, -54, -92, -135, -180, -222, -256, -274, -271, -239,
		-172, -68, 78, 264, 487, 741, 1019, 1308, 1597, 1872, 2120, 2328, 2485,
		2582, 2616, 2582, 2485, 2328, 2120, 1872, 1597, 1308, 1019, 741, 487,
		264, 78, -68, -172, -239, -271, -274, -256, -222, -180, -135, -92, -54,
		-23, 0, 16, 25, 29, 30, 29, 28, 26, };


#define FILTER_LENGTH_FILTER_TAYLOR_4KHZ_FAST (21)

static q15_t filter_taylor_4khz_fast[FILTER_LENGTH_FILTER_TAYLOR_4KHZ_FAST] = {
	486,600,796,1057,1357,1666,
	1960,2216,2414,2541,2584,2541,
	2414,2216,1960,1666,1357,1057,
	796,600,486
};


static struct {
	float Ia;
	float Ib;
	float Ic;
	float Va;
	float Vb;
	float Vc;
	float Vbus;
	float Vgate;
} analogReadings;


typedef struct {
	float id;
	float iq;
	float motorPos;
	float vbus;
	float vq;
	float vd;
} hsLogData_t;

#define HS_LOG_DATA_COUNT (1024)

static uint32_t hsLogIndex = 0;
static bool hsLogEnabled = true;
static hsLogData_t hsLog[HS_LOG_DATA_COUNT];

static focControl_t foc;
static controlParameters_t cp;

static pi_t piIq;
static pi_t piId;

static xTaskHandle debugSendHandle;

#define MOTOR_OFFSET_DELTA_SAMPLE_SIZE (128)
#define MOTOR_OFFSET_DELTA_PERIOD_MS   (2)

static bool motorOffsetMode = false;
static uint32_t motorOffsetIndex = 0;
static bool swapAAndBPhases = false;

static float mOff_Va [MOTOR_OFFSET_DELTA_SAMPLE_SIZE];
static float mOff_Vb [MOTOR_OFFSET_DELTA_SAMPLE_SIZE];
static float mOff_MPTheta [MOTOR_OFFSET_DELTA_SAMPLE_SIZE];
static float mOff_mechVelocityAccum = 0.0f;

/****************************************************************************
 * Private Prototypes
 ***************************************************************************/

static void ControlLoop(void);
static void UpdateAnalogChannels( void );
static void FOCTorqueControl(void);
static void SendDebugData( void * pvParameters);
static void checkForStateChange( void );


static void InitControlLoopConstants( void );

/****************************************************************************
 * Public Functions
 ***************************************************************************/

void ControlLoopInit(void)
{
	d1k_fram_Read(FRAM_ADDRESS_MOTOR_PHASING_SWAPPED,&swapAAndBPhases,sizeof(swapAAndBPhases));

	InitControlLoopConstants();

	cp.drivemode = DRIVEMODE_INVALID;
	cp.pendingDrivemode = DRIVEMODE_INVALID;

	SetDrivemode( DRIVEMODE_DISABLED );

	SetVDMax( 160.0f );
	SetVDMin( -160.0f );
	SetVQMax( 160.0f );
	SetVQMin( -160.0f );

	SetIQRef( 0.0f );
	SetIDRef( 0.0f );

	PI_Init( &piId );
	PI_Init( &piIq );

	piIq.dt = DTI;
	piId.dt = DTI;

	d1k_portal_RegisterFunction("a2d",a2dPortal);
	d1k_portal_RegisterFunction("drivemode", drivemodePortal);
	d1k_portal_RegisterFunction("ppoffcal",PPOffsetAnglePortal);
	d1k_portal_RegisterFunction("dump", PrintHsLog);

	xTaskCreate( SendDebugData, "CANDBG", 2048, NULL, 3, &debugSendHandle );

	pwm_RegisterControlLoopFxn(ControlLoop);
	pwm_Init(PWM_FREQUENCY_HZ);

	pwm_SetPWMOutputEnable( DISABLE, DISABLE, DISABLE );
}

/************ STATE PARAMETERS ************/

void SetDrivemode ( drivemode_t d )
{
	if ( d < DRIVEMODE_INVALID )
	{
		cp.pendingDrivemode = d;
	}
}

void SetDrivemodeU8 ( uint8 d )
{
	if ( d < DRIVEMODE_INVALID )
	{
		cp.pendingDrivemode = (drivemode_t) d;
	}
}

drivemode_t GetDrivemode ( void )   { return cp.drivemode; }
uint8 GetDrivemodeU8 ( void )   { return (uint8) cp.drivemode; }

float rolloffAveragingPeriod = 2;


/************ Limit Parameters ************/

void  SetVDMax ( float vd ) { cp.vdMax = vd;   }
float GetVDMax ( void )     { return piId.outMax; }
void  SetVDMin ( float vd ) { cp.vdMin = vd;   }
float GetVDMin ( void )     { return piId.outMin; }

void  SetVQMax ( float vq ) { cp.vqMax = vq;   }
float GetVQMax ( void )     { return piIq.outMax; }
void  SetVQMin ( float vq ) { cp.vqMin = vq;   }
float GetVQMin ( void )     { return piIq.outMin; }

void  SetKP   ( float k )
{
	piIq.kp = k; piId.kp = k;
	d1k_fram_Write(FRAM_ADDRESS_KP,&k,sizeof(k));
}
void  SetKI   ( float k )
{
	piIq.ki = k; piId.ki = k;
	d1k_fram_Write(FRAM_ADDRESS_KI,&k,sizeof(k));
}

void  SetRolloffAveragingPeriod ( float period )
{
	rolloffAveragingPeriod = period;
	d1k_fram_Write(FRAM_ADDRESS_AVERAGINGPER,&period,sizeof(period));
}

static void InitControlLoopConstants( void )
{
	float k;

	d1k_fram_Read(FRAM_ADDRESS_KP,&k,sizeof(k));
	piIq.kp = k; piId.kp = k;
	d1k_fram_Read(FRAM_ADDRESS_KI,&k,sizeof(k));
	piIq.ki = k; piId.ki = k;
	d1k_fram_Read(FRAM_ADDRESS_AVERAGINGPER,&k,sizeof(k));
	rolloffAveragingPeriod = k;
}

float GetKP   ( void )  { return piIq.kp; }
float GetKI   ( void )  { return piIq.ki; }
float GetRolloffAveragingPeriod ( void ) { return rolloffAveragingPeriod; }

/************************************************************/
/************************************************************/
/************************************************************/

/************ Reference Parameters ************/

void  SetIQRef ( float iq ) { cp.iqRef = limitf32(iq,IQ_LIMIT_A,-IQ_LIMIT_A);   }
float GetIQRef ( void )     { return piIq.reference; }

void  SetIDRef ( float id ) { cp.idRef = limitf32(id,ID_LIMIT_A,-ID_LIMIT_A);   }
float GetIDRef ( void )     { return piId.reference; }

/****************************************************************************
 * Private Functions
 ***************************************************************************/

#define SOFTWARE_CURRENT_FAULT_COUNT	(120) // 3ms

static void ControlLoop(void)
{
	static uint32_t waitCount = 0;
	static uint32_t settleTime = 10000;
	static uint32_t currentErrorFilter = 0;

	waitCount = (waitCount + 1)
				% ( (PWM_FREQUENCY_HZ * 2) / CONTROL_LOOP_FREQUENCY_HZ );

	if ( waitCount == 0 )
	{
		GPIO_SetBits(GPIOB,GPIO_Pin_15);

		// Check to see if we have a new drivemode.  Set the gates accordingly.
		checkForStateChange();

		// Update the analog measurements.
		UpdateAnalogChannels();

		PolePosition_UpdateVelocityCalculation();

		// Check for current faults.  Ignore them for the first settle time.
		if (settleTime > 0) settleTime--;

		if ( diag_GetHardwareOvercurrent() )
		{
			if ( settleTime == 0 )
			{
				diag_ThrowError(ERROR_CURRENT_HARDWARE_FAULT);
			}
		}

		if ( analogReadings.Ia > SOFTWARE_CURRENT_FAULT_SOFT
		  || analogReadings.Ia < -SOFTWARE_CURRENT_FAULT_SOFT
		  || analogReadings.Ib > SOFTWARE_CURRENT_FAULT_SOFT
		  || analogReadings.Ib < -SOFTWARE_CURRENT_FAULT_SOFT
		  || analogReadings.Ic > SOFTWARE_CURRENT_FAULT_SOFT
		  || analogReadings.Ic < -SOFTWARE_CURRENT_FAULT_SOFT )
		{
			if (settleTime == 0)
			{
				if ( analogReadings.Ia > SOFTWARE_CURRENT_FAULT_HARD
					  || analogReadings.Ia < -SOFTWARE_CURRENT_FAULT_HARD
					  || analogReadings.Ib > SOFTWARE_CURRENT_FAULT_HARD
					  || analogReadings.Ib < -SOFTWARE_CURRENT_FAULT_HARD
					  || analogReadings.Ic > SOFTWARE_CURRENT_FAULT_HARD
					  || analogReadings.Ic < -SOFTWARE_CURRENT_FAULT_HARD )
				{
					diag_ThrowError(ERROR_CURRENT_SOFTWARE_FAULT);
				}
				else
				{
					currentErrorFilter++;

					if( currentErrorFilter >= SOFTWARE_CURRENT_FAULT_COUNT )
					{
						diag_ThrowError(ERROR_CURRENT_SOFTWARE_FAULT);
					}
				}
			}
		}
		else if ( currentErrorFilter > 0 )
		{
			currentErrorFilter--;
		}

		// Check to make sure the motor is present.  Otherwise, throw an error.
		(void) motor_CheckIfPresent( );

		// If there are any errors present, immediately float the gates and put the
		// controller in the disable drivemode.
		if ( diag_GetErrorBitmask() != 0x0 )
		{
			pwm_SetPWMOutputEnable(DISABLE,DISABLE,DISABLE);
			SetDrivemode(DRIVEMODE_DISABLED);
			return;
		}

		foc.theta = MotorPosition_GetElectricalPosition() - 90.0f;

		static uint32_t startupWaitTime = CONTROL_LOOP_FREQUENCY_HZ * 3;

		if ( startupWaitTime > 0 )
		{
			startupWaitTime--;
			return;
		}

		static uint32_t logCnt = 0;

		if ( cp.drivemode == DRIVEMODE_FORWARD || cp.drivemode == DRIVEMODE_REVERSE )
		{
			FOCTorqueControl();
/*
			static float lastRef = 0.0f;

			if (fabs(cp.iqRef-lastRef) > 10.0f)
			{
				hsLogEnabled = true;
				hsLogIndex = 0;
			}

			lastRef = cp.iqRef;

			if ( hsLogEnabled & hsLogIndex >= (HS_LOG_DATA_COUNT-1) )
			{
				hsLogEnabled = false;
				hsLogIndex = 0;
			}
*/
			if (hsLogEnabled)
			{
				logCnt = (logCnt + 1) % 5;

				if ( logCnt == 0 )
				{
					hsLog[hsLogIndex].id = foc.Id;
					hsLog[hsLogIndex].iq = foc.Iq;
					hsLog[hsLogIndex].vd = foc.Vd;
					hsLog[hsLogIndex].vq = foc.Vq;
					hsLog[hsLogIndex].vbus = analogReadings.Vbus;
					hsLog[hsLogIndex].motorPos = foc.theta;

					hsLogIndex = (hsLogIndex+1)%HS_LOG_DATA_COUNT;
				}
			}
		}
		else
		{

			float n = analogReadings.Va + analogReadings.Vb + analogReadings.Vc;

			n *= (1.0f/3.0f);

			float van, vbn;

			if ( swapAAndBPhases )
			{
				van = analogReadings.Vb - n;
				vbn = analogReadings.Va - n;
			}
			else
			{
				van = analogReadings.Va - n;
				vbn = analogReadings.Vb - n;
			}

			float valpha = van;
			float vbeta  = ((1.0f/sqrtf(3.0f)) * van)
						 + ((2.0f/sqrtf(3.0f)) * vbn);

			float cosTheta, sinTheta;



			arm_sin_cos_f32(WrapAngle(foc.theta),&sinTheta,&cosTheta);

			float vd =  valpha * cosTheta + vbeta * sinTheta;
			float vq = -valpha * sinTheta + vbeta * cosTheta;

			foc.Vd = vd;
			foc.Vq = vq;

			piId.accumulator = vd;
			piIq.accumulator = vq;
		}

		GPIO_ResetBits(GPIOB,GPIO_Pin_15);
	}
}

static void FOCTorqueControl(void)
{
	static bool brakingEnabled = false;

	float mechVelocity = MotorPosition_GetMechanicalVelocity();

	foc.theta = WrapAngle(foc.theta);

	if ( swapAAndBPhases )
	{
		foc.Ia =  -analogReadings.Ib;
		foc.Ib =  -analogReadings.Ia;
	}
	else
	{
		foc.Ia =  -analogReadings.Ia;
		foc.Ib =  -analogReadings.Ib;
	}

	foc.Ic =  -analogReadings.Ic;
	foc.Vbus = analogReadings.Vbus;

	if ( cp.drivemode == DRIVEMODE_FORWARD )
	{
		if (mechVelocity > 10.0f )
		{
			brakingEnabled = true;
		}
		else if ( mechVelocity < 2.0f )
		{
			brakingEnabled = false;
		}

		// If we are braking while going forward, stop braking at 2 RPM
		if (!brakingEnabled && cp.iqRef < 0 )
		{
			cp.iqRef = 0;
		}

		// Filter the input command.
		piIq.reference = cp.iqRef;
	}
	else if ( cp.drivemode == DRIVEMODE_REVERSE )
	{
		if (mechVelocity < -10.0f )
		{
			brakingEnabled = true;
		}
		else if ( mechVelocity > -2.0f )
		{
			brakingEnabled = false;
		}

		// If we are braking while going forward, stop braking at 2 RPM
		if (!brakingEnabled && cp.iqRef < 0 )
		{
			cp.iqRef = 0;
		}

		piIq.reference = cp.iqRef * -1.0f ;
	}
	else
	{
		// This should never happen.
		cp.iqRef = 0;
	}

	piId.reference = cp.idRef;

	FOC_SinCos( &foc );
	FOC_Clarke( &foc );
	FOC_Park  ( &foc );

	piId.measured = foc.Id;
	piIq.measured = foc.Iq;

	piIq.outMax = limitf32( foc.Vbus * VQ_LIMIT_PERCENT, cp.vqMax, -200.0f);
	piIq.outMin = limitf32( -piIq.outMax, 200.0f, cp.vqMin);

	float vqn = PI_Control( &piIq );
	foc.Vq = (foc.Vq * (rolloffAveragingPeriod-1.0f) + vqn) / rolloffAveragingPeriod;

	// Find the limit of Vd such that the length of vs does not exceed vbus*VDQ_LIMIT_PERCENT

	float VsLimit = foc.Vbus * VDQ_LIMIT_PERCENT;

	float vdLimit = sqrtf( (VsLimit * VsLimit) - (foc.Vq * foc.Vq) );

	piId.outMax = limitf32( vdLimit, cp.vdMax, -200.0f);
	piId.outMin = limitf32( -piId.outMax, 200.0f, cp.vdMin);

	float vdn = PI_Control( &piId );
	foc.Vd = (foc.Vd * (rolloffAveragingPeriod-1.0f) + vdn) / rolloffAveragingPeriod;


	FOC_InvPark  ( &foc );
	FOC_InvClarke( &foc );
	FOC_SVM      ( &foc );

	float oneOverVbus = 1/analogReadings.Vbus;

	if ( swapAAndBPhases )
	{
		pwm_SetDutyCycles( foc.V2*oneOverVbus, foc.V1*oneOverVbus, foc.V3*oneOverVbus );
	}
	else
	{
		pwm_SetDutyCycles( foc.V1*oneOverVbus, foc.V2*oneOverVbus, foc.V3*oneOverVbus );
	}
}


#define FILTER_LEN	(FILTER_LENGTH_FILTER_TAYLOR_4KHZ_FAST)
#define FILTER	    (filter_taylor_4khz_fast)

static void UpdateAnalogChannels(void)
{
	analogReadings.Ia   = adc_GetReading(ADC_CH_INDEX_IA,    FILTER, FILTER_LEN);
	analogReadings.Ib   = adc_GetReading(ADC_CH_INDEX_IB,    FILTER, FILTER_LEN);
	analogReadings.Ic   = adc_GetReading(ADC_CH_INDEX_IC,    FILTER, FILTER_LEN);
	analogReadings.Va   = adc_GetReading(ADC_CH_INDEX_VA,    FILTER, FILTER_LEN);
	analogReadings.Vb   = adc_GetReading(ADC_CH_INDEX_VB,    FILTER, FILTER_LEN);
	analogReadings.Vc   = adc_GetReading(ADC_CH_INDEX_VC,    FILTER, FILTER_LEN);
	analogReadings.Vbus = adc_GetReading(ADC_CH_INDEX_VBUS,  FILTER, FILTER_LEN);
	analogReadings.Vgate= adc_GetReading(ADC_CH_INDEX_VGATE, FILTER, FILTER_LEN);

	static float currentOffset = 0;

	currentOffset = currentOffset * (CONTROL_LOOP_FREQUENCY_HZ-1)
			+ (analogReadings.Ia + analogReadings.Ia + analogReadings.Ic);

	currentOffset *= (1.0f/CONTROL_LOOP_FREQUENCY_HZ);

	float delta = currentOffset * (1.0f/3.0f);

	analogReadings.Ia -= delta;
	analogReadings.Ib -= delta;
	analogReadings.Ic -= delta;
}

static float CalculateBEMFAngle( float va, float vb )
{
	float valpha = va;
	float vbeta  = ((1.0f/sqrtf(3.0f)) * va) + ((2.0f/sqrtf(3.0f)) * vb);

	float angle = atan2f(vbeta,valpha);

	angle *= (180.0f/3.14159f);

	return angle;
}

static void checkForStateChange( void )
{
	if ( cp.pendingDrivemode != DRIVEMODE_INVALID )
	{
		switch (cp.pendingDrivemode)
		{
			case DRIVEMODE_FORWARD:
			case DRIVEMODE_REVERSE:
				pwm_SetPWMOutputEnable(ENABLE,ENABLE,ENABLE);
				break;
			case DRIVEMODE_DISABLED:
			default:
				pwm_SetPWMOutputEnable(DISABLE,DISABLE,DISABLE);
				break;
		}

		cp.drivemode = cp.pendingDrivemode;
		cp.pendingDrivemode = DRIVEMODE_INVALID;
	}
}

void DetermineMotorOffset( void )
{
	motorOffsetIndex = 0;
	mOff_mechVelocityAccum = 0;
	MotorPosition_SetPositionInversion( false );
	MotorPosition_SetElectricalPositionOffset( 0.0f );
	motorOffsetMode = true;
}


static void LogMotorOffsetDataPoint( void )
{
	//analogReadings.Va   = adc_GetReading(ADC_CH_INDEX_VA, FILTER, FILTER_LEN);
	//analogReadings.Vb   = adc_GetReading(ADC_CH_INDEX_VB, FILTER, FILTER_LEN);
	//analogReadings.Vc   = adc_GetReading(ADC_CH_INDEX_VC, FILTER, FILTER_LEN);

	float n = analogReadings.Va + analogReadings.Vb + analogReadings.Vc;
	n *= (1.0f/3.0f);

	mOff_Va[motorOffsetIndex] = analogReadings.Va - n;
	mOff_Vb[motorOffsetIndex] = analogReadings.Vb - n;

	mOff_MPTheta[motorOffsetIndex] = foc.theta + 90.0f;

	printf("%lu %0.3f %0.3f %0.3f\n",motorOffsetIndex,mOff_Va[motorOffsetIndex],mOff_Vb[motorOffsetIndex],mOff_MPTheta[motorOffsetIndex]);

	mOff_mechVelocityAccum += MotorPosition_GetMechanicalVelocity();

	motorOffsetIndex++;
}

static void DetermineMotorPositionOffsets( void )
{
	float phasingTheta[MOTOR_OFFSET_DELTA_SAMPLE_SIZE];
	float phasingVelocity = 0;

	for ( uint32_t i = 0; i < MOTOR_OFFSET_DELTA_SAMPLE_SIZE; i++ )
	{
		phasingTheta[i] = CalculateBEMFAngle(mOff_Va[i],mOff_Vb[i]);

		if ( i > 0 )
		{
			float delta = phasingTheta[i] - phasingTheta[i-1];

			if ( delta > 0.0f ) {
				if ( delta > 180.0f ) {
					delta -= 360.0f;
				}
			} else {
				if ( delta < -180.0f ) {
					delta += 360.0f;
				}
			}

			if ( fabsf(delta) > 180.0f ) {
				// We have a bad angle, ignore it.
				delta = 0.0f;
			}

			phasingVelocity += delta;
		}
	}

	if ( phasingVelocity < 0.0f )
	{
		// If our phasing velocity is negative, we need to flip two
		// phases to make it positive.  If we do, recalculate the phasing
		// theta so that we can use the correct version to find the offset.

		swapAAndBPhases = true;


		for ( uint32_t i = 0; i < MOTOR_OFFSET_DELTA_SAMPLE_SIZE; i++ )
		{
			phasingTheta[i] = CalculateBEMFAngle(mOff_Vb[i],mOff_Va[i]);
		}
	} else {
		swapAAndBPhases = false;
	}

	d1k_fram_Write(FRAM_ADDRESS_MOTOR_PHASING_SWAPPED,&swapAAndBPhases,sizeof(swapAAndBPhases));

	if ( mOff_mechVelocityAccum < 0 )
	{
		// Our PP sensor is going in reverse.  We need to invert its output.

		MotorPosition_SetPositionInversion( true );

		// Change all of the points collected to represent the new inversion.
		for ( uint32_t i = 0; i < MOTOR_OFFSET_DELTA_SAMPLE_SIZE; i++ )
		{
			mOff_MPTheta[i] = 360 - mOff_MPTheta[i];
		}
	}

	printf("\nmOff_mechVelAccum: %f\n",mOff_mechVelocityAccum);
	printf("phasingVelocity: %f\n",phasingVelocity);

	// Now, we have two sets of angles that both have a positive velocity.
	// The last step is to find the the angle offset.

	float deltaAccumSin = 0.0f;
	float deltaAccumCos = 0.0f;

	for (uint32_t i = 0; i < MOTOR_OFFSET_DELTA_SAMPLE_SIZE; i++)
	{
		float delta = phasingTheta[i] - mOff_MPTheta[i];

		delta = WrapAngle(delta);

		printf("%0.1f\n",delta);

		float s,c;

		arm_sin_cos_f32(delta,&s,&c);

		deltaAccumSin += s;
		deltaAccumCos += c;
	}

	deltaAccumSin *= (1.0f/MOTOR_OFFSET_DELTA_SAMPLE_SIZE);
	deltaAccumCos *= (1.0f/MOTOR_OFFSET_DELTA_SAMPLE_SIZE);

	MotorPosition_SetElectricalPositionOffset( (360.0f/(2.0f*3.141592f)) * atan2f(deltaAccumSin,deltaAccumCos) );

	printf("\nCalibration Complete.\n");
	printf("Swap A and B: %s\n", swapAAndBPhases?"YES":"NO" );
	printf("Motor Position Sensor Inverted: %s\n", MotorPosition_GetPositionInversion()?"YES":"NO" );
	printf("Motor Position Offset: %0.3f\n",MotorPosition_GetElectricalPositionOffset() );

}
