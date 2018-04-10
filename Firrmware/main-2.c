#include "stm32xxx_hal.h"

/* USER CODE BEGIN Includes */
#include "main.h"
#include "vl53l1_api.h"

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart2;
VL53L1_Dev_t                   dev;
VL53L1_DEV                     Dev = &dev;
int status;
volatile int IntCount;

#define isAutonomousExample 1  /* Allow to select either autonomous ranging or fast ranging example */
#define isInterrupt 1 /* If isInterrupt = 1 then device working in interrupt mode, else device working in polling mode */
/* USER CODE END PV */

void AutonomousLowPowerRangingTest(void); /* see Autonomous ranging example implementation in USER CODE BEGIN 4 section */


int main(void)
{
  
  VL53L1_RdByte(Dev, 0x010F, &byteData);
  printf("VL53L1X Model_ID: %02X\n\r", byteData);
  VL53L1_RdByte(Dev, 0x0110, &byteData);
  printf("VL53L1X Module_Type: %02X\n\r", byteData);
  VL53L1_RdWord(Dev, 0x010F, &wordData);
  printf("VL53L1X: %02X\n\r", wordData);
  AutonomousLowPowerRangingTest();
}


void AutonomousLowPowerRangingTest(void)
{
  static VL53L1_RangingMeasurementData_t RangingData;
  printf("Autonomous Ranging Test\n");

  status = VL53L1_WaitDeviceBooted(Dev);

  status = VL53L1_DataInit(Dev);

  status = VL53L1_StaticInit(Dev);
  status = VL53L1_SetDistanceMode(Dev, VL53L1_DISTANCEMODE_LONG);


  status = VL53L1_SetMeasurementTimingBudgetMicroSeconds(Dev, 50000);
  status = VL53L1_SetInterMeasurementPeriodMilliSeconds(Dev, 500);
  status = VL53L1_StartMeasurement(Dev);

	if(status){
		printf("VL53L1_StartMeasurement failed \n");
		while(1);
	}	
	if (isInterrupt){
		do // interrupt mode
		{
		 __WFI();
		 if(IntCount !=0 ){
				IntCount=0;
				status = VL53L1_GetRangingMeasurementData(Dev, &RangingData);
0
lives in _api

			/* Get Ranging Data */
1			Status = VL53L1_get_device_results(
lives in _api_core

2					status = VL53L1_get_measurement_results(
lives in _api_core

3						VL53L1_i2c_decode_system_results(
Lives in _register_funcs.c
This contains what each byte means - our decoder ring!
						result__final_crosstalk_corrected_range_mm_sd0
						looks like byte 14 of the response. Two bytes wide, MSB first (14)

2					VL53L1_copy_sys_and_core_results_to_range_results(
						(int32_t)pdev->gain_cal.standard_ranging_gain_factor,
						&(pdev->sys_results),
						&(pdev->core_results),
						presults);


				if(status==0){
					printf("%d,%d,%.2f,%.2f\n", RangingData.RangeStatus,RangingData.RangeMilliMeter,
									RangingData.SignalRateRtnMegaCps/65536.0,RangingData.AmbientRateRtnMegaCps/65336.0);
				}

5				status = VL53L1_ClearInterruptAndStartMeasurement(Dev);
lives in _api

6					Status = VL53L1_clear_interrupt_and_enable_next_range(
lives in _api_core

7						status = VL53L1_init_and_start_range(
lives in _api_core

						i2c_buffer_size_bytes = \
								(VL53L1_SYSTEM_CONTROL_I2C_INDEX + 
								 VL53L1_SYSTEM_CONTROL_I2C_SIZE_BYTES) -
								 i2c_index;

								 VL53L1_SYSTEM_CONTROL_I2C_INDEX = 131
								 VL53L1_SYSTEM_CONTROL_I2C_SIZE_BYTES = 5
								 i2c_index = 5

								looks like there are 5 bytes for the bare minimum config
								#define VL53L1_DEVICEMEASUREMENTMODE_STOP               0x00
								#define VL53L1_DEVICEMEASUREMENTMODE_SINGLESHOT         0x10
								#define VL53L1_DEVICEMEASUREMENTMODE_BACKTOBACK         0x20
								#define VL53L1_DEVICEMEASUREMENTMODE_TIMED              0x40
								#define VL53L1_DEVICEMEASUREMENTMODE_ABORT              0x80


8							VL53L1_i2c_encode_system_control(
lives in _regsiter_functions

VL53L1_SYSTEM_CONTROL_I2C_SIZE_BYTES = 5

These are the minimum 5 bytes to do system control
								*(pbuffer +   0) =
									pdata->power_management__go1_power_force & 0x1;
								*(pbuffer +   1) =
									pdata->system__stream_count_ctrl & 0x1;
								*(pbuffer +   2) =
									pdata->firmware__enable & 0x1;
								*(pbuffer +   3) =
									pdata->system__interrupt_clear & 0x3;
								*(pbuffer +   4) =
									pdata->system__mode_start;
First write:
0x00
0x00
0x01
0x01
0x40
So the last five bytes of the big writes are always 0x 00 00 01 40. Is this the system control?



				}
		}
		while(1);
	}
	else{
		do // polling mode
		{
		  status = VL53L1_WaitMeasurementDataReady(Dev);
			if(!status)
			{
				status = VL53L1_GetRangingMeasurementData(Dev, &RangingData);
				if(status==0){
					printf("%d,%d,%.2f,%.2f\n", RangingData.RangeStatus,RangingData.RangeMilliMeter,
									(RangingData.SignalRateRtnMegaCps/65536.0),RangingData.AmbientRateRtnMegaCps/65336.0);
				}
				status = VL53L1_ClearInterruptAndStartMeasurement(Dev);
			}
		}
		while (1);
	}
//  return status;
}

