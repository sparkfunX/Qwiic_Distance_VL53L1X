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
				if(status==0){
					printf("%d,%d,%.2f,%.2f\n", RangingData.RangeStatus,RangingData.RangeMilliMeter,
									RangingData.SignalRateRtnMegaCps/65536.0,RangingData.AmbientRateRtnMegaCps/65336.0);
				}
				status = VL53L1_ClearInterruptAndStartMeasurement(Dev);
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

