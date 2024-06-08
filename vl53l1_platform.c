
/* 
* This file is part of VL53L1 Platform 
* 
* Copyright (c) 2016, STMicroelectronics - All Rights Reserved 
* 
* License terms: BSD 3-clause "New" or "Revised" License. 
* 
* Redistribution and use in source and binary forms, with or without 
* modification, are permitted provided that the following conditions are met: 
* 
* 1. Redistributions of source code must retain the above copyright notice, this 
* list of conditions and the following disclaimer. 
* 
* 2. Redistributions in binary form must reproduce the above copyright notice, 
* this list of conditions and the following disclaimer in the documentation 
* and/or other materials provided with the distribution. 
* 
* 3. Neither the name of the copyright holder nor the names of its contributors 
* may be used to endorse or promote products derived from this software 
* without specific prior written permission. 
* 
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE 
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. 
* 
*/

#include "vl53l1_platform.h"
#include <string.h>
#include <time.h>
#include <math.h>

#ifdef FREERTOS_ENABLED
#include "freeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#endif

extern I2C_HandleTypeDef VL53L1__PORT;

int8_t VL53L1_WriteMulti( uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count) {
	if (HAL_I2C_Mem_Write(&VL53L1__PORT, dev, index, I2C_MEMADD_SIZE_16BIT, pdata, count, I2C_COMM_TIMEOUT))
		return VL53L1__IO_ERROR;
	else
		return 0;
}

int8_t VL53L1_ReadMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count){
	if (HAL_I2C_Mem_Read(&VL53L1__PORT, dev, index, I2C_MEMADD_SIZE_16BIT, pdata, count, I2C_COMM_TIMEOUT))
		return VL53L1__IO_ERROR;
	else
		return 0;
}

int8_t VL53L1_WrByte(uint16_t dev, uint16_t index, uint8_t data) {
	if (HAL_I2C_Mem_Write(&VL53L1__PORT, dev, index, I2C_MEMADD_SIZE_16BIT, &data, 1, I2C_COMM_TIMEOUT))
		return VL53L1__IO_ERROR;
	else
		return 0;
}

int8_t VL53L1_WrWord(uint16_t dev, uint16_t index, uint16_t data) {
	data=__REVSH(data);
	if (HAL_I2C_Mem_Write(&VL53L1__PORT, dev, index, I2C_MEMADD_SIZE_16BIT, (uint8_t *)&data, 2, I2C_COMM_TIMEOUT))
		return VL53L1__IO_ERROR;
	else
		return 0;
}

int8_t VL53L1_WrDWord(uint16_t dev, uint16_t index, uint32_t data) {
	data=__REV(data);
	if (HAL_I2C_Mem_Write(&VL53L1__PORT, dev, index, I2C_MEMADD_SIZE_16BIT, (uint8_t *)&data, 4, I2C_COMM_TIMEOUT))
		return VL53L1__IO_ERROR;
	else
		return 0;
}

int8_t VL53L1_RdByte(uint16_t dev, uint16_t index, uint8_t *data) {
	if (HAL_I2C_Mem_Read(&VL53L1__PORT, dev, index, I2C_MEMADD_SIZE_16BIT, data, 1, I2C_COMM_TIMEOUT))
		return VL53L1__IO_ERROR;
	else
		return 0;
}

int8_t VL53L1_RdWord(uint16_t dev, uint16_t index, uint16_t *data) {
	if (HAL_I2C_Mem_Read(&VL53L1__PORT, dev, index, I2C_MEMADD_SIZE_16BIT, (uint8_t *)data, 2, I2C_COMM_TIMEOUT))
		return VL53L1__IO_ERROR;
	else {
		*data=__REVSH(*data);
		return 0;
	}
}

int8_t VL53L1_RdDWord(uint16_t dev, uint16_t index, uint32_t *data) {
	if (HAL_I2C_Mem_Read(&VL53L1__PORT, dev, index, I2C_MEMADD_SIZE_16BIT, (uint8_t *)data, 4, I2C_COMM_TIMEOUT))
		return VL53L1__IO_ERROR;
	else {
		*data=__REV(*data);
		return 0;
	}
}

/*int8_t VL53L1_WaitMs(uint16_t dev, int32_t wait_ms){
	return 0; // to be implemented
}*/

uint8_t TOF__Init(){
	uint8_t refRegs[4] = {0,0,0,0};
	uint8_t status = 0;

	// Enable VL53L1 sensor waiting for a complete boot sequence
#ifdef	VL53L1__USING_XSHUT
	status |= VL53L1__Xshut(1);
	if (status) {
		printf("ERROR: Returned status. Smth wrong in XSHUT\r\n");
		return (status);
	}
#endif

#ifdef FREERTOS__ENABLED
	osDelay(4);
#else
	HAL_Delay(4);
#endif

	// Check if VL53L1X is alive and kicking. Remove MASKREV if VL53L1
	VL53L1_ReadMulti(VL53L1__ADDR, VL53L1__MODELID_INDEX, refRegs, 4);
	if ((refRegs[0]!=VL53L1__MODELID_VALUE) || (refRegs[1]!=VL53L1__MODULETYPE_VALUE) || (refRegs[2]!=VL53L1__MASKREV_VALUE)) {
		printf("ERROR: VL53L1X is not alive\r\n");
		return (1);
	}

	// VL53L1X sensor is available
	/* initializing: default setting  */
	status |= VL53L1X_SensorInit(VL53L1__ADDR);
	/* initializing: device calibration settings*/
	status |= VL53L1X_SetOffset(VL53L1__ADDR, VL53L1__CALIB_OFFSET);
	status |= VL53L1X_SetXtalk(VL53L1__ADDR, VL53L1__CALIB_XTALK);
	/* initializing: project settings */
	status |= VL53L1X_SetDistanceMode(VL53L1__ADDR, VL53L1__DISTANCE_MODE);
	status |= VL53L1X_SetTimingBudgetInMs(VL53L1__ADDR, VL53L1__TIMING_BUDGET);
	status |= VL53L1X_SetInterMeasurementInMs(VL53L1__ADDR, VL53L1__INTERMEASUREMENT);
	status |= VL53L1X_SetDistanceThreshold(VL53L1__ADDR,VL53L1__LOWER_THRESHOLD, VL53L1__UPPER_THRESHOLD, VL53L1__WINDOW_MODE, 0);

	//printf("Returned status. Status = %d\r\n", status);
	return (status);
};

uint8_t TOF__GetDistance(uint16_t *Distance){
	uint8_t RangingStatus;
	uint8_t status =0;
	uint32_t testingTime=HAL_GetTick();
	static uint16_t PrevDistance=0;

#ifdef	VL53L1__USING_GPIO
	// VL53L1X data available test if TOF_GPIO pin is available
	while ((!HAL_GPIO_ReadPin(TOF_GPIO_GPIO_Port, TOF_GPIO_Pin)) && ((HAL_GetTick()-testingTime)<=VL53L1__INTERMEASUREMENT)) {};
	if (HAL_GPIO_ReadPin(TOF_GPIO_GPIO_Port, TOF_GPIO_Pin)) {
#else
	// VL53L1X data available test if TOF_GPIO pin is available
	uint8_t dataReady=0;
	while ((dataReady == 0) && ((HAL_GetTick()-testingTime)<VL53L1__INTERMEASUREMENT) && (!status))
		status |= VL53L1X_CheckForDataReady(VL53L1__ADDR, &dataReady);
	if (dataReady && (!status)) {
#endif
		status |= VL53L1X_GetRangeStatus(VL53L1__ADDR, &RangingStatus);
		status |= VL53L1X_GetDistance(VL53L1__ADDR, Distance);
		status |= VL53L1X_ClearInterrupt(VL53L1__ADDR);
		if ((status==0) && (RangingStatus<=VL53L1__RANGE_STATUS_THRESH)) {
			PrevDistance=*Distance;
		} else {
			*Distance=PrevDistance;
			status=1;
		}
	} else{
		*Distance=PrevDistance;
		status=1;
	}
	return status;
}

uint8_t VL53L1__SetTimingBudget(uint16_t levelTB,uint16_t levelIM){
	uint8_t status=0;
	//	status |= VL53L1X_StopRanging(VL53L1__ADDR);
	status |= VL53L1X_SetTimingBudgetInMs(VL53L1__ADDR, levelTB);
	//	status |= VL53L1X_SetInterMeasurementInMs(VL53L1__ADDR, (level+VL53L1__TB_IM_DELTA));
	status |= VL53L1X_SetInterMeasurementInMs(VL53L1__ADDR, (levelIM));
	//	status |= VL53L1X_StartRanging(VL53L1__ADDR);
	return status;
}

uint8_t VL53L1__SetDistanceMode(uint16_t level){
	uint8_t status=0;
//	status |= VL53L1X_StopRanging(VL53L1__ADDR);
	status |= VL53L1X_SetDistanceMode(VL53L1__ADDR, level);
//	status |= VL53L1X_StartRanging(VL53L1__ADDR);
	return status;
}
