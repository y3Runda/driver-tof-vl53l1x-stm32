/**
 * @file  vl53l1_platform.h
 * @brief Those platform functions are platform dependent and have to be implemented by the user
 */
 
#ifndef _VL53L1_PLATFORM_H_
#define _VL53L1_PLATFORM_H_

#include "vl53l1_types.h"
#include "main.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define VL53L1__PORT hi2c1
#define VL53L1__ADDR 0x52

//#define VL53L1__USING_XSHUT
#define VL53L1__USING_GPIO

#define VL53L1__DISTANCE_MODE 		(2) // 1 = short, 2 = long
#define VL53L1__TIMING_BUDGET 	    (20) // in ms
#define VL53L1__TB_IM_DELTA 		(5) // in ms
#define VL53L1__INTERMEASUREMENT 	(VL53L1__TIMING_BUDGET + VL53L1__TB_IM_DELTA) // in ms

#define VL53L1__WINDOW_MODE			(0)	// 0=below, 1=beyond, 2=out of window, 3=inside window
#define VL53L1__LOWER_THRESHOLD		(5000) // (mm) lower window limit. For WINDOW_MODE = 0, 2, 3
#define VL53L1__UPPER_THRESHOLD		(5000) // (mm) upper window limit. For WINDOW_MODE = 1, 2, 3

#define VL53L1__CALIB_OFFSET		(-25) //It must must be detected "una tantum" using VL53L1X_CalibrateOffset()
#define VL53L1__CALIB_XTALK			(0) //It must must be detected "una tantum" using VL53L1X_CalibrateXtalk()

#define VL53L1__RANGE_STATUS_THRESH	(2)		// acceptable values: 0,1,2,4,7

#define VL53L1__MODELID_INDEX		0x010F
#define VL53L1__MODELID_VALUE		0xEA
#define VL53L1__MODULETYPE_INDEX	0x0110
#define VL53L1__MODULETYPE_VALUE	0xCC
#define VL53L1__MASKREV_INDEX		0x0111
#define VL53L1__MASKREV_VALUE		0x10

#define I2C_COMM_TIMEOUT			20 // in ms

#define VL53L1__IO_ERROR			( - 13) // error code returned by I/O interface functions

typedef struct {
	uint32_t dummy;
} VL53L1_Dev_t;

typedef VL53L1_Dev_t *VL53L1_DEV;

uint8_t TOF__Init();
uint8_t TOF__GetDistance(uint16_t *Distance);
uint8_t TOF__SetTimingBudget(uint16_t levelTB,uint16_t levelIM);
uint8_t TOF__SetDistanceMode(uint16_t level);

/** @brief VL53L1_WriteMulti() definition.\n
 * To be implemented by the developer
 */
int8_t VL53L1_WriteMulti(
		uint16_t 			dev,
		uint16_t      index,
		uint8_t      *pdata,
		uint32_t      count);
/** @brief VL53L1_ReadMulti() definition.\n
 * To be implemented by the developer
 */
int8_t VL53L1_ReadMulti(
		uint16_t 			dev,
		uint16_t      index,
		uint8_t      *pdata,
		uint32_t      count);
/** @brief VL53L1_WrByte() definition.\n
 * To be implemented by the developer
 */
int8_t VL53L1_WrByte(
		uint16_t dev,
		uint16_t      index,
		uint8_t       data);
/** @brief VL53L1_WrWord() definition.\n
 * To be implemented by the developer
 */
int8_t VL53L1_WrWord(
		uint16_t dev,
		uint16_t      index,
		uint16_t      data);
/** @brief VL53L1_WrDWord() definition.\n
 * To be implemented by the developer
 */
int8_t VL53L1_WrDWord(
		uint16_t dev,
		uint16_t      index,
		uint32_t      data);
/** @brief VL53L1_RdByte() definition.\n
 * To be implemented by the developer
 */
int8_t VL53L1_RdByte(
		uint16_t dev,
		uint16_t      index,
		uint8_t      *pdata);
/** @brief VL53L1_RdWord() definition.\n
 * To be implemented by the developer
 */
int8_t VL53L1_RdWord(
		uint16_t dev,
		uint16_t      index,
		uint16_t     *pdata);
/** @brief VL53L1_RdDWord() definition.\n
 * To be implemented by the developer
 */
int8_t VL53L1_RdDWord(
		uint16_t dev,
		uint16_t      index,
		uint32_t     *pdata);
/** @brief VL53L1_WaitMs() definition.\n
 * To be implemented by the developer
 */
int8_t VL53L1_WaitMs(
		uint16_t dev,
		int32_t       wait_ms);

#ifdef __cplusplus
}
#endif

#endif
