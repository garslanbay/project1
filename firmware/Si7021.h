#ifndef __SI_7021_H
#define __SI_7021_H

#include "stm32f0xx.h"
#include "main.h"

/** I2C device address for Si7021 */
#define SI7021_ADDR      0x80


/** Device ID value for Si7013 */
#define SI7013_DEVICE_ID 0x0D
/** Device ID value for Si7020 */
#define SI7020_DEVICE_ID 0x14
/** Device ID value for Si7021 */
#define SI7021_DEVICE_ID 0x21

/** Si7013 Read Temperature Command */
#define SI7013_READ_TEMP       0xE0  /* Read previous T data from RH measurement
                                      * command*/
/** Si7013 Read RH Command */
#define SI7013_READ_RH         0xE5  /* Perform RH (and T) measurement. */
/** Si7013 Read RH (no hold) Command */
#define SI7013_READ_RH_NH      0xF5  /* Perform RH (and T) measurement in no hold mode. */
/** Si7013 Read Thermistor Command */
#define SI7013_READ_VIN        0xEE  /* Perform thermistor measurement. */
/** Si7013 Read ID */
#define SI7013_READ_ID1_1      0xFA
#define SI7013_READ_ID1_2      0x0F
#define SI7013_READ_ID2_1      0xFc
#define SI7013_READ_ID2_2      0xc9
/** Si7013 Read Firmware Revision */
#define SI7013_READ_FWREV_1    0x84
#define SI7013_READ_FWREV_2    0xB8



/*******************************************************************************
 *****************************   PROTOTYPES   **********************************
 ******************************************************************************/

int32_t Si7013_MeasureRHAndTemp(I2C_TypeDef *i2c, uint8_t addr,
                                 uint32_t *rhData, int32_t *tData);

int32_t Si7013_GetFirmwareRevision(uint8_t addr, uint8_t *fwRev);


void Si7013_ReadNoHoldRHAndTemp(uint32_t *rhData,int32_t *tData);
int32_t Si7013_StartNoHoldMeasureRHAndTemp(I2C_TypeDef *i2c, uint8_t addr);
int32_t Si7013_MeasureV(I2C_TypeDef *i2c, uint8_t addr, int32_t *vData);
void I2C_Read(int8_t Reg, uint8_t *Data, uint8_t DCnt,uint32_t *hum);


#endif

