/* Copyright (c) 2009 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

#ifndef MPU6050_H
#define MPU6050_H

/*lint ++flb "Enter library region" */

#include <stdbool.h>
#include <stdint.h>
#include "inv_mpu.h"

#define   mpu6050_device_address 0x68;          // !< Device address in bits [7:1]
typedef uint32_t  u32;
typedef uint16_t u16;
typedef uint8_t  u8;

typedef struct {
  int referen_value;
  int current_value;
  uint8_t read_type;
  uint8_t open_mpu_flag;
}mpu_t;


#define MPU6050_SDA_PIN   	8 //current
#define MPU6050_SCL_PIN   	9
#define MPU6050_INT_PIN     10
#define MPU6050_POWER_PIN   18
/*
#define MPU6050_SDA_PIN   	2//debug
#define MPU6050_SCL_PIN   	3
#define MPU6050_INT_PIN     9
#define MPU6050_POWER_PIN   10
*/
bool mpu6050_write(uint8_t device_address, uint8_t register_address, uint8_t 
len, uint8_t *data);
bool mpu6050_read(uint8_t device_address, uint8_t register_address, uint8_t 
number_of_bytes, uint8_t *destination);
/*lint --flb "Leave library region" */


/*******************************
*SCL--P09
*SDA--P08
*INT--P10
*PWR--P18
*******************************/
#define MPU_SELF_TESTX_REG		0X0D	//��??��??��??��X
#define MPU_SELF_TESTY_REG		0X0E	//��??��??��??��Y
#define MPU_SELF_TESTZ_REG		0X0F	//��??��??��??��Z
#define MPU_SELF_TESTA_REG		0X10	//��??��??��??��A
#define MPU_SAMPLE_RATE_REG		0X19	//2��?��?��?����??��?��
#define MPU_CFG_REG				0X1A	//??????��??��
#define MPU_GYRO_CFG_REG		0X1B	//����?Y��???????��??��
#define MPU_ACCEL_CFG_REG		0X1C	//?��?��?��????????��??��
#define MPU_MOTION_DET_REG		0X1F	//???��?��2a����?�̨���????��??��
#define MPU_FIFO_EN_REG			0X23	//FIFO��1?��??��??��
#define MPU_I2CMST_CTRL_REG		0X24	//IIC?��?��??????��??��
#define MPU_I2CSLV0_ADDR_REG	0X25	//IIC�䨮?��0?��?t��??��??��??��
#define MPU_I2CSLV0_REG			0X26	//IIC�䨮?��0��y?Y��??��??��??��
#define MPU_I2CSLV0_CTRL_REG	0X27	//IIC�䨮?��0??????��??��
#define MPU_I2CSLV1_ADDR_REG	0X28	//IIC�䨮?��1?��?t��??��??��??��
#define MPU_I2CSLV1_REG			0X29	//IIC�䨮?��1��y?Y��??��??��??��
#define MPU_I2CSLV1_CTRL_REG	0X2A	//IIC�䨮?��1??????��??��
#define MPU_I2CSLV2_ADDR_REG	0X2B	//IIC�䨮?��2?��?t��??��??��??��
#define MPU_I2CSLV2_REG			0X2C	//IIC�䨮?��2��y?Y��??��??��??��
#define MPU_I2CSLV2_CTRL_REG	0X2D	//IIC�䨮?��2??????��??��
#define MPU_I2CSLV3_ADDR_REG	0X2E	//IIC�䨮?��3?��?t��??��??��??��
#define MPU_I2CSLV3_REG			0X2F	//IIC�䨮?��3��y?Y��??��??��??��
#define MPU_I2CSLV3_CTRL_REG	0X30	//IIC�䨮?��3??????��??��
#define MPU_I2CSLV4_ADDR_REG	0X31	//IIC�䨮?��4?��?t��??��??��??��
#define MPU_I2CSLV4_REG			0X32	//IIC�䨮?��4��y?Y��??��??��??��
#define MPU_I2CSLV4_DO_REG		0X33	//IIC�䨮?��4D�䨺y?Y??��??��
#define MPU_I2CSLV4_CTRL_REG	0X34	//IIC�䨮?��4??????��??��
#define MPU_I2CSLV4_DI_REG		0X35	//IIC�䨮?��4?����y?Y??��??��

#define MPU_I2CMST_STA_REG		0X36	//IIC?��?�����䨬???��??��
#define MPU_INTBP_CFG_REG		0X37	//?D??/???������????��??��
#define MPU_INT_EN_REG			0X38	//?D??��1?��??��??��
#define MPU_INT_STA_REG			0X3A	//?D??���䨬???��??��

#define MPU_ACCEL_XOUTH_REG		0X3B	//?��?��?��?��,X?��??8????��??��
#define MPU_ACCEL_XOUTL_REG		0X3C	//?��?��?��?��,X?���̨�8????��??��
#define MPU_ACCEL_YOUTH_REG		0X3D	//?��?��?��?��,Y?��??8????��??��
#define MPU_ACCEL_YOUTL_REG		0X3E	//?��?��?��?��,Y?���̨�8????��??��
#define MPU_ACCEL_ZOUTH_REG		0X3F	//?��?��?��?��,Z?��??8????��??��
#define MPU_ACCEL_ZOUTL_REG		0X40	//?��?��?��?��,Z?���̨�8????��??��

#define MPU_TEMP_OUTH_REG		0X41	//???��?��??��?????��??��
#define MPU_TEMP_OUTL_REG		0X42	//???��?�̨̦�8????��??��

#define MPU_GYRO_XOUTH_REG		0X43	//����?Y��??��,X?��??8????��??��
#define MPU_GYRO_XOUTL_REG		0X44	//����?Y��??��,X?���̨�8????��??��
#define MPU_GYRO_YOUTH_REG		0X45	//����?Y��??��,Y?��??8????��??��
#define MPU_GYRO_YOUTL_REG		0X46	//����?Y��??��,Y?���̨�8????��??��
#define MPU_GYRO_ZOUTH_REG		0X47	//����?Y��??��,Z?��??8????��??��
#define MPU_GYRO_ZOUTL_REG		0X48	//����?Y��??��,Z?���̨�8????��??��

#define MPU_I2CSLV0_DO_REG		0X63	//IIC�䨮?��0��y?Y??��??��
#define MPU_I2CSLV1_DO_REG		0X64	//IIC�䨮?��1��y?Y??��??��
#define MPU_I2CSLV2_DO_REG		0X65	//IIC�䨮?��2��y?Y??��??��
#define MPU_I2CSLV3_DO_REG		0X66	//IIC�䨮?��3��y?Y??��??��

#define MPU_I2CMST_DELAY_REG	0X67	//IIC?��?��?������1������??��??��
#define MPU_SIGPATH_RST_REG		0X68	//D?o?�����̨�?��????��??��
#define MPU_MDETECT_CTRL_REG	0X69	//???��?��2a??????��??��
#define MPU_USER_CTRL_REG		0X6A	//��??��??????��??��
#define MPU_PWR_MGMT1_REG		0X6B	//��??��1������??��??��1
#define MPU_PWR_MGMT2_REG		0X6C	//��??��1������??��??��2 
#define MPU_FIFO_CNTH_REG		0X72	//FIFO??��y??��??��??��???
#define MPU_FIFO_CNTL_REG		0X73	//FIFO??��y??��??�¦̨���???
#define MPU_FIFO_RW_REG			0X74	//FIFO?��D��??��??��
#define MPU_DEVICE_ID_REG		0X75	//?��?tID??��??��
/**
 * @brief Function for initializing MPU6050 and verifies it's on the bus.
 *
 * @param device_address Device TWI address in bits [6:0].
 * @return
 * @retval true MPU6050 found on the bus and ready for operation.
 * @retval false MPU6050 not found on the bus or communication failure.
 */
bool mpu6050_init(uint8_t device_address);

/**
  @brief Function for writing a MPU6050 register contents over TWI.
  @param[in]  register_address Register address to start writing to
  @param[in] value Value to write to register
  @retval true Register write succeeded
  @retval false Register write failed
*/
//bool mpu6050_register_write(uint8_t register_address, const uint8_t value);

/**
  @brief Function for reading MPU6050 register contents over TWI.
  Reads one or more consecutive registers.
  @param[in]  register_address Register address to start reading from
  @param[in]  number_of_bytes Number of bytes to read
  @param[out] destination Pointer to a data buffer where read data will be 
stored
  @retval true Register read succeeded
  @retval false Register read failed
*/
//bool mpu6050_register_read(uint8_t register_address, uint8_t *destination, 
//uint8_t number_of_bytes);

/**
  @brief Function for reading and verifying MPU6050 product ID.
  @retval true Product ID is what was expected
  @retval false Product ID was not what was expected
*/
bool mpu6050_verify_product_id(void);
uint8_t MPU_Get_Accelerometer(short *ax,short *ay,short *az);
uint8_t MPU_Get_Gyroscope(short *gx,short *gy,short *gz);

uint8_t lock_read_yaw(uint8_t time,float *value);
uint8_t compare_mpu6050_data(int16_t referen,int16_t input);
//void mpu6050_io_init(void);
void mpu6050_gpio_init(void);
void mpu6050_power_on(void);
void mpu6050_power_off(void);
void MX_I2C1_Init(void);
uint8_t mpu6050_power_on_init(void);
#endif /* MPU6050_H */

