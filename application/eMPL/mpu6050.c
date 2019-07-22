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

#include <stdbool.h>
#include <stdint.h>
//
#include "stdlib.h"
#include "stm32f0xx_hal.h"
#include "mpu6050.h"
#include "hal_board.h"
#include "lock_config.h"

#define TWI_READ_BIT              (0x01)        //!< If this bit is set in the address field, transfer direction is from slave to master.
#define TWI_WRITE_BIT			  (0x00)	
/*lint ++flb "Enter library region" */

#define ADDRESS_WHO_AM_I          (0x75U) // !< WHO_AM_I register identifies the device. Expected value is 0x68.
#define ADDRESS_SIGNAL_PATH_RESET (0x68U) // !<
#define I2C1_WRITE_ADDRESS 0xD0
#define I2C1_READ_ADDRESS  0xD1


#define MPU_PWR_Pin			GPIO_PIN_0
#define MPU_PWR_GPIO_Port   GPIOF

static uint8_t   m_device_address = 0xD0; // 0xD0 = 0x68<<1
I2C_HandleTypeDef hi2c1;
/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
void MX_I2C1_Init(void)
{
    
    /* USER CODE BEGIN I2C1_Init 0 */

    /* USER CODE END I2C1_Init 0 */

    /* USER CODE BEGIN I2C1_Init 1 */

    /* USER CODE END I2C1_Init 1 */
    hi2c1.Instance = I2C1;
    hi2c1.Init.Timing = 0x00201D2B;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    {
        //Error_Handler();
    }
    /** Configure Analogue filter
    */
    if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
    {
       // Error_Handler();
    }
    /** Configure Digital filter
    */
    if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
    {
       // Error_Handler();
    }
    /* USER CODE BEGIN I2C1_Init 2 */

    /* USER CODE END I2C1_Init 2 */
}

/**
* @brief I2C MSP Initialization
* This function configures the hardware resources used in this example
* @param hi2c: I2C handle pointer
* @retval None
*/
void HAL_I2C_MspInit(I2C_HandleTypeDef* hi2c)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hi2c->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspInit 0 */

  /* USER CODE END I2C1_MspInit 0 */
  
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**I2C1 GPIO Configuration    
    PA9     ------> I2C1_SCL
    PA10     ------> I2C1_SDA 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Peripheral clock enable */
    __HAL_RCC_I2C1_CLK_ENABLE();
  /* USER CODE BEGIN I2C1_MspInit 1 */

  /* USER CODE END I2C1_MspInit 1 */
  }
}

/**
* @brief I2C MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hi2c: I2C handle pointer
* @retval None
*/
void HAL_I2C_MspDeInit(I2C_HandleTypeDef* hi2c)
{
  if(hi2c->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspDeInit 0 */

  /* USER CODE END I2C1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C1_CLK_DISABLE();
  
    /**I2C1 GPIO Configuration    
    PA9     ------> I2C1_SCL
    PA10     ------> I2C1_SDA 
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

  /* USER CODE BEGIN I2C1_MspDeInit 1 */

  /* USER CODE END I2C1_MspDeInit 1 */
  }

}

// true --> success
bool mpu6050_write(uint8_t device_address, uint8_t register_address, uint8_t len, uint8_t *data)
{
	if(HAL_I2C_Mem_Write(&hi2c1, ((device_address<<1)|0), register_address, 1, data, len, 100)!= HAL_OK)
	{
		return 0;	
	}
	return 1;
}

// true --> success
bool mpu6050_read(uint8_t device_address, uint8_t register_address, uint8_t number_of_bytes, uint8_t *destination)
{
	if(HAL_I2C_Mem_Read(&hi2c1, ((device_address<<1)|1), register_address, 1, destination, number_of_bytes, 100)!= HAL_OK)
	{
		return 0;
	}
	return 1;
}


uint8_t MPU_Write_Byte(uint8_t reg,uint8_t data)
{
    if(mpu6050_write(0x68,reg,1,&data)) 
	{
        return 0;
    }
    return 1;
	
}

uint8_t MPU_Read_Byte(uint8_t reg)
{
    uint8_t res;
    mpu6050_read(0x68, reg, 1, &res);
    return res;
}
//设置MPU6050陀螺仪传感器满量程范围
//fsr:0,±250dps;1,±500dps;2,±1000dps;3,±2000dps
//返回值:0,设置成功
//    其他,设置失败
u8 MPU_Set_Gyro_Fsr(u8 fsr)
{
    return MPU_Write_Byte(MPU_GYRO_CFG_REG,fsr<<3);//éè??íó?Yò??úá?3ì·??§
}
//设置MPU6050加速度传感器满量程范围
//fsr:0,±2g;1,±4g;2,±8g;3,±16g
//返回值:0,设置成功
//    其他,设置失败
u8 MPU_Set_Accel_Fsr(u8 fsr)
{
    return MPU_Write_Byte(MPU_ACCEL_CFG_REG,fsr<<3);//éè???ó?ù?è′??D?÷?úá?3ì·??§
}
//设置MPU6050的数字低通滤波器
//lpf:数字低通滤波频率(Hz)
//返回值:0,设置成功
//    其他,设置失败
u8 MPU_Set_LPF(u16 lpf)
{
    u8 data=0;
    if(lpf>=188)data=1;
    else if(lpf>=98)data=2;
    else if(lpf>=42)data=3;
    else if(lpf>=20)data=4;
    else if(lpf>=10)data=5;
    else data = 6;
    return MPU_Write_Byte(MPU_CFG_REG,data);//éè??êy×?μíí¨??2¨?÷
}
//设置MPU6050的采样率(假定Fs=1KHz)
//rate:4~1000(Hz)
//返回值:0,设置成功
//    其他,设置失败
u8 MPU_Set_Rate(u16 rate)
{
    u8 data;
    if(rate>1000) rate = 1000;
    if(rate<4)rate = 4;
    data = 1000/rate-1;
    data=MPU_Write_Byte(MPU_SAMPLE_RATE_REG,data);	//éè??êy×?μíí¨??2¨?÷
    return MPU_Set_LPF(rate/2);	//×??ˉéè??LPF?a2é?ù?êμ?ò?°?
}
/*lint --flb "Leave library region" */

// !< Device address in bits [7:1]

bool mpu6050_init(uint8_t device_address)
{
    bool transfer_succeeded = true;

    m_device_address = (uint8_t)(device_address << 1);

    // Do a reset on signal paths
    uint8_t reset_value = 0x04U | 0x02U | 0x01U; // Resets gyro, accelerometer and temperature sensor signal paths.
    transfer_succeeded &= mpu6050_write(0x68,ADDRESS_SIGNAL_PATH_RESET,1,&reset_value);
	halTimerDelay(200);
	
    MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X80);	//reset MPU6050
    halTimerDelay(100);

    MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X00);	//wakeup MPU6050
    MPU_Set_Gyro_Fsr(3);					//+-2000dps
    MPU_Set_Accel_Fsr(0);					//g +-2g
    MPU_Set_Rate(50);						//sample rate 50Hz
    MPU_Write_Byte(MPU_INT_EN_REG,0X00);	//disable interupt
    MPU_Write_Byte(MPU_USER_CTRL_REG,0X00);	//I2C master turn off
    MPU_Write_Byte(MPU_FIFO_EN_REG,0X00);	//FIFO turn off
    MPU_Write_Byte(MPU_INTBP_CFG_REG,0X80);	//INT active low

    // Read and verify product ID
    transfer_succeeded &= mpu6050_verify_product_id();

    return transfer_succeeded;
}

bool mpu6050_verify_product_id(void)
{
    uint8_t who_am_i = 0;
    /*
        if ( mpu6050_read(0x68,ADDRESS_WHO_AM_I,1, &who_am_i))
        {
            if (who_am_i != 0x68u)
            {
                return false;
            }else{
                return true;
            }
        }
        else
        {
            return false;
        }
    */
    who_am_i = MPU_Read_Byte(MPU_DEVICE_ID_REG);
    if(who_am_i == 0x68) {
        return true;
    }
    return false;
}

bool mpu6050_register_write(uint8_t register_address, uint8_t value)
{
//    uint8_t w2_data[2];

//    w2_data[0] = register_address;
//    w2_data[1] = value;
//    return twi_master_transfer(m_device_address, w2_data, 2, TWI_ISSUE_STOP);
	
	return mpu6050_write(0x68, register_address, 1, &value);
}

bool mpu6050_register_read(uint8_t register_address, uint8_t * destination, uint8_t number_of_bytes)
{
//    bool transfer_succeeded;
//    transfer_succeeded  = twi_master_transfer(m_device_address, &register_address, 1, TWI_DONT_ISSUE_STOP);
//    transfer_succeeded &= twi_master_transfer(m_device_address|TWI_READ_BIT, destination, number_of_bytes, TWI_ISSUE_STOP);
//    return transfer_succeeded;
	return mpu6050_read(0x68, register_address, number_of_bytes, destination);
}


uint8_t MPU_Get_Accelerometer(short *ax,short *ay,short *az)
{
    uint8_t buf[6];
    bool res;
    //res=MPU_Read_Len(MPU_ADDR,MPU_ACCEL_XOUTH_REG,6,buf);
    res = mpu6050_register_read(MPU_ACCEL_XOUTH_REG,buf,6);
    if(res == 1)
    {
        *ax=((u16)buf[0]<<8)|buf[1];
        *ay=((u16)buf[2]<<8)|buf[3];
        *az=((u16)buf[4]<<8)|buf[5];
    }
    return res;
}

uint8_t MPU_Get_Gyroscope(short *gx,short *gy,short *gz)
{
    uint8_t buf[6],res;
    res = mpu6050_register_read(MPU_GYRO_XOUTH_REG,buf,6);
    if(res == 1)
    {
        *gx=((u16)buf[0]<<8)|buf[1];
        *gy=((u16)buf[2]<<8)|buf[3];
        *gz=((u16)buf[4]<<8)|buf[5];
    }
    return res;
}


//mpu_dmp_get_data(&pitch,&roll,&yaw) == 0
uint8_t lock_read_yaw(uint8_t time,float *value)
{
#if 0

    uint8_t count = 0,read_success_cnt = 0;
    float temp = 0;
    float yaw_val = 0,yaw_sum = 0;
    for( uint8_t i = 0; i < time+1; i ++ )
    {
        for(count=0; count<2; count++)
        {
            if(mpu_dmp_get_data(&temp,&temp,&yaw_val)==0)
            {
                read_success_cnt++;
                if(read_success_cnt > 1)
#if 1
                    yaw_sum += yaw_val;
#else
                    *value = yaw_val;
                return 1;
#endif
            }
            halTimerDelay(20);
        }
    }
    if(read_success_cnt > 1)
    {
        *value = yaw_sum/(read_success_cnt-1);
        return 1;
    }
    else
    {
        return 0;
    }

#else
    uint8_t read_success_cnt = 0;
    float temp = 0;
    float yaw_val = 0,yaw_sum = 0;
   // mpu6050_set_iic_io();
    for( uint8_t i = 0; i < time+3; i ++ )
    {
        if(mpu_dmp_get_data(&temp,&temp,&yaw_val)==0)
        {
            read_success_cnt++;
            //if(read_success_cnt > 1)
            yaw_sum += yaw_val;
            if(read_success_cnt >= time)
                break;
        }
        //nrf_delay_ms(20);
		halTimerDelay(20);
    }
    if(read_success_cnt > 0)
    {
        *value = yaw_sum/(read_success_cnt);
        return 1;
    }
    else
    {
        return 0;
    }

#endif
}


uint8_t compare_mpu6050_data(int16_t referen,int16_t input)
{
    uint8_t res = 0;
    int16_t err;
    err = abs(referen-input);
   // NRF_LOG_INFO("mpu err %d",err);
    if(err <= AUTO_LOCK_ANGLE)
    {
        res = 1;
    }
    return res;
}

//void mpu6050_io_init(void)
//{
//    nrf_gpio_cfg_output(MPU6050_POWER_PIN);
//    nrf_gpio_pin_set(MPU6050_POWER_PIN);

//    /*
//    nrf_gpio_cfg_output(MPU6050_SDA_PIN);
//    nrf_gpio_pin_set(MPU6050_SDA_PIN);

//    nrf_gpio_cfg_output(MPU6050_SCL_PIN);
//    nrf_gpio_pin_set(MPU6050_SCL_PIN);
//    */
//    nrf_gpio_cfg_input(MPU6050_SDA_PIN,NRF_GPIO_PIN_NOPULL);
//    nrf_gpio_cfg_input(MPU6050_SCL_PIN,NRF_GPIO_PIN_NOPULL);

//    nrf_gpio_cfg_input(MPU6050_INT_PIN, NRF_GPIO_PIN_NOPULL );
//}
void mpu6050_gpio_init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();
	
	/*Configure GPIO pins : PA9 PA10 */
	GPIO_InitStruct.Pin  = GPIO_PIN_9|GPIO_PIN_10;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	/*Configure GPIO pins : PF0*/
	GPIO_InitStruct.Pin  = MPU_PWR_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(MPU_PWR_GPIO_Port, &GPIO_InitStruct);
	
	HAL_GPIO_WritePin(MPU_PWR_GPIO_Port, MPU_PWR_Pin,   GPIO_PIN_SET);		
}


void mpu6050_power_on(void)
{

    HAL_GPIO_WritePin(MPU_PWR_GPIO_Port, MPU_PWR_Pin, GPIO_PIN_RESET);	;//turn on mos    
	MX_I2C1_Init();
//halTimerDelay(20);
}

uint8_t mpu6050_power_on_init(void)
{
    uint8_t i = 10;
    uint8_t res = 0;
    for(i=0; i<5; i++)
    {
        res = mpu_dmp_init();
        //printf("mpu_2\n");
		//printf("mpu_dmp_init %d\n\r",res);
        if(res == 0)
        {
            return 1;
        }
        halTimerDelay(2);
    }
    return 0;
}

void mpu6050_power_off(void)
{
	//HAL_I2C_MspDeInit(&hi2c1);
	//mpu6050_gpio_init();	
	HAL_GPIO_WritePin(MPU_PWR_GPIO_Port, MPU_PWR_Pin,   GPIO_PIN_SET);	
}


