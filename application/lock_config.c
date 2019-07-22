#include "main.h"
#include "stm32f0xx_hal.h"
#include "lock_motor.h"
//#include "infrared_detect.h"
#include "lock_config.h"

#define LOCK_CONFIG_ADDR  (0x08003C00+0x4000)//0x0800FC00

static int drag_level_map[] = {MOTOR_DRAG_LEVEL_1,MOTOR_DRAG_LEVEL_2,
                               MOTOR_DRAG_LEVEL_3,MOTOR_DRAG_LEVEL_4,MOTOR_DRAG_LEVEL_5
                              } ;

typedef struct lock_config {
    uint32_t magic;
    uint8_t  ir_direction;
    uint8_t  motor_drag_level;
    uint8_t  unlock_type;
    uint8_t  lock_time;
} lock_config_t;

static lock_config_t m_lock_cfg;

void lock_config_save();
void lock_config_default();

int lock_config_write_flash(uint32_t *data,uint32_t len)
{
    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t PageError = 0;
    uint32_t addr;
    EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.PageAddress = LOCK_CONFIG_ADDR;
    EraseInitStruct.NbPages =  1;
    HAL_FLASH_Unlock();
    if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK)
    {
        return -1;
    }

    for (addr = LOCK_CONFIG_ADDR; addr < LOCK_CONFIG_ADDR + len ; addr = addr + 4)
    {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, *data) != HAL_OK)
        {
            return -1;
        }
        data = data + 1;
    }
    HAL_FLASH_Lock();
    return 0;
}

uint32_t lock_config_read_flash(uint32_t *data,uint32_t len)
{
    uint32_t addr;
    addr = LOCK_CONFIG_ADDR;

    for (addr = LOCK_CONFIG_ADDR; addr < LOCK_CONFIG_ADDR + len ; addr = addr + 4)
    {
        *data = *(__IO uint32_t *)addr;
        data = data + 1;
    }
    return 0;
}

void lock_config_init(void)
{
    osal_memset(&m_lock_cfg,0,sizeof(m_lock_cfg));

    lock_config_read_flash((uint32_t *)&m_lock_cfg,sizeof(m_lock_cfg));
    if ( m_lock_cfg.magic != 0x55aa )
    {
        lock_config_default();
        lock_config_save();
    }

}

void lock_config_default(void)
{
    m_lock_cfg.ir_direction     = BOTH_SIDE_DETECT;
    m_lock_cfg.motor_drag_level = 3;
    m_lock_cfg.unlock_type      = RIGHT_UNLOCK_DOOR;
    m_lock_cfg.lock_time        = 0;
}

void lock_config_save(void)
{
    m_lock_cfg.magic = 0x55aa;
    if (lock_config_write_flash((uint32_t *)&m_lock_cfg,sizeof(m_lock_cfg)) < 0)
    {
        lock_config_default();
    }
}


void lock_config_setting(uint8_t setMask,uint8_t ir_direction,
                         uint8_t motorDrag,uint8_t unlockType,uint8_t lockTime)
{
    if (setMask & SET_IR_DIRECTION )
    {
        m_lock_cfg.ir_direction = ir_direction;
    }

    if (setMask & SET_MOTOR_DRAG )
    {
        if ( motorDrag < 1 || motorDrag > 5)
            return;
        m_lock_cfg.motor_drag_level = motorDrag;
    }

    if (setMask & SET_UNLOCOK_TYPE )
    {
        m_lock_cfg.unlock_type = unlockType;
    }

    if (setMask & SET_LOCK_TIME )
    {
        if (lockTime < 0 || lockTime > 60)
            return;
        m_lock_cfg.lock_time = lockTime;
    }

    if (setMask != 0)
    {
        lock_config_save();
    }
}

uint8_t get_infrared_detect_side(void)
{
    return m_lock_cfg.ir_direction;
}

uint8_t get_gyro_type(void)//used or unused
{
    return m_lock_cfg.ir_direction;
}

uint8_t get_unlock_type(void)//left or right
{
    return m_lock_cfg.unlock_type;
}

int get_motor_drag_value(void)
{
	uint8_t motorDrag = m_lock_cfg.motor_drag_level;
	if (motorDrag < 1 || motorDrag > 5)
		motorDrag = 3;
	return drag_level_map[motorDrag - 1];	
    //return drag_level_map[m_lock_cfg.motor_drag_level - 1];
}

int get_motor_drag_level()
{
    return m_lock_cfg.motor_drag_level;
}


uint8_t get_lock_time(void)
{
    return m_lock_cfg.lock_time;
}


