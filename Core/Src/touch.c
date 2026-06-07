#include "touch.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "i2c.h"
#include "tusb.h"

#define TOUCH_CDC_ITF              0U
#define TOUCH_DEVICE_COUNT         1U
#define TOUCH_REGISTER             0x00U
#define TOUCH_DATA_LENGTH          35U
#define TOUCH_PERIOD_MS            20U
#define TOUCH_FRAME_LENGTH         47U

typedef enum
{
    TOUCH_EVENT_NONE = 0,
    TOUCH_EVENT_COMPLETE,
    TOUCH_EVENT_ERROR
} touch_event_t;

typedef enum
{
    TOUCH_STATUS_OK = 0,
    TOUCH_STATUS_TRANSFER_ERROR,
    TOUCH_STATUS_START_ERROR
} touch_status_t;

static const uint8_t touch_addresses[TOUCH_DEVICE_COUNT] = {0x08U};
static uint8_t touch_data[TOUCH_DEVICE_COUNT][TOUCH_DATA_LENGTH];
static uint8_t touch_status[TOUCH_DEVICE_COUNT];
static uint32_t touch_error[TOUCH_DEVICE_COUNT];
static uint16_t touch_sequence;
static uint8_t touch_device_index;
static uint32_t touch_next_tick;
static bool touch_transfer_active;
static bool touch_batch_ready;
static volatile touch_event_t touch_event;
static volatile uint32_t touch_callback_error;

static void touch_start_read(void);
static void touch_finish_read(touch_status_t status, uint32_t error);
static void touch_send_batch(void);
static void touch_build_frame(uint8_t device_index, uint8_t *frame);

void touch_init(void)
{
    touch_device_index = 0U;
    touch_sequence = 0U;
    touch_transfer_active = false;
    touch_batch_ready = false;
    touch_event = TOUCH_EVENT_NONE;
    touch_next_tick = HAL_GetTick();
}

void touch_task(void)
{
    touch_event_t event;

    if (touch_transfer_active)
    {
        __disable_irq();
        event = touch_event;
        touch_event = TOUCH_EVENT_NONE;
        __enable_irq();

        if (event == TOUCH_EVENT_COMPLETE)
        {
            touch_finish_read(TOUCH_STATUS_OK, HAL_I2C_ERROR_NONE);
        }
        else if (event == TOUCH_EVENT_ERROR)
        {
            touch_finish_read(TOUCH_STATUS_TRANSFER_ERROR,
                    touch_callback_error);
        }
    }

    if (touch_batch_ready)
    {
        touch_send_batch();
    }
    else if (!touch_transfer_active
            && ((int32_t)(HAL_GetTick() - touch_next_tick) >= 0))
    {
        touch_start_read();
    }
}

static void touch_start_read(void)
{
    HAL_StatusTypeDef result;

    memset(touch_data[touch_device_index], 0, TOUCH_DATA_LENGTH);
    touch_event = TOUCH_EVENT_NONE;
    touch_transfer_active = true;

    result = HAL_I2C_Mem_Read_IT(&hi2c1,
            (uint16_t)(touch_addresses[touch_device_index] << 1U),
            TOUCH_REGISTER,
            I2C_MEMADD_SIZE_8BIT,
            touch_data[touch_device_index],
            TOUCH_DATA_LENGTH);

    if (result != HAL_OK)
    {
        touch_finish_read(TOUCH_STATUS_START_ERROR, (uint32_t)result);
    }
}

static void touch_finish_read(touch_status_t status, uint32_t error)
{
    touch_status[touch_device_index] = (uint8_t)status;
    touch_error[touch_device_index] = error;
    touch_transfer_active = false;
    touch_device_index++;

    if (touch_device_index >= TOUCH_DEVICE_COUNT)
    {
        touch_device_index = 0U;
        touch_batch_ready = true;
        touch_sequence++;
    }
}

static void touch_build_frame(uint8_t device_index, uint8_t *frame)
{
    uint8_t checksum = 0U;

    frame[0] = 0xA5U;
    frame[1] = 0x5AU;
    frame[2] = touch_addresses[device_index];
    frame[3] = touch_status[device_index];
    frame[4] = (uint8_t)(touch_error[device_index] >> 0U);
    frame[5] = (uint8_t)(touch_error[device_index] >> 8U);
    frame[6] = (uint8_t)(touch_error[device_index] >> 16U);
    frame[7] = (uint8_t)(touch_error[device_index] >> 24U);
    frame[8] = (uint8_t)(touch_sequence >> 0U);
    frame[9] = (uint8_t)(touch_sequence >> 8U);
    frame[10] = TOUCH_DATA_LENGTH;
    memcpy(&frame[11], touch_data[device_index], TOUCH_DATA_LENGTH);

    for (uint8_t index = 2U; index < (TOUCH_FRAME_LENGTH - 1U); index++)
    {
        checksum ^= frame[index];
    }

    frame[TOUCH_FRAME_LENGTH - 1U] = checksum;
}

static void touch_send_batch(void)
{
    uint8_t frames[TOUCH_DEVICE_COUNT][TOUCH_FRAME_LENGTH];
    const uint32_t batch_length = TOUCH_DEVICE_COUNT * TOUCH_FRAME_LENGTH;

    if (!tud_cdc_n_connected(TOUCH_CDC_ITF)
            || (tud_cdc_n_write_available(TOUCH_CDC_ITF) < batch_length))
    {
        return;
    }

    for (uint8_t index = 0U; index < TOUCH_DEVICE_COUNT; index++)
    {
        touch_build_frame(index, frames[index]);
    }

    if (tud_cdc_n_write(TOUCH_CDC_ITF, frames, batch_length)
            == batch_length)
    {
        tud_cdc_n_write_flush(TOUCH_CDC_ITF);
        touch_batch_ready = false;
        touch_next_tick = HAL_GetTick() + TOUCH_PERIOD_MS;
    }
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if ((hi2c->Instance == I2C1) && touch_transfer_active)
    {
        touch_event = TOUCH_EVENT_COMPLETE;
    }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if ((hi2c->Instance == I2C1) && touch_transfer_active)
    {
        touch_callback_error = HAL_I2C_GetError(hi2c);
        touch_event = TOUCH_EVENT_ERROR;
    }
}
