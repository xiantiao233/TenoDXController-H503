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
#define TOUCH_RETRY_PERIOD_MS      500U
#define TOUCH_TRANSFER_TIMEOUT_MS  50U
#define TOUCH_FRAME_LENGTH         47U
#define TOUCH_ERROR_TIMEOUT        0x80000000U

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
static uint16_t touch_sequence[TOUCH_DEVICE_COUNT];
static uint32_t touch_next_tick[TOUCH_DEVICE_COUNT];
static bool touch_frame_pending[TOUCH_DEVICE_COUNT];
static uint8_t touch_active_device;
static uint8_t touch_schedule_cursor;
static uint32_t touch_transfer_start_tick;
static bool touch_transfer_active;
static volatile touch_event_t touch_event;
static volatile uint32_t touch_callback_error;

static bool touch_start_due_read(void);
static void touch_start_read(uint8_t device_index);
static void touch_finish_read(touch_status_t status, uint32_t error);
static void touch_recover_i2c(void);
static void touch_send_pending_frames(void);
static void touch_build_frame(uint8_t device_index, uint8_t *frame);

void touch_init(void)
{
    uint32_t now = HAL_GetTick();

    memset(touch_sequence, 0, sizeof(touch_sequence));
    memset(touch_frame_pending, 0, sizeof(touch_frame_pending));

    for (uint8_t index = 0U; index < TOUCH_DEVICE_COUNT; index++)
    {
        touch_next_tick[index] = now;
    }

    touch_active_device = 0U;
    touch_schedule_cursor = 0U;
    touch_transfer_active = false;
    touch_event = TOUCH_EVENT_NONE;
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
        else if ((uint32_t)(HAL_GetTick() - touch_transfer_start_tick)
                >= TOUCH_TRANSFER_TIMEOUT_MS)
        {
            touch_recover_i2c();
            touch_finish_read(TOUCH_STATUS_TRANSFER_ERROR,
                    TOUCH_ERROR_TIMEOUT);
        }
    }

    touch_send_pending_frames();

    if (!touch_transfer_active)
    {
        touch_start_due_read();
    }
}

static bool touch_start_due_read(void)
{
    uint32_t now = HAL_GetTick();

    for (uint8_t offset = 0U; offset < TOUCH_DEVICE_COUNT; offset++)
    {
        uint8_t device_index =
                (uint8_t)((touch_schedule_cursor + offset)
                        % TOUCH_DEVICE_COUNT);

        if ((int32_t)(now - touch_next_tick[device_index]) >= 0)
        {
            touch_schedule_cursor =
                    (uint8_t)((device_index + 1U) % TOUCH_DEVICE_COUNT);
            touch_start_read(device_index);
            return true;
        }
    }

    return false;
}

static void touch_start_read(uint8_t device_index)
{
    HAL_StatusTypeDef result;

    if ((HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
            || (__HAL_I2C_GET_FLAG(&hi2c1, I2C_FLAG_BUSY) != RESET))
    {
        touch_recover_i2c();
    }

    touch_active_device = device_index;
    memset(touch_data[device_index], 0, TOUCH_DATA_LENGTH);
    touch_event = TOUCH_EVENT_NONE;
    touch_transfer_start_tick = HAL_GetTick();
    touch_transfer_active = true;

    result = HAL_I2C_Mem_Read_IT(&hi2c1,
            (uint16_t)(touch_addresses[device_index] << 1U),
            TOUCH_REGISTER,
            I2C_MEMADD_SIZE_8BIT,
            touch_data[device_index],
            TOUCH_DATA_LENGTH);

    if (result != HAL_OK)
    {
        touch_finish_read(TOUCH_STATUS_START_ERROR, (uint32_t)result);
    }
}

static void touch_recover_i2c(void)
{
    HAL_I2C_DeInit(&hi2c1);
    __HAL_RCC_I2C1_FORCE_RESET();
    __NOP();
    __HAL_RCC_I2C1_RELEASE_RESET();
    MX_I2C1_Init();
    touch_event = TOUCH_EVENT_NONE;
}

static void touch_finish_read(touch_status_t status, uint32_t error)
{
    uint32_t retry_period = TOUCH_RETRY_PERIOD_MS;

    touch_status[touch_active_device] = (uint8_t)status;
    touch_error[touch_active_device] = error;
    touch_sequence[touch_active_device]++;
    touch_frame_pending[touch_active_device] = true;

    if (status == TOUCH_STATUS_OK)
    {
        retry_period = TOUCH_PERIOD_MS;
    }

    touch_next_tick[touch_active_device] = HAL_GetTick() + retry_period;
    touch_transfer_active = false;
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
    frame[8] = (uint8_t)(touch_sequence[device_index] >> 0U);
    frame[9] = (uint8_t)(touch_sequence[device_index] >> 8U);
    frame[10] = TOUCH_DATA_LENGTH;
    memcpy(&frame[11], touch_data[device_index], TOUCH_DATA_LENGTH);

    for (uint8_t index = 2U; index < (TOUCH_FRAME_LENGTH - 1U); index++)
    {
        checksum ^= frame[index];
    }

    frame[TOUCH_FRAME_LENGTH - 1U] = checksum;
}

static void touch_send_pending_frames(void)
{
    uint8_t frame[TOUCH_FRAME_LENGTH];

    if (!tud_cdc_n_connected(TOUCH_CDC_ITF)
            || (tud_cdc_n_write_available(TOUCH_CDC_ITF)
                    < TOUCH_FRAME_LENGTH))
    {
        return;
    }

    for (uint8_t index = 0U; index < TOUCH_DEVICE_COUNT; index++)
    {
        if (!touch_frame_pending[index]
                || (touch_transfer_active && (index == touch_active_device)))
        {
            continue;
        }

        touch_build_frame(index, frame);

        if (tud_cdc_n_write(TOUCH_CDC_ITF, frame, TOUCH_FRAME_LENGTH)
                == TOUCH_FRAME_LENGTH)
        {
            touch_frame_pending[index] = false;
            tud_cdc_n_write_flush(TOUCH_CDC_ITF);
        }

        break;
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
