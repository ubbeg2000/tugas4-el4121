#include <stdio.h>
#include "driver/gpio.h"
#include "driver/adc.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc_cal.h"
#include "i2c-lcd1602.h"
#include "../../controller/controller.h"
#include "../../fsm/mode_fsm.h"
#include "../../fsm/debounce_fsm.h"
#include "../../fsm/rising_edge_fsm.h"
#include "../../fsm/autorepeat_fsm.h"

#define GPIO_INPUT_PIN_SEL (1ULL << GPIO_NUM_19 | 1ULL << GPIO_NUM_22 | 1ULL << GPIO_NUM_23)

float kp = 10, ki = 60, kd = 0;
float setpoint = 1;
int mode = OPERATING_MODE;

void display_task(void *pvParam)
{
    char lcd_buffer[17];

    smbus_info_t *smbus_info = smbus_malloc();
    i2c_lcd1602_info_t *lcd_info = i2c_lcd1602_malloc();

    smbus_init(smbus_info, I2C_NUM_0, 0x27);
    smbus_set_timeout(smbus_info, 20 / portTICK_PERIOD_MS);

    i2c_lcd1602_init(lcd_info, smbus_info, true, 2, 16, 16);
    i2c_lcd1602_reset(lcd_info);
    i2c_lcd1602_set_backlight(lcd_info, true);

    TickType_t xDelay = 50 / portTICK_PERIOD_MS;

    for (;;)
    {
        TickType_t xLastWakeTime = xTaskGetTickCount();

        switch (mode)
        {
        case OPERATING_MODE:
        {
            i2c_lcd1602_move_cursor(lcd_info, 0, 0);
            i2c_lcd1602_write_string(lcd_info, "OPERATING ");
            i2c_lcd1602_move_cursor(lcd_info, 0, 1);
            sprintf(lcd_buffer, "%16f", setpoint);
            i2c_lcd1602_write_string(lcd_info, lcd_buffer);
            break;
        }
        case SET_P_MODE:
        {
            i2c_lcd1602_move_cursor(lcd_info, 0, 0);
            i2c_lcd1602_write_string(lcd_info, "SETTING KP");
            i2c_lcd1602_move_cursor(lcd_info, 0, 1);
            sprintf(lcd_buffer, "%16f", kp);
            i2c_lcd1602_write_string(lcd_info, lcd_buffer);
            break;
        }
        case SET_I_MODE:
        {
            i2c_lcd1602_move_cursor(lcd_info, 0, 0);
            i2c_lcd1602_write_string(lcd_info, "SETTING KI");
            i2c_lcd1602_move_cursor(lcd_info, 0, 1);
            sprintf(lcd_buffer, "%16f", ki);
            i2c_lcd1602_write_string(lcd_info, lcd_buffer);
            break;
        }
        case SET_D_MODE:
        {
            i2c_lcd1602_move_cursor(lcd_info, 0, 0);
            i2c_lcd1602_write_string(lcd_info, "SETTING KD");
            i2c_lcd1602_move_cursor(lcd_info, 0, 1);
            sprintf(lcd_buffer, "%16f", kd);
            i2c_lcd1602_write_string(lcd_info, lcd_buffer);
            break;
        }

        default:
            break;
        }

        xTaskDelayUntil(&xLastWakeTime, xDelay);
    }
}

void pid_task(void *pvParam)
{
    char lcd_buffer[17];

    smbus_info_t *smbus_info = smbus_malloc();
    i2c_lcd1602_info_t *lcd_info = i2c_lcd1602_malloc();

    smbus_init(smbus_info, I2C_NUM_0, 0x27);
    smbus_set_timeout(smbus_info, 20 / portTICK_PERIOD_MS);

    i2c_lcd1602_init(lcd_info, smbus_info, true, 2, 16, 16);
    i2c_lcd1602_reset(lcd_info);
    i2c_lcd1602_set_backlight(lcd_info, true);

    float speed = 0, position = 0;
    float control_signal = 0;
    int temp = 0;

    TickType_t xDelay = TS * 1000 / portTICK_PERIOD_MS;

    PIDParams pid_params = {
        .kp = kp,
        .ki = ki,
        .kd = kd,
        .derivative = 0,
        .integral = 0,
        .error = 0,
        .prev_error = 0,
    };

    for (;;)
    {
        TickType_t xLastWakeTime = xTaskGetTickCount();

        pid_params.kp = kp;
        pid_params.ki = ki;
        pid_params.kd = kd;

        adc2_get_raw(ADC_CHANNEL_3, ADC_WIDTH_10Bit, &temp);
        setpoint = (float)temp;

        if (1 == scanf("%f\r\n", &speed))
        {
            if (mode == OPERATING_MODE)
            {
                pid_params.prev_error = pid_params.error;
                pid_params.error = setpoint - speed;
                control_signal = controller(&pid_params);

                printf("%.8f;%.8f\r\n", control_signal, setpoint);
            }
            else
            {
                pid_params.integral = 0;
                pid_params.derivative = 0;

                printf("%.8f;%.8f\r\n", 0.0, setpoint);
            }
        }

        switch (mode)
        {
        case OPERATING_MODE:
        {
            i2c_lcd1602_move_cursor(lcd_info, 0, 0);
            i2c_lcd1602_write_string(lcd_info, "OPERATING ");
            i2c_lcd1602_move_cursor(lcd_info, 0, 1);
            sprintf(lcd_buffer, "%16f", setpoint);
            i2c_lcd1602_write_string(lcd_info, lcd_buffer);
            break;
        }
        case SET_P_MODE:
        {
            i2c_lcd1602_move_cursor(lcd_info, 0, 0);
            i2c_lcd1602_write_string(lcd_info, "SETTING KP");
            i2c_lcd1602_move_cursor(lcd_info, 0, 1);
            sprintf(lcd_buffer, "%16f", kp);
            i2c_lcd1602_write_string(lcd_info, lcd_buffer);
            break;
        }
        case SET_I_MODE:
        {
            i2c_lcd1602_move_cursor(lcd_info, 0, 0);
            i2c_lcd1602_write_string(lcd_info, "SETTING KI");
            i2c_lcd1602_move_cursor(lcd_info, 0, 1);
            sprintf(lcd_buffer, "%16f", ki);
            i2c_lcd1602_write_string(lcd_info, lcd_buffer);
            break;
        }
        case SET_D_MODE:
        {
            i2c_lcd1602_move_cursor(lcd_info, 0, 0);
            i2c_lcd1602_write_string(lcd_info, "SETTING KD");
            i2c_lcd1602_move_cursor(lcd_info, 0, 1);
            sprintf(lcd_buffer, "%16f", kd);
            i2c_lcd1602_write_string(lcd_info, lcd_buffer);
            break;
        }

        default:
            break;
        }

        xTaskDelayUntil(&xLastWakeTime, xDelay);
    }
}

void control_task(void *pvParam)
{
    int debounce_state = IDLE_STATE,
        debounce_output = 0,
        cnt = 0;

    int rising_edge_state = LOW_STATE,
        rising_edge_output = 0;

    int up_debounce_state = IDLE_STATE,
        up_debounce_output = 0,
        up_debounce_cnt = 0;

    int up_autorepeat_state = AUTOREPEAT_OFF_STATE,
        up_autorepeat_output = 0,
        up_autorepeat_cnt = 0;

    int down_debounce_state = IDLE_STATE,
        down_debounce_output = 0,
        down_debounce_cnt = 0;

    int down_autorepeat_state = AUTOREPEAT_OFF_STATE,
        down_autorepeat_output = 0,
        down_autorepeat_cnt = 0;

    TickType_t xDelay = TS * 1000 / portTICK_PERIOD_MS;

    for (;;)
    {
        TickType_t xLastWakeTime = xTaskGetTickCount();

        debounce_fsm(&debounce_state, &cnt, gpio_get_level(GPIO_NUM_19), &debounce_output);
        rising_edge_fsm(&rising_edge_state, debounce_output, &rising_edge_output);
        mode_fsm(&mode, rising_edge_output);

        debounce_fsm(&up_debounce_state, &up_debounce_cnt, gpio_get_level(GPIO_NUM_22), &up_debounce_output);
        autorepeat_fsm(&up_autorepeat_state, &up_autorepeat_cnt, up_debounce_output, &up_autorepeat_output);

        debounce_fsm(&down_debounce_state, &down_debounce_cnt, gpio_get_level(GPIO_NUM_23), &down_debounce_output);
        autorepeat_fsm(&down_autorepeat_state, &down_autorepeat_cnt, down_debounce_output, &down_autorepeat_output);

        switch (mode)
        {
        case SET_P_MODE:
        {
            if (up_autorepeat_output == 1 && down_autorepeat_output == 0)
            {
                kp += 1;
            }
            if (up_autorepeat_output == 0 && down_autorepeat_output == 1)
            {
                kp -= 1;
            }
            break;
        }
        case SET_I_MODE:
        {
            if (up_autorepeat_output == 1 && down_autorepeat_output == 0)
            {
                ki += 1;
            }
            if (up_autorepeat_output == 0 && down_autorepeat_output == 1)
            {
                ki -= 1;
            }
            break;
        }
        case SET_D_MODE:
        {
            if (up_autorepeat_output == 1 && down_autorepeat_output == 0)
            {
                kd += 1;
            }
            if (up_autorepeat_output == 0 && down_autorepeat_output == 1)
            {
                kd -= 1;
            }
            break;
        }

        default:
            break;
        }

        xTaskDelayUntil(&xLastWakeTime, xDelay);
    }
}

void app_main()
{
    gpio_config_t io_conf;
    i2c_config_t i2c_conf;
    esp_adc_cal_characteristics_t adc_conf;

    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    gpio_config(&io_conf);

    esp_adc_cal_characterize(ADC_UNIT_2, ADC_ATTEN_11db, ADC_WIDTH_10Bit, 0, &adc_conf);
    adc2_config_channel_atten(ADC_CHANNEL_3, ADC_ATTEN_11db);

    i2c_conf.mode = I2C_MODE_MASTER;
    i2c_conf.sda_io_num = GPIO_NUM_21;
    i2c_conf.scl_io_num = GPIO_NUM_18;
    i2c_conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    i2c_conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    i2c_conf.master.clk_speed = 100000;
    i2c_conf.clk_flags = 0;
    i2c_param_config(I2C_NUM_0, &i2c_conf);
    i2c_driver_install(I2C_NUM_0, i2c_conf.mode, 0, 0, ESP_INTR_FLAG_SHARED);

    xTaskCreatePinnedToCore(pid_task, "PID Task", 2048, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(control_task, "Control Task", 2048, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(display_task, "Display Task", 2048, NULL, 3, NULL, 1);
}