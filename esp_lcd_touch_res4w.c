/*
 * SPDX-FileCopyrightText: 2024 dvosully (github.com/dvosully)
 *
 * SPDX-License-Identifier: MIT
 */

#include <driver/gpio.h>
#include <esp_check.h>
#include <esp_err.h>
#include <esp_lcd_panel_io.h>
#include <esp_rom_gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
// This must be included after FreeRTOS includes due to missing include
// for portMUX_TYPE
#include <esp_lcd_touch.h>
#include <memory.h>
#include "esp_lcd_touch_res4w.h"
#include "esp_adc/adc_oneshot.h"

#include "sdkconfig.h"

static const char *TAG = "res4w";


#if CONFIG_RES4W_ENABLE_LOCKING
#define RES4W_LOCK(lock) portENTER_CRITICAL(lock)
#define RES4W_UNLOCK(lock) portEXIT_CRITICAL(lock)
#else
#define RES4W_LOCK(lock)
#define RES4W_UNLOCK(lock)
#endif

static const uint16_t RES4W_ADC_LIMIT = 4096;
static esp_err_t res4w_read_data(esp_lcd_touch_handle_t tp);
static bool res4w_get_xy(esp_lcd_touch_handle_t tp,
                           uint16_t *x, uint16_t *y,
                           uint16_t *strength,
                           uint8_t *point_num,
                           uint8_t max_point_num);
static esp_err_t res4w_del(esp_lcd_touch_handle_t tp);

typedef struct
{
  gpio_num_t io_YP;    // Analog and Digital capable
  gpio_num_t io_YM;    // Digital capable
  gpio_num_t io_XP;    // Digital capable
  gpio_num_t io_XM;    // Analog and Digital capable
  adc_oneshot_unit_handle_t adc_xm_handle;
  adc_oneshot_unit_handle_t adc_yp_handle;
  adc_channel_t adc_xm_channel;
  adc_channel_t adc_yp_channel;
} res4w_data_t;



esp_err_t esp_lcd_touch_new_res4w(const esp_lcd_touch_config_t *config,
                                  const res4w_cfg_data_t *res4w_cfg,
                                  esp_lcd_touch_handle_t *out_touch)
{
    esp_err_t ret = ESP_OK;
    esp_lcd_touch_handle_t handle = NULL;

    ESP_GOTO_ON_FALSE(config, ESP_ERR_INVALID_ARG, err, TAG,
                      "esp_lcd_touch_config_t must not be NULL");
    ESP_GOTO_ON_FALSE(res4w_cfg, ESP_ERR_INVALID_ARG, err, TAG,
                      "res4w_cfg_data_t must not be NULL");

    handle = (esp_lcd_touch_handle_t)calloc(1, sizeof(esp_lcd_touch_t));
    ESP_GOTO_ON_FALSE(handle, ESP_ERR_NO_MEM, err, TAG,
                      "No memory available for RES4W state");
    handle->io = (esp_lcd_panel_io_handle_t)calloc(1, sizeof(res4w_data_t));
    ESP_GOTO_ON_FALSE(handle->io, ESP_ERR_NO_MEM, err, TAG,
                      "No memory available for RES4W state");
    handle->read_data = res4w_read_data;
    handle->get_xy = res4w_get_xy;
    handle->del = res4w_del;
    handle->data.lock.owner = portMUX_FREE_VAL;
    memcpy(&handle->config, config, sizeof(esp_lcd_touch_config_t));


    ESP_GOTO_ON_FALSE(GPIO_IS_VALID_OUTPUT_GPIO(res4w_cfg->io_YP), ESP_ERR_INVALID_ARG, err, TAG,
                      "io_YP must be a GPIO output");
    ESP_GOTO_ON_FALSE(GPIO_IS_VALID_OUTPUT_GPIO(res4w_cfg->io_YM), ESP_ERR_INVALID_ARG, err, TAG,
                      "io_YM must be a GPIO output");
    ESP_GOTO_ON_FALSE(GPIO_IS_VALID_OUTPUT_GPIO(res4w_cfg->io_XP), ESP_ERR_INVALID_ARG, err, TAG,
                      "io_XP must be a GPIO output");
    ESP_GOTO_ON_FALSE(GPIO_IS_VALID_OUTPUT_GPIO(res4w_cfg->io_XM), ESP_ERR_INVALID_ARG, err, TAG,
                      "io_XM must be a GPIO output");


    res4w_data_t* io = (res4w_data_t*)handle->io;

    adc_unit_t xm_unit;
    adc_unit_t yp_unit;
    ESP_GOTO_ON_FALSE(ESP_OK == adc_oneshot_io_to_channel(res4w_cfg->io_YP, &yp_unit, &io->adc_yp_channel), ESP_ERR_INVALID_ARG, err, TAG,
                      "io_YP must be an ADC input");
    ESP_GOTO_ON_FALSE(ESP_OK == adc_oneshot_io_to_channel(res4w_cfg->io_XM, &xm_unit, &io->adc_xm_channel), ESP_ERR_INVALID_ARG, err, TAG,
                      "io_XM must be an ADC input");

    io->io_YP = res4w_cfg->io_YP;
    io->io_YM = res4w_cfg->io_YM;
    io->io_XP = res4w_cfg->io_XP;
    io->io_XM = res4w_cfg->io_XM;

    adc_oneshot_unit_init_cfg_t adc_init_cfg = {};
    adc_init_cfg.unit_id = xm_unit;
    adc_init_cfg.ulp_mode = ADC_ULP_MODE_DISABLE;
    ESP_GOTO_ON_FALSE(
      ESP_OK == adc_oneshot_new_unit(&adc_init_cfg, &io->adc_xm_handle),
                      ESP_ERR_INVALID_ARG, err, TAG, "Create XM ADC handle failed");
    if (xm_unit == yp_unit)
        {
        io->adc_yp_handle = io->adc_xm_handle;
        }
    else
        {
        adc_init_cfg.unit_id = yp_unit;
        ESP_GOTO_ON_FALSE(
          ESP_OK == adc_oneshot_new_unit(&adc_init_cfg, &io->adc_yp_handle),
                          ESP_ERR_INVALID_ARG, err, TAG, "Create YP ADC handle failed");
        }
    adc_oneshot_chan_cfg_t adc_ch_cfg = {};
    adc_ch_cfg.bitwidth = ADC_BITWIDTH_12;
    adc_ch_cfg.atten = ADC_ATTEN_DB_12;
    adc_oneshot_config_channel(io->adc_yp_handle, io->adc_yp_channel, &adc_ch_cfg);
    adc_oneshot_config_channel(io->adc_xm_handle, io->adc_xm_channel, &adc_ch_cfg);



err:
    if (ret != ESP_OK)
    {
        if (handle)
        {
            res4w_del(handle);
            handle = NULL;
        }
    }

    *out_touch = handle;

    return ret;
}

static esp_err_t res4w_del(esp_lcd_touch_handle_t tp)
{
    if (tp != NULL)
    {
        if (tp->io != NULL)
        {
            res4w_data_t* io = (res4w_data_t*)tp->io;
            if (io->adc_yp_handle == io->adc_xm_handle)
            {
                adc_oneshot_del_unit(io->adc_yp_handle);
            }
            else
            {
                adc_oneshot_del_unit(io->adc_xm_handle);
                adc_oneshot_del_unit(io->adc_yp_handle);
            }
            free(tp->io);
        }
        if (tp->config.int_gpio_num != GPIO_NUM_NC)
        {
            gpio_reset_pin(tp->config.int_gpio_num);
        }
    }
    free(tp);

    return ESP_OK;
}


// Helper functions to control the pins - set them high and low and read the ADC
//==============================================================================

// Set Y+ pin high
static void set_yp_high(res4w_data_t *data)
{
    gpio_reset_pin(data->io_YP);
    gpio_set_direction(data->io_YP, GPIO_MODE_OUTPUT);
    gpio_set_level(data->io_YP, 1);
}

// Set Y+ pin ADC input and read
static int set_yp_input(res4w_data_t *data)
{
    int retval = 0;
    // Reset pin to high impedance (it may be configured as an output and driven high)
    gpio_reset_pin(data->io_YP);

    adc_oneshot_read(data->adc_yp_handle, data->adc_yp_channel, &retval);

    return retval;
}

// Set Y- pin high
static void set_ym_high(res4w_data_t *data)
{
    gpio_reset_pin(data->io_YM);
    gpio_set_direction(data->io_YM, GPIO_MODE_OUTPUT);
    gpio_set_level(data->io_YM, 1);
}

// Set Y- pin low
static void set_ym_low(res4w_data_t *data)
{
    gpio_reset_pin(data->io_YM);
    gpio_set_direction(data->io_YM, GPIO_MODE_OUTPUT);
    gpio_set_level(data->io_YM, 0);
}

// Set Y- pin high impedance
static void set_ym_hiz(res4w_data_t *data)
{
    gpio_reset_pin(data->io_YM);
    gpio_set_direction(data->io_YM, GPIO_MODE_INPUT);
}

// Set X+ pin high
static void set_xp_high(res4w_data_t *data)
{
    gpio_reset_pin(data->io_XP);
    gpio_set_direction(data->io_XP, GPIO_MODE_OUTPUT);
    gpio_set_level(data->io_XP, 1);
}

// Set X+ pin low
static void set_xp_low(res4w_data_t *data)
{
    gpio_reset_pin(data->io_XP);
    gpio_set_direction(data->io_XP, GPIO_MODE_OUTPUT);
    gpio_set_level(data->io_XP, 0);
}

// Set X+ pin high impedance
static void set_xp_hiz(res4w_data_t *data)
{
    gpio_reset_pin(data->io_XP);
    gpio_set_direction(data->io_XP, GPIO_MODE_INPUT);
}

// Set X- pin low
static void set_xm_low(res4w_data_t *data)
{
    gpio_reset_pin(data->io_XM);
    gpio_set_direction(data->io_XM, GPIO_MODE_OUTPUT);
    gpio_set_level(data->io_XM, 0);
}

// Set X- pin ADC input and read
static int set_xm_input(res4w_data_t *data)
{
    int retval = 0;
    // Reset pin to high impedance (it may be configured as an output and driven low)
    gpio_reset_pin(data->io_XM);

    adc_oneshot_read(data->adc_xm_handle, data->adc_xm_channel, &retval);

    return retval;
}



// Get the X value of the touchscreen
// Set Y- high impedance
// Set X+ high
// Set X- low
// Read Y+ ADC (discard first 2 results)
static uint32_t get_x_val(res4w_data_t *data)
{
    uint32_t x = 0;

    // Set pin status
    set_ym_hiz(data);
    set_xp_high(data);
    set_xm_low(data);

    // Wait for levels to settle
    set_yp_input(data);
    set_yp_input(data);
    // Read input
    x = RES4W_ADC_LIMIT - set_yp_input(data);

    return x;
}

// Get the Y value of the touchscreen
// Set X+ high impedance
// Set Y+ high
// Set Y- low
// Read X- ADC (discard first 2 results)
static uint32_t get_y_val(res4w_data_t *data)
{
    uint32_t y = 0;

    // Set pin status
    set_xp_hiz(data);
    set_yp_high(data);
    set_ym_low(data);

    // Wait for levels to settle
    set_xm_input(data);
    set_xm_input(data);

    // Read input
    y = set_xm_input(data);

    return RES4W_ADC_LIMIT - y;
}

// Get the Z value of the touchscreen
// Set X+ low
// Set Y- high
// Set X- high impedance (adc)
// Read Y+ ADC (discard first results)
// Read X- ADC (discard first 2 results)
static uint32_t get_z_val(res4w_data_t *data)
{
    uint32_t xm = 0;
    uint32_t yp = 0;
    uint32_t z = 0;

    // Set pin status
    set_xp_low(data);
    set_ym_high(data);
    set_xm_input(data);
    set_yp_input(data);

    // Wait for levels to settle
    set_xm_input(data);

    // Read inputs
    xm = set_xm_input(data);
    yp = set_yp_input(data);

    z = RES4W_ADC_LIMIT - (yp - xm);
    return z;
}




static esp_err_t res4w_read_data(esp_lcd_touch_handle_t tp)
{
    uint32_t x = 0, y = 0, z = 0;
    uint8_t point_count = 0;

    res4w_data_t* io = (res4w_data_t*)tp->io;

    z = get_z_val(io);

    // If the Z (pressure) exceeds the threshold it is likely the user has
    // pressed the screen, read in and average the positions.
    if (z >= CONFIG_RES4W_Z_THRESHOLD)
    {
        for (uint8_t idx = 0; idx < CONFIG_ESP_LCD_TOUCH_MAX_POINTS; idx++)
        {
            // Read X position
            uint16_t x_temp = get_x_val(io);
            // Read Y position
            uint16_t y_temp = get_y_val(io);

            // Test if the readings are valid (50 < reading < max - 50)
            if ((x_temp >= 50) && (x_temp <= RES4W_ADC_LIMIT - 50) && (y_temp >= 50) && (y_temp <= RES4W_ADC_LIMIT - 50))
            {
#if CONFIG_RES4W_CONVERT_ADC_TO_COORDS
                // Convert the raw ADC value into a screen coordinate and store it
                // for averaging.
                x += ((x_temp / (double)RES4W_ADC_LIMIT) * tp->config.x_max);
                y += ((y_temp / (double)RES4W_ADC_LIMIT) * tp->config.y_max);
#else
                // store the raw ADC values and let the user convert them to screen
                // coordinates.
                x += x_temp;
                y += y_temp;
#endif // CONFIG_RES4W_CONVERT_ADC_TO_COORDS
                point_count++;
            }
        }

        // Check we had enough valid values
        const int minimum_count = (1 == CONFIG_ESP_LCD_TOUCH_MAX_POINTS ? 1 : CONFIG_ESP_LCD_TOUCH_MAX_POINTS/2);
        if (point_count >= minimum_count)
        {
            // Average the accumulated coordinate data points.
            x /= point_count;
            y /= point_count;
            point_count = 1;
        }
        else
        {
            z = 0;
            point_count = 0;
        }
    }

    RES4W_LOCK(&tp->data.lock);
    tp->data.coords[0].x = x;
    tp->data.coords[0].y = y;
    tp->data.coords[0].strength = z;
    tp->data.points = point_count;
    RES4W_UNLOCK(&tp->data.lock);

    return ESP_OK;
}

static bool res4w_get_xy(esp_lcd_touch_handle_t tp, uint16_t *x, uint16_t *y,
                           uint16_t *strength, uint8_t *point_num,
                           uint8_t max_point_num)
{
    RES4W_LOCK(&tp->data.lock);

    // Determine how many touch points that are available.
    if (tp->data.points > max_point_num)
    {
        *point_num = max_point_num;
    }
    else
    {
        *point_num = tp->data.points;
    }

    for (size_t i = 0; i < *point_num; i++)
    {
        x[i] = tp->data.coords[i].x;
        y[i] = tp->data.coords[i].y;

        if (strength)
        {
            strength[i] = tp->data.coords[i].strength;
        }
    }

    // Invalidate stored touch data.
    tp->data.points = 0;

    RES4W_UNLOCK(&tp->data.lock);

    if (*point_num)
    {
        ESP_LOGD(TAG, "Touch point: %dx%d", x[0], y[0]);
    }
    else
    {
        ESP_LOGV(TAG, "No touch points");
    }

    return (*point_num > 0);
}

