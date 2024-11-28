# esp_lcd_touch driver for bare 4 wire resistive touchscreen devices

Implementation of the bare 4 wire resistive touchscreen driver with esp_lcd_touch component.

## Adding this component in your project

This package can be added to your project in two ways:

1. Using [Espressif's component service](https://components.espressif.com/) as:
```
dependencies:
  dvosully/esp_lcd_touch_res4w: "~0.1.0"
```

2. Using the git repository directly:

```
dependencies:
  esp_lcd_touch_res4w:
    git: https://github.com/dvosully/esp_lcd_touch_res4w.git
```

For more information on the usage of the `idf_component.yml` file please refer to [Espressif's documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/tools/idf-component-manager.html).

## Supported platforms

At this time testing is limited to ESP32, other ESP32 variants should work but are not tested.

## Configuration options

`Kconfig.projbuild` contains a handful of options to allow customization of the RES4W interface:

* XPT2046_Z_THRESHOLD - This is the minimum ADC threshold to use for detecting touch points.
* XPT2046_CONVERT_ADC_TO_COORDS - This option enables / disables the conversion of raw ADC values into screen coordinates. When disabled it will be necessary to define the `process_coordinates` method in `esp_lcd_touch_config_t`. This can be useful for applying calibration or other offsets to adjust the screen coordinates.
* XPT2046_ENABLE_LOCKING - This option enables / disables the usage of critical sections to protect internal data structures. This has been seen to cause conflicts with other components at times.

## Example usage

### Initialization

```
    esp_lcd_touch_handle_t tp = NULL;

    esp_lcd_touch_config_t tp_cfg = {
        .x_max = CONFIG_LCD_HRES,
        .y_max = CONFIG_LCD_VRES,
        .rst_gpio_num = -1,
        .int_gpio_num = -1,
        .flags = {
            .swap_xy = 0,
            .mirror_x = 0,
            .mirror_y = 0,
        },
    };
    res4w_cfg_data_t res_cfg = {
        .io_YP = TOUCH_YP_PIN,
        .io_YM = TOUCH_YM_PIN,
        .io_XP = TOUCH_XP_PIN,
        .io_XM = TOUCH_XM_PIN,
    };

    ESP_LOGI(TAG, "Initialize touch controller RES4W");
    ESP_ERROR_CHECK(esp_lcd_touch_new_spi_res4w(&tp_cfg, &res_cfg, &tp));
```

### Updating touch point data

This will read new data from the touch controller and store it in SRAM. This method
should be called on a regular basis.

```
    ESP_ERROR_CHECK(esp_lcd_touch_read_data(tp));
```

### Retrieving touch point(s)

This will retrieve the latest averaged touch point data from SRAM.

```
    uint16_t x[1];
    uint16_t y[1];
    uint16_t  strength[1];
    uint8_t count = 0;

    bool touchpad_pressed = esp_lcd_touch_get_coordinates(tp, x, y, strength, &count, 1);
```

### Integrating with LVGL

This driver can be integrated with LVGL using code similar to below:

```
void touch_driver_read(lv_indev_drv_t *drv, lv_indev_data_t *data)
{
    uint16_t x[1];
    uint16_t y[1];
    uint16_t strength[1];
    uint8_t count = 0;

    // Update touch point data.
    ESP_ERROR_CHECK(esp_lcd_touch_read_data(tp));

    data->state = LV_INDEV_STATE_REL;

    if (esp_lcd_touch_get_coordinates(tp, x, y, strength, &count, 1))
    {
        data->point.x = x[0];
        data->point.y = y[0];
        data->state = LV_INDEV_STATE_PR;
    }

    data->continue_reading = false;
}

void initialize_input()
{
    lv_indev_drv_t indev_drv;
    lv_indev_drv_init( &indev_drv );
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = touch_driver_read;
    lv_indev_drv_register( &indev_drv );
}

```