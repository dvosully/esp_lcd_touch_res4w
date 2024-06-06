/*
 * SPDX-FileCopyrightText: 2024 dvosully (github.com/dvosully)
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include "esp_idf_version.h"
#include "esp_lcd_touch.h"
#include "esp_lcd_panel_io.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  gpio_num_t io_YP;    // Analog and Digital capable
  gpio_num_t io_YM;    // Digital capable
  gpio_num_t io_XP;    // Digital capable
  gpio_num_t io_XM;    // Analog and Digital capable
} res4w_cfg_data_t;

/**
 * @brief Create a new RES4W touch driver
 *
 * @param config: Touch configuration.
 * @param io: Touch panel IO configuration.
 * @param out_touch: RES4W instance handle.
 * @return
 *      - ESP_OK                    on success
 *      - ESP_ERR_NO_MEM            if there is insufficient memory for allocating main structure.
 *      - ESP_ERR_INVALID_ARG       if @param io or @param config are null.
 */
esp_err_t esp_lcd_touch_new_res4w(const esp_lcd_touch_config_t *config,
                                  const res4w_cfg_data_t *res4w_cfg,
                                  esp_lcd_touch_handle_t *out_touch);

#ifdef __cplusplus
}
#endif
