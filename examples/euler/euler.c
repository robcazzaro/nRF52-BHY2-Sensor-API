/**
 *
 * Copyright (C) 2022 Roberto Cazzaro <https://github.com/robcazzaro>
 * 
 * Bosch BHY2 libraries ported to nRF5 SDK (17.1.0)
 *
 *
 * 
 *
 * Copyright (c) 2020 Bosch Sensortec GmbH. All rights reserved.
 *
 * BSD-3-Clause
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * @file    euler.c
 * @date    24 Mar 2020
 * @brief   Euler data stream example for the BHI260/BHA260
 *
 */

#include "bhy2.h"
#include "bhy2_parse.h"
#include "bhy2_hif.h"
#include "bhy2_defs.h"
#include "common.h"

#include "nrf_delay.h"
#include "nrf_drv_gpiote.h"
#include "nrf_gpio.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "Bosch_SHUTTLE_BHI260_aux_BMM150.fw.h"

#include "nrf_pwr_mgmt.h"

#define WORK_BUFFER_SIZE   2048
#define MAX_READ_WRITE_LEN 256

#define EULER_SENSOR_ID    BHY2_SENSOR_ID_ORI_WU

static void parse_euler(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref);
static void parse_meta_event(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref);
static void print_api_error(int8_t rslt, struct bhy2_dev *dev);

static void gpio_init(void);
static void mems_int_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action);

struct bhy2_dev bhy2;
uint8_t work_buffer[WORK_BUFFER_SIZE];

int main_euler(void)
{
    uint8_t product_id = 0;
    uint16_t version = 0;
    int8_t rslt;
// moved to global    struct bhy2_dev bhy2;
// moved to global    uint8_t work_buffer[WORK_BUFFER_SIZE];
    uint8_t hintr_ctrl, hif_ctrl, boot_status;
    uint8_t accuracy; /* Accuracy is reported as a meta event. It is being printed alongside the data */

    // Setup SPI peripheral for nRF52. See readme for BHI260 connections
    setup_SPI();

    rslt = bhy2_init(BHY2_SPI_INTERFACE, bhy2_spi_read, bhy2_spi_write, bhy2_delay_us, MAX_READ_WRITE_LEN, NULL, &bhy2);
    print_api_error(rslt, &bhy2);

    nrf_gpio_cfg_output(BSP_MEMS_nRESET);   // GPIO is connected to BHI260 RESETN pin
    nrf_gpio_pin_clear(BSP_MEMS_nRESET);    // BHI260 reset is active  low
    nrf_delay_us(1);                        // Wait minimum safe time for reset (min 100nsec)
    nrf_gpio_pin_set(BSP_MEMS_nRESET);      // End of reset sequence
    // The default protocol used by BHI260 after power-up or RESET is I2C. Pulling the HCSB (Host Chip Select) pin LOW 
    // at any time switches the host interface (HIF) into SPI mode, where it remains until next RESET or power-on cycle
    nrf_gpio_cfg_output(BSP_MEMS_CS);   // BHI260 HCSB
    nrf_gpio_pin_clear(BSP_MEMS_CS);
    nrf_delay_us(1);

    rslt = bhy2_soft_reset(&bhy2);
    print_api_error(rslt, &bhy2);

    rslt = bhy2_get_product_id(&product_id, &bhy2);
    print_api_error(rslt, &bhy2);

    /* Check for a valid product ID */
    if (product_id != BHY2_PRODUCT_ID)
    {
        NRF_LOG_INFO("Product ID read %X. Expected %X", product_id, BHY2_PRODUCT_ID);
    }
    else
    {
        NRF_LOG_INFO("BHI260/BHA260 found. Product ID read %X", product_id);
    }

    // Disable status and debug, and temporarily disable interrupts to prevent issues with interrupt-driven code. Page 121 datasheet
    hintr_ctrl = BHY2_ICTL_DISABLE_STATUS_FIFO | BHY2_ICTL_DISABLE_DEBUG | BHY2_ICTL_DISABLE_FIFO_W | BHY2_ICTL_DISABLE_FIFO_NW | BHY2_ICTL_DISABLE_FAULT | BHY2_ICTL_OPEN_DRAIN;

    rslt = bhy2_set_host_interrupt_ctrl(hintr_ctrl, &bhy2);
    print_api_error(rslt, &bhy2);
    rslt = bhy2_get_host_interrupt_ctrl(&hintr_ctrl, &bhy2);
    print_api_error(rslt, &bhy2);

    NRF_LOG_INFO("Host interrupt control");
    NRF_LOG_INFO("    Wake up FIFO %s.", (hintr_ctrl & BHY2_ICTL_DISABLE_FIFO_W) ? "disabled" : "enabled");
    NRF_LOG_INFO("    Non wake up FIFO %s.", (hintr_ctrl & BHY2_ICTL_DISABLE_FIFO_NW) ? "disabled" : "enabled");
    NRF_LOG_INFO("    Status FIFO %s.", (hintr_ctrl & BHY2_ICTL_DISABLE_STATUS_FIFO) ? "disabled" : "enabled");
    NRF_LOG_INFO("    Debugging %s.", (hintr_ctrl & BHY2_ICTL_DISABLE_DEBUG) ? "disabled" : "enabled");
    NRF_LOG_INFO("    Fault %s.", (hintr_ctrl & BHY2_ICTL_DISABLE_FAULT) ? "disabled" : "enabled");
    NRF_LOG_INFO("    Interrupt is %s.", (hintr_ctrl & BHY2_ICTL_ACTIVE_LOW) ? "active low" : "active high");
    NRF_LOG_INFO("    Interrupt is %s triggered.", (hintr_ctrl & BHY2_ICTL_EDGE) ? "pulse" : "level");
    NRF_LOG_INFO("    Interrupt pin drive is %s.", (hintr_ctrl & BHY2_ICTL_OPEN_DRAIN) ? "open drain" : "push-pull");

    /* Configure the host interface */
    hif_ctrl = 0;
    rslt = bhy2_set_host_intf_ctrl(hif_ctrl, &bhy2);
    print_api_error(rslt, &bhy2);

    /* Check if the sensor is ready to load firmware */
    rslt = bhy2_get_boot_status(&boot_status, &bhy2);
    print_api_error(rslt, &bhy2);

    if (boot_status & BHY2_BST_HOST_INTERFACE_READY)
    {
        uint8_t sensor_error;
        int8_t temp_rslt;
        NRF_LOG_INFO("Loading firmware into RAM.");

        rslt = bhy2_upload_firmware_to_ram(bhy2_firmware_image, sizeof(bhy2_firmware_image), &bhy2);
        temp_rslt = bhy2_get_error_value(&sensor_error, &bhy2);
        if (sensor_error)
        {
            NRF_LOG_INFO("%s", get_sensor_error_text(sensor_error));
        }
        print_api_error(rslt, &bhy2);
        print_api_error(temp_rslt, &bhy2);

        NRF_LOG_INFO("Booting from RAM.");

        rslt = bhy2_boot_from_ram(&bhy2);
        temp_rslt = bhy2_get_error_value(&sensor_error, &bhy2);
        if (sensor_error)
        {
            NRF_LOG_INFO("%s", get_sensor_error_text(sensor_error));
        }
        print_api_error(rslt, &bhy2);
        print_api_error(temp_rslt, &bhy2);

        rslt = bhy2_get_kernel_version(&version, &bhy2);
        print_api_error(rslt, &bhy2);
        if ((rslt == BHY2_OK) && (version != 0))
        {
            NRF_LOG_INFO("Boot successful. Kernel version %u.", version);
        }

        // Disabled mete event parsing: sometimes too many messages cause problems, can be re-enabled for test
//        rslt = bhy2_register_fifo_parse_callback(BHY2_SYS_ID_META_EVENT, parse_meta_event, (void*)&accuracy, &bhy2);
//        print_api_error(rslt, &bhy2);
//        rslt = bhy2_register_fifo_parse_callback(BHY2_SYS_ID_META_EVENT_WU, parse_meta_event, (void*)&accuracy, &bhy2);
//        print_api_error(rslt, &bhy2);
        rslt = bhy2_register_fifo_parse_callback(EULER_SENSOR_ID, parse_euler, (void*)&accuracy, &bhy2);
        print_api_error(rslt, &bhy2);

        rslt = bhy2_get_and_process_fifo(work_buffer, WORK_BUFFER_SIZE, &bhy2);
        print_api_error(rslt, &bhy2);
    }
    else
    {
        NRF_LOG_INFO("Host interface not ready. Exiting");

        return 0;
    }

    /* Update the callback table to enable parsing of sensor data */
    rslt = bhy2_update_virtual_sensor_list(&bhy2);
    print_api_error(rslt, &bhy2);

    float sample_rate = 1.0; /* Read out data measured at 1Hz. Avoid high rates when using NRF_LOG */
    uint32_t report_latency_ms = 0; /* Report immediately */
    rslt = bhy2_set_virt_sensor_cfg(EULER_SENSOR_ID, sample_rate, report_latency_ms, &bhy2);
    print_api_error(rslt, &bhy2);
    printf("Enable %s at %.2fHz.\r\n", get_sensor_name(EULER_SENSOR_ID), sample_rate);

// Nordic specific code to enable GPIOTE
    hintr_ctrl = BHY2_ICTL_DISABLE_STATUS_FIFO | BHY2_ICTL_DISABLE_DEBUG | BHY2_ICTL_ACTIVE_LOW | BHY2_ICTL_EDGE; // re-enable interrupts. Page 121 datasheet
    rslt = bhy2_set_host_interrupt_ctrl(hintr_ctrl, &bhy2);
    print_api_error(rslt, &bhy2);

    gpio_init();

    // Enter main loop. Never exits. Code is interrupt driven from the BHI260, every time data is ready, it's printed by the parse_euler() function
	// in real use, parse_euler() will load data into variables to be used by the rest of the code
    for (;;)
    {
        if (NRF_LOG_PROCESS() == false)
        {
            nrf_pwr_mgmt_run();
        }
    }
}


static void parse_euler(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
    (void)callback_ref;
    struct bhy2_data_orientation data;
    uint32_t s, ns;
    uint8_t *accuracy = (uint8_t*)callback_ref;
    if (callback_info->data_size != 7) /* Check for a valid payload size. Includes sensor ID */
    {
        return;
    }

    bhy2_parse_orientation(callback_info->data_ptr, &data);

    uint64_t timestamp = *callback_info->time_stamp; /* Store the last timestamp */

    timestamp = timestamp * 15625; /* Timestamp is now in nanoseconds */
    s = (uint32_t)(timestamp / UINT64_C(1000000000));
    ns = (uint32_t)(timestamp - (s * UINT64_C(1000000000)));

    if (accuracy)
    {
/*        NRF_LOG_INFO("SID: T: %u.%09u; h: %s%d.%02d, p: %s%d.%02d, r: %s%d.%02d; acc: %u",
               s,
               ns,
               NRF_LOG_FLOAT(data.heading * 360.0f / 32768.0f),
               NRF_LOG_FLOAT(data.pitch * 360.0f / 32768.0f),
               NRF_LOG_FLOAT(data.roll * 360.0f / 32768.0f),
               *accuracy);*/

        NRF_LOG_INFO("SID: T: %u.%09u; h: %d, p: %d, r: %d; acc: %u",
               s,
               ns,
               data.heading,
               data.pitch,
               data.roll,
               *accuracy);



/*        NRF_LOG_INFO("SID: %u; T: %u.%09u; h: %f, p: %f, r: %f; acc: %u",
               callback_info->sensor_id,
               s,
               ns,
               data.heading * 360.0f / 32768.0f,
               data.pitch * 360.0f / 32768.0f,
               data.roll * 360.0f / 32768.0f,
               *accuracy);*/
    }
    else
    {
/*        NRF_LOG_INFO("SID: T: %u.%09u; h: %f, p: %f, r: %f",
               s,
               ns,
               data.heading * 360.0f / 32768.0f,
               data.pitch * 360.0f / 32768.0f,
               data.roll * 360.0f / 32768.0f);
*/

/*        NRF_LOG_INFO("SID: %u; T: %u.%09u; h: %f, p: %f, r: %f",
               callback_info->sensor_id,
               s,
               ns,
               data.heading * 360.0f / 32768.0f,
               data.pitch * 360.0f / 32768.0f,
               data.roll * 360.0f / 32768.0f);*/
    }
}


static void parse_meta_event(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
    (void)callback_ref;
    uint8_t meta_event_type = callback_info->data_ptr[0];
    uint8_t byte1 = callback_info->data_ptr[1];
    uint8_t byte2 = callback_info->data_ptr[2];
    uint8_t *accuracy = (uint8_t*)callback_ref;
    char *event_text;

    if (callback_info->sensor_id == BHY2_SYS_ID_META_EVENT)
    {
        event_text = "[META EVENT]";
    }
    else if (callback_info->sensor_id == BHY2_SYS_ID_META_EVENT_WU)
    {
        event_text = "[META EVENT WAKE UP]";
    }
    else
    {
        return;
    }

    switch (meta_event_type)
    {
        case BHY2_META_EVENT_FLUSH_COMPLETE:
            NRF_LOG_INFO("%s Flush complete for sensor id %u", event_text, byte1);
            break;
        case BHY2_META_EVENT_SAMPLE_RATE_CHANGED:
            NRF_LOG_INFO("%s Sample rate changed for sensor id %u", event_text, byte1);
            break;
        case BHY2_META_EVENT_POWER_MODE_CHANGED:
            NRF_LOG_INFO("%s Power mode changed for sensor id %u", event_text, byte1);
            break;
        case BHY2_META_EVENT_ALGORITHM_EVENTS:
            NRF_LOG_INFO("%s Algorithm event", event_text);
            break;
        case BHY2_META_EVENT_SENSOR_STATUS:
            NRF_LOG_INFO("%s Accuracy for sensor id %u changed to %u", event_text, byte1, byte2);
            if (accuracy)
            {
                *accuracy = byte2;
            }
            break;
        case BHY2_META_EVENT_BSX_DO_STEPS_MAIN:
            NRF_LOG_INFO("%s BSX event (do steps main)", event_text);
            break;
        case BHY2_META_EVENT_BSX_DO_STEPS_CALIB:
            NRF_LOG_INFO("%s BSX event (do steps calib)", event_text);
            break;
        case BHY2_META_EVENT_BSX_GET_OUTPUT_SIGNAL:
            NRF_LOG_INFO("%s BSX event (get output signal)", event_text);
            break;
        case BHY2_META_EVENT_SENSOR_ERROR:
            NRF_LOG_INFO("%s Sensor id %u reported error 0x%02X", event_text, byte1, byte2);
            break;
        case BHY2_META_EVENT_FIFO_OVERFLOW:
            NRF_LOG_INFO("%s FIFO overflow", event_text);
            break;
        case BHY2_META_EVENT_DYNAMIC_RANGE_CHANGED:
            NRF_LOG_INFO("%s Dynamic range changed for sensor id %u", event_text, byte1);
            break;
        case BHY2_META_EVENT_FIFO_WATERMARK:
            NRF_LOG_INFO("%s FIFO watermark reached", event_text);
            break;
        case BHY2_META_EVENT_INITIALIZED:
            NRF_LOG_INFO("%s Firmware initialized. Firmware version %u", event_text, ((uint16_t)byte2 << 8) | byte1);
            break;
        case BHY2_META_TRANSFER_CAUSE:
            NRF_LOG_INFO("%s Transfer cause for sensor id %u", event_text, byte1);
            break;
        case BHY2_META_EVENT_SENSOR_FRAMEWORK:
            NRF_LOG_INFO("%s Sensor framework event for sensor id %u", event_text, byte1);
            break;
        case BHY2_META_EVENT_RESET:
            NRF_LOG_INFO("%s Reset event", event_text);
            break;
        case BHY2_META_EVENT_SPACER:
            break;
        default:
            NRF_LOG_INFO("%s Unknown meta event with id: %u", event_text, meta_event_type);
            break;
    }
}

static void print_api_error(int8_t rslt, struct bhy2_dev *dev)
{
    if (rslt != BHY2_OK)
    {
        NRF_LOG_INFO("%s", get_api_error(rslt));
        if ((rslt == BHY2_E_IO) && (dev != NULL))
        {
            NRF_LOG_INFO("%s", get_api_error(dev->hif.intf_rslt));
            dev->hif.intf_rslt = BHY2_INTF_RET_SUCCESS;
        }
        exit(0);
    }
}

static void gpio_init(void)
{
    ret_code_t err_code;

    if(!nrf_drv_gpiote_is_init())
    {
        err_code = nrf_drv_gpiote_init();
        APP_ERROR_CHECK(err_code);
    }

    nrf_drv_gpiote_in_config_t mems_int;
    mems_int.sense = GPIOTE_CONFIG_POLARITY_HiToLo;
    mems_int.pull = NRF_GPIO_PIN_PULLUP;
    mems_int.hi_accuracy = true;
    mems_int.is_watcher = false; //Don't change this
    mems_int.skip_gpio_setup = false; //Don't change this

    err_code = nrf_drv_gpiote_in_init((nrfx_gpiote_pin_t)BSP_MEMS_INT, &mems_int, mems_int_handler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(BSP_MEMS_INT, true);
}

static void mems_int_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    int8_t rslt;
    rslt = bhy2_get_and_process_fifo(work_buffer, WORK_BUFFER_SIZE, &bhy2);
    print_api_error(rslt, &bhy2);
}
