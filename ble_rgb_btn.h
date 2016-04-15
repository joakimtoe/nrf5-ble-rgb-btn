/* Copyright (c) 2016 joakimtoe
 *
 * This software may be modified and distributed under the terms
 * of the MIT license.  See the LICENSE file for details.
 */

#ifndef __BLE_RGB_BTN_H__
#define __BLE_RGB_BTN_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"

#define RGB_BTN_UUID_BASE {0xB4, 0xE7, 0x1C, 0x33, 0xBB, 0x67, 0xCA, 0x84, 0x32, 0x4C, 0x2A, 0x6F, 0x00, 0x00, 0xAE, 0xB6}
#define RGB_BTN_UUID_SERVICE 0x0001
#define RGB_BTN_UUID_LED_CHAR 0x0002
#define RGB_BTN_UUID_BUTTON_CHAR 0x0003

// Forward declaration of the ble_rgb_btn_t type. 
typedef struct ble_rgb_btn_s ble_rgb_btn_t;

typedef void (*ble_rgb_btn_handler_t) (ble_rgb_btn_t * p_rgb_btn, uint8_t * p_rgb);

typedef struct
{
    ble_rgb_btn_handler_t rgb_handler; /**< Event handler to be called when RGB characteristic is written. */
} ble_rgb_btn_init_t;

/**@brief RGB Button Service structure. This contains various status information for the service. */
typedef struct ble_rgb_btn_s
{
    uint16_t                    service_handle;
    ble_gatts_char_handles_t    rgb_char_handles;
    ble_gatts_char_handles_t    btn_char_handles;
    uint8_t                     uuid_type;
    uint16_t                    conn_handle;
    ble_rgb_btn_handler_t       rgb_handler;
    bool                        is_notification_enabled;
} ble_rgb_btn_t;

/**@brief Function for initializing the RGB Button Service.
 *
 * @param[out]  p_rgb_btn   RGB Button Service structure. This structure will have to be supplied by
 *                          the application. It will be initialized by this function, and will later
 *                          be used to identify this particular service instance.
 * @param[in]   p_rgb_btn_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
uint32_t ble_rgb_btn_init(ble_rgb_btn_t * p_rgb_btn, const ble_rgb_btn_init_t * p_rgb_btn_init);

/**@brief Function for handling the Application's BLE Stack events.
 *
 * @details Handles all events from the BLE stack of interest to the RGB Button Service.
 *
 *
 * @param[in]   p_rgb_btn  RGB Button Service structure.
 * @param[in]   p_ble_evt  Event received from the BLE stack.
 */
void ble_rgb_btn_on_ble_evt(ble_rgb_btn_t * p_rgb_btn, ble_evt_t * p_ble_evt);

/**@brief Function for sending a button state notification.
 */
uint32_t ble_rgb_btn_set(ble_rgb_btn_t * p_rgb_btn, uint8_t button_state);

#endif // __BLE_RGB_BTN_H__
