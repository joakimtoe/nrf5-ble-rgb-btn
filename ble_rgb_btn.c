/* Copyright (c) 2016 joakimtoe
 *
 * This software may be modified and distributed under the terms
 * of the MIT license.  See the LICENSE file for details.
 */

#include "ble_rgb_btn.h"
#include <string.h>
#include "sdk_common.h"
#include "ble_srv_common.h"
#include "app_util.h"


/**@brief Function for handling the Connect event.
 *
 * @param[in]   p_rgb_btn   RGB Button Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_connect(ble_rgb_btn_t * p_rgb_btn, ble_evt_t * p_ble_evt)
{
    p_rgb_btn->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}


/**@brief Function for handling the Disconnect event.
 *
 * @param[in]   p_rgb_btn   RGB Button Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_disconnect(ble_rgb_btn_t * p_rgb_btn, ble_evt_t * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_rgb_btn->conn_handle = BLE_CONN_HANDLE_INVALID;
}


/**@brief Function for handling the Write event.
 *
 * @param[in]   p_rgb_btn   RGB Button Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_write(ble_rgb_btn_t * p_rgb_btn, ble_evt_t * p_ble_evt)
{
    ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
    
    if ((p_evt_write->handle == p_rgb_btn->btn_char_handles.cccd_handle) &&
        (p_evt_write->len == 2)
       )
    {
        if (ble_srv_is_notification_enabled(p_evt_write->data))
        {
            p_rgb_btn->is_notification_enabled = true;
        }
        else
        {
            p_rgb_btn->is_notification_enabled = false;
        }
    }    
    else if ((p_evt_write->handle == p_rgb_btn->rgb_char_handles.value_handle) &&
             (p_evt_write->len == 3) &&
             (p_rgb_btn->rgb_handler != NULL))
    {
        p_rgb_btn->rgb_handler(p_rgb_btn, p_evt_write->data);
    }
}


void ble_rgb_btn_on_ble_evt(ble_rgb_btn_t * p_rgb_btn, ble_evt_t * p_ble_evt)
{
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_rgb_btn, p_ble_evt);
            break;
            
        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_rgb_btn, p_ble_evt);
            break;
            
        case BLE_GATTS_EVT_WRITE:
            on_write(p_rgb_btn, p_ble_evt);
            break;
            
        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for adding the LED characteristic.
 *
 */
static uint32_t rgb_char_add(ble_rgb_btn_t * p_rgb_btn, const ble_rgb_btn_init_t * p_rgb_btn_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;
    
    uint8_t default_rgb[3] = {0, 0, 0};

    memset(&char_md, 0, sizeof(char_md));
    
    char_md.char_props.read   = 1;
    char_md.char_props.write  = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = NULL;
    char_md.p_sccd_md         = NULL;
    
    ble_uuid.type = p_rgb_btn->uuid_type;
    ble_uuid.uuid = RGB_BTN_UUID_LED_CHAR;
    
    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;
    
    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid       = &ble_uuid;
    attr_char_value.p_attr_md    = &attr_md;
    attr_char_value.init_len     = sizeof(uint8_t)*3;
    attr_char_value.init_offs    = 0;
    attr_char_value.max_len      = sizeof(uint8_t)*3;
    attr_char_value.p_value      = default_rgb;
    
    return sd_ble_gatts_characteristic_add(p_rgb_btn->service_handle, &char_md,
                                               &attr_char_value,
                                               &p_rgb_btn->rgb_char_handles);
}

/**@brief Function for adding the Button characteristic.
 *
 */
static uint32_t btn_char_add(ble_rgb_btn_t * p_rgb_btn, const ble_rgb_btn_init_t * p_rgb_btn_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&cccd_md, 0, sizeof(cccd_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    
    cccd_md.vloc = BLE_GATTS_VLOC_STACK;
    
    memset(&char_md, 0, sizeof(char_md));
    
    char_md.char_props.read   = 1;
    char_md.char_props.notify = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md;
    char_md.p_sccd_md         = NULL;
    
    ble_uuid.type = p_rgb_btn->uuid_type;
    ble_uuid.uuid = RGB_BTN_UUID_BUTTON_CHAR;
    
    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;
    
    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid       = &ble_uuid;
    attr_char_value.p_attr_md    = &attr_md;
    attr_char_value.init_len     = sizeof(uint8_t);
    attr_char_value.init_offs    = 0;
    attr_char_value.max_len      = sizeof(uint8_t);
    attr_char_value.p_value      = NULL;
    
    return sd_ble_gatts_characteristic_add(p_rgb_btn->service_handle, &char_md,
                                               &attr_char_value,
                                               &p_rgb_btn->btn_char_handles);
}

uint32_t ble_rgb_btn_init(ble_rgb_btn_t * p_rgb_btn, const ble_rgb_btn_init_t * p_rgb_btn_init)
{
    uint32_t   err_code;
    ble_uuid_t ble_uuid;

    // Initialize service structure
    p_rgb_btn->conn_handle             = BLE_CONN_HANDLE_INVALID;
    p_rgb_btn->rgb_handler             = p_rgb_btn_init->rgb_handler;
    p_rgb_btn->is_notification_enabled = false;
    
    // Add service
    ble_uuid128_t base_uuid = {RGB_BTN_UUID_BASE};
    err_code = sd_ble_uuid_vs_add(&base_uuid, &p_rgb_btn->uuid_type);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    ble_uuid.type = p_rgb_btn->uuid_type;
    ble_uuid.uuid = RGB_BTN_UUID_SERVICE;

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_rgb_btn->service_handle);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    err_code = btn_char_add(p_rgb_btn, p_rgb_btn_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    err_code = rgb_char_add(p_rgb_btn, p_rgb_btn_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    return NRF_SUCCESS;
}

uint32_t ble_rgb_btn_set(ble_rgb_btn_t * p_rgb_btn, uint8_t btn_state)
{
    ble_gatts_hvx_params_t params;
    uint16_t length = 1;
    
    VERIFY_PARAM_NOT_NULL(p_rgb_btn);

    if ((p_rgb_btn->conn_handle == BLE_CONN_HANDLE_INVALID) || (!p_rgb_btn->is_notification_enabled))
    {
        return NRF_ERROR_INVALID_STATE;
    }
    
    memset(&params, 0, sizeof(params));
    params.type = BLE_GATT_HVX_NOTIFICATION;
    params.handle = p_rgb_btn->btn_char_handles.value_handle;
    params.p_data = &btn_state;
    params.p_len = &length;
    
    return sd_ble_gatts_hvx(p_rgb_btn->conn_handle, &params);
}
