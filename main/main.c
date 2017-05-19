#include "esp_system.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <stdio.h>
#include <string.h>

#include "bt.h"
#include "bta_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"

#define BLINK_DEVICE_NAME "Blink"
#define GATTS_SERVICE_UUID_BLINK 0x00FF
#define GATTS_CHAR_UUID_BLINK 0xFF01
#define GATTS_DESCR_UUID_BLINK 0x3333
#define GATTS_NUM_HANDLE_BLINK 4

#define BLINK_APP_ID 0

#define BLINK_ATTR_MAX_LEN 0x10

static void gatts_profile_blink_event_handler(esp_gatts_cb_event_t event,
					      esp_gatt_if_t gatts_if,
					      esp_ble_gatts_cb_param_t *param);

static uint8_t blink_attr[BLINK_ATTR_MAX_LEN] = {0};

static esp_attr_value_t blink_value = {
    .attr_max_len = BLINK_ATTR_MAX_LEN,
    .attr_len = sizeof(blink_attr),
    .attr_value = blink_attr,
};

static uint8_t test_service_uuid128[32] = {
    // first uuid, 16bit, [12],[13] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00,
    0xAB, 0xCD, 0x00, 0x00,
    // second uuid, 32bit, [12], [13], [14], [15] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00,
    0xAB, 0xCD, 0xAB, 0xCD,
};

static struct gatts_profile_inst {
  esp_gatts_cb_t gatts_cb;
  uint16_t gatts_if;
  uint16_t app_id;
  uint16_t conn_id;
  uint16_t service_handle;
  esp_gatt_srvc_id_t service_id;
  uint16_t char_handle;
  esp_bt_uuid_t char_uuid;
  esp_gatt_perm_t perm;
  esp_gatt_char_prop_t property;
  uint16_t descr_handle;
  esp_bt_uuid_t descr_uuid;
} blink_profiles[] = {
	[BLINK_APP_ID] =
	    {
		.gatts_cb = gatts_profile_blink_event_handler,
		.gatts_if = ESP_GATT_IF_NONE,
	    },
};

static esp_ble_adv_params_t blink_adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x40,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static esp_ble_adv_data_t test_adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x20,
    .max_interval = 0x40,
    .appearance = 0x00,
    .manufacturer_len = 0,
    .p_manufacturer_data = NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = 32,
    .p_service_uuid = test_service_uuid128,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static void gatts_profile_blink_event_handler(esp_gatts_cb_event_t event,
					      esp_gatt_if_t gatts_if,
					      esp_ble_gatts_cb_param_t *param) {
  uint16_t length = 0;
  const uint8_t *prf_char;
  esp_gatt_rsp_t rsp;
  printf("gatts_event_handler: %d\n", event);
  switch (event) {
  case ESP_GATTS_REG_EVT:
    printf("REGISTER_APP_EVT, status %d, app_id %d\n", param->reg.status,
	   param->reg.app_id);
    blink_profiles[0].service_id.is_primary = true;
    blink_profiles[0].service_id.id.inst_id = 0x00;
    blink_profiles[0].service_id.id.uuid.len = ESP_UUID_LEN_16;
    blink_profiles[0].service_id.id.uuid.uuid.uuid16 = GATTS_SERVICE_UUID_BLINK;

    esp_ble_gap_set_device_name(BLINK_DEVICE_NAME);
    esp_ble_gap_config_adv_data(&test_adv_data);
    esp_ble_gatts_create_service(gatts_if, &blink_profiles[0].service_id,
				 GATTS_NUM_HANDLE_BLINK);
    break;
  case ESP_GATTS_READ_EVT:
    printf("GATT_READ_EVT, conn_id %d, trans_id %d, handle %d\n",
	   param->read.conn_id, param->read.trans_id, param->read.handle);
    memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
    rsp.attr_value.handle = param->read.handle;
    rsp.attr_value.len = BLINK_ATTR_MAX_LEN;
    memcpy(rsp.attr_value.value, blink_value.attr_value, BLINK_ATTR_MAX_LEN);
    esp_ble_gatts_send_response(gatts_if, param->read.conn_id,
				param->read.trans_id, ESP_GATT_OK, &rsp);
    break;
  case ESP_GATTS_WRITE_EVT:
    printf("GATT_WRITE_EVT, conn_id %d, trans_id %d, handle %d, len %d\n",
	   param->write.conn_id, param->write.trans_id, param->write.handle,
	   param->write.len);
    if (param->write.len <= BLINK_ATTR_MAX_LEN) {
      memcpy(blink_value.attr_value, param->write.value, param->write.len);
      esp_ble_gatts_send_response(gatts_if, param->write.conn_id,
				  param->write.trans_id, ESP_GATT_OK, NULL);
    }
    break;
  case ESP_GATTS_CREATE_EVT:
    printf("CREATE_SERVICE_EVT, status %d,  service_handle %d\n",
	   param->create.status, param->create.service_handle);
    blink_profiles[0].service_handle = param->create.service_handle;
    blink_profiles[0].char_uuid.len = ESP_UUID_LEN_16;
    blink_profiles[0].char_uuid.uuid.uuid16 = GATTS_CHAR_UUID_BLINK;

    esp_ble_gatts_start_service(blink_profiles[0].service_handle);

    esp_ble_gatts_add_char(
	blink_profiles[0].service_handle, &blink_profiles[0].char_uuid,
	ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
	ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE |
	    ESP_GATT_CHAR_PROP_BIT_NOTIFY,
	&blink_value, NULL);
    break;
  case ESP_GATTS_ADD_CHAR_EVT:
    printf("ADD_CHAR_EVT, status %d,  attr_handle %d, service_handle %d\n",
	   param->add_char.status, param->add_char.attr_handle,
	   param->add_char.service_handle);
    blink_profiles[0].char_handle = param->add_char.attr_handle;
    blink_profiles[0].descr_uuid.len = ESP_UUID_LEN_16;
    blink_profiles[0].descr_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
    esp_ble_gatts_add_char_descr(
	blink_profiles[0].service_handle, &blink_profiles[0].descr_uuid,
	ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, NULL, NULL);
    break;
  case ESP_GATTS_CONNECT_EVT:
    printf("SERVICE_START_EVT, conn_id %d, remote "
	   "%02x:%02x:%02x:%02x:%02x:%02x:, is_conn %d\n",
	   param->connect.conn_id, param->connect.remote_bda[0],
	   param->connect.remote_bda[1], param->connect.remote_bda[2],
	   param->connect.remote_bda[3], param->connect.remote_bda[4],
	   param->connect.remote_bda[5], param->connect.is_connected);
    blink_profiles[0].conn_id = param->connect.conn_id;
    break;
  case ESP_GATTS_DISCONNECT_EVT:
    esp_ble_gap_start_advertising(&blink_adv_params);
    break;
  case ESP_GATTS_EXEC_WRITE_EVT:
  case ESP_GATTS_MTU_EVT:
  case ESP_GATTS_CONF_EVT:
  case ESP_GATTS_UNREG_EVT:
  case ESP_GATTS_ADD_INCL_SRVC_EVT:
  case ESP_GATTS_ADD_CHAR_DESCR_EVT:
  case ESP_GATTS_DELETE_EVT:
  case ESP_GATTS_START_EVT:
  case ESP_GATTS_STOP_EVT:
  case ESP_GATTS_OPEN_EVT:
  case ESP_GATTS_CANCEL_OPEN_EVT:
  case ESP_GATTS_CLOSE_EVT:
  case ESP_GATTS_LISTEN_EVT:
  case ESP_GATTS_CONGEST_EVT:
  default:
    break;
  }
}

static void gap_event_handler(esp_gap_ble_cb_event_t event,
			      esp_ble_gap_cb_param_t *param) {
  printf("gap_event_handler: %d\n", event);
  switch (event) {
  case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
  case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
  case ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT:
    esp_ble_gap_start_advertising(&blink_adv_params);
    break;
  default:
    break;
  }
}

static void gatts_event_handler(esp_gatts_cb_event_t event,
				esp_gatt_if_t gatts_if,
				esp_ble_gatts_cb_param_t *param) {
  gatts_profile_blink_event_handler(event, gatts_if, param);
}

int ble_setup() {
  int r;

  /* Initialize BLE */
  esp_bt_controller_init();
  r = esp_bt_controller_enable(ESP_BT_MODE_BTDM);
  if (r != 0) {
    printf("esp_bt_controller_enable: %d\n", r);
    return r;
  }
  r = esp_bluedroid_init();
  if (r != 0) {
    printf("esp_bluedroid_init: %d\n", r);
    return r;
  }
  r = esp_bluedroid_enable();
  if (r != 0) {
    printf("esp_bluedroid_enable: %d\n", r);
    return r;
  }

  /* Register BLE callbacks */
  esp_ble_gatts_register_callback(gatts_event_handler);
  esp_ble_gap_register_callback(gap_event_handler);
  esp_ble_gatts_app_register(BLINK_APP_ID);
  return 0;
}

void app_main() {
  int r;

  r = ble_setup();
  if (r != 0) {
    printf("reboot...\n");
    fflush(stdout);
    esp_restart();
    return;
  }
}
