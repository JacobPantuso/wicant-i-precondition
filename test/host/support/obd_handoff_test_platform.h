#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "esp_timer.h"
#define BIT0 1
#define BIT1 2
#define BIT2 4
#define BIT3 8
#define BIT4 16
#define pdFALSE 0
#define pdPASS 1
#define pdMS_TO_TICKS(x) (x)
typedef int portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED 0
#define portENTER_CRITICAL(x) ((void)(x))
#define portEXIT_CRITICAL(x) ((void)(x))
typedef void *TaskHandle_t;
typedef uint32_t EventBits_t;
typedef uint32_t *EventGroupHandle_t;
EventGroupHandle_t xEventGroupCreate(void);
EventBits_t xEventGroupClearBits(EventGroupHandle_t,EventBits_t);
EventBits_t xEventGroupSetBits(EventGroupHandle_t,EventBits_t);
EventBits_t xEventGroupWaitBits(EventGroupHandle_t,EventBits_t,int,int,int);
BaseType_t xTaskCreate(void(*)(void*),const char*,unsigned,void*,unsigned,TaskHandle_t*);
void vTaskDelay(int);
void vTaskDelete(void*);
#define ESP_OK 0
#define ESP_GATT_OK 0
#define ESP_GATT_IF_NONE 255
#define ESP_UUID_LEN_16 2
#define ESP_UUID_LEN_128 16
#define ESP_GATT_WRITE_TYPE_RSP 1
#define ESP_GATT_WRITE_TYPE_NO_RSP 0
#define ESP_GATT_AUTH_REQ_NONE 0
#define ESP_GATT_CHAR_PROP_BIT_WRITE 8
#define ESP_GATT_CHAR_PROP_BIT_WRITE_NR 4
#define ESP_GATT_CHAR_PROP_BIT_NOTIFY 16
#define ESP_GATT_UUID_CHAR_CLIENT_CONFIG 0x2902
#define BLE_SCAN_TYPE_ACTIVE 1
#define BLE_ADDR_TYPE_PUBLIC 0
#define BLE_SCAN_FILTER_ALLOW_ALL 0
#define BLE_SCAN_DUPLICATE_DISABLE 0
#define ESP_BT_STATUS_SUCCESS 0
#define ESP_BLE_AD_TYPE_NAME_CMPL 9
typedef int esp_ble_addr_type_t;
typedef uint8_t esp_bd_addr_t[6];
typedef int esp_gatt_if_t;
typedef struct {unsigned len;union {uint16_t uuid16;uint8_t uuid128[16];}uuid;} esp_bt_uuid_t;
typedef struct {uint16_t char_handle;unsigned properties;esp_bt_uuid_t uuid;} esp_gattc_char_elem_t;
typedef struct {uint16_t handle;} esp_gattc_descr_elem_t;
typedef struct {int scan_type,own_addr_type,scan_filter_policy,scan_interval,scan_window,scan_duplicate;} esp_ble_scan_params_t;
typedef enum {ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT,ESP_GAP_BLE_SCAN_RESULT_EVT}esp_gap_ble_cb_event_t;
enum {ESP_GAP_SEARCH_INQ_CMPL_EVT,ESP_GAP_SEARCH_INQ_RES_EVT};
typedef struct {struct {int status;}scan_param_cmpl;struct {int search_evt;uint8_t ble_adv[62];uint8_t adv_data_len,scan_rsp_len;esp_bd_addr_t bda;int ble_addr_type;}scan_rst;} esp_ble_gap_cb_param_t;
typedef enum {ESP_GATTC_REG_EVT,ESP_GATTC_OPEN_EVT,ESP_GATTC_SEARCH_RES_EVT,ESP_GATTC_SEARCH_CMPL_EVT,ESP_GATTC_REG_FOR_NOTIFY_EVT,ESP_GATTC_WRITE_DESCR_EVT,ESP_GATTC_NOTIFY_EVT,ESP_GATTC_DISCONNECT_EVT}esp_gattc_cb_event_t;
typedef struct {
 struct {int status,app_id;}reg;
 struct {int status;uint16_t conn_id;esp_bd_addr_t remote_bda;}open;
 struct {uint16_t conn_id,start_handle,end_handle;struct {esp_bt_uuid_t uuid;}srvc_id;}search_res;
 struct {int status;uint16_t conn_id;}search_cmpl;
 struct {int status;uint16_t handle;}reg_for_notify;
 struct {int status;uint16_t conn_id,handle;}write;
 struct {uint16_t conn_id,handle,value_len;uint8_t *value;}notify;
 struct {uint16_t conn_id;}disconnect;
}esp_ble_gattc_cb_param_t;
int esp_ble_gattc_write_char(int,int,int,int,uint8_t*,int,int);
int esp_ble_gattc_search_service(int,int,void*);
int esp_ble_gattc_close(int,int);
int esp_ble_gattc_get_all_char(int,int,int,int,esp_gattc_char_elem_t*,uint16_t*,int);
int esp_ble_gattc_register_for_notify(int,uint8_t*,int);
int esp_ble_gattc_unregister_for_notify(int,uint8_t*,int);
int esp_ble_gattc_get_descr_by_char_handle(int,int,int,esp_bt_uuid_t,esp_gattc_descr_elem_t*,uint16_t*);
int esp_ble_gattc_write_char_descr(int,int,int,int,uint8_t*,int,int);
int esp_ble_gap_start_scanning(int);
uint8_t *esp_ble_resolve_adv_data_by_type(uint8_t*,uint16_t,int,uint8_t*);
int esp_ble_gap_set_scan_params(esp_ble_scan_params_t*);
int esp_ble_gattc_open(int,uint8_t*,int,bool);
int esp_ble_gap_stop_scanning(void);
int esp_ble_gattc_register_callback(void(*)(esp_gattc_cb_event_t,esp_gatt_if_t,esp_ble_gattc_cb_param_t*));
int esp_ble_gattc_app_register(int);
