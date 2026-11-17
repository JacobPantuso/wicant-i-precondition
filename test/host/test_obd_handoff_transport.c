// Production BLE-client lifecycle with synchronous fake controller callbacks.
#define OBD_HANDOFF_HOST_TEST 1
#include "test_support.h"
#include "../../main/obd_handoff.c"
static uint32_t bits;
static int close_calls,scan_calls,search_calls,write_calls,hold_calls,observe_calls;
static bool phone=true,authorized=true;
static EventBits_t wait_result;
static bool scripted,ack_close,wrong_vin,owner_script;
static unsigned bc03_reads;
static int lease_started,lease_ended,owner_calls;static bool lease_touched;
void standby_note_lease_started(void){lease_started++;}
void standby_note_lease_ended(bool touched_vehicle){lease_ended++;lease_touched=touched_vehicle;}
void autolock_owner_locked(void){owner_calls++;}
static unsigned body_reads;
EventGroupHandle_t xEventGroupCreate(void){return &bits;}
EventBits_t xEventGroupClearBits(EventGroupHandle_t g,EventBits_t b){EventBits_t old=*g;*g&=~b;return old;}
EventBits_t xEventGroupSetBits(EventGroupHandle_t g,EventBits_t b){*g|=b;return *g;}
static int silent_scans,scan_param_calls;
EventBits_t xEventGroupWaitBits(EventGroupHandle_t g,EventBits_t b,int clear_on,int all,int timeout){if(scripted&&(b&SCANNED)){if(silent_scans>0)silent_scans--;else matches=1;*g|=SCANNED;} EventBits_t result=wait_result?wait_result:(*g&b);if(clear_on)*g&=~b;return result;}
BaseType_t xTaskCreate(void(*fn)(void*),const char*n,unsigned stack,void*arg,unsigned priority,TaskHandle_t*out){*out=(void*)1;return pdPASS;}
void vTaskDelay(int ms){fake_now+=(int64_t)ms*1000;}
void vTaskDelete(void*p){}
bool ble_connected(void){return phone;}
bool autolock_can_handoff(void){return authorized;}
bool autolock_command(const char*a,char*r,size_t cap){if(!strcmp(a,"b1"))hold_calls++;snprintf(r,cap,"OK");return true;}
void autolock_handoff_invalidate(void){authorized=false;}
void autolock_handoff_observation(bool on,bool blocked){observe_calls++;}
void bodycmd_get_status(bodycmd_status_t*s){memset(s,0,sizeof(*s));if(scripted&&body_reads++&&!owner_script)s->locks_completed=1;}
int esp_ble_gattc_write_char(int a,int b,int c,int d,uint8_t*e,int f,int g){write_calls++;
 if(scripted){
   char cmd[32];memcpy(cmd,e,(size_t)d);cmd[d-1]=0;
   if(!strcmp(cmd,"0902")){
     size_t off=(size_t)snprintf(reply,sizeof(reply),"490201");
     const char *v=wrong_vin?"KMHM141AAN0999999":vin;
     for(int i=0;i<17;i++)off+=(size_t)snprintf(reply+off,sizeof(reply)-off,"%02X",(unsigned char)v[i]);
     snprintf(reply+off,sizeof(reply)-off,"\r>");
   }else if(!strcmp(cmd,"22BC03")){
     // Owner script: the driver's door opens on the first read, then shuts
     // and every door reads locked.
     bc03_reads++;
     snprintf(reply,sizeof(reply),owner_script&&bc03_reads==1?"62BC03000000000100\r>":"62BC03000000000000\r>");
   }
   else if(!strcmp(cmd,"22BC04"))snprintf(reply,sizeof(reply),"62BC040000000000\r>");
   else snprintf(reply,sizeof(reply),"OK\r>");
   reply_len=strlen(reply);bits|=REPLY;
 }
 return ESP_OK;}
int esp_ble_gattc_search_service(int a,int b,void*c){search_calls++;return ESP_OK;}
int esp_ble_gattc_close(int a,int b){close_calls++;if(scripted&&ack_close){link_up=false;bits|=CLOSED;}return ESP_OK;}
int esp_ble_gattc_get_all_char(int a,int b,int c,int d,esp_gattc_char_elem_t*e,uint16_t*f,int g){*f=0;return ESP_OK;}
int esp_ble_gattc_register_for_notify(int a,uint8_t*b,int c){return ESP_OK;}
int esp_ble_gattc_unregister_for_notify(int a,uint8_t*b,int c){return ESP_OK;}
int esp_ble_gattc_get_descr_by_char_handle(int a,int b,int c,esp_bt_uuid_t d,esp_gattc_descr_elem_t*e,uint16_t*f){*f=0;return ESP_OK;}
int esp_ble_gattc_write_char_descr(int a,int b,int c,int d,uint8_t*e,int f,int g){return ESP_OK;}
int esp_ble_gap_start_scanning(int a){scan_calls++;return ESP_OK;}
uint8_t *esp_ble_resolve_adv_data_by_type(uint8_t*p,uint16_t len,int type,uint8_t*n){if(len>=2&&p[0]&&p[0]<len&&p[1]==type){*n=p[0]-1;return p+2;}return NULL;}
int esp_ble_gap_set_scan_params(esp_ble_scan_params_t*p){scan_param_calls++;return ESP_OK;}
int esp_ble_gattc_open(int a,uint8_t*b,int c,bool d){if(scripted){opening=false;link_up=true;write_handle=1;bits|=READY;}return ESP_OK;}
int esp_ble_gap_stop_scanning(void){return ESP_OK;}
int esp_ble_gattc_register_callback(void(*cb)(esp_gattc_cb_event_t,esp_gatt_if_t,esp_ble_gattc_cb_param_t*)){return ESP_OK;}
int esp_ble_gattc_app_register(int a){return ESP_OK;}
static char out[128];
static void reset(void){
 fake_now=1000000;bits=0;events=&bits;worker=NULL;client=3;conn=7;
 active=link_up=opening=cancelled=managed=identity_ok=clear=false;
 observed_at=doors_at=0;observation_serial=applied_serial=0;state="idle";doors="unknown";
 strcpy(vin,"KMHM141AAN0123456");strcpy(token,"-");memset(selected,0,6);
 close_calls=scan_calls=search_calls=write_calls=hold_calls=observe_calls=0;
 wait_result=0;phone=authorized=true;scripted=false;ack_close=true;wrong_vin=false;body_reads=0;
 owner_script=false;bc03_reads=0;owner_locked=false;touched=false;silent_scans=0;scan_param_calls=0;
 lease_started=lease_ended=owner_calls=0;lease_touched=false;
}
static void start(void){CHECK(obd_handoff_command("nabcdef12,4f4244",out,sizeof(out)));}
int main(void){
 reset();start();CHECK(active&&managed&&worker);CHECK(!strcmp(token,"ABCDEF12"));CHECK(obd_handoff_blocks());
 CHECK(!obd_handoff_command("n12345678,4f4244",out,sizeof(out)));
 CHECK(!obd_handoff_command("vKMHM141AAN0123456",out,sizeof(out)));
 // Cancellation cannot revive scan or turn a late OPEN into discovery.
 CHECK(obd_handoff_command("0",out,sizeof(out)));CHECK(cancelled&&obd_handoff_blocks());
 esp_ble_gap_cb_param_t gp={0};gp.scan_param_cmpl.status=ESP_BT_STATUS_SUCCESS;
 obd_handoff_gap(ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT,&gp);CHECK(scan_calls==0);
 esp_ble_gattc_cb_param_t cp={0};cp.open.status=ESP_GATT_OK;cp.open.conn_id=7;
 client_event(ESP_GATTC_OPEN_EVT,3,&cp);CHECK(close_calls==1&&search_calls==0);
 // No new lease/configuration until outstanding open/connection is closed.
 active=false;worker=NULL;opening=true;CHECK(!obd_handoff_command("vKMHM141AAN0123456",out,sizeof(out)));
 CHECK(!obd_handoff_command("n12345678,4f4244",out,sizeof(out)));
 CHECK(!obd_handoff_command("r",out,sizeof(out)));
 opening=false;link_up=true;state="closing";
 cp.disconnect.conn_id=9;client_event(ESP_GATTC_DISCONNECT_EVT,3,&cp);CHECK(link_up&&!strcmp(state,"closing"));
 cp.disconnect.conn_id=7;client_event(ESP_GATTC_DISCONNECT_EVT,3,&cp);CHECK(!link_up&&!strcmp(state,"failed"));
 CHECK(obd_handoff_command("r",out,sizeof(out)));CHECK(!managed&&!authorized);
 // A different peer's OPEN never becomes our adapter or starts discovery.
 reset();start();cp=(esp_ble_gattc_cb_param_t){0};cp.open.status=ESP_GATT_OK;cp.open.conn_id=12;cp.open.remote_bda[0]=1;
 client_event(ESP_GATTC_OPEN_EVT,3,&cp);CHECK(close_calls==1&&!link_up&&search_calls==0);
 // Only active connection callbacks may mutate discovery/readiness.
 reset();start();link_up=true;service_count=0;
 cp=(esp_ble_gattc_cb_param_t){0};cp.search_res.conn_id=8;cp.search_res.srvc_id.uuid.len=2;cp.search_res.srvc_id.uuid.uuid.uuid16=0xfff0;
 client_event(ESP_GATTC_SEARCH_RES_EVT,3,&cp);CHECK(service_count==0);
 cp.search_res.conn_id=7;client_event(ESP_GATTC_SEARCH_RES_EVT,3,&cp);CHECK(service_count==1);
 cccd=22;cp.write.conn_id=8;cp.write.handle=22;cp.write.status=ESP_GATT_OK;
 client_event(ESP_GATTC_WRITE_DESCR_EVT,3,&cp);CHECK(!(bits&READY));
 // Valid identity + fresh clear is a short-lived lease, never cached consent.
 identity_ok=clear=true;observed_at=fake_now;CHECK(!obd_handoff_blocks());
 fake_now+=FRESH_US+1;CHECK(obd_handoff_blocks());obd_handoff_tick();CHECK(hold_calls>0);
 // Old unconsumed observations are discarded at new lease start.
 reset();observation_serial=4;applied_serial=3;observation_on=true;start();obd_handoff_tick();CHECK(observe_calls==0);
 // Overflow/failure combined with prompt cannot satisfy a new request.
 link_up=true;write_handle=1;wait_result=REPLY|DROPPED;char r[32];CHECK(!send_cmd("22BC03",r,sizeof(r)));CHECK(write_calls==1);
 // A fresh app ON cancels direct ownership without releasing the live client.
 obd_handoff_note_on();CHECK(cancelled&&managed&&obd_handoff_blocks());
 // Complete names in a padded scan response are resolved independently of AD.
 reset();start();matches=0;gp=(esp_ble_gap_cb_param_t){0};gp.scan_rst.search_evt=ESP_GAP_SEARCH_INQ_RES_EVT;
 gp.scan_rst.adv_data_len=5;gp.scan_rst.scan_rsp_len=5;gp.scan_rst.ble_adv[5]=4;gp.scan_rst.ble_adv[6]=9;
 memcpy(gp.scan_rst.ble_adv+7,"OBD",3);gp.scan_rst.bda[0]=4;
 obd_handoff_gap(ESP_GAP_BLE_SCAN_RESULT_EVT,&gp);CHECK(matches==1&&selected[0]==4);
 gp.scan_rst.bda[0]=5;obd_handoff_gap(ESP_GAP_BLE_SCAN_RESULT_EVT,&gp);CHECK(matches==2);
 // Registration and callback guards isolate unrelated GATT applications.
 reset();cp=(esp_ble_gattc_cb_param_t){0};cp.reg.status=ESP_GATT_OK;cp.reg.app_id=0x99;
 client_event(ESP_GATTC_REG_EVT,8,&cp);CHECK(client==3);
 // Run the actual worker end to end: only both physical lock reads and
 // acknowledged GATT close may produce a terminal successful handoff.
 reset();start();scripted=true;run(NULL);
 CHECK(!active&&!link_up&&!worker&&!strcmp(state,"done")&&!strcmp(doors,"locked"));
 CHECK(managed&&obd_handoff_blocks());CHECK(body_reads>=2&&doors_at>0);
 obd_handoff_tick();CHECK(managed&&obd_handoff_blocks());   // done: ownership stays
 reset();start();scripted=true;ack_close=false;run(NULL);
 CHECK(!active&&link_up&&!strcmp(state,"closing"));
 CHECK(!obd_handoff_command("n12345678,4f4244",out,sizeof(out)));
 obd_handoff_tick();CHECK(managed);   // adapter link still up: not yet
 cp=(esp_ble_gattc_cb_param_t){0};cp.disconnect.conn_id=7;client_event(ESP_GATTC_DISCONNECT_EVT,3,&cp);
 CHECK(!strcmp(state,"failed")&&!link_up);
 obd_handoff_tick();CHECK(!managed);
 // A complete VIN reply from another vehicle never reaches safety polling.
 reset();start();scripted=true;wrong_vin=true;run(NULL);
 CHECK(!strcmp(state,"failed")&&body_reads==0&&observe_calls==0&&obd_handoff_blocks());
 // A failed lease hands safety back to the app on the next tick, with the
 // hold it latched at finish still on (2026-11-16 21:00 field failure).
 hold_calls=0;obd_handoff_tick();
 CHECK(!managed&&!obd_handoff_blocks()&&!obd_handoff_managed());
 CHECK(lease_ended==1&&lease_touched);   // VIN probing touched the car: no refund
 // Every started lease uses the departure's lease; a scan that never finds
 // the adapter ends without touching the car.
 reset();start();CHECK(lease_started==1);run(NULL);
 CHECK(!strcmp(state,"failed")&&lease_ended==1&&!lease_touched&&write_calls==0);
 // WiCAN's own lock: success, but not an owner lock.
 reset();start();scripted=true;run(NULL);
 CHECK(!strcmp(state,"done")&&owner_calls==0);obd_handoff_tick();CHECK(owner_calls==0);
 // Owner lock: door opened after the drive, then all locked with no lock
 // burst from WiCAN. The adapter is released at once and auto-lock hears
 // about it on its own task.
 reset();start();scripted=true;owner_script=true;run(NULL);
 CHECK(!strcmp(state,"done")&&!strcmp(doors,"locked")&&owner_locked&&!link_up);
 CHECK(bc03_reads==2&&lease_ended==1&&lease_touched);
 obd_handoff_tick();CHECK(owner_calls==1&&!owner_locked);
 obd_handoff_tick();CHECK(owner_calls==1);
 // All locked without any door opening first (a car that locked itself
 // while driving, driver still seated): not an owner lock; keep monitoring
 // until the lease runs out.
 reset();start();scripted=true;owner_script=true;bc03_reads=1;run(NULL);
 CHECK(owner_calls==0&&!owner_locked&&strcmp(doors,"locked")!=0);
 // The phone's iOS link to the adapter can take seconds to drop; an adapter
 // that starts advertising on the third 2 s scan still completes the lease.
 reset();start();scripted=true;silent_scans=2;run(NULL);
 CHECK(scan_param_calls==3&&!strcmp(state,"done")&&!strcmp(doors,"locked"));
 // Never advertising: four scans, then a failed lease that hands back.
 reset();start();scripted=true;silent_scans=99;run(NULL);
 CHECK(scan_param_calls==4&&!strcmp(state,"failed")&&write_calls==0&&!lease_touched);
 obd_handoff_tick();CHECK(!managed);
 return test_report("OBD handoff transport");
}
