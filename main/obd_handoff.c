// Experimental BLE central. The phone remains a GATT client of WiCAN while
// this independent client owns the selected ELM pipe for a bounded lease.
#include "obd_handoff.h"
#include "obd_handoff_decode.h"
#include "autolock.h"
#include "bodycmd.h"
#include "ble.h"
#include "standby.h"
#ifndef OBD_HANDOFF_HOST_TEST
#include "esp_gattc_api.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#endif
#include <string.h>
#include <stdio.h>
#include <ctype.h>

#define READY BIT0
#define REPLY BIT1
#define DROPPED BIT2
#define SCANNED BIT3
#define CLOSED BIT4
#define GATT_APP 0x48
#define LEASE_US 90000000LL
// Switched off (2026-11-16): on the WiCAN's single radio, scanning for and
// connecting to the OBD adapter starved the phone's own link. Its commands
// timed out and it dropped within seconds of every lease (21:00, 21:34 field
// logs); the one lease that worked had the phone's Bluetooth off. The phone
// keeps its OBD adapter and forwards door/belt readings instead. ATXOH then
// reports XOH:0, which the app reads as "unavailable" before releasing
// anything. Host tests switch it back on to keep covering the worker.
#ifndef OBD_HANDOFF_ENABLED
#define OBD_HANDOFF_ENABLED 0
#endif
static bool handoff_enabled = OBD_HANDOFF_ENABLED;
#define FRESH_US 1500000LL
static const char *TAG="obd_handoff";
static EventGroupHandle_t events;
static TaskHandle_t worker;
static esp_gatt_if_t client=ESP_GATT_IF_NONE;
static uint16_t conn,write_handle,notify_handle,cccd;
static bool write_response;
static volatile bool link_up,opening,active,cancelled,identity_ok,clear;
static bool managed;
static char token[9]="-";
static int64_t doors_at;
static volatile int64_t deadline,observed_at;
static const char *state="idle", *doors="unknown";
static char target_name[30], vin[18];
static esp_bd_addr_t selected;
static esp_ble_addr_type_t selected_type;
static unsigned matches;
static uint16_t starts[8],ends[8];static unsigned service_count;
static char reply[512];static size_t reply_len;
static portMUX_TYPE mux=portMUX_INITIALIZER_UNLOCKED;
static uint32_t locks_before;
static uint32_t observation_serial,applied_serial;
static bool observation_on,observation_blocked;
// Set by the worker when the owner locked the car during the lease; consumed
// by obd_handoff_tick on the auto-lock task.
static volatile bool owner_locked;
// The lease sent at least one diagnostic request (ATZ onwards), so a failure
// can't give the departure's lease back.
static volatile bool touched;
// A missing or incomplete observation always blocks while ownership is ours.
bool obd_handoff_blocks(void) {
    portENTER_CRITICAL(&mux);
    bool blocked=managed&&(!active||cancelled||!identity_ok||!clear||observed_at==0||esp_timer_get_time()-observed_at>FRESH_US);
    portEXIT_CRITICAL(&mux);return blocked;
}
bool obd_handoff_managed(void) { return managed; }
void obd_handoff_note_on(void) { if(active)obd_handoff_stop();else managed=false; }
static void hold(void) { char response[48]; autolock_command("b1",response,sizeof(response)); }
void obd_handoff_stop(void) {if(!active)return;cancelled=true;clear=false;hold();if(events)xEventGroupSetBits(events,DROPPED);}
static bool alive(void){return active&&!cancelled&&esp_timer_get_time()<deadline;}
static bool send_cmd(const char *cmd,char *out,size_t cap) {
    if(!alive()||!link_up||!write_handle)return false;
    char wire[32];int len=snprintf(wire,sizeof(wire),"%s\r",cmd);if(len<=0||(size_t)len>=sizeof(wire))return false;
    xEventGroupClearBits(events,REPLY);
    portENTER_CRITICAL(&mux);reply_len=0;reply[0]=0;portEXIT_CRITICAL(&mux);
    if(esp_ble_gattc_write_char(client,conn,write_handle,len,(uint8_t*)wire,
        write_response?ESP_GATT_WRITE_TYPE_RSP:ESP_GATT_WRITE_TYPE_NO_RSP,ESP_GATT_AUTH_REQ_NONE)!=ESP_OK)return false;
    EventBits_t bits=xEventGroupWaitBits(events,REPLY|DROPPED,pdTRUE,pdFALSE,pdMS_TO_TICKS(!strcmp(cmd,"ATZ")?4000:1200));
    if(!(bits&REPLY)||(bits&DROPPED)||!alive()||!link_up)return false;
    portENTER_CRITICAL(&mux);snprintf(out,cap,"%s",reply);portEXIT_CRITICAL(&mux);
    return true;
}
static bool ok(const char *cmd){
    char r[64];if(!send_cmd(cmd,r,sizeof(r)))return false;
    char compact[64];size_t n=0;
    for(size_t i=0;r[i]&&n+1<sizeof(compact);i++)if(!isspace((unsigned char)r[i])&&r[i]!='>')compact[n++]=r[i];
    compact[n]=0;return !strcmp(compact,"OK");
}
static bool setup_header(const char *header) {
    char cmd[24];snprintf(cmd,sizeof(cmd),"ATSH%s",header);if(!ok(cmd))return false;
    snprintf(cmd,sizeof(cmd),"ATFCSH%s",header);
    return ok(cmd)&&ok("ATCRA778")&&ok("ATFCSD300000")&&ok("ATFCSM1");
}
static void client_event(esp_gattc_cb_event_t ev,esp_gatt_if_t gi,esp_ble_gattc_cb_param_t *p) {
    if(ev==ESP_GATTC_REG_EVT){if(p->reg.app_id==GATT_APP&&p->reg.status==ESP_GATT_OK)client=gi;return;}
    if(gi!=client)return;
    if(ev!=ESP_GATTC_OPEN_EVT&&ev!=ESP_GATTC_DISCONNECT_EVT&&(!alive()||!link_up))return;
    switch(ev){
    case ESP_GATTC_OPEN_EVT:
        opening=false;
        if(p->open.status!=ESP_GATT_OK){if(!active&&!worker)state="failed";xEventGroupSetBits(events,DROPPED);break;}
        if(memcmp(p->open.remote_bda,selected,6)!=0){esp_ble_gattc_close(client,p->open.conn_id);xEventGroupSetBits(events,DROPPED);break;}
        conn=p->open.conn_id;link_up=true;
        if(!alive()){esp_ble_gattc_close(client,conn);break;}
        // Discovery needs no increased MTU; short ELM writes are <=20 bytes.
        service_count=0;esp_ble_gattc_search_service(client,conn,NULL);break;
    case ESP_GATTC_SEARCH_RES_EVT: {
        if(p->search_res.conn_id!=conn)break;
        esp_bt_uuid_t *u=&p->search_res.srvc_id.uuid;
        bool uart=u->len==ESP_UUID_LEN_16&&(u->uuid.uuid16==0xfff0||u->uuid.uuid16==0xffe0||u->uuid.uuid16==0x18f0||u->uuid.uuid16==0xffe5);
        static const uint8_t nus[16]={0x9e,0xca,0xdc,0x24,0x0e,0xe5,0xa9,0xe0,0x93,0xf3,0xa3,0xb5,0x01,0x00,0x40,0x6e};
        uart|=u->len==ESP_UUID_LEN_128&&memcmp(u->uuid.uuid128,nus,16)==0;
        if(uart&&service_count<8){starts[service_count]=p->search_res.start_handle;ends[service_count++]=p->search_res.end_handle;}break;
    }
    case ESP_GATTC_SEARCH_CMPL_EVT: {
        if(p->search_cmpl.conn_id!=conn)break;
        if(p->search_cmpl.status!=ESP_GATT_OK){xEventGroupSetBits(events,DROPPED);break;}
        write_handle=notify_handle=0;
        for(unsigned s=0;s<service_count&&!write_handle;s++){
            esp_gattc_char_elem_t chars[12];uint16_t n=12;
            if(esp_ble_gattc_get_all_char(client,conn,starts[s],ends[s],chars,&n,0)!=ESP_GATT_OK)continue;
            uint16_t wr=0,nt=0;bool rsp=false;
            for(unsigned i=0;i<n;i++){
                if(chars[i].properties&(ESP_GATT_CHAR_PROP_BIT_WRITE|ESP_GATT_CHAR_PROP_BIT_WRITE_NR)){wr=chars[i].char_handle;rsp=(chars[i].properties&ESP_GATT_CHAR_PROP_BIT_WRITE)!=0;}
                if(chars[i].properties&ESP_GATT_CHAR_PROP_BIT_NOTIFY)nt=chars[i].char_handle;
            }
            if(wr&&nt){write_handle=wr;notify_handle=nt;write_response=rsp;}
        }
        if(!write_handle||!notify_handle){xEventGroupSetBits(events,DROPPED);break;}
        if (esp_ble_gattc_register_for_notify(client, selected, notify_handle) != ESP_OK) {
            xEventGroupSetBits(events, DROPPED);
        }
        break;
    }
    case ESP_GATTC_REG_FOR_NOTIFY_EVT: {
        if(p->reg_for_notify.handle!=notify_handle)break;
        if(p->reg_for_notify.status!=ESP_GATT_OK){xEventGroupSetBits(events,DROPPED);break;}
        esp_bt_uuid_t uuid={.len=ESP_UUID_LEN_16,.uuid.uuid16=ESP_GATT_UUID_CHAR_CLIENT_CONFIG};
        esp_gattc_descr_elem_t d;uint16_t n=1;
        if(esp_ble_gattc_get_descr_by_char_handle(client,conn,notify_handle,uuid,&d,&n)!=ESP_GATT_OK||n!=1){xEventGroupSetBits(events,DROPPED);break;}
        cccd=d.handle;uint8_t value[2]={1,0};
        if (esp_ble_gattc_write_char_descr(client, conn, cccd, 2, value,
                ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE) != ESP_OK) {
            xEventGroupSetBits(events, DROPPED);
        }
        break;
    }
    case ESP_GATTC_WRITE_DESCR_EVT:
        if (p->write.conn_id == conn && p->write.handle == cccd) {
            xEventGroupSetBits(events, p->write.status == ESP_GATT_OK ? READY : DROPPED);
        }
        break;
    case ESP_GATTC_NOTIFY_EVT:
        if(p->notify.conn_id!=conn||p->notify.handle!=notify_handle||!active)break;
        portENTER_CRITICAL(&mux);
        if(reply_len+p->notify.value_len>=sizeof(reply)){reply_len=0;reply[0]=0;portEXIT_CRITICAL(&mux);xEventGroupSetBits(events,DROPPED);break;}
        memcpy(reply+reply_len,p->notify.value,p->notify.value_len);reply_len+=p->notify.value_len;reply[reply_len]=0;
        bool complete=memchr(p->notify.value,'>',p->notify.value_len)!=NULL;
        portEXIT_CRITICAL(&mux);if(complete)xEventGroupSetBits(events,REPLY);break;
    case ESP_GATTC_DISCONNECT_EVT:
        if(p->disconnect.conn_id==conn){link_up=false;clear=false;if(!active&&!worker)state="failed";xEventGroupSetBits(events,DROPPED|CLOSED);}break;
    default:break;
    }
}
void obd_handoff_gap(esp_gap_ble_cb_event_t ev,esp_ble_gap_cb_param_t *p) {
    if(!alive())return;
    if(ev==ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT){
        if(p->scan_param_cmpl.status!=ESP_BT_STATUS_SUCCESS||esp_ble_gap_start_scanning(2)!=ESP_OK)xEventGroupSetBits(events,DROPPED);
    } else if(ev==ESP_GAP_BLE_SCAN_RESULT_EVT){
        if(p->scan_rst.search_evt==ESP_GAP_SEARCH_INQ_CMPL_EVT){xEventGroupSetBits(events,SCANNED);return;}
        if(p->scan_rst.search_evt!=ESP_GAP_SEARCH_INQ_RES_EVT)return;
        uint8_t len=0;uint8_t *name=esp_ble_resolve_adv_data_by_type(p->scan_rst.ble_adv,p->scan_rst.adv_data_len,ESP_BLE_AD_TYPE_NAME_CMPL,&len);
        if(!name&&p->scan_rst.scan_rsp_len)name=esp_ble_resolve_adv_data_by_type(p->scan_rst.ble_adv+p->scan_rst.adv_data_len,p->scan_rst.scan_rsp_len,ESP_BLE_AD_TYPE_NAME_CMPL,&len);
        // Only complete names qualify; shortened prefix matching is unsafe.
        if(!name||len!=strlen(target_name)||memcmp(name,target_name,len)!=0)return;
        if(matches&&memcmp(selected,p->scan_rst.bda,6)==0)return;
        if(matches<2)matches++;
        memcpy(selected,p->scan_rst.bda,6);selected_type=p->scan_rst.ble_addr_type;
    }
}
static bool connect_adapter(void) {
    state="scanning";matches=0;write_handle=notify_handle=0;xEventGroupClearBits(events,0xff);
    esp_ble_scan_params_t params={.scan_type=BLE_SCAN_TYPE_ACTIVE,.own_addr_type=BLE_ADDR_TYPE_PUBLIC,
      .scan_filter_policy=BLE_SCAN_FILTER_ALLOW_ALL,.scan_interval=0x50,.scan_window=0x30,.scan_duplicate=BLE_SCAN_DUPLICATE_DISABLE};
    // An adapter still connected to the phone doesn't advertise, and iOS can
    // take seconds to drop the link after the app lets go. One 2 s window
    // failed the whole lease (2026-11-16 21:00); keep scanning in 2 s windows
    // for up to four tries while nothing matches. Two matches stay a failure.
    EventBits_t b=0;
    for(int attempt=0;attempt<4&&alive();attempt++){
        xEventGroupClearBits(events,SCANNED);
        if(esp_ble_gap_set_scan_params(&params)!=ESP_OK)return false;
        b=xEventGroupWaitBits(events,SCANNED|DROPPED,pdTRUE,pdFALSE,pdMS_TO_TICKS(3500));
        if(!(b&SCANNED)||(b&DROPPED)||matches!=0)break;
        ESP_LOGI(TAG,"adapter not advertising yet (scan %d of 4)",attempt+1);
    }
    if(!(b&SCANNED)||(b&DROPPED)||matches!=1||!alive())return false;
    state="connecting";
    opening=true;
    if(esp_ble_gattc_open(client,selected,selected_type,true)!=ESP_OK){opening=false;return false;}
    b=xEventGroupWaitBits(events,READY|DROPPED,pdTRUE,pdFALSE,pdMS_TO_TICKS(5000));
    return (b&READY)&&!(b&DROPPED)&&alive()&&link_up;
}
static void run(void *unused) {
    // Give the phone time to release the adapter after receiving the ACK.
    vTaskDelay(pdMS_TO_TICKS(750));
    char r[512];bool success=false;
    if(!alive()||!connect_adapter())goto finished;
    state="probing";touched=true;
    // Reset the inherited ELM state before sending any diagnostic request.
    if(!send_cmd("ATZ",r,sizeof(r))||!ok("ATE0")||!ok("ATL0")||!ok("ATS0")||!ok("ATH0")||!ok("ATSP6")||!ok("ATAL"))goto finished;
    if(!ok("ATSH7DF")||!send_cmd("0902",r,sizeof(r))||!obd_handoff_vin(r,vin))goto finished;
    identity_ok=true;
    if(!setup_header("770"))goto finished;
    bodycmd_status_t status;bodycmd_get_status(&status);locks_before=status.locks_completed;
    bool saw_opening=false; // someone left the car after the drive
    while(alive()&&link_up){
        bool on=false,blocked=true,rear=false,front=false;
        if(!send_cmd("22BC03",r,sizeof(r))||!obd_handoff_safety(r,&on,&blocked,&rear))goto finished;
        // Publish only positively decoded observations. A real ON is never
        // fabricated to force an OFF-only handoff to arm.
        portENTER_CRITICAL(&mux);clear=!blocked;observed_at=esp_timer_get_time();portEXIT_CRITICAL(&mux);
        portENTER_CRITICAL(&mux);observation_on=on;observation_blocked=blocked;observation_serial++;portEXIT_CRITICAL(&mux);
        bool open_now=false;
        if(obd_handoff_opening(r,&open_now)&&open_now)saw_opening=true;
        state="monitoring";
        bodycmd_get_status(&status);
        if(status.locks_completed>locks_before){
            state="verifying";
            if(!send_cmd("22BC03",r,sizeof(r))||!obd_handoff_safety(r,&on,&blocked,&rear))goto finished;
            portENTER_CRITICAL(&mux);clear=!blocked;observed_at=esp_timer_get_time();portEXIT_CRITICAL(&mux);
            portENTER_CRITICAL(&mux);observation_on=on;observation_blocked=blocked;observation_serial++;portEXIT_CRITICAL(&mux);
            if(!send_cmd("22BC04",r,sizeof(r))||!obd_handoff_front(r,&front))goto finished;
            portENTER_CRITICAL(&mux);doors=rear&&front?"locked":"unlocked";doors_at=esp_timer_get_time();portEXIT_CRITICAL(&mux);
            if(rear&&front){success=true;goto finished;}
        } else if(saw_opening&&!on&&!blocked&&rear){
            // Owner lock: a door opened after the drive, now everything is
            // shut, the car is off, nobody is belted and the rear doors read
            // locked. Confirm the fronts, then release the adapter at once.
            // Requiring the earlier opening keeps a driver still seated in a
            // car that locked itself while driving from counting as a lock.
            if(!send_cmd("22BC04",r,sizeof(r))||!obd_handoff_front(r,&front))goto finished;
            bodycmd_get_status(&status);
            if(front&&status.locks_completed==locks_before){
                portENTER_CRITICAL(&mux);doors="locked";doors_at=esp_timer_get_time();portEXIT_CRITICAL(&mux);
                owner_locked=true;success=true;goto finished;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(250));
    }
finished:
    clear=false;identity_ok=false;hold();
    // Close our client only. The phone's GATT server connection is independent.
    esp_ble_gap_stop_scanning();
    if(link_up){
        if(notify_handle)esp_ble_gattc_unregister_for_notify(client,selected,notify_handle);
        state="closing";xEventGroupClearBits(events,CLOSED);esp_ble_gattc_close(client,conn);
        xEventGroupWaitBits(events,CLOSED,pdTRUE,pdFALSE,pdMS_TO_TICKS(2000));
        if(link_up){success=false;ESP_LOGW(TAG,"client close not acknowledged");}
    } // ESP-IDF has no public cancel-open API. A late OPEN callback sees
    // alive()==false and immediately closes; another lease is blocked meanwhile.
    state=success&&!link_up&&!opening?"done":(link_up||opening?"closing":"failed");
    active=false;worker=NULL;
    standby_note_lease_ended(touched);
    ESP_LOGI(TAG,"handoff %s, doors %s%s",state,doors,owner_locked?" (owner)":"");
    vTaskDelete(NULL);
}
void obd_handoff_tick(void) {
    if(owner_locked){owner_locked=false;autolock_owner_locked();}
    // A failed lease hands safety ownership back to the app. Leaving it
    // managed swallowed every later ATXALP0/ATXALB0 while the guard kept
    // blocking, so one failed handoff vetoed walk-away until the next drive
    // (2026-11-16 21:00 field log). The hold latched at finish stays on: only
    // a reading the app sends after this point can clear it, so stale data
    // still never authorizes a lock. A successful lease ("done") keeps
    // ownership; there is nothing left to lock.
    if(managed&&!active&&!worker&&!link_up&&!opening&&!strcmp(state,"failed")){
        managed=false;
        ESP_LOGI(TAG,"handoff failed; the app's next fresh reading decides the lock");
    }
    if(active){
        bool apply=false,on=false,blocked=true;
        portENTER_CRITICAL(&mux);
        if(observation_serial!=applied_serial){
            applied_serial=observation_serial;apply=true;on=observation_on;blocked=observation_blocked;
        }
        portEXIT_CRITICAL(&mux);
        if(apply&&alive())autolock_handoff_observation(on,blocked);
        if(!alive()||obd_handoff_blocks())hold();
    }
}
void obd_handoff_init(void) {
    if (!handoff_enabled) return;   // no GATT client registered at all
    if (!events) {
        events = xEventGroupCreate();
    }
    if (!events) return;
    client=ESP_GATT_IF_NONE;link_up=false;opening=false;
    xEventGroupClearBits(events,0xff);
    if(esp_ble_gattc_register_callback(client_event)!=ESP_OK)return;
    esp_ble_gattc_app_register(GATT_APP);
}
bool obd_handoff_shutdown(void) {
    obd_handoff_stop();
    int64_t until=esp_timer_get_time()+4200000LL;
    while(worker&&esp_timer_get_time()<until)vTaskDelay(pdMS_TO_TICKS(10));
    if(worker){ESP_LOGW(TAG,"BLE shutdown deferred until OBD worker exits");return false;}
    // The caller now destroys the host/controller, which terminates pending
    // connections too. Ignore callbacks from that old client registration.
    client=ESP_GATT_IF_NONE;link_up=false;opening=false;
    return true;
}
static int hex(char c){if(c>='0'&&c<='9')return c-'0';c=(char)tolower((unsigned char)c);return c>='a'&&c<='f'?c-'a'+10:-1;}
bool obd_handoff_command(const char *args,char *out,size_t cap) {
    if(!handoff_enabled){
        if(!*args){snprintf(out,cap,"XOH:0,disabled");return true;}
        return false;
    }
    if(!*args){int remain=active?(int)((deadline-esp_timer_get_time())/1000000):0;
        portENTER_CRITICAL(&mux);int64_t sample_at=doors_at;portEXIT_CRITICAL(&mux);
        snprintf(out,cap,"XOH:1,%s,%d,%s,%s,%s,%lld",state,remain>0?remain:0,
            active?(obd_handoff_blocks()?"blocked":"clear"):"unknown",doors,token,
            sample_at?(long long)((esp_timer_get_time()-sample_at)/1000):-1LL);return true;}
    if(!strcmp(args,"r")){if(active||worker||opening||link_up)return false;autolock_handoff_invalidate();managed=false;snprintf(out,cap,"OK");return true;}
    if(!strcmp(args,"0")){if(active)obd_handoff_stop();snprintf(out,cap,"OK");return true;}
    if(args[0]=='v'){
        if(active||worker||link_up||opening||strlen(args+1)!=17)return false;
        for(int i=0;i<17;i++){char c=(char)toupper((unsigned char)args[i+1]);if(!isalnum((unsigned char)c)||c=='I'||c=='O'||c=='Q')return false;vin[i]=c;}
        vin[17]=0;snprintf(out,cap,"OK");return true;
    }
    if(args[0]=='n'){
        size_t len=strlen(args);if(len<12||args[9]!=',')return false;
        for(int i=0;i<8;i++){if(hex(args[1+i])<0)return false;}
        const char *name=args+10;size_t n=strlen(name);
        if(active||worker||link_up||opening||client==ESP_GATT_IF_NONE||!events||!ble_connected()||!autolock_can_handoff()||strlen(vin)!=17||n<2||n>58||n%2)return false;
        for(size_t i=0;i<n/2;i++){int h=hex(name[2*i]),l=hex(name[1+2*i]);if(h<0||l<0||!(h|l))return false;target_name[i]=(char)((h<<4)|l);}target_name[n/2]=0;
        for (int i = 0; i < 8; i++) {
            token[i] = (char)toupper((unsigned char)args[i + 1]);
        }
        token[8] = 0;
        cancelled=false;identity_ok=false;clear=false;observed_at=0;doors_at=0;doors="unknown";state="scanning";
        owner_locked=false;touched=false;
        portENTER_CRITICAL(&mux);applied_serial=observation_serial;portEXIT_CRITICAL(&mux);
        deadline=esp_timer_get_time()+LEASE_US;managed=true;active=true;hold();
        if(xTaskCreate(run,"obd_handoff",4096,NULL,4,&worker)!=pdPASS){active=false;state="failed";worker=NULL;return false;}
        standby_note_lease_started();
        snprintf(out,cap,"OK");return true;
    }
    return false;
}
