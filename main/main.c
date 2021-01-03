#include <stdio.h> 
#include <stdlib.h> 
#include <string.h>
#include "freertos/FreeRTOS.h" 
#include "freertos/task.h" 
#include "freertos/event_groups.h" 
#include "esp_system.h"
#include "esp_wifi.h" 
#include "driver/gpio.h" 
#include "driver/adc.h" 
#include "esp_event_loop.h" 
#include "esp_log.h" 
#include "sdkconfig.h" 
#include "nvs.h"
#include "nvs_flash.h" 
#include "esp_http_client.h" 
#include "lwip/err.h" 
#include "esp_sntp.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"
#include "time.h"
#include "sys/time.h"

#include "esp_attr.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"

#if CONFIG_IDF_TARGET_ESP32
#include "esp_adc_cal.h" 
#endif

#if CONFIG_IDF_TARGET_ESP32
static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t channel = ADC_CHANNEL_6;	//GPIO34 if ADC1, GPIO14 if ADC2
#elif CONFIG_IDF_TARGET_ESP32S2BETA
static const adc_channel_t channel = ADC_CHANNEL_6;	// GPIO7 if ADC1, GPIO17 if ADC2
#endif
static const adc_atten_t atten = ADC_ATTEN_DB_0; 
static const adc_unit_t unit = ADC_UNIT_1;

#define LED_RED 18
#define LED_GREEN 21
#define LED_BLUE 19
#define LED 22
#define BUZZ 23
#define DEFAULT_VREF 1100	//Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES 64	//Multisampling 
#define MAX_HTTP_RECV_BUFFER 512
#define WIFI_SSID "INI-HPKU" 
#define WIFI_PASS "12345678"
#define GPIO_PWM0A_OUT 15   //Set GPIO 15 as PWM0A

#define SPP_TAG "SPP_ACCEPTOR_DEMO"
#define SPP_SERVER_NAME "SPP_SERVER"
#define EXAMPLE_DEVICE_NAME "INI-ESPKU"
#define SPP_SHOW_DATA 0
#define SPP_SHOW_SPEED 1
#define SPP_SHOW_MODE SPP_SHOW_SPEED


//Global Variable
uint32_t adc_reading,voltage,celcius;
static char strftime_buf[32],led[6],Site[300],target[48],bt_data[5];

static const esp_spp_mode_t esp_spp_mode = ESP_SPP_MODE_CB;

static struct timeval time_new, time_old;
static long data_num = 0;

static const esp_spp_sec_t sec_mask = ESP_SPP_SEC_AUTHENTICATE;
static const esp_spp_role_t role_slave = ESP_SPP_ROLE_SLAVE;
static void bt_main(void);

#if CONFIG_IDF_TARGET_ESP32
static void check_efuse(void)
{
    //Check TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) { 
        printf("eFuse Two Point: Supported\n");
    } else {
        printf("eFuse Two Point: NOT supported\n");
    }

    //Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) { 
        printf("eFuse Vref: Supported\n");
    } else {
        printf("eFuse Vref: NOT supported\n");
    }
}

static void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) { 
        printf("Characterized using Two Point Value\n");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) { 
        printf("Characterized using eFuse Vref\n");
    } else {
        printf("Characterized using Default Vref\n");
    }
}
#endif

static void initialize_sntp(void); 
static void initialise_wifi(void);
static esp_err_t event_handler(void *ctx, system_event_t *event); 
esp_err_t _http_event_handler(esp_http_client_event_t *evt); 
static void trigger_http_request(const char *url);
static void http_request_task(void *pvParameters); 
static void get_time(void);

/* FreeRTOS event group to signal when we are connected & ready to make a request */
static EventGroupHandle_t wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about one event - are we connected to the AP with an IP? */
const int CONNECTED_BIT = BIT0; 
static const char *TAG = "Cloud Suhu";

/* @brief: Function to initialise WiFi
*  @param:
*  @retval:
*/
static void get_time(void){ 
setenv("TZ", "UTC", 1);
    time_t now;
    struct tm timeinfo; 
    time(&now);  
    localtime_r(&now, &timeinfo);
    // Is time set? If not, tm_year will be (1970 - 1900). 
    if (timeinfo.tm_year < (2016 - 1900)) {
        ESP_LOGI(TAG, "Time is not set yet. Connecting to WiFi and getting time over NTP.");
    while (sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET) { 
        ESP_LOGI(TAG, "Waiting for system time to be set... "); 
        vTaskDelay(2000 / portTICK_PERIOD_MS);
}
time(&now);  
localtime_r(&now, &timeinfo);
    time(&now);
}
tzset();
localtime_r(&now, &timeinfo); 
time_t t = now + (3600 * 7 ); 
struct tm timeinfo_utc7; 
localtime_r(&t , &timeinfo_utc7);
strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo_utc7 );

int y = 0;
    for (int i = 0; strftime_buf[i] != NULL; i++){ 
        if (strftime_buf[i] == ' '){
            target[y] = '%';
            target[y + 1] = '2';
            target[y + 2] = '0';
            y += 3; 
        }
        else
        {
            target[y] = strftime_buf[i]; 
            y++;
        }
    }
}
static void initialise_wifi(void)
{
    tcpip_adapter_init();
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) ); 
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT(); 
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) ); 
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };
    ESP_LOGI(TAG, "Setting WiFi configuration SSID %s...", wifi_config.sta.ssid);
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) ); ESP_ERROR_CHECK( esp_wifi_start() );
}

static void initialize_sntp(void)
{
    ESP_LOGI(TAG, "Initializing SNTP"); 
    sntp_setoperatingmode(SNTP_OPMODE_POLL); 
    sntp_setservername(0, "pool.ntp.org"); 
    sntp_init();
}

//============================== PWM ============================
static void mcpwm_example_gpio_initialize(void)
{
    printf("initializing mcpwm gpio...\n");
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_PWM0A_OUT);
}

static void brushed_motor_forward(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num , float duty_cycle)
{
    mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_A, duty_cycle);
    mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_OPR_A, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state
}

static void brushed_motor_stop(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num)
{
    mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_A);
}

//============================ END PWM ===========================

//===========================bt=======================
static void delay(int number_of_seconds) 
{ 
    // Converting time into milli_seconds 
    int milli_seconds = 1000 * number_of_seconds; 
  
    // Storing start time 
    clock_t start_time = clock(); 
  
    // looping till required time is not achieved 
    while (clock() < start_time + milli_seconds); 
} 

static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
    switch (event) {
    case ESP_SPP_INIT_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_INIT_EVT");
        esp_bt_dev_set_device_name(EXAMPLE_DEVICE_NAME);
        esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
        esp_spp_start_srv(sec_mask,role_slave, 0, SPP_SERVER_NAME);
        break;
    case ESP_SPP_DISCOVERY_COMP_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_DISCOVERY_COMP_EVT");
        break;
    case ESP_SPP_OPEN_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_OPEN_EVT");
        break;
    case ESP_SPP_CLOSE_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_CLOSE_EVT");
        break;
    case ESP_SPP_START_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_START_EVT");
        break;
    case ESP_SPP_CL_INIT_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_CL_INIT_EVT");
        break;
    case ESP_SPP_DATA_IND_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_DATA_IND_EVT len=%d handle=%d",
                 param->data_ind.len, param->data_ind.handle);
        esp_log_buffer_hex("",param->data_ind.data,param->data_ind.len);

        strncpy(bt_data, (char*) param-> data_ind.data, sizeof(bt_data));
        bt_data[param->data_ind.len]=NULL;

        char id[2] = {0};
        int time;

        sscanf(bt_data,"%[a-z];%d",id,&time);

        if((strcmp(bt_data,"LON") == 0)){
            gpio_set_level(LED, 1);
        }
        else if((strcmp(bt_data,"LOFF") == 0)){
            gpio_set_level(LED, 0);
        }
        
        else if((strcmp(id,"t") == 0)){
            delay(time);
            gpio_set_level(BUZZ, 1);
        }

        else{
            gpio_set_level(LED, 0);
            gpio_set_level(BUZZ, 0);
        }
        break;
    case ESP_SPP_CONG_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_CONG_EVT");
        break;
    case ESP_SPP_WRITE_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_WRITE_EVT");
        break;
    case ESP_SPP_SRV_OPEN_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_SRV_OPEN_EVT");
        gettimeofday(&time_old, NULL);
        break;
    case ESP_SPP_SRV_STOP_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_SRV_STOP_EVT");
        break;
    case ESP_SPP_UNINIT_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_UNINIT_EVT");
        break;
    default:
        break;
    }
}

void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_BT_GAP_AUTH_CMPL_EVT:{
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(SPP_TAG, "authentication success: %s", param->auth_cmpl.device_name);
            esp_log_buffer_hex(SPP_TAG, param->auth_cmpl.bda, ESP_BD_ADDR_LEN);
        } else {
            ESP_LOGE(SPP_TAG, "authentication failed, status:%d", param->auth_cmpl.stat);
        }
        break;
    }
    case ESP_BT_GAP_PIN_REQ_EVT:{
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_PIN_REQ_EVT min_16_digit:%d", param->pin_req.min_16_digit);
        if (param->pin_req.min_16_digit) {
            ESP_LOGI(SPP_TAG, "Input pin code: 0000 0000 0000 0000");
            esp_bt_pin_code_t pin_code = {0};
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 16, pin_code);
        } else {
            ESP_LOGI(SPP_TAG, "Input pin code: 1234");
            esp_bt_pin_code_t pin_code;
            pin_code[0] = '1';
            pin_code[1] = '2';
            pin_code[2] = '3';
            pin_code[3] = '4';
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin_code);
        }
        break;
    }

#if (CONFIG_BT_SSP_ENABLED == true)
    case ESP_BT_GAP_CFM_REQ_EVT:
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_CFM_REQ_EVT Please compare the numeric value: %d", param->cfm_req.num_val);
        esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
        break;
    case ESP_BT_GAP_KEY_NOTIF_EVT:
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_KEY_NOTIF_EVT passkey:%d", param->key_notif.passkey);
        break;
    case ESP_BT_GAP_KEY_REQ_EVT:
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_KEY_REQ_EVT Please enter passkey!");
        break;
#endif

    case ESP_BT_GAP_MODE_CHG_EVT:
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_MODE_CHG_EVT mode:%d", param->mode_chg.mode);
        break;

    default: {
        ESP_LOGI(SPP_TAG, "event: %d", event);
        break;
    }
    }
    return;
}

static void bt_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bluedroid_init()) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s initialize bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bluedroid_enable()) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s enable bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bt_gap_register_callback(esp_bt_gap_cb)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s gap register failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_spp_register_callback(esp_spp_cb)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s spp register failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_spp_init(esp_spp_mode)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s spp init failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

#if (CONFIG_BT_SSP_ENABLED == true)
    /* Set default parameters for Secure Simple Pairing */
    esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
    esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_IO;
    esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));
#endif

    /*
     * Set default parameters for Legacy Pairing
     * Use variable pin, input pin code when pairing
     */
    esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_VARIABLE;
    esp_bt_pin_code_t pin_code;
    esp_bt_gap_set_pin(pin_type, 0, pin_code);
}



static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch(event->event_id) { 
    case SYSTEM_EVENT_STA_START:
        esp_wifi_connect(); 
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        xEventGroupSetBits(wifi_event_group, CONNECTED_BIT); 
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        /* This is a workaround as ESP32 WiFi libs don't currently auto-reassociate. */
        esp_wifi_connect(); xEventGroupClearBits(wifi_event_group, CONNECTED_BIT); 
        break;
    default:
        break;
    }
    return ESP_OK;
}

esp_err_t _http_event_handler(esp_http_client_event_t *evt)
{
    switch(evt->event_id) { 
        case HTTP_EVENT_ERROR:
            ESP_LOGD(TAG, "HTTP_EVENT_ERROR");
            break;
        case HTTP_EVENT_ON_CONNECTED:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_CONNECTED");
            break;
        case HTTP_EVENT_HEADER_SENT:
            ESP_LOGD(TAG, "HTTP_EVENT_HEADER_SENT");
            break;
        case HTTP_EVENT_ON_HEADER:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s",evt->header_key, evt->header_value);
            break;
        case HTTP_EVENT_ON_DATA:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len); 
            if (!esp_http_client_is_chunked_response(evt->client)) {
            // Write out data
            // printf("%.s", evt->data_len, (char)evt->data);
            }
            break;
        case HTTP_EVENT_ON_FINISH:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_FINISH");
            break;
        case HTTP_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "HTTP_EVENT_DISCONNECTED");
            break;
    }
    return ESP_OK;
}

static void run_program(void)
{
    mcpwm_example_gpio_initialize();
    printf("Configuring Initial Parameters of mcpwm...\n");
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 1000;    //frequency = 500Hz,
    pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);    //Configure PWM0A & PWM0B with above settings
    
    while (1) {
        get_time();
            adc_reading = 0;
        //Multisampling  
        for (int i = 0; i < NO_OF_SAMPLES; i++) { 
            if (unit == ADC_UNIT_1) {
                adc_reading += adc1_get_raw((adc1_channel_t)channel);
            } else {
                int raw;
                adc2_get_raw((adc2_channel_t)channel, ADC_WIDTH_BIT_12, &raw); 
                adc_reading += raw;
            }
        }
        adc_reading /= NO_OF_SAMPLES; 
        
        //Convert adc_reading to voltage in mV
        voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars); 
        celcius = voltage / 20;

        if (celcius < 30) {
            strcat(led, "BLUE");
            gpio_set_level(LED_RED, 0);
            gpio_set_level(LED_GREEN, 0);
            gpio_set_level(LED_BLUE, 1);
            brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, 50.0);
        } 
        
        else if (celcius >= 30 && celcius <= 36) {
            strcat(led, "GREEN");
            gpio_set_level(LED_RED, 0);
            gpio_set_level(LED_GREEN, 1);
            gpio_set_level(LED_BLUE, 0);
            brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, 90.0);
        } 
        else if(celcius > 36) {
            strcat(led, "RED");
            gpio_set_level(LED_RED, 1);
            gpio_set_level(LED_GREEN, 0);
            gpio_set_level(LED_BLUE, 0);
            brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, 99.0);
        }

        printf("Nilai ADC: %d  Tegangan LM741: %dmV  Tegangan LM35: %dmV  Temperature : %d  LED: %s\n", adc_reading, voltage, voltage/2, celcius, led); 

        vTaskDelay(55000/portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase()); 
    ret = nvs_flash_init();
    } 
    ESP_ERROR_CHECK(ret);

    //Configure ADC
    if (unit == ADC_UNIT_1) { 
        adc1_config_width(ADC_WIDTH_BIT_12); 
        adc1_config_channel_atten(channel, atten);
    } else {
    adc2_config_channel_atten((adc2_channel_t)channel, atten);
    }

    #if CONFIG_IDF_TARGET_ESP32
    //Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t)); 
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten,ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars); 
    print_char_val_type(val_type);
    #endif

    gpio_reset_pin(LED_RED);
    gpio_reset_pin(LED_GREEN);
    gpio_reset_pin(LED_BLUE);
    gpio_reset_pin(LED);
    gpio_reset_pin(BUZZ);

    gpio_set_direction(LED_RED, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_GREEN, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_BLUE, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED, GPIO_MODE_OUTPUT);
    gpio_set_direction(BUZZ, GPIO_MODE_OUTPUT);

    bt_main();
    initialise_wifi(); 
    initialize_sntp();

    run_program();
}
