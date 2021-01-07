/*
Author      : Ujang Supriyadi
Last Edited : 7/1/2020
*/

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
#include "esp_adc_cal.h"

#define LED_RED 25
#define LED_GREEN 33
#define LED_BLUE 32
#define LED 27
#define BUZZER 13
#define GPIO_PWM0A_OUT 214

#define WIFI_SSID "Wifi kopi"
#define WIFI_PASS "kopikeputih"
#define ALARM_TIME "00:50:00"

#define SPP_TAG "SPP_ACCEPTOR"
#define SPP_SERVER_NAME "SPP_SERVER"
#define EXAMPLE_DEVICE_NAME "INI-ESPKU"
#define SPP_SHOW_DATA 0
#define SPP_SHOW_SPEED 1
#define SPP_SHOW_MODE SPP_SHOW_SPEED
#define SPP_DATA_LEN 20

static uint8_t spp_data[SPP_DATA_LEN];
static bool bWriteAfterOpenEvt = true;
static bool bWriteAfterWriteEvt = false;
static bool bWriteAfterSvrOpenEvt = true;
static bool bWriteAfterDataReceived = true;

static const esp_spp_mode_t esp_spp_mode = ESP_SPP_MODE_CB;
static const esp_spp_sec_t sec_mask = ESP_SPP_SEC_AUTHENTICATE;
static const esp_spp_role_t role_slave = ESP_SPP_ROLE_SLAVE;

static void initialize_sntp(void);
static void initialise_wifi(void);
static esp_err_t event_handler(void *ctx, system_event_t *event);
esp_err_t _http_event_handler(esp_http_client_event_t *evt);
static void get_time(void);

static void sensor_process(void);

static EventGroupHandle_t wifi_event_group;
const int CONNECTED_BIT = BIT0;
static const char *TAG = "My Project";

static char strftime_buf[32], strftime_set[32], target[48];
int val_lm, val_opamp, counter = 0;
float mv_lm, mv_opamp, cel, speed;
bool Alarm = 0, On;

static void get_time(void)
{
    setenv("TZ", "UTC", 1);
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);
    // Is time set? If not, tm_year will be (1970 - 1900).
    if (timeinfo.tm_year < (2016 - 1900))
    {
        ESP_LOGI(TAG, "Time is not set yet. Connecting to WiFi and getting time over NTP.");
        while (sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET)
        {
            ESP_LOGI(TAG, "Waiting for system time to be set... ");
            vTaskDelay(2000 / portTICK_PERIOD_MS);
        }
        time(&now);
        localtime_r(&now, &timeinfo);
        time(&now);
    }
    tzset();
    localtime_r(&now, &timeinfo);
    time_t t = now + (3600 * 7);
    struct tm timeinfo_utc7;
    localtime_r(&t, &timeinfo_utc7);

    strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo_utc7);
    strftime(strftime_set, sizeof(strftime_set), "%H:%M:%S", &timeinfo_utc7);

    // strftime(buffer, sizeof(buffer), "%a %b %d %H:%M:%S %Y", &your_tm);

    int y = 0;
    for (int i = 0; strftime_buf[i] != NULL; i++)
    {
        if (strftime_buf[i] == ' ')
        {
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
    ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };
    ESP_LOGI(TAG, "Setting WiFi configuration SSID %s...", wifi_config.sta.ssid);
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}

static void initialize_sntp(void)
{
    ESP_LOGI(TAG, "Initializing SNTP");
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "pool.ntp.org");
    sntp_init();
}

static void mcpwm_example_gpio_initialize(void)
{
    printf("initializing mcpwm gpio...\n");
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_PWM0A_OUT);
}

static void brushed_motor_forward(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num, float duty_cycle)
{
    mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_A, duty_cycle);
    mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
}

uint8_t *createTestBuffer(void)
{
    static const uint8_t buffer[] = {5, 7, 3, 4, 9, 1, 3};
    uint8_t *rv = malloc(sizeof(buffer));
    if (rv != 0)
        memmove(rv, buffer, sizeof(buffer));
    return rv;
}

static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
    ESP_LOGI(SPP_TAG, "Start of: static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)");
    switch (event)
    {
    case ESP_SPP_INIT_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_INIT_EVT");
        ESP_LOGI(SPP_TAG, "Call esp_bt_dev_set_device_name(EXAMPLE_DEVICE_NAME)");
        esp_bt_dev_set_device_name(EXAMPLE_DEVICE_NAME);
        ESP_LOGI(SPP_TAG, "Call esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE)");
        esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
        ESP_LOGI(SPP_TAG, "Call esp_spp_start_srv(sec_mask,role_slave, 0, SPP_SERVER_NAME)");
        esp_spp_start_srv(sec_mask, role_slave, 0, SPP_SERVER_NAME);
        break;
    case ESP_SPP_DISCOVERY_COMP_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_DISCOVERY_COMP_EVT");
        break;
    case ESP_SPP_OPEN_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_OPEN_EVT");

        ESP_LOGI(SPP_TAG, "bWriteAfterOpenEvt: %d: ", bWriteAfterOpenEvt);
        if (bWriteAfterOpenEvt)
        {
            ESP_LOGI(SPP_TAG, "bWriteAfterOpenEvt = true");
            ESP_LOGI(SPP_TAG, "Call esp_spp_write(param->srv_open.handle, SPP_DATA_LEN, spp_data)");

            //esp_err_tesp_spp_write(uint32_t handle, int len, uint8_t *p_data)
            esp_spp_write(param->srv_open.handle, SPP_DATA_LEN, spp_data);

            //Test 1
            //uint8_t myData[10];

            //Test 2
            char *c = "Hello\n";
            // "Hello" is in fact just { 'H', 'e', 'l', 'l', 'o', '\0' }
            uint8_t *u = (uint8_t *)c;
            //uint8_t x = u[1];
            // x is 101, which is the ASCII char code of 'e'

            esp_spp_write(param->srv_open.handle, 6, u);

            //Test, compiles ok
            int intTest = 6;
            intTest = 2 + 4;
            esp_spp_write(param->srv_open.handle, intTest, u);
        }
        else
        {
            ESP_LOGI(SPP_TAG, "bWriteAfterOpenEvt = false");
        }
        break;
    case ESP_SPP_CLOSE_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_CLOSE_EVT");
        break;
    case ESP_SPP_START_EVT: //Short before connection is established
        ESP_LOGI(SPP_TAG, "ESP_SPP_START_EVT");
        break;
    case ESP_SPP_CL_INIT_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_CL_INIT_EVT");
        break;
    case ESP_SPP_DATA_IND_EVT: //When SPP connection received data, the event comes, only for ESP_SPP_MODE_CB
        ESP_LOGI(SPP_TAG, "ESP_SPP_DATA_IND_EVT len=%d handle=%d", param->data_ind.len, param->data_ind.handle);
        ESP_LOGI(SPP_TAG, "Call esp_log_buffer_hex("
                          ",param->data_ind.data,param->data_ind.len)");

        esp_log_buffer_hex("Received HEX Data", param->data_ind.data, param->data_ind.len);
        esp_log_buffer_char("Received String Data", param->data_ind.data, param->data_ind.len);
        //hexToString(param->data_ind.len, hasil, param->data_ind.data);

        if (bWriteAfterDataReceived)
        {
            ESP_LOGI(SPP_TAG, "bWriteAfterDataReceived = true");

            char hasil_convert[30];
            for (int i = 0; i < param->data_ind.len; ++i)
            {
                //printf("%c", (char)p_data[i]);
                hasil_convert[i] = (char)param->data_ind.data[i];
            }
            printf("%s", (char *)hasil_convert);
            printf("\n");
            char *pesan = "aaaaaaa";
            char *ON = "ON";
            char *OFF = "OFF";

            if (!strcmp(hasil_convert, ON))
            {
                pesan = "ON";
                gpio_set_level(LED, 1);
            }
            else if (!strcmp(hasil_convert, OFF))
            {
                pesan = "OFF";
                gpio_set_level(LED, 0);
            }
            else
            {
                // pesan = "Wrong command :(";
                if (!On)
                {
                    gpio_set_level(LED, 1);
                    pesan = "LED is ON\n";
                    On = 1;
                }
                else
                {
                    gpio_set_level(LED, 0);
                    pesan = "LED is OFF\n";
                    On = 0;
                }
            }

            esp_spp_write(param->srv_open.handle, strlen(pesan), (uint8_t *)pesan);
        }
        else
        {
            ESP_LOGI(SPP_TAG, "bWriteAfterDataReceived = false");
        }

        break;
    case ESP_SPP_CONG_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_CONG_EVT");
        break;
    case ESP_SPP_WRITE_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_WRITE_EVT len=%d cong=%d", param->write.len, param->write.cong);
        esp_log_buffer_hex("HEX Data was sent", spp_data, SPP_DATA_LEN);

        ESP_LOGI(SPP_TAG, "if param->write.cong ...");
        if (param->write.cong == 0)
        {
            ESP_LOGI(SPP_TAG, "param->write.cong == 0");
            if (bWriteAfterWriteEvt)
            {
                ESP_LOGI(SPP_TAG, "bWriteAfterWriteEvt = true");
                ESP_LOGI(SPP_TAG, "Call esp_spp_write(param->write.handle, SPP_DATA_LEN, spp_data)");
                esp_spp_write(param->write.handle, SPP_DATA_LEN, spp_data);
            }
            else
            {
                ESP_LOGI(SPP_TAG, "bWriteAfterWriteEvt = false");
            }
        }
        else
        {
            ESP_LOGI(SPP_TAG, "param->write.cong <> 0");
        }
        break;
    case ESP_SPP_SRV_OPEN_EVT: //After connection is established, short before data is received
        ESP_LOGI(SPP_TAG, "ESP_SPP_SRV_OPEN_EVT");

        ESP_LOGI(SPP_TAG, "bWriteAfterOpenEvt: %d: ", bWriteAfterSvrOpenEvt);
        //Added code from Initiator - Start
        if (bWriteAfterSvrOpenEvt)
        {
            ESP_LOGI(SPP_TAG, "bWriteAfterSvrOpenEvt = true");

            //https://stackoverflow.com/questions/40579902/how-to-turn-a-character-array-into-uint8-t
            char *c = "Hello"; // "Hello" is in fact just { 'H', 'e', 'l', 'l', 'o', '\0' }
            uint8_t *u = (uint8_t *)c;
            //uint8_t x = u[1];
            ESP_LOGI(SPP_TAG, "Call esp_spp_write(param->srv_open.handle, 5, Hello)");
            //esp_spp_write(param->srv_open.handle, 6, u);    //Works but shows special character after Hello
            esp_spp_write(param->srv_open.handle, 5, u); //Works, but maybe it needs something like CR
        }
        else
        {
            ESP_LOGI(SPP_TAG, "bWriteAfterSvrOpenEvt = false");
        }
        break;
    default:
        break;
    }
    ESP_LOGI(SPP_TAG, "End of: static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)");
}

void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    ESP_LOGI(SPP_TAG, "Start of: void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)");
    switch (event)
    {
    case ESP_BT_GAP_AUTH_CMPL_EVT:
    {
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGI(SPP_TAG, "authentication success: %s", param->auth_cmpl.device_name);
            esp_log_buffer_hex(SPP_TAG, param->auth_cmpl.bda, ESP_BD_ADDR_LEN);
        }
        else
        {
            ESP_LOGE(SPP_TAG, "authentication failed, status:%d", param->auth_cmpl.stat);
        }
        break;
    }
    case ESP_BT_GAP_PIN_REQ_EVT:
    {
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_PIN_REQ_EVT min_16_digit:%d", param->pin_req.min_16_digit);
        if (param->pin_req.min_16_digit)
        {
            ESP_LOGI(SPP_TAG, "Input pin code: 0000 0000 0000 0000");
            esp_bt_pin_code_t pin_code = {0};
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 16, pin_code);
        }
        else
        {
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

    default:
    {
        ESP_LOGI(SPP_TAG, "event: %d", event);
        break;
    }
    }
    return;
}

void startClassicBtSpp(void)
{
    for (int i = 0; i < SPP_DATA_LEN; ++i)
    {
        //spp_data[i] = i;
        spp_data[i] = i + 65;
    }

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_LOGI(SPP_TAG, "Call esp_bt_controller_init(&bt_cfg)");
    if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK)
    {
        ESP_LOGE(SPP_TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }
    else
    {
        ESP_LOGI(SPP_TAG, "Initialize controller ok");
    }

    ESP_LOGI(SPP_TAG, "Call esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT)");
    if ((ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT)) != ESP_OK)
    {
        ESP_LOGE(SPP_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }
    else
    {
        ESP_LOGI(SPP_TAG, "Enable controller ok");
    }

    ESP_LOGI(SPP_TAG, "Call esp_bluedroid_init()");
    if ((ret = esp_bluedroid_init()) != ESP_OK)
    {
        ESP_LOGE(SPP_TAG, "%s initialize bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }
    else
    {
        ESP_LOGI(SPP_TAG, "Initialize bluedroid ok");
    }

    ESP_LOGI(SPP_TAG, "Call esp_bluedroid_enable()");
    if ((ret = esp_bluedroid_enable()) != ESP_OK)
    {
        ESP_LOGE(SPP_TAG, "%s enable bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }
    else
    {
        ESP_LOGI(SPP_TAG, "Enable bluedroid ok");
    }

    ESP_LOGI(SPP_TAG, "Call esp_bt_gap_register_callback(esp_bt_gap_cb)");
    if ((ret = esp_bt_gap_register_callback(esp_bt_gap_cb)) != ESP_OK)
    {
        ESP_LOGE(SPP_TAG, "%s gap register failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }
    else
    {
        ESP_LOGI(SPP_TAG, "Gap register ok");
    }

    ESP_LOGI(SPP_TAG, "Call esp_spp_register_callback(esp_spp_cb)");
    if ((ret = esp_spp_register_callback(esp_spp_cb)) != ESP_OK)
    {
        ESP_LOGE(SPP_TAG, "%s spp register failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }
    else
    {
        ESP_LOGI(SPP_TAG, "spp register ok");
    }

    ESP_LOGI(SPP_TAG, "Call esp_spp_init(esp_spp_mode)");
    if ((ret = esp_spp_init(esp_spp_mode)) != ESP_OK)
    {
        ESP_LOGE(SPP_TAG, "%s spp init failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }
    else
    {
        ESP_LOGI(SPP_TAG, "spp init ok");
    }

    ESP_LOGI(SPP_TAG, "CONFIG_BT_SSP_ENABLED == true");

    ESP_LOGI(SPP_TAG, "Set default parameters for Secure Simple Pairing");
    esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
    esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_IO;
    esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));

    ESP_LOGI(SPP_TAG, "Set default parameters");
    esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_VARIABLE;
    esp_bt_pin_code_t pin_code;
    esp_bt_gap_set_pin(pin_type, 0, pin_code);
}

static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch (event->event_id)
    {
    case SYSTEM_EVENT_STA_START:
        esp_wifi_connect();
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
        break;
    default:
        break;
    }
    return ESP_OK;
}

esp_err_t _http_event_handler(esp_http_client_event_t *evt)
{
    switch (evt->event_id)
    {
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
            ESP_LOGD(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
            break;
        case HTTP_EVENT_ON_DATA:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
            if (!esp_http_client_is_chunked_response(evt->client))
            {
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



static void sensor_process(void)
{
    val_opamp = adc1_get_raw(ADC1_CHANNEL_0);
    val_lm = val_opamp / 2;
    mv_lm = (val_lm / 4096.0) * 1100;
    mv_opamp = (val_opamp / 4096.0) * 1100;
    cel = mv_lm / 10;
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_0);

    gpio_reset_pin(LED_RED);
    gpio_reset_pin(LED_GREEN);
    gpio_reset_pin(LED_BLUE);
    gpio_reset_pin(LED);
    gpio_reset_pin(BUZZER);

    gpio_set_direction(LED_RED, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_GREEN, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_BLUE, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED, GPIO_MODE_OUTPUT);
    gpio_set_direction(BUZZER, GPIO_MODE_OUTPUT);

    startClassicBtSpp();
    initialise_wifi();
    initialize_sntp();

    mcpwm_example_gpio_initialize();
    printf("Configuring Initial Parameters of mcpwm...\n");
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 1000; //frequency = 500Hz,
    pwm_config.cmpr_a = 0;       //duty cycle of PWMxA = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config); //Configure PWM0A & PWM0B with above settings

    while (1)
    {
        get_time();
        sensor_process();

        if (cel > 37)
        {
            gpio_set_level(LED_RED, 1);
            gpio_set_level(LED_GREEN, 0);
            gpio_set_level(LED_BLUE, 0);

            speed = 99.0;
            brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, speed);
            printf("RAW: %d\tLM741: %.2f\tLM35: %.2f\n\tLED: RED", val_lm, mv_lm, mv_opamp);

        }
        else if (cel >= 30 && cel <= 37)
        {
            gpio_set_level(LED_RED, 0);
            gpio_set_level(LED_GREEN, 1);
            gpio_set_level(LED_BLUE, 0);

            speed = 50.0;
            brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, speed);
            printf("RAW: %d\tLM741: %.2f\tLM35: %.2f\n\tLED: GREEN", val_lm, mv_lm, mv_opamp);

        }
        else if (cel < 30)
        {
            gpio_set_level(LED_RED, 0);
            gpio_set_level(LED_GREEN, 0);
            gpio_set_level(LED_BLUE, 1);

            speed = 0.0;
            brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, speed);
            printf("RAW: %d\tLM741: %.2f\tLM35: %.2f\n\tLED: BLUE", val_lm, mv_lm, mv_opamp);
        }

        if (strcmp(strftime_set, ALARM_TIME) == 0)
            Alarm = 1;

        if (Alarm)
        {
            gpio_set_level(BUZZER, 1);
            counter++;
            if (counter >= 60)
            {
                counter = 0;
                Alarm = 0;
            }
        }
        else
        {
            gpio_set_level(BUZZER, 0);
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

}
