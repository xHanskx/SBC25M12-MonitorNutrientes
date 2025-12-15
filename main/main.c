// main.c - EC + AS7263 + ThingsBoard con FreeRTOS
// Versión con tareas paralelas para ESP-IDF

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "mqtt_client.h"
#include "driver/i2c.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/gpio.h"

// =========================== CONFIGURACIÓN ===========================

// Wi-Fi
#define WIFI_SSID    "12345678"
#define WIFI_PASS    "********"  

// ThingsBoard
#define TB_SERVER       "demo.thingsboard.io"
#define TB_TOKEN        "4OSALLVLq5mQr3WU7QCf"

// Pines
#define EC_PIN_GPIO     34
#define I2C_SDA         21
#define I2C_SCL         22
#define RELAY_PIN1      GPIO_NUM_2
#define RELAY_PIN2      GPIO_NUM_4
#define RELAY_PIN3      GPIO_NUM_5

// Valores K y umbrales
static const float K_HIGH = 20497.16992f;
#define EC_THRESHOLD    30000.0f     // Umbral de electroconductividad (mS/cm)
#define NIR_T_THRESHOLD 2       // Umbral para el canal T del NIR

// ===================== AS7263 =====================
#define I2C_MASTER_NUM      I2C_NUM_0
#define I2C_MASTER_FREQ_HZ  100000
#define AS7263_ADDR         0x49

// Registros del AS7263
#define I2C_AS72XX_SLAVE_STATUS_REG 0x00
#define I2C_AS72XX_SLAVE_WRITE_REG  0x01
#define I2C_AS72XX_SLAVE_READ_REG   0x02

#define R_HIGH_REG          0x08
#define R_LOW_REG           0x09
#define S_HIGH_REG          0x0A
#define S_LOW_REG           0x0B
#define T_HIGH_REG          0x0C
#define T_LOW_REG           0x0D
#define U_HIGH_REG          0x0E
#define U_LOW_REG           0x0F
#define V_HIGH_REG          0x10
#define V_LOW_REG           0x11
#define W_HIGH_REG          0x12
#define W_LOW_REG           0x13
#define VIRTUAL_REG_DEVICE_TEMP 0x06

#define AS72XX_CONFIG_CONTROL_REG 0x04
#define AS72XX_GAIN_64X           0b00011000
#define AS72XX_MEASUREMENT_MODE   0b00000010

#define AS72XX_LED_CONFIG         0x07

// Bits del registro LED_CONFIG
#define LED_CURRENT_LIMIT_12_5MA  0b00
#define LED_CURRENT_LIMIT_25MA    0b01
#define LED_CURRENT_LIMIT_50MA    0b10
#define LED_CURRENT_LIMIT_100MA   0b11

#define LED_INDICATOR_CURRENT_LIMIT_1MA 0b00
#define LED_INDICATOR_CURRENT_LIMIT_2MA 0b01
#define LED_INDICATOR_CURRENT_LIMIT_4MA 0b10
#define LED_INDICATOR_CURRENT_LIMIT_8MA 0b11

#define LED_INDICATOR_ENABLE    (1 << 6)
#define LED_ILLUMINATION_ENABLE (1 << 7)

static const char* TAG = "NUTRIENT_MONITOR";

// ===================== ESTRUCTURAS DE DATOS =====================

typedef struct {
    float   ec;
    uint16_t r;
    uint16_t s;
    uint16_t t;
    uint16_t u;
    uint16_t v;
    uint16_t w;
    uint8_t temp;
    bool    relay1_state;
    bool    relay2_state;
    bool    relay3_state;
} sensor_data_t;

// ===================== VARIABLES GLOBALES =====================

static esp_mqtt_client_handle_t mqtt_client;
static adc_oneshot_unit_handle_t adc1_handle;
static QueueHandle_t sensor_data_queue;
static SemaphoreHandle_t i2c_mutex;
static SemaphoreHandle_t relay_mutex;

static bool relay1_active = false;
static bool relay2_active = false;
static bool relay3_active = false;

static TaskHandle_t ec_task_handle   = NULL;
static TaskHandle_t nir_task_handle  = NULL;
static TaskHandle_t mqtt_task_handle = NULL;

// ===================== DECLARACIONES =====================

static void i2c_init(void);
static esp_err_t i2c_master_read_slave_reg(uint8_t reg_addr, uint8_t *data);
static esp_err_t i2c_master_write_slave_reg(uint8_t reg_addr, uint8_t data);
static void write_virtual_register(uint8_t reg, uint8_t value);
static uint8_t read_virtual_register(uint8_t reg);
static void as7263_init(void);
static void wifi_init(void);
static void mqtt_init(void);
static void mqtt_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
static void init_relays(void);
static void control_relay1(bool state);
static void control_relay2(bool state);
static void control_relay3(bool state);
static void update_relays_based_on_data(float ec_value, uint16_t nir_t_value);

// ===================== TAREAS =====================

// Tarea para leer EC
static void ec_reading_task(void *pvParameters) {
    ESP_LOGI(TAG, "Tarea EC iniciada");

    while (1) {
        uint32_t sum = 0;
        int adc_raw = 0;

        for (int i = 0; i < 15; i++) {
            esp_err_t ret = adc_oneshot_read(adc1_handle, ADC_CHANNEL_6, &adc_raw);
            if (ret == ESP_OK) {
                sum += adc_raw;
            }
            vTaskDelay(5 / portTICK_PERIOD_MS);
        }

        float voltage = (sum / 15.0f) * (3.3f / 4095.0f);
        float raw_ec  = 1000.0f * voltage / 820.0f / 200.0f;
        float ec_value = raw_ec * K_HIGH;

        update_relays_based_on_data(ec_value, 0);

        sensor_data_t data = {0};
        data.ec = ec_value;
        data.r  = 0xFFFF;  // Marcador: solo viene EC

        if (sensor_data_queue != NULL) {
            xQueueSend(sensor_data_queue, &data, portMAX_DELAY);
        }

        ESP_LOGI(TAG, "EC: %.3f mS/cm (voltaje: %.3f V)", ec_value, voltage);
        ESP_LOGI(TAG, "Relé 2 (EC>%.1f): %s", EC_THRESHOLD, relay2_active ? "ON" : "OFF");

        vTaskDelay(5000 / portTICK_PERIOD_MS); // Cada 5 segundos
    }
}

// Tarea para leer NIR
static void nir_reading_task(void *pvParameters) {
    ESP_LOGI(TAG, "Tarea NIR iniciada");

    i2c_init();
    as7263_init();

    while (1) {
        sensor_data_t data = {0};

        // Forzar una medición (ONE-SHOT)
        write_virtual_register(AS72XX_CONFIG_CONTROL_REG, AS72XX_GAIN_64X | 0x02);

        // Esperar datos listos
        uint8_t status;
        do {
            status = read_virtual_register(0x00);
            vTaskDelay(10 / portTICK_PERIOD_MS);
        } while (!(status & 0x40));

        if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
            data.temp = read_virtual_register(VIRTUAL_REG_DEVICE_TEMP);

            uint8_t r_low  = read_virtual_register(R_LOW_REG);
            uint8_t r_high = read_virtual_register(R_HIGH_REG);
            data.r = (r_high << 8) | r_low;

            uint8_t s_low  = read_virtual_register(S_LOW_REG);
            uint8_t s_high = read_virtual_register(S_HIGH_REG);
            data.s = (s_high << 8) | s_low;

            uint8_t t_low  = read_virtual_register(T_LOW_REG);
            uint8_t t_high = read_virtual_register(T_HIGH_REG);
            data.t = (t_high << 8) | t_low;

            uint8_t u_low  = read_virtual_register(U_LOW_REG);
            uint8_t u_high = read_virtual_register(U_HIGH_REG);
            data.u = (u_high << 8) | u_low;

            uint8_t v_low  = read_virtual_register(V_LOW_REG);
            uint8_t v_high = read_virtual_register(V_HIGH_REG);
            data.v = (v_high << 8) | v_low;

            uint8_t w_low  = read_virtual_register(W_LOW_REG);
            uint8_t w_high = read_virtual_register(W_HIGH_REG);
            data.w = (w_high << 8) | w_low;

            xSemaphoreGive(i2c_mutex);

            update_relays_based_on_data(0.0f, data.t);

            data.relay1_state = relay1_active;
            data.relay2_state = relay2_active;
            data.relay3_state = relay3_active;

            if (sensor_data_queue != NULL) {
                xQueueSend(sensor_data_queue, &data, portMAX_DELAY);
            }

            ESP_LOGI(TAG, "NIR: R=%u S=%u T=%u U=%u V=%u W=%u Temp=%d",
                     data.r, data.s, data.t, data.u, data.v, data.w, data.temp);
        }

        ESP_LOGI(TAG, "Relé 3 (NIR T>%d): %s", NIR_T_THRESHOLD, relay3_active ? "ON" : "OFF");

        vTaskDelay(5000 / portTICK_PERIOD_MS); // Cada 5 segundos
    }
}

// Tarea para enviar datos por MQTT
static void mqtt_sending_task(void *pvParameters) {
    ESP_LOGI(TAG, "Tarea MQTT iniciada");

    vTaskDelay(5000 / portTICK_PERIOD_MS);

    sensor_data_t last_data = {0};
    int send_counter = 0;

    while (1) {
        sensor_data_t data;

        if (xQueueReceive(sensor_data_queue, &data, pdMS_TO_TICKS(1000)) == pdTRUE) {
            if (data.r != 0xFFFF) {  
                last_data.r     = data.r;
                last_data.s     = data.s;
                last_data.t     = data.t;
                last_data.u     = data.u;
                last_data.v     = data.v;
                last_data.w     = data.w;
                last_data.temp  = data.temp;
                last_data.relay1_state = data.relay1_state;
                last_data.relay2_state = data.relay2_state;
                last_data.relay3_state = data.relay3_state;
            } else {  
                last_data.ec = data.ec;
            }

            send_counter++;

            if (send_counter >= 5 || data.r != 0xFFFF) {
                char payload[512];
                snprintf(payload, sizeof(payload),
                    "{"
                    "\"EC\":%.3f,"
                    "\"R\":%u,\"S\":%u,\"T\":%u,\"U\":%u,\"V\":%u,\"W\":%u,"
                    "\"temp\":%u,"
                    "\"relay1\":%s,\"relay2\":%s,\"relay3\":%s"
                    "}",
                    last_data.ec,
                    last_data.r, last_data.s, last_data.t,
                    last_data.u, last_data.v, last_data.w,
                    last_data.temp,
                    last_data.relay1_state ? "true" : "false",
                    last_data.relay2_state ? "true" : "false",
                    last_data.relay3_state ? "true" : "false");

                esp_mqtt_client_publish(mqtt_client, "v1/devices/me/telemetry",
                                        payload, 0, 1, 0);

                ESP_LOGI(TAG, "Datos enviados: EC=%.3f, R=%u, Relay1=%s",
                         last_data.ec, last_data.r,
                         last_data.relay1_state ? "ON" : "OFF");

                send_counter = 0;
            }
        }

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

// ===================== FUNCIONES =====================

static void i2c_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA,
        .scl_io_num = I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(I2C_MASTER_NUM, &conf);
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0));
}

static esp_err_t i2c_master_read_slave_reg(uint8_t reg_addr, uint8_t *data) {
    esp_err_t ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AS7263_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) return ret;

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AS7263_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, data, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);

    return ret;
}

static esp_err_t i2c_master_write_slave_reg(uint8_t reg_addr, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AS7263_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);

    return ret;
}

static void write_virtual_register(uint8_t reg, uint8_t value) {
    uint8_t status;

    do {
        i2c_master_read_slave_reg(I2C_AS72XX_SLAVE_STATUS_REG, &status);
    } while (status & 0x02);

    uint8_t reg_with_write = reg | 0x80;
    i2c_master_write_slave_reg(I2C_AS72XX_SLAVE_WRITE_REG, reg_with_write);

    do {
        i2c_master_read_slave_reg(I2C_AS72XX_SLAVE_STATUS_REG, &status);
    } while (status & 0x02);

    i2c_master_write_slave_reg(I2C_AS72XX_SLAVE_WRITE_REG, value);
}

static uint8_t read_virtual_register(uint8_t reg) {
    uint8_t status, data;

    do {
        i2c_master_read_slave_reg(I2C_AS72XX_SLAVE_STATUS_REG, &status);
    } while (status & 0x02);

    i2c_master_write_slave_reg(I2C_AS72XX_SLAVE_WRITE_REG, reg);

    do {
        i2c_master_read_slave_reg(I2C_AS72XX_SLAVE_STATUS_REG, &status);
    } while (!(status & 0x01));

    i2c_master_read_slave_reg(I2C_AS72XX_SLAVE_READ_REG, &data);
    return data;
}

static void as7263_init(void) {
    ESP_LOGI(TAG, "Inicializando AS7263...");

    if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(2000)) == pdTRUE) {
        write_virtual_register(0x04, 0x80);  // Soft reset
        vTaskDelay(pdMS_TO_TICKS(1000));

        write_virtual_register(0x04, 0b01011110);  
        write_virtual_register(0x05, 35);          

        // LED blanco a 100mA
        uint8_t led_config = (1 << 7) | (1 << 3) | 0b11;  
        write_virtual_register(0x07, led_config);

        vTaskDelay(pdMS_TO_TICKS(100));
        uint8_t check = read_virtual_register(0x07);
        ESP_LOGI(TAG, "Registro LED_CONFIG escrito: 0x%02X (esperado ~0x8B)", check);

        xSemaphoreGive(i2c_mutex);
        ESP_LOGI(TAG, "AS7263 inicializado - LED blanco a 100 mA");
    } else {
        ESP_LOGE(TAG, "No se pudo tomar mutex I2C para inicializar AS7263");
    }
}

static void init_relays(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << RELAY_PIN1) | (1ULL << RELAY_PIN2) | (1ULL << RELAY_PIN3),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = false,
        .pull_down_en = false,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    gpio_set_level(RELAY_PIN1, 0);
    gpio_set_level(RELAY_PIN2, 0);
    gpio_set_level(RELAY_PIN3, 0);

    relay1_active = relay2_active = relay3_active = false;

    ESP_LOGI(TAG, "3 relés inicializados (todos apagados)");
}

static void control_relay1(bool state) {
    if (xSemaphoreTake(relay_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        gpio_set_level(RELAY_PIN1, state ? 1 : 0);
        relay1_active = state;
        ESP_LOGI(TAG, "Relé 1 (ThingsBoard) %s", state ? "ACTIVADO" : "DESACTIVADO");
        xSemaphoreGive(relay_mutex);
    }
}

static void control_relay2(bool state) {
    if (xSemaphoreTake(relay_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        gpio_set_level(RELAY_PIN2, state ? 1 : 0);
        relay2_active = state;
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "Relé 2 (EC>%.1f) %s", EC_THRESHOLD, state ? "ACTIVADO" : "DESACTIVADO");
        xSemaphoreGive(relay_mutex);
    }
}

static void control_relay3(bool state) {
    if (xSemaphoreTake(relay_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        gpio_set_level(RELAY_PIN3, state ? 1 : 0);
        relay3_active = state;
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "Relé 3 (NIR T>%d) %s", NIR_T_THRESHOLD, state ? "ACTIVADO" : "DESACTIVADO");
        xSemaphoreGive(relay_mutex);
    }
}

static void update_relays_based_on_data(float ec_value, uint16_t nir_t_value) {
    if (ec_value > 0.0f) {
        control_relay2(ec_value > EC_THRESHOLD);
    }

    if (nir_t_value > 0) {
        control_relay3(nir_t_value > NIR_T_THRESHOLD);
    }
}

// ===================== MQTT =====================

static void mqtt_event_handler(void *args, esp_event_base_t base, int32_t event_id, void *event_data) {
    esp_mqtt_event_handle_t event = event_data;

    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT conectado!");
            esp_mqtt_client_subscribe(mqtt_client, "v1/devices/me/rpc/request/+", 1);
            ESP_LOGI(TAG, "Suscrito a RPCs");
            break;

        case MQTT_EVENT_DATA:
            if (strstr(event->topic, "rpc/request")) {
                int request_id = atoi(event->topic + strlen("v1/devices/me/rpc/request/"));

                bool new_state;
                if (strstr(event->data, "true") || strstr(event->data, "\"value\":true")) {
                    new_state = true;
                } else if (strstr(event->data, "false") || strstr(event->data, "\"value\":false")) {
                    new_state = false;
                } else if (strstr(event->data, "toggle")) {
                    new_state = !relay1_active;
                } else {
                    break;
                }

                control_relay1(new_state);

                char response_topic[64];
                snprintf(response_topic, sizeof(response_topic), "v1/devices/me/rpc/response/%d", request_id);

                char response[128];
                snprintf(response, sizeof(response),
                    "{\"result\":\"OK\",\"relay1\":%s,\"relay2\":%s,\"relay3\":%s}",
                    relay1_active ? "true" : "false",
                    relay2_active ? "true" : "false",
                    relay3_active ? "true" : "false");

                esp_mqtt_client_publish(mqtt_client, response_topic, response, 0, 1, 0);
                ESP_LOGI(TAG, "RPC %d respondido", request_id);
            }
            break;

        default:
            break;
    }
}

static void wifi_init(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_connect());

    ESP_LOGI(TAG, "WiFi iniciado");
}

static void mqtt_init(void) {
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = "mqtt://" TB_SERVER,
        .credentials.username = TB_TOKEN,
    };

    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(mqtt_client);

    ESP_LOGI(TAG, "MQTT iniciado");
}

// ===================== APP MAIN =====================

void app_main(void) {
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_LOGI(TAG, "Sistema iniciando...");

    i2c_mutex    = xSemaphoreCreateMutex();
    relay_mutex  = xSemaphoreCreateMutex();
    sensor_data_queue = xQueueCreate(10, sizeof(sensor_data_t));

    if (!i2c_mutex || !relay_mutex || !sensor_data_queue) {
        ESP_LOGE(TAG, "Error creando objetos RTOS");
        return;
    }

    init_relays();

    // ADC
    adc_oneshot_unit_init_cfg_t init_config1 = { .unit_id = ADC_UNIT_1 };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_6, &config));
    ESP_LOGI(TAG, "ADC inicializado");

    // WiFi y MQTT
    wifi_init();
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    mqtt_init();
    vTaskDelay(2000 / portTICK_PERIOD_MS);

    // Tareas
    xTaskCreate(ec_reading_task,   "EC_Task",   4096, NULL, 3, &ec_task_handle);
    xTaskCreate(nir_reading_task,  "NIR_Task",  4096, NULL, 2, &nir_task_handle);
    xTaskCreate(mqtt_sending_task, "MQTT_Task", 4096, NULL, 1, &mqtt_task_handle);

    ESP_LOGI(TAG, "Todas las tareas creadas. Sistema funcionando en paralelo.");

    while (1) {
        ESP_LOGI(TAG, "Sistema activo - EC: %p, NIR: %p, MQTT: %p",
                 ec_task_handle, nir_task_handle, mqtt_task_handle);
        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }
}
