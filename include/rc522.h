#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <esp_event.h>
#include <driver/spi_master.h>
#include <driver/i2c.h> // TODO: Log warning: This driver is an old driver, please migrate your application code to adapt `driver/i2c_master.h`

#define RC522_I2C_ADDRESS (0x28)

#define RC522_DEFAULT_SCAN_INTERVAL_MS (125)
#define RC522_DEFAULT_TASK_STACK_SIZE (4 * 1024)
#define RC522_DEFAULT_TASK_STACK_PRIORITY (4)
#define RC522_DEFAULT_SPI_CLOCK_SPEED_HZ (5000000)
#define RC522_DEFAULT_I2C_RW_TIMEOUT_MS (1000)
#define RC522_DEFAULT_I2C_CLOCK_SPEED_HZ (100000)

ESP_EVENT_DECLARE_BASE(RC522_EVENTS);

typedef enum {
    SERIAL_NO_TYPE_NONE =   0,
    SERIAL_NO_TYPE_4 =      4,
    SERIAL_NO_TYPE_7 =      7,
} serial_no_type_t;

typedef struct {
    serial_no_type_t type;
    uint8_t data[7];
} serial_no_t;

// A struct used for passing a MIFARE Crypto1 key
typedef struct {
    uint8_t		keyByte[6]; // A Mifare Crypto1 key is 6 bytes
} MIFARE_Key;


typedef struct rc522 * rc522_handle_t;

typedef enum {
    RC522_TRANSPORT_SPI,
    RC522_TRANSPORT_I2C,
} rc522_transport_t;

typedef struct {
    uint16_t scan_interval_ms;         /*<! How fast will ESP32 scan for nearby tags, in miliseconds */
    size_t task_stack_size;            /*<! Stack size of rc522 task */
    uint8_t task_priority;             /*<! Priority of rc522 task */
    rc522_transport_t transport;       /*<! Transport that will be used. Defaults to SPI */
    union {
        struct {
            spi_host_device_t host;
            gpio_num_t miso;
            gpio_num_t mosi;
            gpio_num_t sclk;
            gpio_num_t cs;
            int clock_speed_hz;
            uint32_t device_flags;     /*<! Bitwise OR of SPI_DEVICE_* flags */
            /**
             * @brief Set to true if the bus is already initialized. 
             *        NOTE: This property will be removed in future,
             *        once when https://github.com/espressif/esp-idf/issues/8745 is resolved
             * 
             */
            bool bus_is_initialized;
        } spi;
        struct {
            i2c_port_t port;
            gpio_num_t sda;
            gpio_num_t scl;
            int rw_timeout_ms;
            uint32_t clock_speed_hz;
        } i2c;
    };
} rc522_config_t;

typedef enum {
    RC522_EVENT_ANY = ESP_EVENT_ANY_ID,
    RC522_EVENT_NONE,
    RC522_EVENT_TAG_SCANNED,             /*<! Tag scanned */
} rc522_event_t;

typedef struct {
    rc522_handle_t rc522;
    void* ptr;
} rc522_event_data_t;

typedef struct {
    uint64_t serial_number;
} rc522_tag_t;

/**
 * 初始化RC522 RFID模块。
 * 
 * 该函数负责根据提供的配置参数初始化RC522模块，包括配置复制、传输层创建、事件循环创建和任务创建。
 * 如果初始化成功，RC522模块将准备好进行进一步的操作。
 * 
 * @param config 指向RC522配置参数的指针。不能为空。
 * @param out_rc522 指向一个变量的指针，该变量将接收RC522模块的句柄。不能为空。
 * @return 返回初始化操作的结果，如果返回ESP_OK表示成功，其他值表示失败。
 */
esp_err_t rc522_create(const rc522_config_t * config, rc522_handle_t* out_rc522);

esp_err_t rc522_register_events(rc522_handle_t rc522, rc522_event_t event, esp_event_handler_t event_handler, void* event_handler_arg);

esp_err_t rc522_unregister_events(rc522_handle_t rc522, rc522_event_t event, esp_event_handler_t event_handler);

/**
 * @brief Start to scan tags. If already started, ESP_OK will just be returned. Initialization function had to be
 *        called before this one.
 * @param rc522 Handle
 * @return ESP_OK on success
 */
esp_err_t rc522_start(rc522_handle_t rc522);

/**
 * @brief Start to scan tags. If already started, ESP_OK will just be returned.
 * @param rc522 Handle
 * @return ESP_OK on success
 */
#define rc522_resume(rc522) rc522_start(rc522)

/**
 * @brief Pause scan tags. If already paused, ESP_OK will just be returned.
 * @param rc522 Handle
 * @return ESP_OK on success
 */
esp_err_t rc522_pause(rc522_handle_t rc522);

/**
 * @brief Destroy RC522 and free all resources. Cannot be called from event handler.
 * @param rc522 Handle
 */
esp_err_t rc522_destroy(rc522_handle_t rc522);

#ifdef __cplusplus
}
#endif
