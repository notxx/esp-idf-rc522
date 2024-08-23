#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <esp_log.h>
#include <string.h>

#include "rc522.h"
#include "rc522_registers.h"
#include "rc522_commands.h"
#include "iso_iec_14443_protocol.h"
#include "guards.h"

static const char * TAG = "rc522";

#define EVENT_BIT_HARDWARE_READY BIT0

ESP_EVENT_DEFINE_BASE(RC522_EVENTS);

typedef rc522_t * rc522_handle_t;

static esp_err_t rc522_spi_send(rc522_handle_t rc522, uint8_t * buffer, uint8_t length);
static esp_err_t rc522_spi_receive(rc522_handle_t rc522, uint8_t * buffer, uint8_t length, uint8_t addr);
static esp_err_t rc522_i2c_send(rc522_handle_t rc522, uint8_t * buffer, uint8_t length);
static esp_err_t rc522_i2c_receive(rc522_handle_t rc522, uint8_t * buffer, uint8_t length, uint8_t addr);

static void rc522_task(void* arg);

/**
 * @brief RC522模块发送数据函数
 * 
 * 该函数通过指定的传输方式(SPI或I2C)将数据发送到RC522模块。
 * 
 * @param rc522 RC522模块句柄，包含模块配置和状态信息
 * @param addr RC522模块寄存器地址
 * @param data 指向要发送的数据的指针
 * @param n 要发送的数据字节数
 * @return esp_err_t 返回操作状态，ESP_OK表示成功，其他值表示错误
 */
static esp_err_t rc522_send(rc522_handle_t rc522, rc522_reg_t addr, const uint8_t * data, uint8_t n) {
    esp_err_t err = ESP_OK; // 初始化错误码为ESP_OK，表示还没有错误发生
    
    uint8_t buffer[n + 1]; // 创建一个足够容纳地址和数据的缓冲区

    buffer[0] = addr; // 缓冲区的第一个字节是模块寄存器地址
    memcpy(buffer + 1, data, n); // 将数据复制到缓冲区的剩余部分

    // 根据RC522模块的配置选择合适的传输方式
    switch(rc522->config.transport) {
        case RC522_TRANSPORT_SPI: // 如果使用SPI传输
            err = rc522_spi_send(rc522, buffer, n + 1); // 调用SPI发送函数发送数据
            break;
        case RC522_TRANSPORT_I2C: // 如果使用I2C传输
            err = rc522_i2c_send(rc522, buffer, n + 1); // 调用I2C发送函数发送数据
            break;
        default: // 如果传输方式未知
            err = ESP_ERR_INVALID_STATE; // 设置错误码为未知状态
            break;
    }

    // 如果发送过程中发生错误
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write data (err: %s)", esp_err_to_name(err)); // 记录错误日志
    }
   
    return err; // 返回操作状态
}

/**
 * @brief 向RC522模块写入单个寄存器的值。
 * 
 * 本函数封装了向RC522模块发送数据的操作，用于向指定的寄存器地址写入一个8位的值。
 * 它使用了底层的rc522_send函数来完成实际的数据发送任务。
 * 
 * @param rc522 RC522模块的句柄，标识具体的RC522模块实例。
 * @param addr 要写入值的寄存器地址。
 * @param val 要写入寄存器的8位值。
 * @return esp_err_t 返回操作结果，表示写入操作是否成功。
 */
static inline esp_err_t rc522_write(rc522_handle_t rc522, rc522_reg_t addr, uint8_t val) {
    return rc522_send(rc522, addr, &val, 1);
}

/**
 * @brief 从RC522模块接收数据
 * 
 * 此函数根据配置的传输方式(SPI或I2C)，从RC522模块的指定地址接收数据。
 * 它使用适当的传输层函数来执行读取操作，并记录任何错误。
 * 
 * @param rc522 RC522模块的句柄，包含配置和状态信息
 * @param addr RC522模块中的起始地址，从该地址开始读取数据
 * @param buffer 指向用于存储接收到的数据的缓冲区
 * @param n 要接收的字节数
 * @return esp_err_t 返回操作的结果，ESP_OK表示成功，其他值表示错误
 */
static esp_err_t rc522_receive(rc522_handle_t rc522, rc522_reg_t addr, uint8_t * buffer, uint8_t n) {
    esp_err_t err; // 初始化错误码变量

    // 根据配置的传输方式，调用相应的接收函数
    switch(rc522->config.transport) {
        case RC522_TRANSPORT_SPI:
            err = rc522_spi_receive(rc522, buffer, n, addr);
            break;
        case RC522_TRANSPORT_I2C:
            err = rc522_i2c_receive(rc522, buffer, n, addr);
            break;
        default:
            err = ESP_ERR_INVALID_STATE; // 当传输方式未知时，返回错误状态
            break;
    }

    // 如果接收操作失败，记录错误
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read data (err: %s)", esp_err_to_name(err));
    }

    return err; // 返回接收操作的结果
}

static inline esp_err_t rc522_read(rc522_handle_t rc522, rc522_reg_t addr, uint8_t * value_ref)
{
    return rc522_receive(rc522, addr, value_ref, 1);
}

static esp_err_t rc522_set_bitmask(rc522_handle_t rc522, rc522_reg_t addr, uint8_t mask)
{
    esp_err_t err = ESP_OK;
    uint8_t tmp;

    ESP_ERR_RET_GUARD(rc522_read(rc522, addr, &tmp));

    return rc522_write(rc522, addr, tmp | mask);
}

/**
 * @brief RC522寄存器中清除指定位
 * 
 * 该函数用于在RC522的指定寄存器中清除指定的位。它首先从寄存器读取当前值，
 * 然后通过与非操作清除特定的位，最后将修改后的新值写回寄存器。
 * 
 * @param rc522 RC522设备的句柄
 * @param addr 要操作的寄存器地址
 * @param mask 要清除的位的掩码，即哪些位要清除
 * @return esp_err_t 返回操作结果，ESP_OK表示成功，其他值表示错误
 */
static esp_err_t rc522_clear_bitmask(rc522_handle_t rc522, rc522_reg_t addr, uint8_t mask) {
    esp_err_t err = ESP_OK; // 初始化错误码为ESP_OK，表示还没有错误发生
    uint8_t tmp; // 用于存储从寄存器读取的原始值

    // 从RC522的指定寄存器读取当前值，如果读取失败，则直接返回错误码
    ESP_ERR_RET_GUARD(rc522_read(rc522, addr, &tmp));

    // 使用与非操作清除指定的位，并将结果写回寄存器
    return rc522_write(rc522, addr, tmp & ~mask);
}

/**
 * @brief 读取RC522模块的固件信息
 * 
 * 本函数通过调用rc522_read函数，读取RC522模块的版本寄存器（RC522_REG_VERSION），
 * 以获取模块的固件信息。结果存储在result参数指向的缓冲区中。
 * 
 * @param rc522 RC522模块的句柄，用于标识和操作具体的RC522模块
 * @param result 用于存储读取的固件信息的缓冲区
 * @return esp_err_t 表示操作结果，ESP_OK表示成功，其他值表示错误
 */
static inline esp_err_t rc522_firmware(rc522_handle_t rc522, uint8_t * result) {
    return rc522_read(rc522, RC522_REG_VERSION, result);
}

/**
 * @brief 启动RC522的天线
 * 
 * 该函数负责启动RC522射频模块的天线。它首先检查TX控制寄存器是否已配置，
 * 如果没有，它会设置TX控制寄存器以启用天线。然后，它设置RF配置寄存器以
 * 设置适当的增益，从而完成天线的启动过程。
 * 
 * @param rc522 RC522设备的句柄，用于操作RC522芯片。
 * @return esp_err_t 返回操作结果，ESP_OK表示成功，其他值表示错误代码。
 */
static esp_err_t rc522_antenna_on(rc522_handle_t rc522) {
    esp_err_t err = ESP_OK; // 初始化错误码为ESP_OK
    uint8_t tmp; // 用于存储从RC522_REG_TX_CONTROL读取的值

    // 读取RC522的TX控制寄存器的值，用于后续判断是否需要设置位掩码
    ESP_ERR_RET_GUARD(rc522_read(rc522, RC522_REG_TX_CONTROL, &tmp));

    // 如果TX控制寄存器的低两位没有被设置，则设置位掩码以启用天线
    if (~(tmp & 0x03)) {
        ESP_ERR_RET_GUARD(rc522_set_bitmask(rc522, RC522_REG_TX_CONTROL, 0x03));
    }

    // 设置RF配置寄存器以获得43dB的增益，这是启动天线所需的配置
    return rc522_write(rc522, RC522_REG_RF_CFG, RC522_RX_43DB << 4); // 43db gain
}

static void config_cpy(rc522_config_t * dst, const rc522_config_t * src) {
    memcpy(dst, src, sizeof(rc522_config_t));
    // defaults
    dst->scan_interval_ms = src->scan_interval_ms < 50 ? RC522_DEFAULT_SCAN_INTERVAL_MS : src->scan_interval_ms;
    dst->task_stack_size = src->task_stack_size == 0 ? RC522_DEFAULT_TASK_STACK_SIZE : src->task_stack_size;
    dst->task_priority = src->task_priority == 0 ? RC522_DEFAULT_TASK_STACK_PRIORITY : src->task_priority;
    dst->transport = src->transport;
    if (src->transport == RC522_TRANSPORT_SPI) {
        dst->spi.clock_speed_hz = src->spi.clock_speed_hz == 0 ? RC522_DEFAULT_SPI_CLOCK_SPEED_HZ : src->spi.clock_speed_hz;
    } else if (src->transport == RC522_TRANSPORT_I2C) {
        dst->i2c.rw_timeout_ms = src->i2c.rw_timeout_ms == 0 ? RC522_DEFAULT_I2C_RW_TIMEOUT_MS : src->i2c.rw_timeout_ms;
        dst->i2c.clock_speed_hz = src->i2c.clock_speed_hz == 0 ? RC522_DEFAULT_I2C_CLOCK_SPEED_HZ : src->i2c.clock_speed_hz;
    }
}

static esp_err_t rc522_create_transport(rc522_handle_t rc522) {
    esp_err_t err = ESP_OK;

    switch(rc522->config.transport) {
        case RC522_TRANSPORT_SPI: {
                spi_device_interface_config_t devcfg = {
                    .clock_speed_hz = rc522->config.spi.clock_speed_hz,
                    .mode = 0,
                    .spics_io_num = rc522->config.spi.cs,
                    .queue_size = 7,
                    .flags = rc522->config.spi.device_flags,
                };

                rc522->bus_initialized_by_user = rc522->config.spi.bus_is_initialized;

                if (!rc522->bus_initialized_by_user) {
                    spi_bus_config_t buscfg = {
                        .miso_io_num = rc522->config.spi.miso,
                        .mosi_io_num = rc522->config.spi.mosi,
                        .sclk_io_num = rc522->config.spi.sclk,
                        .quadwp_io_num = -1,
                        .quadhd_io_num = -1,
                    };

                    ESP_ERR_RET_GUARD(spi_bus_initialize(rc522->config.spi.host, &buscfg, 0));
                }

                ESP_ERR_RET_GUARD(spi_bus_add_device(rc522->config.spi.host, &devcfg, &rc522->spi_handle));
            }
            break;
        case RC522_TRANSPORT_I2C: {
                i2c_config_t conf = {
                    .mode = I2C_MODE_MASTER,
                    .sda_io_num = rc522->config.i2c.sda,
                    .scl_io_num = rc522->config.i2c.scl,
                    .sda_pullup_en = GPIO_PULLUP_ENABLE,
                    .scl_pullup_en = GPIO_PULLUP_ENABLE,
                    .master.clk_speed = rc522->config.i2c.clock_speed_hz,
                };

                ESP_ERR_RET_GUARD(i2c_param_config(rc522->config.i2c.port, &conf));
                ESP_ERR_RET_GUARD(i2c_driver_install(rc522->config.i2c.port, conf.mode, false, false, 0x00));
            }
            break;
        default: {
            ESP_LOGE(TAG, "create_transport: Unknown transport");
            err = ESP_ERR_INVALID_STATE; // unknown transport
            break;
        }
    }

    return err;
}

esp_err_t rc522_create(const rc522_config_t * config, rc522_t * rc522) {
    // 检查输入参数是否有效
    if (!config || !rc522) {
        return ESP_ERR_INVALID_ARG;
    }

    // 初始化错误码为成功
    esp_err_t err = ESP_OK;

    rc522->running = false;
    config_cpy(&(rc522->config), config); // 复制配置参数
    rc522->task_handle = NULL;
    rc522->event_handle = NULL;
    rc522->spi_handle = NULL;
    rc522->initialized = false;
    rc522->scanning = false;
    rc522->tag_was_present_last_time = false;
    rc522->bus_initialized_by_user = false;

    // 创建传输层
    err = rc522_create_transport(rc522);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Fail to create transport");
        rc522_destroy(rc522);
        return err;
    }

    // 初始化事件循环参数
    esp_event_loop_args_t event_args = {
        .queue_size = 1,
        .task_name = NULL
    };

    // 创建事件循环
    err = esp_event_loop_create(&event_args, &rc522->event_handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Fail to create event loop");
        rc522_destroy(rc522);
        return err;
    }

    // 标记RC522为运行状态
    rc522->running = true;

    // 创建RC522任务线程
    BaseType_t ret = xTaskCreate(
        rc522_task,
        "rc522_task",
        rc522->config.task_stack_size,
        rc522,
        rc522->config.task_priority,
        &rc522->task_handle
    );
    if (pdTRUE != ret) {
        ESP_LOGW(TAG, "Fail to create task");
        rc522_destroy(rc522);
        return ESP_ERR_NO_MEM;
    }

    // 返回错误码
    return err;
}

/**
 * 注册RC522模块的事件处理函数。
 * 
 * 本函数用于将指定的事件处理函数注册到RC522模块的事件循环中。
 * 这样，当指定事件发生时，可以自动调用相应的处理函数。
 * 
 * @param rc522 RC522模块的句柄，用于标识具体的RC522模块实例。
 * @param event 需要注册的事件类型。
 * @param event_handler 事件处理函数，当指定事件发生时会被调用。
 * @param event_handler_arg 事件处理函数的参数，可以在事件处理函数中使用。
 * 
 * @return 如果操作成功，返回ESP_OK；如果参数无效，返回ESP_ERR_INVALID_ARG。
 */
esp_err_t rc522_register_events(rc522_handle_t rc522, rc522_event_t event, esp_event_handler_t event_handler, void* event_handler_arg) {
    // 检查rc522句柄是否为空，为空则返回参数错误
    if (!rc522) {
        return ESP_ERR_INVALID_ARG;
    }

    // 注册事件处理函数
    return esp_event_handler_register_with(rc522->event_handle, RC522_EVENTS, event, event_handler, event_handler_arg);
}

/**
 * @brief 取消注册RC522事件的处理函数
 * 
 * 该函数用于从事件循环中移除指定的事件处理函数。这对于不再需要事件更新或避免内存泄漏是必要的。
 * 
 * @param rc522 RC522设备的句柄，用于识别特定的RC522设备。
 * @param event 需要取消注册的事件类型。
 * @param event_handler 需要被移除的事件处理函数。
 * 
 * @return esp_err_t 返回操作结果，可能的值为ESP_OK（成功）或ESP_ERR_INVALID_ARG（输入参数无效）。
 */
esp_err_t rc522_unregister_events(rc522_handle_t rc522, rc522_event_t event, esp_event_handler_t event_handler) {
    // 检查rc522句柄是否为空，为空则返回无效参数错误
    if (!rc522) {
        return ESP_ERR_INVALID_ARG;
    }

    // 从事件循环中取消注册指定的事件处理函数
    return esp_event_handler_unregister_with(rc522->event_handle, RC522_EVENTS, event, event_handler);
}

/* 
static uint64_t rc522_sn_to_u64(uint8_t* sn)
{
    uint64_t result = 0;

    if (!sn) {
        return 0;
    }

    for(int i = 4; i >= 0; i--) {
        result |= ((uint64_t) sn[i] << (i * 8));
    }

    return result;
}
 */

/**
 * 计算CRC值
 * 
 * @brief 使用RC522模块的CRC计算功能，对给定的数据进行CRC计算
 * 
 * @param rc522 RC522模块的句柄
 * @param data 需要计算CRC的数据数组
 * @param n 数据数组的长度
 * @param result 存储计算结果的缓冲区（数组长度至少为2）
 * @return esp_err_t 返回操作结果，ESP_OK表示成功
 */
static esp_err_t rc522_calculate_crc(rc522_handle_t rc522, uint8_t * data, uint8_t n, uint8_t (*result)[2]) {
    esp_err_t err = ESP_OK; // 初始化操作结果为成功
    uint8_t i = 255; // 用于延时循环的计数器
    uint8_t nn = 0; // 读取的中断请求寄存器的值

    // 准备CRC计算
    ESP_ERR_RET_GUARD(rc522_clear_bitmask(rc522, RC522_REG_DIV_IRQ, 0x04)); // 清除DIV_IRQ_REQ寄存器的特定位
    ESP_ERR_RET_GUARD(rc522_set_bitmask(rc522, RC522_REG_FIFO_LEVEL, 0x80)); // 设置FIFO_LEVEL寄存器的特定位
    ESP_ERR_RET_GUARD(rc522_send(rc522, RC522_REG_FIFO_DATA, data, n)); // 将数据发送到FIFO数据寄存器
    ESP_ERR_RET_GUARD(rc522_write(rc522, RC522_REG_COMMAND, RC522_CMD_CALC_CRC)); // 写入命令寄存器，开始CRC计算

    // 循环等待CRC计算完成
    for(;;) {
        ESP_ERR_RET_GUARD(rc522_read(rc522, RC522_REG_DIV_IRQ, &nn)); // 读取DIV_IRQ_REQ寄存器的值

        i--; // 延时循环计数器递减

        // 检查是否CRC计算完成
        if (!(i != 0 && !(nn & 0x04))) {
            break;
        }
    }

    uint8_t tmp; // 临时变量，用于读取CRC结果

    // 读取CRC结果
    ESP_ERR_RET_GUARD(rc522_read(rc522, RC522_REG_CRC_RESULT_LSB, &tmp)); // 读取CRC结果的低字节
    (*result)[0] = tmp;
    ESP_ERR_RET_GUARD(rc522_read(rc522, RC522_REG_CRC_RESULT_MSB, &tmp)); // 读取CRC结果的高字节
    (*result)[1] = tmp;

    return ESP_OK; // 返回操作成功
}

/**
 * @brief RC522卡写入函数
 * @param rc522: RC522设备句柄
 * @param cmd: 发送给RC522的命令
 * @param data: 发送的数据
 * @param n: 发送数据的长度
 * @param rxbits: 接收数据的位长度
 * @return: 操作结果，ESP_OK表示成功
 */
static esp_err_t rc522_card_write(rc522_handle_t rc522, rc522_cmd_t cmd, uint8_t * data, uint8_t n, uint8_t * rxbits) {
    esp_err_t err = ESP_OK; // 错误码
    uint8_t irq = 0x00; // 中断请求寄存器值
    uint8_t irq_wait = 0x00; // 中断等待寄存器值
    uint8_t last_bits = 0; // 最后几位变量
    uint8_t nn = 0; // 临时变量
    uint8_t tmp; // 临时变量
    
    // 根据命令设置中断请求和中断等待寄存器值
    if (cmd == RC522_CMD_AUTHENT) {
        irq = 0x12;
        irq_wait = 0x10;
    } else if (cmd == RC522_CMD_TRANSCEIVE) {
        irq = 0x77;
        irq_wait = 0x30;
    }

    ESP_ERR_RET_GUARD(rc522_write(rc522, RC522_REG_COMM_IRQ_EN, irq | 0x80)); // 设置通信中断使能寄存器
    ESP_ERR_RET_GUARD(rc522_clear_bitmask(rc522, RC522_REG_COMM_IRQ, 0x80)); // 清除通信中断请求寄存器的相应位
    ESP_ERR_RET_GUARD(rc522_set_bitmask(rc522, RC522_REG_FIFO_LEVEL, 0x80)); // 设置FIFO水平寄存器的相应位
    ESP_ERR_RET_GUARD(rc522_write(rc522, RC522_REG_COMMAND, RC522_CMD_IDLE)); // 发送空闲命令
    ESP_ERR_RET_GUARD(rc522_send(rc522, RC522_REG_FIFO_DATA, data, n)); // 发送数据到FIFO
    ESP_ERR_RET_GUARD(rc522_write(rc522, RC522_REG_COMMAND, cmd)); // 发送命令

    if (cmd == RC522_CMD_TRANSCEIVE) { // 如果是TRANSCEIVE命令，设置位帧寄存器
        ESP_ERR_RET_GUARD(rc522_set_bitmask(rc522, RC522_REG_BIT_FRAMING, 0x80));
    }

    uint16_t i = 1000; // 设置超时计数器

    for(;;) { // 等待中断请求
        ESP_ERR_RET_GUARD(rc522_read(rc522, RC522_REG_COMM_IRQ, &nn)); // 读取通信中断请求寄存器

        i--; // 超时计数减一

        // 判断是否满足中断条件或超时
        if (!(i != 0 && (((nn & 0x01) == 0) && ((nn & irq_wait) == 0)))) {
            break;
        }
    }

    ESP_ERR_RET_GUARD(rc522_clear_bitmask(rc522, RC522_REG_BIT_FRAMING, 0x80)); // 清除位帧寄存器的相应位

    if (i != 0) { // 如果没有超时
        ESP_ERR_RET_GUARD(rc522_read(rc522, RC522_REG_ERROR, &tmp)); // 读取错误寄存器

        if ((tmp & 0x1B) == 0) { // 如果没有错误
            if (cmd == RC522_CMD_TRANSCEIVE) {
                ESP_ERR_RET_GUARD(rc522_read(rc522, RC522_REG_FIFO_LEVEL, &nn)); // 读取FIFO水平寄存器
                ESP_ERR_RET_GUARD(rc522_read(rc522, RC522_REG_CONTROL, &tmp)); // 读取控制寄存器

                last_bits = tmp & 0x07; // 获取最后几位

                if (last_bits != 0) { // 如果最后几位不为0
                    ESP_LOGI(TAG, "last_bits: %d", last_bits);
                    *rxbits = ((nn - 1) << 3) + last_bits; // 计算接收数据长度
                } else {
                    *rxbits = nn << 3; // 直接使用FIFO水平寄存器的值
                }

                if (nn == 0) nn = 1;

                memset(rc522->rxbuf, 0, sizeof(rc522->rxbuf));
                for (i = 0; i < nn; i++) { // 逐个读取数据
                    ESP_ERR_RET_GUARD(rc522_read(rc522, RC522_REG_FIFO_DATA, &tmp));
                    rc522->rxbuf[i] = tmp;
                }
            }
        }
    } else {
        err = ESP_ERR_TIMEOUT;
    }

    // 返回操作结果
    return err;
}

/**
 * @brief RC522请求函数
 * 
 * 该函数用于根据指定的请求模式req_mode，从RC522读取标签响应。函数通过设置RC522的位帧寄存器，
 * 并发送一个传输接收命令来实现。成功执行后，会返回标签的响应数据和响应数据的长度。
 * 
 * @param rc522 RC522设备的句柄
 * @param req_mode 请求模式，用于指定如何从RC522读取标签响应
 * @param tag_type 卡类型
 * @return esp_err_t 返回操作结果，ESP_OK表示成功，其他值表示错误
 */
static esp_err_t rc522_request(rc522_handle_t rc522, uint8_t req_mode, uint8_t (*tag_type)[2]) {
    esp_err_t err = ESP_OK;
    // uint8_t _result[2] = { 0 }; // 接收结果的指针
    uint8_t _rxbits; // 初始化接收结果的长度

    ESP_ERR_RET_GUARD(rc522_clear_bitmask(rc522, RC522_REG_STATUS_2, 0x08));
    // 设置RC522的位帧寄存器，准备接收数据
    ESP_ERR_RET_GUARD(rc522_write(rc522, RC522_REG_BIT_FRAMING, 0x07));
    ESP_ERR_RET_GUARD(rc522_set_bitmask(rc522, RC522_REG_TX_CONTROL, 0x03));
    
    // 向RC522发送传输接收命令
    ESP_ERR_RET_GUARD(rc522_card_write(rc522, RC522_CMD_TRANSCEIVE, &req_mode, 1, &_rxbits));

    // 检查接收到的数据长度是否符合预期，不符合则释放资源并返回错误
    if (_rxbits != 16) {
        return ESP_ERR_INVALID_STATE;
    }

    (*tag_type)[0] = rc522->rxbuf[0];
    (*tag_type)[1] = rc522->rxbuf[1];

    return err; // 返回操作结果
}

static esp_err_t rc522_select(rc522_handle_t rc522, uint8_t uid[5]) {
    esp_err_t err = ESP_OK;
    uint8_t data[9] = { 0x93, 0x70, uid[0], uid[1], uid[2], uid[3], uid[4], 0, 0 };
    // uint8_t data[2] = { 0x93, 0x70 };
    uint8_t _rxbits;

    ESP_ERR_RET_GUARD(rc522_write(rc522, RC522_REG_BIT_FRAMING, 0x00));
    ESP_ERR_RET_GUARD(rc522_calculate_crc(rc522, data, 7, &data[7])); // TODO
    ESP_ERR_RET_GUARD(rc522_write(rc522, RC522_REG_STATUS_2, 0x08));
    ESP_ERR_RET_GUARD(rc522_card_write(rc522, RC522_CMD_TRANSCEIVE, data, sizeof(data), &_rxbits));

    if (_rxbits != 0x18) {
        ESP_LOGI(TAG, "##select ? _rxbits %02X", _rxbits);
        return ESP_ERR_INVALID_RESPONSE;
    }

    return err;
}

/* 
static esp_err_t rc522_picc_wakeup(rc522_handle_t rc522) {
    esp_err_t err = ESP_OK;

    uint8_t tag_type[2];

    ESP_ERR_RET_GUARD(rc522_request(rc522, MIFARE_WUPA, &tag_type));

    if(tag_type[1] != 0x00 || tag_type[0] != 0x04) {
        err = ESP_ERR_INVALID_STATE; 
    } else {
        printf(" >> Wakeup  0x%02x%02x \n", tag_type[1], tag_type[0]);
    }
    
    return err;
}
 */

/**
 * @brief 实现RFID卡的防冲突算法
 * 
 * 该函数用于在多张RFID卡同时进入读写器的射频场时，实现卡的识别和防冲突。
 * 通过读写器的通信接口实现卡的防冲突操作，获取唯一卡的UID。
 * 
 * @param rc522 读写器的句柄，用于操作RFID读写器
 * @param result 返回识别出的RFID卡的UID指针
 * @return esp_err_t 返回操作的错误码
 */
static esp_err_t rc522_anticoll(rc522_handle_t rc522, uint8_t (*result)[5]) {
    esp_err_t err = ESP_OK; // 初始化错误码为成功
    uint8_t _rxbits; // 存储返回的长度

    ESP_ERR_RET_GUARD(rc522_clear_bitmask(rc522, RC522_REG_STATUS_2, 0x08));
    // 设置位帧寄存器为0x00，准备进行防冲突操作
    ESP_ERR_RET_GUARD(rc522_write(rc522, RC522_REG_BIT_FRAMING, 0x00));
    ESP_ERR_RET_GUARD(rc522_clear_bitmask(rc522, RC522_REG_COLL, 0x80));

    // 向读写器发送防冲突命令和相关数据，获取唯一卡的UID
    ESP_ERR_RET_GUARD(rc522_card_write(rc522, RC522_CMD_TRANSCEIVE, (uint8_t[]) MIFARE_ANTICOLLISION_CL_1, 2, &_rxbits));

    if (_rxbits != 40) { // 5 bytes
        return ESP_ERR_INVALID_RESPONSE;
    }

    ESP_ERR_RET_GUARD(rc522_set_bitmask(rc522, RC522_REG_COLL, 0x80));

    uint8_t bcrc = 0;
    for (size_t i = 0; i < 5; i++) {
        bcrc ^= rc522->rxbuf[i];
    }
    if (bcrc != 0) {
        return ESP_ERR_INVALID_CRC;
    }
    
    memcpy(*result, rc522->rxbuf, 5);

    return err;
}

static esp_err_t rc522_anticoll2(rc522_handle_t rc522, uint8_t (*result)[4]) {
    esp_err_t err = ESP_OK; // 初始化错误码为成功
    uint8_t _rxbits; // 存储返回的长度

    // ESP_ERR_RET_GUARD(rc522_clear_bitmask(rc522, RC522_REG_STATUS_2, 0x08));
    // 设置位帧寄存器为0x00，准备进行防冲突操作
    ESP_ERR_RET_GUARD(rc522_write(rc522, RC522_REG_BIT_FRAMING, 0x00));
    // ESP_ERR_RET_GUARD(rc522_clear_bitmask(rc522, RC522_REG_COLL, 0x80));

    // 向读写器发送防冲突命令和相关数据，获取唯一卡的UID
    ESP_ERR_RET_GUARD(rc522_card_write(rc522, RC522_CMD_TRANSCEIVE, (uint8_t[]) MIFARE_ANTICOLLISION_CL_2, 2, &_rxbits));

    ESP_ERR_RET_GUARD(rc522_set_bitmask(rc522, RC522_REG_COLL, 0x80));

    uint8_t bcrc = 0;
    for (size_t i = 0; i < 5; i++) {
        bcrc ^= rc522->rxbuf[i];
    }
    if (bcrc != 0) {
        return ESP_ERR_INVALID_CRC;
    }
    
    memcpy(*result, rc522->rxbuf, 4);

    return err;
}

static esp_err_t rc522_stop_picc_communication(rc522_handle_t rc522) {

    esp_err_t err = ESP_OK;
    /*
        This part stops the communication with the picc by issuing a HALT_A as well as STOP_CRYPTO1
    */
    uint8_t _rxbits;

    // Issue a HALT_A
    uint8_t buf[4];
    memcpy(buf, (uint8_t[])MIFARE_HALT, 2);
    memcpy(buf + 2, (uint8_t[]){ 0x00, 0x00 }, 2);
    ESP_ERR_RET_GUARD(rc522_calculate_crc(rc522, buf, 2, buf + 2));
    ESP_ERR_RET_GUARD(rc522_card_write(rc522, RC522_CMD_TRANSCEIVE, buf, 4, &_rxbits));
    // Stop CYPTO1
    // Clear MFCrypto1On bit
    ESP_ERR_RET_GUARD(rc522_clear_bitmask(rc522, RC522_REG_STATUS_2, 0x08));

    return err;
}

/**
 * 从RC522读取标签的序列号
 * 
 * 此函数通过发送请求、防冲突和选择标签等步骤，来识别并返回标签的序列号
 * 它处理两种类型的序列号：4字节和7字节，并在通信结束后停止与标签的通信
 * 
 * @param rc522 RC522设备的句柄
 * @param result 指向存储标签序列号的变量指针
 * @return esp_err_t 类型的错误代码，指示函数执行的结果
 */
static esp_err_t rc522_get_tag(rc522_handle_t rc522, serial_no_t * result) {
    esp_err_t err = ESP_OK;

    uint8_t tag_type[2] = { 0 };
    // 发送请求以激活标签
    ESP_ERR_RET_GUARD(rc522_request(rc522, MIFARE_REQA, &tag_type));

    uint8_t segment[5] = { 0 }; // 卡号（没有折叠的情况下）或者卡号的第一部分（折叠的情况下）
    ESP_ERR_RET_GUARD(rc522_anticoll(rc522, &segment));
    ESP_ERR_RET_GUARD(rc522_select(rc522, segment));

    if (segment[0] != 0x88) { // 卡号没有折叠
        result->type = SERIAL_NO_TYPE_4;
        memcpy(result->data, segment, 4);
    } else {
        uint8_t segment2[4] = { 0 }; // 卡号的第二部分，三级折叠的情况下还有第三部分
        ESP_ERR_RET_GUARD(rc522_anticoll2(rc522, &segment2));

        if (segment2[0] != 0x88) { // 卡号没有三级折叠
            result->type = SERIAL_NO_TYPE_7;
            memcpy(result->data, &segment[1], 3);
            memcpy(&(result->data[3]), segment2, 4);
        } else {
            // TODO
        }
    }
    // 停止与标签的通信
    ESP_ERR_RET_GUARD(rc522_stop_picc_communication(rc522));

    return err;
}

/* 
static esp_err_t rc522_authenticate(rc522_handle_t rc522, uint8_t auth_key, uint8_t blockAddr, MIFARE_Key * key) {
    esp_err_t err = ESP_OK;
    uint8_t _rxbits;

    uint8_t buf[12];
    buf[0] = auth_key;
    buf[1] = blockAddr;
    memcpy(buf + 2, key->keyByte, 6);
    memcpy(buf + 8, rc522->tag_uid, 4);

    ESP_ERR_RET_GUARD(rc522_write(rc522, RC522_REG_BIT_FRAMING, 0x00));
    ESP_ERR_RET_GUARD(rc522_card_write(rc522, RC522_CMD_AUTHENT, buf, 12, &_rxbits));

    return err;
}

static esp_err_t  rc522_read_block_from_picc(rc522_handle_t rc522, uint8_t blockAddr,
											uint8_t *buffer,		///< The buffer to store the data in
											uint8_t *bufferSize	///< Buffer size, at least 18 bytes. Also number of bytes returned if STATUS_OK.
										) {
    esp_err_t err = ESP_OK;
    uint8_t _rxbits;

    // Sanity checks
    if (buffer == NULL) {
        ESP_LOGE(TAG, "The buffer pointer is null, PICC's block cannot be read");
        return ESP_ERR_INVALID_ARG;
    }
    if (*bufferSize < 18) {
        ESP_LOGE(TAG, "The buffer reading a block from the PICC is small, increase the size to atleast 18 bytes");
        return ESP_ERR_INVALID_ARG;
    }

	// Build command buffer
	buffer[0] = MIFARE_READ;
	buffer[1] = blockAddr;
	// Calculate CRC_A
    ESP_ERR_RET_GUARD(rc522_calculate_crc(rc522, buffer, 2, &buffer[2]));

    // Transmit the buffer and receive the response, validate CRC_A.
    ESP_ERR_RET_GUARD(rc522_write(rc522, RC522_REG_BIT_FRAMING, 0x00));
    ESP_ERR_RET_GUARD(rc522_card_write(rc522, RC522_CMD_TRANSCEIVE, buffer, 4, &_rxbits));

    return err;
}

static esp_err_t rc522_read_sector_from_picc(rc522_handle_t rc522, MIFARE_Key* key, uint8_t sector) {
    esp_err_t err = ESP_OK;

    uint8_t no_of_blocks = 4; // Number of blocks in sector, 4 in MIFARE cards
    uint8_t firstBlock = sector * no_of_blocks; // Adress of first block of a sector, we will always read 5 sectors (MIFARE_MINI): 0, 4, 8, 12, 16

	// The access bits are stored in a peculiar fashion.
	// There are four groups:
	//		g[3]	Access bits for the sector trailer, block 3 (for sectors 0-31) or block 15 (for sectors 32-39)
	//		g[2]	Access bits for block 2 (for sectors 0-31) or blocks 10-14 (for sectors 32-39)
	//		g[1]	Access bits for block 1 (for sectors 0-31) or blocks 5-9 (for sectors 32-39)
	//		g[0]	Access bits for block 0 (for sectors 0-31) or blocks 0-4 (for sectors 32-39)
	// Each group has access bits [C1 C2 C3]. In this code C1 is MSB and C3 is LSB.
	// The four CX bits are stored together in a nible cx and an inverted nible cx_.
	uint8_t c1, c2, c3;		// Nibbles
	uint8_t c1_, c2_, c3_;		// Inverted nibbles

	uint8_t g[4];				// Access bits for each of the four groups.
	uint8_t group;				// 0-3 - active group for access bits
	bool firstInGroup;		// True for the first block dumped in the group
	

    // Dump blocks, highest address first.
	uint8_t byteCount;
	uint8_t buffer[18];
    uint8_t tag_type[2];
	uint8_t blockAddr;
	bool isSectorTrailer = true;
	bool invertedError = false;	// Avoid "unused variable" warning.

    for (int8_t blockOffset = no_of_blocks - 1; blockOffset >= 0; blockOffset--) {
		blockAddr = firstBlock + blockOffset;
        // Sector number - only on first line
        if (isSectorTrailer) {
			printf("  "); // Pad with spaces
			printf("%d", sector);
			printf("   ");
		} else {
            printf("      ");
        }
        
        // Block number
		if (blockAddr < 10)
			printf("   "); // Pad with spaces
		else {
			printf("  "); // Pad with spaces
		}
		printf("%d", blockAddr);
		printf("  ");
        // Establish encrypted communications before reading the first block
		if (isSectorTrailer) {
            ESP_ERR_RET_GUARD(rc522_request(rc522, MIFARE_WUPA, &tag_type));
            if (tag_type[0] != 0) {
                ESP_ERR_RET_GUARD(rc522_authenticate(rc522, MIFARE_AUTH_KEY_A, firstBlock, key));
            }
		}
        // Read block
		byteCount = sizeof(buffer);
        ESP_ERR_RET_GUARD(rc522_request(rc522, MIFARE_WUPA, &tag_type));
        if (tag_type[0] != 0) {
		    ESP_ERR_RET_GUARD(rc522_read_block_from_picc(rc522, blockAddr, buffer, &byteCount));
        }
		// Dump data
		for (uint8_t index = 0; index < 16; index++) {
			printf(" 0x%02x", buffer[index]);
			if ((index % 4) == 3) {
				printf(" ");
			}
		}
        // Parse sector trailer data
		if (isSectorTrailer) {
            // printf(" << SECTOR TRAILER >> ");
			c1  = buffer[7] >> 4;
			c2  = buffer[8] & 0xF;
			c3  = buffer[8] >> 4;
			c1_ = buffer[6] & 0xF;
			c2_ = buffer[6] >> 4;
			c3_ = buffer[7] & 0xF;
			invertedError = (c1 != (~c1_ & 0xF)) || (c2 != (~c2_ & 0xF)) || (c3 != (~c3_ & 0xF));
			g[0] = ((c1 & 1) << 2) | ((c2 & 1) << 1) | ((c3 & 1) << 0);
			g[1] = ((c1 & 2) << 1) | ((c2 & 2) << 0) | ((c3 & 2) >> 1);
			g[2] = ((c1 & 4) << 0) | ((c2 & 4) >> 1) | ((c3 & 4) >> 2);
			g[3] = ((c1 & 8) >> 1) | ((c2 & 8) >> 2) | ((c3 & 8) >> 3);
			isSectorTrailer = false;
		}
		
		// Which access group is this block in?
		if (no_of_blocks == 4) {
			group = blockOffset;
			firstInGroup = true;
		}
		else {
			group = blockOffset / 5;
			firstInGroup = (group == 3) || (group != (blockOffset + 1) / 5);
		}
		
		if (firstInGroup) {
			// Print access bits
			printf(" [ ");
		    printf("%d", (g[group] >> 2) & 1); printf(" ");
			printf("%d", (g[group] >> 1) & 1); printf(" ");
			printf("%d", (g[group] >> 0) & 1);
			printf(" ] ");
			if (invertedError) {
				ESP_LOGW(TAG, " Inverted access bits did not match! ");
			}
		}
		
		if (group != 3 && (g[group] == 1 || g[group] == 6)) { // Not a sector trailer, a value block
			int32_t value = ((int32_t)(buffer[3])<<24) | ((int32_t)(buffer[2])<<16) | ((int32_t)(buffer[1])<<8) | ((int32_t)(buffer[0]));
			printf(" Value=0x%lx", value);
			printf(" Adr=0x%02x", buffer[12]);
		}
		printf("\n");
    }

    return err;
}

static esp_err_t rc522_read_data_from_picc(rc522_handle_t rc522, MIFARE_Key* key)
{
    esp_err_t err = ESP_OK;
    
    // For the sake of simplicity we will treat all PICCs as MIFARE_MINI
    // Has 5 sectors * 4 blocks/sector * 16 bytes/block = 320 bytes.
    // For now, even if we read/write for other type of PICC, we will use only 320 bytes.
    uint8_t no_of_sectors = 5;

    // 5 sectors are available for read/write
    printf("Sector Block    0    1    2    3     4    5    6    7     8    9   10   11    12   13   14   15  AccessBits\n");
    for (int8_t i = no_of_sectors - 1; i >= 0; i--) {
        ESP_ERR_JMP_GUARD(rc522_read_sector_from_picc(rc522, key, i));
    }
    
    ESP_ERR_JMP_GUARD(rc522_stop_picc_communication(rc522));

    JMP_GUARD_GATES({
        // TODO
    }, {
        // Nothing to do if everything went fine
    });
    
    return err;
}
 */

// 开始RC522模块，进入扫描模式。
// 如果模块尚未初始化，则在开始前进行初始化。
// 参数:
// - rc522: RC522模块的句柄。
// 返回值:
// - esp_err_t: 操作的错误代码。
esp_err_t rc522_start(rc522_handle_t rc522) {
    esp_err_t err = ESP_OK;

    if (!rc522) {
        return ESP_ERR_INVALID_ARG;
    }

    if (rc522->scanning) { // 已经处于扫描模式
        return ESP_OK;
    }

    uint8_t tmp = 0;

    if (!rc522->initialized) {
        // 初始化仅执行一次，在首次调用启动函数时完成。

        // TODO: 将测试提取到专用函数中
        // ---------- 读写测试 ------------
        // TODO: 使用敏感度较低的寄存器进行测试，或在测试结束时将寄存器恢复到原始状态
        const rc522_reg_t test_addr = RC522_REG_MOD_WIDTH, test_val = 0x25;
        // uint8_t pass = 0;

        for (uint8_t i = test_val; i < test_val + 2; i++) {
            // ESP_LOGI(TAG, "rc522_write(%p, %u, %u)", rc522, test_addr, i);
            err = rc522_write(rc522, test_addr, i);

            if (err != ESP_OK) {
                ESP_LOGE(TAG, "RW test failed on write %u", i);
                rc522_destroy(rc522);

                return err;
            } else {
                err = rc522_read(rc522, test_addr, &tmp);

                if (err != ESP_OK) {
                    ESP_LOGE(TAG, "RW test failed on read %u", i);
                    rc522_destroy(rc522);

                    return err;
                } else if (tmp != i) {
                    ESP_LOGE(TAG, "RW test failed %u", i);
                    rc522_destroy(rc522);

                    return err;
                }
            }
        }
        // ------- 读写测试结束 --------

        // 执行必要的寄存器配置以初始化RC522模块。
        ESP_ERR_RET_GUARD(rc522_write(rc522, RC522_REG_COMMAND, RC522_CMD_SOFTRESET));
        ESP_ERR_RET_GUARD(rc522_write(rc522, RC522_REG_TIMER_MODE, 0x8D));
        ESP_ERR_RET_GUARD(rc522_write(rc522, RC522_REG_TIMER_PRESCALER, 0x3E));
        ESP_ERR_RET_GUARD(rc522_write(rc522, RC522_REG_TIMER_RELOAD_LSB, 0x1E));
        ESP_ERR_RET_GUARD(rc522_write(rc522, RC522_REG_TIMER_RELOAD_MSB, 0x00));
        ESP_ERR_RET_GUARD(rc522_write(rc522, RC522_REG_TX_ASK, 0x40));
        ESP_ERR_RET_GUARD(rc522_write(rc522, RC522_REG_MODE, 0x3D));

        // 打开天线并加载固件。
        ESP_ERR_RET_GUARD(rc522_antenna_on(rc522));
        ESP_ERR_RET_GUARD(rc522_firmware(rc522, &tmp));

        rc522->initialized = true;

        // 记录固件版本。
        ESP_LOGI(TAG, "Initialized (firmware v%d.0)", (tmp & 0x03));
        xTaskNotify(rc522->task_handle, EVENT_BIT_HARDWARE_READY, eSetBits);
    }

    rc522->scanning = true;

    return ESP_OK;
}

/**
 * 暂停RFID扫描功能
 * 
 * 此函数用于暂停RFID扫描如果RFID扫描已经暂停，则无需进行任何操作
 * 
 * @param rc522 RFID处理句柄，指向RFID操作结构体的指针
 * @return 返回操作状态，可能的值为：
 *         - ESP_OK：操作成功
 *         - ESP_ERR_INVALID_ARG：输入参数无效（rc522为NULL）
 */
esp_err_t rc522_pause(rc522_handle_t rc522) {
    // 检查输入参数是否有效，如果rc522为NULL，则返回错误
    if (!rc522) {
        return ESP_ERR_INVALID_ARG;
    }

    // 如果扫描已经暂停，则无需进行任何操作，直接返回成功
    if (!rc522->scanning) {
        return ESP_OK;
    }

    // 将扫描状态设置为false，暂停扫描
    rc522->scanning = false;

    // 操作成功，返回ESP_OK
    return ESP_OK;
}

/**
 * @brief 销毁RC522传输层设备
 * 
 * 根据RC522模块使用的传输层类型（SPI或I2C），执行相应的销毁操作。
 * 如果是SPI类型，将从总线上移除设备，如果总线是由用户初始化的，还将释放总线。
 * 如果是I2C类型，将删除I2C驱动程序。
 * 如果传输层类型未知，将返回错误状态。
 * 
 * @param rc522 RC522模块的句柄，用于访问模块的配置和状态信息
 * @return esp_err_t 返回操作的结果，成功返回ESP_OK，失败返回相应的错误码
 */
static esp_err_t rc522_destroy_transport(rc522_handle_t rc522) {
    esp_err_t err;

    // 根据配置的传输层类型执行相应的销毁操作
    switch(rc522->config.transport) {
        case RC522_TRANSPORT_SPI:
            err = spi_bus_remove_device(rc522->spi_handle); // 从SPI总线上移除设备
            // 如果总线是由用户初始化的，释放总线
            if (rc522->bus_initialized_by_user) {
                err = spi_bus_free(rc522->config.spi.host);
            }
            break;
        case RC522_TRANSPORT_I2C:
            // 删除I2C驱动程序
            err = i2c_driver_delete(rc522->config.i2c.port);
            break;
        default:
            // 如果传输层类型未知，记录警告并返回错误状态
            ESP_LOGW(TAG, "destroy_transport: Unknown transport");
            err = ESP_ERR_INVALID_STATE;
    }

    return err;
}

/**
 * @brief 释放RC522资源
 * 
 * 该函数用于安全地停止并释放与RC522相关的所有资源，包括暂停任务、释放传输资源、删除事件循环和配置数据。
 * 
 * @param rc522 RC522设备句柄，指向包含RC522设备相关信息的结构体。
 * @return esp_err_t 返回操作结果，ESP_OK表示成功，其他值表示错误代码。
 */
esp_err_t rc522_destroy(rc522_handle_t rc522) {
    esp_err_t err = ESP_OK; // 初始化错误码为成功

    if (!rc522) {
        return ESP_ERR_INVALID_ARG; // 如果传入的句柄为空，返回无效参数错误
    }

    if (xTaskGetCurrentTaskHandle() == rc522->task_handle) {
        ESP_LOGE(TAG, "Cannot destroy rc522 from event handler"); // 如果当前任务句柄与RC522的任务句柄相同，记录错误日志

        return ESP_ERR_INVALID_STATE; // 返回无效状态错误
    }

    err = rc522_pause(rc522); // 停止任务
    rc522->running = false; // 标记RC522为停止状态，任务将自行删除

    // TODO: 在这里等待任务退出

    err = rc522_destroy_transport(rc522); // 释放RC522传输资源

    if (rc522->event_handle) {
        err = esp_event_loop_delete(rc522->event_handle); // 如果事件句柄存在，删除事件循环
        rc522->event_handle = NULL; // 将事件句柄置空
    }

    free(rc522); // 释放RC522设备句柄内存

    return err; // 返回操作结果
}

static esp_err_t rc522_dispatch_event(rc522_handle_t rc522, rc522_event_t event, void* data) {
    esp_err_t err;

    if (!rc522) {
        return ESP_ERR_INVALID_ARG;
    }

    rc522_event_data_t e_data = {
        .rc522 = rc522,
        .ptr = data,
    };

    ESP_ERR_RET_GUARD(esp_event_post_to(
        rc522->event_handle,
        RC522_EVENTS,
        event,
        &e_data,
        sizeof(rc522_event_data_t),
        portMAX_DELAY
    ));

    return esp_event_loop_run(rc522->event_handle, 0);
}

/**
 * @brief RC522 SPI总线数据发送函数
 * 
 * 本函数负责通过SPI总线向RC522发送命令或数据。它首先对发送缓冲区中的数据进行格式调整，
 * 然后通过SPI设备句柄进行数据传输。
 * 
 * @param rc522 RC522设备句柄，包含SPI设备句柄等信息
 * @param buffer 发送缓冲区，包含待发送的数据
 * @param length 发送数据的长度，单位为字节
 * @return esp_err_t 返回spi_device_transmit函数的执行结果，表示发送操作是否成功
 * 
 * @note 发送缓冲区的第一个字节会在发送前进行位移和屏蔽操作，以符合RC522的通信协议格式。
 */
static esp_err_t rc522_spi_send(rc522_handle_t rc522, uint8_t * buffer, uint8_t length) {
    // 调整发送数据的第一个字节，符合RC522的通信协议要求
    buffer[0] = (buffer[0] << 1) & 0x7E;

    // 构建并发起SPI传输请求
    return spi_device_transmit(rc522->spi_handle, &(spi_transaction_t){
        .length = 8 * length, // 设置传输的位长度
        .tx_buffer = buffer, // 指定发送缓冲区
    });
}

/**
 * @brief 通过SPI接收数据
 * 
 * 该函数用于通过SPI接口从RC522芯片接收数据。它支持半双工和全双工两种SPI模式。
 * 在半双工模式下，数据的发送和接收是分开进行的；而在全双工模式下，可以同时发送和接收数据。
 * 
 * @param rc522 RC522设备句柄，包含SPI配置信息
 * @param buffer 接收数据的缓冲区
 * @param length 要接收的字节数
 * @param addr RC522芯片的地址
 * @return esp_err_t 返回操作状态，ESP_OK表示成功，其他值表示错误
 */
static esp_err_t rc522_spi_receive(rc522_handle_t rc522, uint8_t * buffer, uint8_t length, uint8_t addr) {
    esp_err_t err = ESP_OK; // 初始化错误码为ESP_OK

    // 根据RC522芯片的通信协议，将地址位左移一位，并设置读操作标志位
    addr = ((addr << 1) & 0x7E) | 0x80;

    // 检查SPI设备是否配置为半双工模式
    if (SPI_DEVICE_HALFDUPLEX & rc522->config.spi.device_flags) {
        // 在半双工模式下，一次传输中同时发送地址和接收数据
        ESP_ERR_RET_GUARD(spi_device_transmit(rc522->spi_handle, &(spi_transaction_t) {
            .flags = SPI_TRANS_USE_TXDATA,
            .length = 8,
            .tx_data[0] = addr,
            .rxlength = 8 * length,
            .rx_buffer = buffer,
        }));
    } else { // 全双工模式
        // 首先发送地址，不接收数据
        ESP_ERR_RET_GUARD(spi_device_transmit(rc522->spi_handle, &(spi_transaction_t) {
            .flags = SPI_TRANS_USE_TXDATA,
            .length = 8,
            .tx_data[0] = addr,
        }));

        // 然后进行数据接收
        ESP_ERR_RET_GUARD(spi_device_transmit(rc522->spi_handle, &(spi_transaction_t) {
            .flags = 0x00,
            .length = 8,
            .rxlength = 8 * length,
            .rx_buffer = buffer,
        }));
    }

    return err; // 返回操作状态
}

/**
 * 通过I2C总线向RC522发送数据
 * 
 * 此函数封装了通过I2C总线向RC522芯片发送数据的逻辑，使用了底层的I2C主写函数实现
 * 它是静态内联函数，旨在提高运行时的效率，减少函数调用的开销
 * 
 * @param rc522 RC522设备的句柄，包含了配置信息等
 * @param buffer 指向发送数据的缓冲区的指针
 * @param length 要发送的数据长度（字节数）
 * @return 返回操作结果，esp_err_t类型的错误代码
 * 
 * 调用此函数是为了将数据从微控制器通过I2C总线发送到RC522芯片中
 * 它依赖于提供的RC522设备句柄来获取I2C端口信息，并使用全局定义的RC522_I2C_ADDRESS地址进行通信
 * 超时设置通过配置中的读写超时转换为tick来实现，确保了函数调用的灵活性和稳定性
 */
static inline esp_err_t rc522_i2c_send(rc522_handle_t rc522, uint8_t* buffer, uint8_t length) {
    return i2c_master_write_to_device(
        rc522->config.i2c.port,
        RC522_I2C_ADDRESS,
        buffer,
        length,
        rc522->config.i2c.rw_timeout_ms / portTICK_PERIOD_MS
    );
}

/**
 * @brief 通过I2C接收数据
 * 
 * 该函数用于从RC522设备通过I2C总线接收指定长度的数据。它内部调用了`i2c_master_write_read_device`函数，
 * 该函数需要传入I2C端口、设备地址、用于写入的地址指针、写入长度、用于读取的缓冲区、读取长度以及超时时间。
 * 
 * @param rc522 RC522设备句柄，包含了配置信息
 * @param buffer 接收数据的缓冲区
 * @param length 要接收的数据长度
 * @param addr I2C设备内部的寄存器地址
 * 
 * @return esp_err_t 返回操作结果，可能的错误代码
 */
static inline esp_err_t rc522_i2c_receive(rc522_handle_t rc522, uint8_t * buffer, uint8_t length, uint8_t addr) {
    // 调用i2c_master_write_read_device函数进行读操作
    // 参数依次为：I2C端口、设备地址、写入的寄存器地址、写入长度、读取缓冲区、读取长度、超时时间（毫秒）
    return i2c_master_write_read_device(
        rc522->config.i2c.port,
        RC522_I2C_ADDRESS,
        &addr,
        1,
        buffer,
        length,
        rc522->config.i2c.rw_timeout_ms / portTICK_PERIOD_MS
    );
}

/**
 * rc522_task函数是RC522读卡器模块的任务函数。
 * 它负责初始化RC522模块，然后进入一个循环，不断扫描是否有RFID标签靠近读卡器。
 * 如果有标签靠近，它将读取标签的序列号，并触发一个事件通知上层应用。
 * 
 * 参数:
 * arg - 传递给任务的参数，这里是rc522_handle_t类型的指针，用于访问RC522模块的内部状态。
 */
static void rc522_task(void* arg) {
    // 将arg转换为rc522_handle_t类型，并保存在本地变量中，以便于访问RC522模块的状态。
    rc522_handle_t rc522 = (rc522_handle_t) arg;

    // 输出日志，指示RC522任务开始运行，并等待读写测试完成。
    ESP_LOGI(TAG, "Starting rc522 task, wait for RW test...");
    // 等待硬件准备就绪的事件通知。
    xTaskNotifyWait(0, EVENT_BIT_HARDWARE_READY, NULL, portMAX_DELAY);

    // 循环运行，直到RC522模块停止。
    while (rc522->running) {
        // 如果没有进行扫描，则跳过当前循环，等待下一次扫描。
        if (!rc522->scanning) {
            vTaskDelay(100 / portTICK_PERIOD_MS);
            continue;
        }

        // 用于存储标签的序列号的指针。
        serial_no_t serial_no = { 0 };

        // 调用rc522_get_tag函数尝试获取标签的序列号。
        if (ESP_OK != rc522_get_tag(rc522, &serial_no)) {
            // 如果获取失败，可能是没有标签靠近或者通信问题。
            // TODO: 需要实现逻辑来判断是标签不存在还是其他协议问题。
        }
        
        // 检查是否成功获取到标签的序列号。
        if (serial_no.type == SERIAL_NO_TYPE_NONE) {
            // 如果没有获取到序列号，重置上次标签存在状态。
            rc522->tag_was_present_last_time = false;
        } else if (!rc522->tag_was_present_last_time) {
            // 如果上次没有标签，现在获取到了，则记录标签信息并触发事件。
            rc522_dispatch_event(rc522, RC522_EVENT_TAG_SCANNED, &serial_no);
            rc522->tag_was_present_last_time = true;

            // 尝试进行身份验证。
            /*
            MIFARE_Key key;
            for (uint8_t i = 0; i < 6; i++) {
                key.keyByte[i] = 0xff;
            }
            rc522_read_data_from_picc(rc522, &key);
            */
        }

        // 计算下一次扫描的延迟间隔。
        int delay_interval_ms = rc522->config.scan_interval_ms;
        if (rc522->tag_was_present_last_time) {
            // 如果上次有标签存在，则增加延迟间隔，以减少扫描频率。
            delay_interval_ms <<= 1;
        }

        // 延迟一定时间，等待下一次扫描。
        vTaskDelay(delay_interval_ms / portTICK_PERIOD_MS);
    }

    // 任务结束时删除自身。
    vTaskDelete(NULL);
}
