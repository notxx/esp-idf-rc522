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

static const char* TAG = "rc522";

/**
 * @brief Macro for safely freeing memory.
 *        This macro checks if the pointer is not NULL before calling the free function.
 *        After freeing the memory, it sets the pointer to NULL to prevent dangling pointer issues.
 *        This helps in avoiding double free errors and improves the safety of memory management.
 * @param ptr Pointer to the memory to be freed.
 */
#define FREE(ptr) \
    if(ptr) { free(ptr); ptr = NULL; }

struct rc522 {
    bool running;                          /*<! Indicates whether rc522 task is running or not */
    rc522_config_t* config;                /*<! Configuration */
    TaskHandle_t task_handle;              /*<! Handle of task */
    esp_event_loop_handle_t event_handle;  /*<! Handle of event loop */
    spi_device_handle_t spi_handle;
    // TODO: Use new 'status' field, instead of initialized, scanning, etc...
    bool initialized;                      /*<! Set on the first start() when configuration is sent to rc522 */
    bool scanning;                         /*<! Whether the rc522 is in scanning or idle mode */
    bool tag_was_present_last_time;
    uint8_t* tag_uid; 
    bool bus_initialized_by_user;          /*<! Whether the bus has been initialized manually by the user, before calling rc522_create function */
};

ESP_EVENT_DEFINE_BASE(RC522_EVENTS);

static esp_err_t rc522_spi_send(rc522_handle_t rc522, uint8_t* buffer, uint8_t length);
static esp_err_t rc522_spi_receive(rc522_handle_t rc522, uint8_t* buffer, uint8_t length, uint8_t addr);
static esp_err_t rc522_i2c_send(rc522_handle_t rc522, uint8_t* buffer, uint8_t length);
static esp_err_t rc522_i2c_receive(rc522_handle_t rc522, uint8_t* buffer, uint8_t length, uint8_t addr);

static void rc522_task(void* arg);

static esp_err_t rc522_write_n(rc522_handle_t rc522, uint8_t addr, uint8_t n, uint8_t *data)
{
    esp_err_t err = ESP_OK;
    uint8_t* buffer = NULL;
    
    // TODO: Find a way to send address + data without memory allocation
    ALLOC_JMP_GUARD(buffer = (uint8_t*) malloc(n + 1));

    buffer[0] = addr;
    memcpy(buffer + 1, data, n);

    switch(rc522->config->transport) {
        case RC522_TRANSPORT_SPI:
            ESP_ERR_JMP_GUARD(rc522_spi_send(rc522, buffer, n + 1));
            break;
        case RC522_TRANSPORT_I2C:
            ESP_ERR_JMP_GUARD(rc522_i2c_send(rc522, buffer, n + 1));
            break;
        default:
            ESP_ERR_LOG_AND_JMP_GUARD(ESP_ERR_INVALID_STATE, "write: Unknown transport");
            break;
    }

    JMP_GUARD_GATES({
        ESP_LOGE(TAG, "Failed to write data (err: %s)", esp_err_to_name(err));
    }, {});

    FREE(buffer);

    return err;
}

static inline esp_err_t rc522_write(rc522_handle_t rc522, uint8_t addr, uint8_t val)
{
    return rc522_write_n(rc522, addr, 1, &val);
}

static esp_err_t rc522_read_n(rc522_handle_t rc522, uint8_t addr, uint8_t n, uint8_t* buffer)
{
    esp_err_t err;

    switch(rc522->config->transport) {
        case RC522_TRANSPORT_SPI:
            ESP_ERR_JMP_GUARD(rc522_spi_receive(rc522, buffer, n, addr));
            break;
        case RC522_TRANSPORT_I2C:
            ESP_ERR_JMP_GUARD(rc522_i2c_receive(rc522, buffer, n, addr));
            break;
        default:
            ESP_ERR_LOG_AND_JMP_GUARD(ESP_ERR_INVALID_STATE, "read: Unknown transport");
            break;
    }

    JMP_GUARD_GATES({
        ESP_LOGE(TAG, "Failed to read data (err: %s)", esp_err_to_name(err));
    }, {});

    return err;
}

static inline esp_err_t rc522_read(rc522_handle_t rc522, uint8_t addr, uint8_t* value_ref)
{
    return rc522_read_n(rc522, addr, 1, value_ref);
}

static esp_err_t rc522_set_bitmask(rc522_handle_t rc522, uint8_t addr, uint8_t mask)
{
    esp_err_t err = ESP_OK;
    uint8_t tmp;

    ESP_ERR_RET_GUARD(rc522_read(rc522, addr, &tmp));

    return rc522_write(rc522, addr, tmp | mask);
}

static esp_err_t rc522_clear_bitmask(rc522_handle_t rc522, uint8_t addr, uint8_t mask)
{
    esp_err_t err = ESP_OK;
    uint8_t tmp;

    ESP_ERR_RET_GUARD(rc522_read(rc522, addr, &tmp));

    return rc522_write(rc522, addr, tmp & ~mask);
}

static inline esp_err_t rc522_firmware(rc522_handle_t rc522, uint8_t* result)
{
    return rc522_read(rc522, RC522_VERSION_REG, result);
}

static esp_err_t rc522_antenna_on(rc522_handle_t rc522)
{
    esp_err_t err = ESP_OK;
    uint8_t tmp;

    ESP_ERR_RET_GUARD(rc522_read(rc522, RC522_TX_CONTROL_REG, &tmp));

    if(~ (tmp & 0x03)) {
        ESP_ERR_RET_GUARD(rc522_set_bitmask(rc522, RC522_TX_CONTROL_REG, 0x03));
    }

    return rc522_write(rc522, RC522_RF_CFG_REG, 0x60); // 43dB gain
}

static esp_err_t rc522_clone_config(rc522_config_t* config, rc522_config_t** result)
{
    rc522_config_t* _clone_config = NULL;
    
    ALLOC_RET_GUARD(_clone_config = calloc(1, sizeof(rc522_config_t)));
    
    memcpy(_clone_config, config, sizeof(rc522_config_t));

    // defaults
    _clone_config->scan_interval_ms = config->scan_interval_ms < 50 ? RC522_DEFAULT_SCAN_INTERVAL_MS : config->scan_interval_ms;
    _clone_config->task_stack_size = config->task_stack_size == 0 ? RC522_DEFAULT_TASK_STACK_SIZE : config->task_stack_size;
    _clone_config->task_priority = config->task_priority == 0 ? RC522_DEFAULT_TASK_STACK_PRIORITY : config->task_priority;
    _clone_config->spi.clock_speed_hz = config->spi.clock_speed_hz == 0 ? RC522_DEFAULT_SPI_CLOCK_SPEED_HZ : config->spi.clock_speed_hz;
    _clone_config->i2c.rw_timeout_ms = config->i2c.rw_timeout_ms == 0 ? RC522_DEFAULT_I2C_RW_TIMEOUT_MS : config->i2c.rw_timeout_ms;
    _clone_config->i2c.clock_speed_hz = config->i2c.clock_speed_hz == 0 ? RC522_DEFAULT_I2C_CLOCK_SPEED_HZ : config->i2c.clock_speed_hz;

    *result = _clone_config;

    return ESP_OK;
}

static esp_err_t rc522_create_transport(rc522_handle_t rc522)
{
    esp_err_t err = ESP_OK;

    switch(rc522->config->transport) {
        case RC522_TRANSPORT_SPI: {
                spi_device_interface_config_t devcfg = {
                    .clock_speed_hz = rc522->config->spi.clock_speed_hz,
                    .mode = 0,
                    .spics_io_num = rc522->config->spi.sda_gpio,
                    .queue_size = 7,
                    .flags = rc522->config->spi.device_flags,
                };

                rc522->bus_initialized_by_user = rc522->config->spi.bus_is_initialized;

                if(! rc522->bus_initialized_by_user) {
                    spi_bus_config_t buscfg = {
                        .miso_io_num = rc522->config->spi.miso_gpio,
                        .mosi_io_num = rc522->config->spi.mosi_gpio,
                        .sclk_io_num = rc522->config->spi.sck_gpio,
                        .quadwp_io_num = -1,
                        .quadhd_io_num = -1,
                    };

                    ESP_ERR_RET_GUARD(spi_bus_initialize(rc522->config->spi.host, &buscfg, 0));
                }

                ESP_ERR_RET_GUARD(spi_bus_add_device(rc522->config->spi.host, &devcfg, &rc522->spi_handle));
            }
            break;
        case RC522_TRANSPORT_I2C: {
                i2c_config_t conf = {
                    .mode = I2C_MODE_MASTER,
                    .sda_io_num = rc522->config->i2c.sda_gpio,
                    .scl_io_num = rc522->config->i2c.scl_gpio,
                    .sda_pullup_en = GPIO_PULLUP_ENABLE,
                    .scl_pullup_en = GPIO_PULLUP_ENABLE,
                    .master.clk_speed = rc522->config->i2c.clock_speed_hz,
                };

                ESP_ERR_RET_GUARD(i2c_param_config(rc522->config->i2c.port, &conf));
                ESP_ERR_RET_GUARD(i2c_driver_install(rc522->config->i2c.port, conf.mode, false, false, 0x00));
            }
            break;
        default:
            ESP_LOGE(TAG, "create_transport: Unknown transport");
            err = ESP_ERR_INVALID_STATE; // unknown transport
            break;
    }

    return err;
}

esp_err_t rc522_create(rc522_config_t* config, rc522_handle_t* out_rc522)
{
    if(! config || ! out_rc522) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = ESP_OK;
    rc522_handle_t rc522 = NULL;
    
    ALLOC_RET_GUARD(rc522 = calloc(1, sizeof(struct rc522)));

    ESP_ERR_LOG_AND_JMP_GUARD(rc522_clone_config(
        config, 
        &(rc522->config)
    ), "Fail to clone config");

    ESP_ERR_LOG_AND_JMP_GUARD(rc522_create_transport(
        rc522
    ), "Fail to create transport");

    esp_event_loop_args_t event_args = {
        .queue_size = 1,
        .task_name = NULL, // no task will be created
    };

    ESP_ERR_LOG_AND_JMP_GUARD(esp_event_loop_create(
        &event_args,
        &rc522->event_handle
    ), "Fail to create event loop");

    rc522->running = true;

    CONDITION_LOG_AND_JMP_GUARD(pdTRUE != xTaskCreate(
        rc522_task,
        "rc522_task",
        rc522->config->task_stack_size,
        rc522,
        rc522->config->task_priority,
        &rc522->task_handle
    ), "Fail to create task");

    JMP_GUARD_GATES({
        rc522_destroy(rc522);
        rc522 = NULL;
    }, {
        *out_rc522 = rc522;
    });

    return err;
}

esp_err_t rc522_register_events(rc522_handle_t rc522, rc522_event_t event, esp_event_handler_t event_handler, void* event_handler_arg)
{
    if(! rc522) {
        return ESP_ERR_INVALID_ARG;
    }

    return esp_event_handler_register_with(rc522->event_handle, RC522_EVENTS, event, event_handler, event_handler_arg);
}

esp_err_t rc522_unregister_events(rc522_handle_t rc522, rc522_event_t event, esp_event_handler_t event_handler)
{
    if(! rc522) {
        return ESP_ERR_INVALID_ARG;
    }

    return esp_event_handler_unregister_with(rc522->event_handle, RC522_EVENTS, event, event_handler);
}

static uint64_t rc522_sn_to_u64(uint8_t* sn)
{
    uint64_t result = 0;

    if(! sn) {
        return 0;
    }

    for(int i = 4; i >= 0; i--) {
        result |= ((uint64_t) sn[i] << (i * 8));
    }

    return result;
}

// Buffer should be length of 2, or more
// Only first 2 elements will be used where the result will be stored
// TODO: Use 2+ bytes data type instead of buffer array
static esp_err_t rc522_calculate_crc(rc522_handle_t rc522, uint8_t *data, uint8_t n, uint8_t* buffer)
{
    esp_err_t err = ESP_OK;
    uint8_t i = 255;
    uint8_t nn = 0;

    ESP_ERR_RET_GUARD(rc522_clear_bitmask(rc522, RC522_DIV_INT_REQ_REG, 0x04));
    ESP_ERR_RET_GUARD(rc522_set_bitmask(rc522, RC522_FIFO_LEVEL_REG, 0x80));
    ESP_ERR_RET_GUARD(rc522_write_n(rc522, RC522_FIFO_DATA_REG, n, data));
    ESP_ERR_RET_GUARD(rc522_write(rc522, RC522_COMMAND_REG, RC522_CMD_CALC_CRC));

    for(;;) {
        ESP_ERR_RET_GUARD(rc522_read(rc522, RC522_DIV_INT_REQ_REG, &nn));

        i--;

        if(! (i != 0 && ! (nn & 0x04))) {
            break;
        }
    }

    uint8_t tmp;

    ESP_ERR_RET_GUARD(rc522_read(rc522, RC522_CRC_RESULT_LSB_REG, &tmp));
    buffer[0] = tmp;

    ESP_ERR_RET_GUARD(rc522_read(rc522, RC522_CRC_RESULT_MSB_REG, &tmp));
    buffer[1] = tmp;

    return ESP_OK;
}

static esp_err_t rc522_card_write(rc522_handle_t rc522, uint8_t cmd, uint8_t *data, uint8_t n, uint8_t* res_n, uint8_t** result)
{
    esp_err_t err = ESP_OK;
    uint8_t* _result = NULL;
    uint8_t _res_n = 0;
    uint8_t irq = 0x00;
    uint8_t irq_wait = 0x00;
    uint8_t last_bits = 0;
    uint8_t nn = 0;
    uint8_t tmp;
    
    if(cmd == RC522_CMD_AUTHENT) {
        irq = 0x12;
        irq_wait = 0x10;
    }
    else if(cmd == RC522_CMD_TRANSCEIVE) {
        irq = 0x77;
        irq_wait = 0x30;
    }

    ESP_ERR_JMP_GUARD(rc522_write(rc522, RC522_COMM_INT_EN_REG, irq | 0x80));
    ESP_ERR_JMP_GUARD(rc522_clear_bitmask(rc522, RC522_COMM_INT_REQ_REG, 0x80));
    ESP_ERR_JMP_GUARD(rc522_set_bitmask(rc522, RC522_FIFO_LEVEL_REG, 0x80));
    ESP_ERR_JMP_GUARD(rc522_write(rc522, RC522_COMMAND_REG, RC522_CMD_IDLE));
    ESP_ERR_JMP_GUARD(rc522_write_n(rc522, RC522_FIFO_DATA_REG, n, data));
    ESP_ERR_JMP_GUARD(rc522_write(rc522, RC522_COMMAND_REG, cmd));

    if(cmd == RC522_CMD_TRANSCEIVE) {
        ESP_ERR_JMP_GUARD(rc522_set_bitmask(rc522, RC522_BIT_FRAMING_REG, 0x80));
    }

    uint16_t i = 1000;

    for(;;) {
        ESP_ERR_JMP_GUARD(rc522_read(rc522, RC522_COMM_INT_REQ_REG, &nn));

        i--;

        if(! (i != 0 && (((nn & 0x01) == 0) && ((nn & irq_wait) == 0)))) {
            break;
        }
    }

    ESP_ERR_JMP_GUARD(rc522_clear_bitmask(rc522, RC522_BIT_FRAMING_REG, 0x80));

    if(i != 0) {
        ESP_ERR_JMP_GUARD(rc522_read(rc522, RC522_ERROR_REG, &tmp));

        if((tmp & 0x1B) == 0x00) {
            if(cmd == RC522_CMD_TRANSCEIVE) {
                ESP_ERR_JMP_GUARD(rc522_read(rc522, RC522_FIFO_LEVEL_REG, &nn));
                ESP_ERR_JMP_GUARD(rc522_read(rc522, RC522_CONTROL_REG, &tmp));

                last_bits = tmp & 0x07;

                if (last_bits != 0) {
                    _res_n = (nn - 1) + last_bits;
                } else {
                    _res_n = nn;
                }

                if(_res_n > 0) {
                    ALLOC_JMP_GUARD(_result = (uint8_t*) malloc(_res_n));

                    for(i = 0; i < _res_n; i++) {
                        ESP_ERR_JMP_GUARD(rc522_read(rc522, RC522_FIFO_DATA_REG, &tmp));
                        _result[i] = tmp;
                    }
                }
            }
        }
    }

    JMP_GUARD_GATES({
        FREE(_result);
        _res_n = 0;
    }, {
        *res_n = _res_n;
        *result = _result;
    });

    return err;
}

static esp_err_t rc522_request(rc522_handle_t rc522, uint8_t* res_n, uint8_t** result)
{
    esp_err_t err = ESP_OK;
    uint8_t* _result = NULL;
    uint8_t _res_n = 0;
    uint8_t req_mode = MIFARE_REQA;

    ESP_ERR_RET_GUARD(rc522_write(rc522, RC522_BIT_FRAMING_REG, 0x07));
    ESP_ERR_RET_GUARD(rc522_card_write(rc522, RC522_CMD_TRANSCEIVE, &req_mode, 1, &_res_n, &_result));

    if(_res_n * 8 != 0x10) {
        free(_result);
        
        return ESP_ERR_INVALID_STATE;
    }

    *res_n = _res_n;
    *result = _result;

    return err;
}

static esp_err_t rc522_anticoll(rc522_handle_t rc522, uint8_t** result)
{
    esp_err_t err = ESP_OK;
    uint8_t* _result = NULL;
    uint8_t _res_n;

    ESP_ERR_JMP_GUARD(rc522_write(rc522, RC522_BIT_FRAMING_REG, 0x00));
    ESP_ERR_JMP_GUARD(rc522_card_write(rc522, RC522_CMD_TRANSCEIVE, (uint8_t[]) MIFARE_ANTICOLLISION_CL_1, 2, &_res_n, &_result));

    // TODO: Some cards have length of 4, and some of them have length of 7 bytes
    //       here we are using one extra byte which is not part of UID.
    //       Implement logic to determine the length of the UID and use that info
    //       to retrieve the serial number aka UID
    if(_result && _res_n != 5) { // all cards/tags serial numbers is 5 bytes long (??)
        ESP_ERR_LOG_AND_JMP_GUARD(ESP_ERR_INVALID_RESPONSE, "invalid length of serial number");
    }

    JMP_GUARD_GATES({
        FREE(_result);
        _res_n = 0;
    }, {
        *result = _result;
    });

    return err;
}

static esp_err_t rc522_stop_picc_communication(rc522_handle_t rc522)
{

    esp_err_t err = ESP_OK;
    /*
        This part stops the communication with the picc by issuing a HALT_A as well as STOP_CRYPTO1
    */
    uint8_t* res_data = NULL;
    uint8_t res_data_n;

    // Issue a HALT_A
    uint8_t buf[4];
    memcpy(buf, (uint8_t[])MIFARE_HALT, 2);
    memcpy(buf + 2, (uint8_t[]){ 0x00, 0x00 }, 2);
    ESP_ERR_JMP_GUARD(rc522_calculate_crc(rc522, buf, 2, buf + 2));
    ESP_ERR_JMP_GUARD(rc522_card_write(rc522, RC522_CMD_TRANSCEIVE, buf, 4, &res_data_n, &res_data));
    FREE(res_data);
    // Stop CYPTO1
    // Clear MFCrypto1On bit
    ESP_ERR_JMP_GUARD(rc522_clear_bitmask(rc522, RC522_STATUS_2_REG, 0x08));

    JMP_GUARD_GATES({
        FREE(res_data);
    }, {
        // Nothing to do if everything went fine
    });

    return err;
}

static esp_err_t rc522_get_tag(rc522_handle_t rc522, uint8_t** result)
{
    esp_err_t err = ESP_OK;
    uint8_t* _result = NULL;
    uint8_t* res_data = NULL;
    uint8_t res_data_n;

    ESP_ERR_JMP_GUARD(rc522_request(rc522, &res_data_n, &res_data));

    if(res_data != NULL) {
        FREE(res_data);
        ESP_ERR_JMP_GUARD(rc522_anticoll(rc522, &_result));

        if(_result != NULL) {
            ESP_ERR_JMP_GUARD(rc522_stop_picc_communication(rc522));
        }
    }

    JMP_GUARD_GATES({
        FREE(_result);
        FREE(res_data);
    }, {
        *result = _result;
    });

    return err;
}


static esp_err_t rc522_authenticate(rc522_handle_t rc522, uint8_t auth_key, uint8_t blockAddr, MIFARE_Key* key)
{
    esp_err_t err = ESP_OK;
    uint8_t* res_data = NULL;
    uint8_t res_data_n;

    uint8_t buf[12];
    buf[0] = auth_key;
    buf[1] = blockAddr;
    memcpy(buf + 2, key->keyByte, 6);
    memcpy(buf + 8, rc522->tag_uid, 4);

    ESP_ERR_JMP_GUARD(rc522_write(rc522, RC522_BIT_FRAMING_REG, 0x00));
    ESP_ERR_JMP_GUARD(rc522_card_write(rc522, RC522_CMD_AUTHENT, buf, 12, &res_data_n, &res_data));

    JMP_GUARD_GATES({
        FREE(res_data);
    }, {
        // Nothing to do if the authentication worked
    });

    return err;
}


static esp_err_t  rc522_read_block_from_picc(rc522_handle_t rc522, uint8_t blockAddr,
											uint8_t *buffer,		///< The buffer to store the data in
											uint8_t *bufferSize	///< Buffer size, at least 18 bytes. Also number of bytes returned if STATUS_OK.
										) {
    esp_err_t err = ESP_OK;

    // Sanity checks
    CONDITION_LOG_AND_JMP_GUARD((buffer == NULL), "The buffer pointer is null, PICC's block cannot be read");
    CONDITION_LOG_AND_JMP_GUARD((*bufferSize < 18), "The buffer reading a block from the PICC is small, increase the size to atleast 18 bytes");

	// Build command buffer
	buffer[0] = MIFARE_READ;
	buffer[1] = blockAddr;
	// Calculate CRC_A
    ESP_ERR_JMP_GUARD(rc522_calculate_crc(rc522, buffer, 2, buffer + 2));

    // Transmit the buffer and receive the response, validate CRC_A.
    ESP_ERR_JMP_GUARD(rc522_card_write(rc522, RC522_CMD_TRANSCEIVE, buffer, 4, bufferSize, &buffer));

	JMP_GUARD_GATES({
        FREE(buffer);
        *bufferSize = 0;
    }, {
        // Nothing to do if everything went fine
    });

    return err;
}

static esp_err_t rc522_read_sector_from_picc(rc522_handle_t rc522, MIFARE_Key* key, uint8_t sector)
{
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
            printf("       ");
        }
        
        // Block number
		if(blockAddr < 10)
			printf("   "); // Pad with spaces
		else {
			printf("  "); // Pad with spaces
		}
		printf("%d", blockAddr);
		printf("  ");
        // Establish encrypted communications before reading the first block
		if (isSectorTrailer) {
			ESP_ERR_JMP_GUARD(rc522_authenticate(rc522, MIFARE_AUTH_KEY_A, firstBlock, key));
		}
        // Read block
		byteCount = sizeof(buffer);
		ESP_ERR_JMP_GUARD(rc522_read_block_from_picc(rc522, blockAddr, buffer, &byteCount));
		// Dump data
		for (uint8_t index = 0; index < 16; index++) {
			if(buffer[index] < 0x10)
				printf(" 0");
			else
				printf(" ");
			printf("0x%02x", buffer[index]);
			if ((index % 4) == 3) {
				printf(" ");
			}
		}
        // Parse sector trailer data
		if (isSectorTrailer) {
            printf(" << SECTOR TRAILER >> ");
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

    JMP_GUARD_GATES({
        // TODO
    }, {
        // Nothing to do if everything went fine
    });
    
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
    ESP_LOGI(TAG, "Sector Block   0  1  2  3   4  5  6  7   8  9 10 11  12 13 14 15  AccessBits");
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

esp_err_t rc522_start(rc522_handle_t rc522)
{
    esp_err_t err = ESP_OK;

    if(! rc522) {
        return ESP_ERR_INVALID_ARG;
    }

    if(rc522->scanning) { // Already in scan mode
        return ESP_OK;
    }

    uint8_t tmp = 0;

    if(! rc522->initialized) {
        // Initialization will be done only once, on the first call of start function

        // TODO: Extract test in dedicated function
        // ---------- RW test ------------
        // TODO: Use less sensitive register for the test, or return the value
        //       of this register to the previous state at the end of the test
        const uint8_t test_addr = RC522_MOD_WIDTH_REG, test_val = 0x25;
        uint8_t pass = 0;

        for(uint8_t i = test_val; i < test_val + 2; i++) {
            err = rc522_write(rc522, test_addr, i);

            if(err == ESP_OK) {
                err = rc522_read(rc522, test_addr, &tmp);

                if(err == ESP_OK && tmp == i) {
                    pass = 1;
                }
            }

            if(pass != 1) {
                ESP_LOGE(TAG, "Read/write test failed");
                rc522_destroy(rc522);

                return err;
            }
        }
        // ------- End of RW test --------

        ESP_ERR_RET_GUARD(rc522_write(rc522, RC522_COMMAND_REG, RC522_CMD_SOFTRESET));
        ESP_ERR_RET_GUARD(rc522_write(rc522, RC522_TIMER_MODE_REG, 0x8D));
        ESP_ERR_RET_GUARD(rc522_write(rc522, RC522_TIMER_PRESCALER_REG, 0x3E));
        ESP_ERR_RET_GUARD(rc522_write(rc522, RC522_TIMER_RELOAD_LSB_REG, 0x1E));
        ESP_ERR_RET_GUARD(rc522_write(rc522, RC522_TIMER_RELOAD_MSB_REG, 0x00));
        ESP_ERR_RET_GUARD(rc522_write(rc522, RC522_TX_ASK_REG, 0x40));
        ESP_ERR_RET_GUARD(rc522_write(rc522, RC522_MODE_REG, 0x3D));

        ESP_ERR_RET_GUARD(rc522_antenna_on(rc522));
        ESP_ERR_RET_GUARD(rc522_firmware(rc522, &tmp));

        rc522->initialized = true;

        ESP_LOGI(TAG, "Initialized (firmware v%d.0)", (tmp & 0x03));
    }

    rc522->scanning = true;

    return ESP_OK;
}

esp_err_t rc522_pause(rc522_handle_t rc522)
{
    if(! rc522) {
        return ESP_ERR_INVALID_ARG;
    }

    if(! rc522->scanning) {
        return ESP_OK;
    }

    rc522->scanning = false;

    return ESP_OK;
}

static esp_err_t rc522_destroy_transport(rc522_handle_t rc522)
{
    esp_err_t err;

    switch(rc522->config->transport) {
        case RC522_TRANSPORT_SPI:
            err = spi_bus_remove_device(rc522->spi_handle);
            if(rc522->bus_initialized_by_user) {
                err = spi_bus_free(rc522->config->spi.host);
            }
            break;
        case RC522_TRANSPORT_I2C:
            err = i2c_driver_delete(rc522->config->i2c.port);
            break;
        default:
            ESP_LOGW(TAG, "destroy_transport: Unknown transport");
            err = ESP_ERR_INVALID_STATE;
    }

    return err;
}

esp_err_t rc522_destroy(rc522_handle_t rc522)
{
    esp_err_t err = ESP_OK;

    if(! rc522) {
        return ESP_ERR_INVALID_ARG;
    }

    if(xTaskGetCurrentTaskHandle() == rc522->task_handle) {
        ESP_LOGE(TAG, "Cannot destroy rc522 from event handler");

        return ESP_ERR_INVALID_STATE;
    }

    err = rc522_pause(rc522); // stop task
    rc522->running = false; // stop rc522 -> task will delete itself

    // TODO: Wait here for task to exit

    err = rc522_destroy_transport(rc522);

    if(rc522->event_handle) {
        err = esp_event_loop_delete(rc522->event_handle); 
        rc522->event_handle = NULL;
    }

    FREE(rc522->config);
    FREE(rc522);

    return err;
}

static esp_err_t rc522_dispatch_event(rc522_handle_t rc522, rc522_event_t event, void* data)
{
    esp_err_t err;

    if(! rc522) {
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

static esp_err_t rc522_spi_send(rc522_handle_t rc522, uint8_t* buffer, uint8_t length)
{
    buffer[0] = (buffer[0] << 1) & 0x7E;

    return spi_device_transmit(rc522->spi_handle, &(spi_transaction_t){
        .length = 8 * length,
        .tx_buffer = buffer,
    });
}

static esp_err_t rc522_spi_receive(rc522_handle_t rc522, uint8_t* buffer, uint8_t length, uint8_t addr)
{
    esp_err_t err = ESP_OK;

    addr = ((addr << 1) & 0x7E) | 0x80;

    if(SPI_DEVICE_HALFDUPLEX & rc522->config->spi.device_flags) {
        ESP_ERR_RET_GUARD(spi_device_transmit(rc522->spi_handle, &(spi_transaction_t) {
            .flags = SPI_TRANS_USE_TXDATA,
            .length = 8,
            .tx_data[0] = addr,
            .rxlength = 8 * length,
            .rx_buffer = buffer,
        }));
    } else { // Fullduplex
        ESP_ERR_RET_GUARD(spi_device_transmit(rc522->spi_handle, &(spi_transaction_t) {
            .flags = SPI_TRANS_USE_TXDATA,
            .length = 8,
            .tx_data[0] = addr,
        }));

        ESP_ERR_RET_GUARD(spi_device_transmit(rc522->spi_handle, &(spi_transaction_t) {
            .flags = 0x00,
            .length = 8,
            .rxlength = 8 * length,
            .rx_buffer = buffer,
        }));
    }

    return err;
}

static inline esp_err_t rc522_i2c_send(rc522_handle_t rc522, uint8_t* buffer, uint8_t length)
{
    return i2c_master_write_to_device(
        rc522->config->i2c.port,
        RC522_I2C_ADDRESS,
        buffer,
        length,
        rc522->config->i2c.rw_timeout_ms / portTICK_PERIOD_MS
    );
}

static inline esp_err_t rc522_i2c_receive(rc522_handle_t rc522, uint8_t* buffer, uint8_t length, uint8_t addr)
{
    return i2c_master_write_read_device(
        rc522->config->i2c.port,
        RC522_I2C_ADDRESS,
        &addr,
        1,
        buffer,
        length,
        rc522->config->i2c.rw_timeout_ms / portTICK_PERIOD_MS
    );
}

static void rc522_task(void* arg)
{
    rc522_handle_t rc522 = (rc522_handle_t) arg;

    while(rc522->running) {
        if(! rc522->scanning) {
            // Idling...
            vTaskDelay(100 / portTICK_PERIOD_MS);
            continue;
        }

        uint8_t* serial_no_array = NULL;

        if(ESP_OK != rc522_get_tag(rc522, &serial_no_array)) {
            // Tag is not present
            //
            // TODO: Implement logic to know when the error is due to
            //       tag absence or some other protocol issue
        }
        
        if(! serial_no_array) {
            rc522->tag_was_present_last_time = false;
        } else if(! rc522->tag_was_present_last_time) {
            rc522->tag_uid = (uint8_t*) malloc(32);
            memcpy(rc522->tag_uid, serial_no_array, 32);
            rc522_tag_t tag = {
                .serial_number = rc522_sn_to_u64(serial_no_array),
            };
            FREE(serial_no_array);
            rc522_dispatch_event(rc522, RC522_EVENT_TAG_SCANNED, &tag);
            rc522->tag_was_present_last_time = true;

            // TEST AUTHENTICATION
            MIFARE_Key key;
            // All keys are set to FFFFFFFFFFFFh at chip delivery from the factory.
			for (uint8_t i = 0; i < 6; i++) {
				key.keyByte[i] = 0xff;
			}
            rc522_read_data_from_picc(rc522, &key);
        } else {
            FREE(serial_no_array);
        }

        int delay_interval_ms = rc522->config->scan_interval_ms;

        if(rc522->tag_was_present_last_time) {
            delay_interval_ms *= 2; // extra scan-bursting prevention
        }

        vTaskDelay(delay_interval_ms / portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL);
}
