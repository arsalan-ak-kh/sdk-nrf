/*
 * Copyright (c) 2025 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/mspi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/ztest.h>

LOG_MODULE_REGISTER(mspi_slave, LOG_LEVEL_DBG);

#define MSPI_PERIPHERAL_NODE 	DT_NODELABEL(peripheral)
#define MSPI_CONTROLLER_NODE 	DT_NODELABEL(controller)

#define DATA_LINES_MAX 4
#define SCK_FREQUENCY MHZ(1)
#define CMD_LEN_MAX 2
#define ADDR_LEN_MAX 4
#define DATA_LEN_MAX 52

static uint8_t packet_buf[DATA_LEN_MAX];
static uint8_t rx_buff[DATA_LEN_MAX];
static const struct device *mspi_peripheral_dev = DEVICE_DT_GET(MSPI_PERIPHERAL_NODE);
static const struct device *mspi_controller_dev = DEVICE_DT_GET(MSPI_CONTROLLER_NODE);

static const struct mspi_dev_id mspi_id_tx = { .dev_idx = 0 };
static const struct mspi_dev_id mspi_id_rx = { .dev_idx = 0 };

struct user_context {
	uint32_t status;
	uint32_t total_packets;
};
//--------------------------------------------------------------------------------
static void *setup_buffer(void)
{
	for (int i = 0; i < DATA_LEN_MAX; ++i) {
		packet_buf[i] = (uint8_t)i;
	}
	return NULL;
}
//----------------------------------------------------------------------------
static void before(void *fixture)
{
	ARG_UNUSED(fixture);

	zassert_true(device_is_ready(mspi_peripheral_dev),
		"MSPI device %s is not ready", mspi_peripheral_dev->name);
	zassert_true(device_is_ready(mspi_controller_dev),
		"MSPI device %s is not ready", mspi_controller_dev->name);
}
//--------------------------------------------------------------------------
static void print_rx_buff(uint8_t *input_buff)
{
	/*
	for (size_t i = 0; i < DATA_LEN_MAX; i++) {
		printk("returned buffer [%u] = %u\n", i, input_buff[i]);
	}
		*/
}
//---------------------------------------------------------------------
static void async_cb(struct mspi_callback_context *mspi_cb_ctx)
{
	printk("Callback triggered\n");
	print_rx_buff(mspi_cb_ctx->mspi_evt.evt_data.packet->data_buf);

}
//-------------------------------------------------------------------
static void test_tx_transfers(enum mspi_io_mode io_mode)
{
	struct mspi_dev_cfg rx_dev_cfg = {
		.ce_num = 1,
		.freq = SCK_FREQUENCY,
		.io_mode = io_mode,
		.data_rate = MSPI_DATA_RATE_SINGLE,
		.cpp = MSPI_CPP_MODE_0,
		.endian = MSPI_XFER_BIG_ENDIAN,
		.ce_polarity = MSPI_CE_ACTIVE_LOW,
	};

	struct mspi_xfer_packet rx_packet = {
		.dir = MSPI_RX, // direction - tells us its being received 
		.cmd = 0xA5, // transfer command (0)
		.address = 0x123456,// transfer address (0)
		.data_buf = rx_buff, // where to store the received data buffer 
		.num_bytes = DATA_LEN_MAX, // number of bytes to recieve 
	};

	struct mspi_xfer rx_xfer = { // Describe how the RX buffer should be received 
		.xfer_mode   = MSPI_PIO,  // tramsfer mode 
		.packets     = &rx_packet, // actual packet being recieved 
		.num_packet  = 1, // number of transfer packets
		.timeout     = 1, // transfer timeout 
		.async       = true, // tells us its async 
		.cmd_length  = 1, // command length 
		.addr_length = 3, // address length 
	};

	struct mspi_dev_cfg tx_dev_cfg = {
		.ce_num = 1, 
		.freq = SCK_FREQUENCY,
		.io_mode = io_mode,
		.data_rate = MSPI_DATA_RATE_SINGLE,
		.cpp = MSPI_CPP_MODE_0,
		.endian = MSPI_XFER_BIG_ENDIAN,
		.ce_polarity = MSPI_CE_ACTIVE_LOW,
	};

	struct mspi_xfer_packet tx_packet = { //the format of the TX buffer ( packet being sent over )
		.dir = MSPI_TX, // direction - tells us its being transferred 
		.cmd = 0xFF, // transfer command
		.address = 0x87654321, // transfer address
		.data_buf = packet_buf, // the data buffer we want to send over 
		.num_bytes = DATA_LEN_MAX, //tells us the number of bytes to transfer 
	};

	struct mspi_xfer tx_xfer = { // Describe how the TX buffer (being sent) should be sent
		.xfer_mode   = MSPI_PIO, // transfer mode 
		.packets     = &tx_packet, // actual packet being sent over 
		.num_packet  = 1, // number of transfer pakcet
		.timeout     = 10, //transfer timeout
		.cmd_length  = 1, // command length ( 8 bit command )
		.addr_length = 3, // address length ( 24 bit address )
	};

	volatile struct user_context read_ctx;
	struct mspi_callback_context cb_ctx;
	uint8_t cmd_lines, addr_lines, data_lines;
	int rc;

	switch (io_mode) {
	default:
	case MSPI_IO_MODE_SINGLE:
		cmd_lines = 1;
		addr_lines = 1;
		data_lines = 1;
		break;
	case MSPI_IO_MODE_DUAL:
		cmd_lines = 2;
		addr_lines = 2;
		data_lines = 2;
		break;
	case MSPI_IO_MODE_DUAL_1_1_2:
		cmd_lines = 1;
		addr_lines = 1;
		data_lines = 2;
		break;
	case MSPI_IO_MODE_DUAL_1_2_2:
		cmd_lines = 1;
		addr_lines = 2;
		data_lines = 2;
		break;
	case MSPI_IO_MODE_QUAD:
		cmd_lines = 4;
		addr_lines = 4;
		data_lines = 4;
		break;
	case MSPI_IO_MODE_QUAD_1_1_4:
		cmd_lines = 1;
		addr_lines = 1;
		data_lines = 4;
		break;
	case MSPI_IO_MODE_QUAD_1_4_4:
		cmd_lines = 1;
		addr_lines = 4;
		data_lines = 4;
		break;
	}

	uint8_t cmd_addr_cycles = (tx_xfer.cmd_length * 8 / cmd_lines)
			+ (tx_xfer.addr_length * 8 / addr_lines);
	tx_xfer.tx_dummy = 8 - (cmd_addr_cycles % 8);
	read_ctx.total_packets = rx_xfer.num_packet;
	read_ctx.status = ~0;
	cb_ctx.ctx = (void *)&read_ctx;
	// tx_xfer.tx_dummy = 0;
	rc = mspi_dev_config(mspi_peripheral_dev, &mspi_id_rx,
			     MSPI_DEVICE_CONFIG_ALL, &rx_dev_cfg);
	zassert_equal(rc, 0, "Failed to config peripheral");

	rc = mspi_dev_config(mspi_controller_dev, &mspi_id_tx,
			     MSPI_DEVICE_CONFIG_ALL, &tx_dev_cfg);
	zassert_equal(rc, 0, "Failed to config controller");

	rc = mspi_register_callback(mspi_peripheral_dev, &mspi_id_rx,
				    MSPI_BUS_XFER_COMPLETE, (mspi_callback_handler_t)async_cb, &cb_ctx);
	zassert_equal(rc, 0, "Failed to register callback");

	printk("Setting Async RX...\n");
	rc = mspi_transceive(mspi_peripheral_dev, &mspi_id_rx, &rx_xfer);
	zassert_equal(rc, 0, "Peripheral transceive failed");

	k_msleep(5);
	printf ("POOOOOOOOOOOOOOOO");
	printk("Setting Controller TX...\n");
	rc = mspi_transceive(mspi_controller_dev, &mspi_id_tx, &tx_xfer);
	zassert_equal(rc, 0, "Controller transceive failed");
	
    //----------------------------------------
	//----------------------------------------

	printk("Comparing TX and RX buffers...\n");

	// Print TX buffer
	printk("TX data Buffer: ");
	for (int i = 0; i < DATA_LEN_MAX; ++i) {
		printk("%02X ", tx_packet.data_buf[i]); // this should only print out the data being sent over 
	}
	printk("\n");
	

	// Print RX buffer 
	printk("RX Buffer: ");
	for (int i = 0; i < sizeof(rx_buff); ++i) {
		printk("%02X ", rx_buff[i]); // this should print out total stuff recieved ( includes data + cmd + address )
	}
	printk("\n");
	////----------------------------- Comparing & validations-------------------------------- 
	uint32_t stream_pos = 0;  // tally position 

	// Command bytes
	for (int i = 0; i < tx_xfer.cmd_length; i++) {
		uint8_t expected = (tx_xfer.packets[0].cmd >> ((tx_xfer.cmd_length - 1 - i) * 8)) & 0xFF;
		uint8_t received = rx_buff[stream_pos++];
		printk("CMD Byte %d: Expected 0x%02X, Received 0x%02X\n", i, expected, received);
		//zassert_equal(received, expected, "Command mismatch at index %d", i);
	}

	// Address bytes
	for (int i = 0; i < tx_xfer.addr_length; i++) {
		uint8_t expected = (tx_xfer.packets[0].address >> ((tx_xfer.addr_length - 1 - i) * 8)) & 0xFF;
		uint8_t received = rx_buff[stream_pos++];
		printk("ADDR Byte %d: Expected 0x%02X, Received 0x%02X\n", i, expected, received);
		//zassert_equal(received, expected, "Address mismatch at index %d", i);
	}

	// This added tx_dummy cycles to the tally as it goes through the recieved packet 
	stream_pos += tx_xfer.tx_dummy;

	// DATA bytes
	for (int i = 0; i < DATA_LEN_MAX; i++) {
		uint8_t expected = tx_packet.data_buf[i];
		uint8_t received = rx_buff[stream_pos++];
		printk("DATA Byte %d: Expected 0x%02X, Received 0x%02X\n", i, expected, received);
		//zassert_equal(received, expected, "Data mismatch at index %d", i);
	}
	
}

/*
ZTEST(mspi_with_spis, test_tx_single)
{
	printk("IO MODE SINGLE...\n");
	test_tx_transfers(MSPI_IO_MODE_SINGLE);
}
*/

//-------------------------------------------------------------------------
/*
ZTEST(mspi_with_spis, test_tx_dual_1_1_2)
{
	printk("IO MODE 1_1_2...\n");
	test_tx_transfers(MSPI_IO_MODE_DUAL_1_1_2);
}

ZTEST(mspi_with_spis, test_tx_dual_1_2_2)
{
	printk("IO MODE 1_2_2...\n");
	test_tx_transfers(MSPI_IO_MODE_DUAL_1_2_2);
}
*/
ZTEST(mspi_with_spis, test_tx_quad_1_1_4)
{
	printk("IO MODE 1_1_4...\n");
	test_tx_transfers(MSPI_IO_MODE_QUAD_1_1_4);
}

// ZTEST(mspi_with_spis, test_tx_quad_1_4_4)
// {
// 	printk("IO MODE 1_4_4...\n");
// 	test_tx_transfers(MSPI_IO_MODE_QUAD_1_4_4);
// }

ZTEST_SUITE(mspi_with_spis, NULL, setup_buffer, before, NULL, NULL);

//rx_packet->data_buf