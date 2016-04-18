/***************************************************************************
 *   Copyright (C) 2007 by Juergen Stuber <juergen@jstuber.net>            *
 *   based on Dominic Rath's and Benedikt Sauter's usbprog.c               *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   Copyright (C) 2011 by Jean-Christophe PLAGNIOL-VIILARD                *
 *   plagnioj@jcrosoft.com                                                 *
 *                                                                         *
 *   Copyright (C) 2015 by Marc Schink                                     *
 *   openocd-dev@marcschink.de                                             *
 *                                                                         *
 *   Copyright (C) 2015 by Paul Fertser                                    *
 *   fercerpav@gmail.com                                                   *
 *                                                                         *
 *   Copyright (C) 2015-2016 by Forest Crossman                            *
 *   cyrozap@gmail.com                                                     *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stdint.h>

#include <hidapi.h>

#include <jtag/interface.h>
#include <jtag/swd.h>
#include <jtag/commands.h>

#include "libusb_common.h"

#define VID 0x04b4
#define PID 0xf139

#define EP_IN  1
#define EP_OUT 2

#define CONTROL_TYPE_READ  0x01
#define CONTROL_TYPE_WRITE 0x02

#define CONTROL_COMMAND_PROGRAM 0x07

#define CONTROL_MODE_POLL_PROGRAMMER_STATUS  0x01
#define CONTROL_MODE_RESET_TARGET            0x04
#define CONTROL_MODE_SET_PROGRAMMER_PROTOCOL 0x40
#define CONTROL_MODE_SYNCHRONIZE_TRANSFER    0x41
#define CONTROL_MODE_ACQUIRE_SWD_TARGET      0x42
#define CONTROL_MODE_RESET_SWD_BUS           0x43

#define PROTOCOL_JTAG 0x00
#define PROTOCOL_SWD  0x01

#define DEVICE_PSOC4   0x00
#define DEVICE_UNKNOWN 0x01
#define DEVICE_PSOC5   0x03

#define ACQUIRE_MODE_RESET       0x00
#define ACQUIRE_MODE_POWER_CYCLE 0x01

#define PROGRAMMER_NOK_NACK 0x00
#define PROGRAMMER_OK_ACK   0x01

#define HID_TYPE_WRITE 0x00
#define HID_TYPE_READ  0x01
#define HID_TYPE_START 0x02

#define HID_COMMAND_POWER      0x80
#define HID_COMMAND_VERSION    0x81
#define HID_COMMAND_RESET      0x82
#define HID_COMMAND_CONFIGURE  0x8f
#define HID_COMMAND_BOOTLOADER 0xa0

/* 512 bytes seems to work reliably */
#define SWD_MAX_BUFFER_LENGTH 512

struct kitprog {
	hid_device *hid_handle;
	struct jtag_libusb_device_handle *usb_handle;
	uint16_t packet_size;
	uint16_t packet_index;
	uint8_t *packet_buffer;
	wchar_t *serial;
	uint8_t hardware_version;
	uint8_t minor_version;
	uint8_t major_version;
	uint16_t millivolts;
};

struct pending_transfer_result {
	uint8_t cmd;
	uint32_t data;
	void *buffer;
};

static char *kitprog_serial;

static int pending_transfer_count, pending_queue_len;
static struct pending_transfer_result *pending_transfers;

static int queued_retval;

static struct kitprog *kitprog_handle;

static int kitprog_usb_open(void);
static void kitprog_usb_close(void);
static int kitprog_hid_command(uint8_t *command, size_t command_length,
		uint8_t *data, size_t data_length);

static int kitprog_get_version(void);
static int kitprog_get_millivolts(void);

static int kitprog_set_protocol(uint8_t protocol);
static int kitprog_get_status(void);
static int kitprog_set_unknown(void);
static int kitprog_acquire_psoc(uint8_t psoc_type, uint8_t acquire_mode,
		uint8_t max_attempts);
static int kitprog_reset_target(void);

static int kitprog_swd_reset(void);

static int kitprog_swd_run_queue(void);
static void kitprog_swd_queue_cmd(uint8_t cmd, uint32_t *dst, uint32_t data);
static int kitprog_swd_switch_seq(enum swd_special_seq seq);


static int kitprog_init(void)
{
	kitprog_handle = malloc(sizeof(struct kitprog));

	if (kitprog_usb_open() != ERROR_OK) {
		LOG_ERROR("Can't find a KitProg device! Please check device connections and permissions.");
		return ERROR_JTAG_INIT_FAILED;
	}

	/* Get the device version information */
	if (kitprog_get_version() != ERROR_OK)
		return ERROR_FAIL;

	/* Get the current reported target voltage */
	if (kitprog_get_millivolts() != ERROR_OK)
		return ERROR_FAIL;

	/* I have no idea what this does */
	if (kitprog_set_unknown() != ERROR_OK)
		return ERROR_FAIL;

	/* Set the protocol to SWD */
	if (kitprog_set_protocol(PROTOCOL_SWD) != ERROR_OK)
		return ERROR_FAIL;

	/* Reset the SWD bus */
	if (kitprog_swd_reset() != ERROR_OK)
		return ERROR_FAIL;

	/* To enable the SWDIO and SWCLK pins as outputs, the acquire function
	 * *must* be run with a max_attempts >= 1.
	 */

	/* Here we try to acquire any device that will respond */
	do {
		if (kitprog_acquire_psoc(DEVICE_PSOC4, ACQUIRE_MODE_RESET, 3) != ERROR_OK)
			return ERROR_FAIL;

		if (kitprog_get_status() == ERROR_OK)
			break;

		if (kitprog_acquire_psoc(DEVICE_UNKNOWN, ACQUIRE_MODE_RESET, 3) != ERROR_OK)
			return ERROR_FAIL;

		if (kitprog_get_status() == ERROR_OK)
			break;

		if (kitprog_acquire_psoc(DEVICE_PSOC5, ACQUIRE_MODE_RESET, 3) != ERROR_OK)
			return ERROR_FAIL;

		if (kitprog_get_status() == ERROR_OK)
			break; else {
			LOG_ERROR("No PSoC devices found");
			return ERROR_FAIL;
		}
	} while (0);

	/* Allocate packet buffers and queues */
	kitprog_handle->packet_buffer = malloc(SWD_MAX_BUFFER_LENGTH);
	kitprog_handle->packet_size = SWD_MAX_BUFFER_LENGTH;

	pending_queue_len = SWD_MAX_BUFFER_LENGTH / 5;
	pending_transfers = malloc(pending_queue_len * sizeof(*pending_transfers));
	if (!pending_transfers) {
		LOG_ERROR("Unable to allocate memory for KitProg queue");
		return ERROR_FAIL;
	}

	/* Display KitProg info */
	LOG_INFO("KitProg v%u.%02u",
		kitprog_handle->major_version, kitprog_handle->minor_version);
	LOG_INFO("Hardware version: %u",
		kitprog_handle->hardware_version);
	LOG_INFO("VTARG = %u.%03u V",
		kitprog_handle->millivolts / 1000, kitprog_handle->millivolts % 1000);

	return ERROR_OK;
}

static int kitprog_quit(void)
{
	kitprog_usb_close();

	free(kitprog_handle);

	return ERROR_OK;
}

/*************** jtag wrapper functions *********************/

static int kitprog_swd_init(void)
{
	return ERROR_OK;
}

static void kitprog_swd_write_reg(uint8_t cmd, uint32_t value, uint32_t ap_delay_clk)
{
	assert(!(cmd & SWD_CMD_RnW));
	kitprog_swd_queue_cmd(cmd, NULL, value);
}

static void kitprog_swd_read_reg(uint8_t cmd, uint32_t *value, uint32_t ap_delay_clk)
{
	assert(cmd & SWD_CMD_RnW);
	kitprog_swd_queue_cmd(cmd, value, 0);
}

/*************** jtag lowlevel functions ********************/

static int kitprog_get_usb_serial(void)
{
	int retval;
	const uint8_t str_index = 128; /* This seems to be a constant */
	char desc_string[256+1]; /* Max size of string descriptor */

	retval = libusb_get_string_descriptor_ascii(kitprog_handle->usb_handle,
			str_index, (unsigned char *)desc_string, sizeof(desc_string)-1);
	if (retval < 0) {
		LOG_ERROR("libusb_get_string_descriptor_ascii() failed with %d", retval);
		return ERROR_FAIL;
	}

	/* Null terminate descriptor string */
	desc_string[retval] = '\0';

	/* Allocate memory for the serial number */
	kitprog_handle->serial = calloc(retval + 1, sizeof(wchar_t));
	if (kitprog_handle->serial == NULL) {
		LOG_ERROR("unable to allocate memory for serial");
		return ERROR_FAIL;
	}

	/* Convert the ASCII serial number into a (wchar_t *) */
	if (mbstowcs(kitprog_handle->serial, desc_string, retval + 1) == (size_t)-1) {
		free(kitprog_handle->serial);
		kitprog_serial = NULL;
		LOG_ERROR("unable to convert serial");
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int kitprog_usb_open(void)
{
	const uint16_t vids[] = { VID, 0 };
	const uint16_t pids[] = { PID, 0 };

	if (jtag_libusb_open(vids, pids, kitprog_serial,
			&kitprog_handle->usb_handle) != ERROR_OK) {
		LOG_ERROR("Failed to open or find the device");
		return ERROR_FAIL;
	}

	/* Claim the KitProg Programmer (bulk transfer) interface */
	if (jtag_libusb_claim_interface(kitprog_handle->usb_handle, 1) != ERROR_OK) {
		LOG_ERROR("Failed to claim KitProg Programmer (bulk transfer) interface");
		return ERROR_FAIL;
	}

	/* Get the serial number for the device so the HID interface will point to
	 * the same device.
	 */
	if (kitprog_get_usb_serial() != ERROR_OK)
		LOG_ERROR("Failed to get KitProg serial number");

	/* Use HID for the KitBridge interface */
	kitprog_handle->hid_handle = hid_open(VID, PID, kitprog_handle->serial);
	if (kitprog_handle->hid_handle == NULL) {
		LOG_ERROR("Failed to open KitBridge (HID) interface");
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static void kitprog_usb_close(void)
{
	hid_close(kitprog_handle->hid_handle);
	hid_exit();

	jtag_libusb_close(kitprog_handle->usb_handle);
}

static int kitprog_hid_command(uint8_t *command, size_t command_length,
		uint8_t *data, size_t data_length)
{
	int ret;

	ret = hid_write(kitprog_handle->hid_handle, command, command_length);
	if (ret < 0) {
		LOG_DEBUG("HID write returned %i", ret);
		return ERROR_FAIL;
	}

	ret = hid_read(kitprog_handle->hid_handle, data, data_length);
	if (ret < 0) {
		LOG_DEBUG("HID read returned %i", ret);
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int kitprog_get_version(void)
{
	int ret;

	unsigned char command[3] = {HID_TYPE_START | HID_TYPE_WRITE, 0x00, HID_COMMAND_VERSION};
	unsigned char data[64];

	ret = kitprog_hid_command(command, sizeof command, data, sizeof data);
	if (ret != ERROR_OK)
		return ERROR_FAIL;

	kitprog_handle->hardware_version = data[1];
	kitprog_handle->minor_version = data[2];
	kitprog_handle->major_version = data[3];

	return ERROR_OK;
}

static int kitprog_get_millivolts(void)
{
	int ret;

	unsigned char command[3] = {HID_TYPE_START | HID_TYPE_READ, 0x00, HID_COMMAND_POWER};
	unsigned char data[64];

	ret = kitprog_hid_command(command, sizeof command, data, sizeof data);
	if (ret != ERROR_OK)
		return ERROR_FAIL;

	kitprog_handle->millivolts = (data[4] << 8) | data[3];

	return ERROR_OK;
}

static int kitprog_set_protocol(uint8_t protocol)
{
	int transferred;
	char status = PROGRAMMER_NOK_NACK;

	transferred = jtag_libusb_control_transfer(kitprog_handle->usb_handle,
		LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
		CONTROL_TYPE_WRITE,
		(CONTROL_MODE_SET_PROGRAMMER_PROTOCOL << 8) | CONTROL_COMMAND_PROGRAM,
		protocol, &status, 1, 0);

	if (transferred == 0) {
		LOG_DEBUG("Zero bytes transferred");
		return ERROR_FAIL;
	}

	if (status != PROGRAMMER_OK_ACK) {
		LOG_DEBUG("Programmer did not respond OK");
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int kitprog_get_status(void)
{
	int transferred = 0;
	char status = PROGRAMMER_NOK_NACK;

	/* Try a maximum of three times */
	for (int i = 0; (i < 3) && (transferred == 0); i++) {
		transferred = jtag_libusb_control_transfer(kitprog_handle->usb_handle,
			LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
			CONTROL_TYPE_READ,
			(CONTROL_MODE_POLL_PROGRAMMER_STATUS << 8) | CONTROL_COMMAND_PROGRAM,
			0, &status, 1, 0);
	}

	if (transferred == 0) {
		LOG_DEBUG("Zero bytes transferred");
		return ERROR_FAIL;
	}

	if (status != PROGRAMMER_OK_ACK) {
		LOG_DEBUG("Programmer did not respond OK");
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int kitprog_set_unknown(void)
{
	int transferred;
	char status = PROGRAMMER_NOK_NACK;

	transferred = jtag_libusb_control_transfer(kitprog_handle->usb_handle,
		LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
		CONTROL_TYPE_WRITE,
		(0x03 << 8) | 0x04,
		0, &status, 1, 0);

	if (transferred == 0) {
		LOG_DEBUG("Zero bytes transferred");
		return ERROR_FAIL;
	}

	if (status != PROGRAMMER_OK_ACK) {
		LOG_DEBUG("Programmer did not respond OK");
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int kitprog_acquire_psoc(uint8_t psoc_type, uint8_t acquire_mode,
		uint8_t max_attempts)
{
	int transferred;
	char status = PROGRAMMER_NOK_NACK;

	transferred = jtag_libusb_control_transfer(kitprog_handle->usb_handle,
		LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
		CONTROL_TYPE_WRITE,
		(CONTROL_MODE_ACQUIRE_SWD_TARGET << 8) | CONTROL_COMMAND_PROGRAM,
		(max_attempts << 8) | (acquire_mode << 4) | psoc_type, &status, 1, 0);

	if (transferred == 0) {
		LOG_DEBUG("Zero bytes transferred");
		return ERROR_FAIL;
	}

	if (status != PROGRAMMER_OK_ACK) {
		LOG_DEBUG("Programmer did not respond OK");
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int kitprog_reset_target(void)
{
	int transferred;
	char status = PROGRAMMER_NOK_NACK;

	transferred = jtag_libusb_control_transfer(kitprog_handle->usb_handle,
		LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
		CONTROL_TYPE_WRITE,
		(CONTROL_MODE_RESET_TARGET << 8) | CONTROL_COMMAND_PROGRAM,
		0, &status, 1, 0);

	if (transferred == 0) {
		LOG_DEBUG("Zero bytes transferred");
		return ERROR_FAIL;
	}

	if (status != PROGRAMMER_OK_ACK) {
		LOG_DEBUG("Programmer did not respond OK");
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int kitprog_swd_reset(void)
{
	int transferred;
	char status = PROGRAMMER_NOK_NACK;

	transferred = jtag_libusb_control_transfer(kitprog_handle->usb_handle,
		LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
		CONTROL_TYPE_WRITE,
		(CONTROL_MODE_RESET_SWD_BUS << 8) | CONTROL_COMMAND_PROGRAM,
		0, &status, 1, 0);

	if (transferred == 0) {
		LOG_DEBUG("Zero bytes transferred");
		return ERROR_FAIL;
	}

	if (status != PROGRAMMER_OK_ACK) {
		LOG_DEBUG("Programmer did not respond OK");
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int kitprog_swd_switch_seq(enum swd_special_seq seq)
{
	switch (seq) {
		case LINE_RESET:
			LOG_DEBUG("SWD line reset");
			if (kitprog_swd_reset() != ERROR_OK)
				return ERROR_FAIL;
			break;
		default:
			LOG_ERROR("Sequence %d not supported.", seq);
			return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int kitprog_swd_run_queue(void)
{
	int ret;

	size_t read_count = 0;
	size_t read_index = 0;
	size_t write_count = 0;
	uint8_t *buffer = kitprog_handle->packet_buffer;

	do {
		LOG_DEBUG("Executing %d queued transactions", pending_transfer_count);

		if (queued_retval != ERROR_OK) {
			LOG_DEBUG("Skipping due to previous errors: %d", queued_retval);
			break;
		}

		if (!pending_transfer_count)
			break;

		for (int i = 0; i < pending_transfer_count; i++) {
			uint8_t cmd = pending_transfers[i].cmd;
			uint32_t data = pending_transfers[i].data;

#if 0
			LOG_DEBUG("%s %s reg %x %"PRIx32,
					cmd & SWD_CMD_APnDP ? "AP" : "DP",
					cmd & SWD_CMD_RnW ? "read" : "write",
				  (cmd & SWD_CMD_A32) >> 1, data);
#endif

			buffer[write_count++] = (cmd | SWD_CMD_START | SWD_CMD_PARK) & ~SWD_CMD_STOP;
			read_count++;
			if (!(cmd & SWD_CMD_RnW)) {
				buffer[write_count++] = (data) & 0xff;
				buffer[write_count++] = (data >> 8) & 0xff;
				buffer[write_count++] = (data >> 16) & 0xff;
				buffer[write_count++] = (data >> 24) & 0xff;
			} else {
				read_count += 4;
			}
		}

		ret = jtag_libusb_bulk_write(kitprog_handle->usb_handle,
				EP_OUT, (char *)buffer, write_count, 0);
		if (ret > 0) {
			queued_retval = ERROR_OK;
		} else {
			LOG_DEBUG("Bulk write failed");
			queued_retval = ERROR_FAIL;
			break;
		}

		/* We use the maximum buffer size here because the KitProg sometimes
		 * doesn't like bulk reads of fewer than 62 bytes. (?!?!)
		 */
		ret = jtag_libusb_bulk_write(kitprog_handle->usb_handle,
				EP_IN | LIBUSB_ENDPOINT_IN, (char *)buffer,
				SWD_MAX_BUFFER_LENGTH, 0);
		if (ret > 0) {
			/* Handle garbage data by offsetting the initial read index */
			if ((unsigned int)ret > read_count)
				read_index = ret - read_count;
			queued_retval = ERROR_OK;
		} else {
			LOG_DEBUG("Bulk read failed");
			queued_retval = ERROR_FAIL;
			break;
		}

		for (int i = 0; i < pending_transfer_count; i++) {
			if (pending_transfers[i].cmd & SWD_CMD_RnW) {
				uint32_t data = le_to_h_u32(&buffer[read_index]);

#if 0
				LOG_DEBUG("Read result: %"PRIx32, data);
#endif

				if (pending_transfers[i].buffer)
					*(uint32_t *)pending_transfers[i].buffer = data;

				read_index += 4;
			}
			read_index++;
		}
	} while (0);

	pending_transfer_count = 0;
	int retval = queued_retval;
	queued_retval = ERROR_OK;

	return retval;
}

static void kitprog_swd_queue_cmd(uint8_t cmd, uint32_t *dst, uint32_t data)
{
	if (pending_transfer_count == pending_queue_len) {
		/* Not enough room in the queue. Run the queue. */
		queued_retval = kitprog_swd_run_queue();
	}

	if (queued_retval != ERROR_OK)
		return;

	pending_transfers[pending_transfer_count].data = data;
	pending_transfers[pending_transfer_count].cmd = cmd;
	if (cmd & SWD_CMD_RnW) {
		/* Queue a read transaction */
		pending_transfers[pending_transfer_count].buffer = dst;
	}
	pending_transfer_count++;
}

COMMAND_HANDLER(kitprog_handle_info_command)
{
	if (kitprog_get_version() == ERROR_OK) {
		LOG_INFO("KitProg v%u.%02u",
			kitprog_handle->major_version, kitprog_handle->minor_version);
		LOG_INFO("Hardware version: %u",
			kitprog_handle->hardware_version);
	} else {
		LOG_ERROR("Failed to get KitProg version");
		return ERROR_FAIL;
	}

	if (kitprog_get_millivolts() == ERROR_OK) {
		LOG_INFO("VTARG = %u.%03u V",
			kitprog_handle->millivolts / 1000, kitprog_handle->millivolts % 1000);
	} else {
		LOG_ERROR("Failed to get target voltage");
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

COMMAND_HANDLER(kitprog_handle_reset_target_command)
{
	int retval = kitprog_reset_target();

	return retval;
}

COMMAND_HANDLER(kitprog_handle_serial_command)
{
	if (CMD_ARGC == 1) {
		size_t len = strlen(CMD_ARGV[0]);
		kitprog_serial = calloc(len + 1, sizeof(char));
		if (kitprog_serial == NULL) {
			LOG_ERROR("unable to allocate memory");
			return ERROR_OK;
		}
		strncpy(kitprog_serial, CMD_ARGV[0], len + 1);
	} else {
		LOG_ERROR("expected exactly one argument to kitprog_serial <serial-number>");
	}

	return ERROR_OK;
}

static const struct command_registration kitprog_subcommand_handlers[] = {
	{
		.name = "info",
		.handler = &kitprog_handle_info_command,
		.mode = COMMAND_EXEC,
		.usage = "",
		.help = "show KitProg info",
	},
	{
		.name = "reset_target",
		.handler = &kitprog_handle_reset_target_command,
		.mode = COMMAND_EXEC,
		.usage = "",
		.help = "reset the connected device using the KitProg's built-in target reset function",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration kitprog_command_handlers[] = {
	{
		.name = "kitprog",
		.mode = COMMAND_ANY,
		.help = "perform KitProg management",
		.usage = "<cmd>",
		.chain = kitprog_subcommand_handlers,
	},
	{
		.name = "kitprog_serial",
		.handler = &kitprog_handle_serial_command,
		.mode = COMMAND_CONFIG,
		.help = "set the serial number of the adapter",
		.usage = "serial_string",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct swd_driver kitprog_swd = {
	.init = kitprog_swd_init,
	.switch_seq = kitprog_swd_switch_seq,
	.read_reg = kitprog_swd_read_reg,
	.write_reg = kitprog_swd_write_reg,
	.run = kitprog_swd_run_queue,
};

static const char * const kitprog_transports[] = { "swd", NULL };

struct jtag_interface kitprog_interface = {
	.name = "kitprog",
	.commands = kitprog_command_handlers,
	.transports = kitprog_transports,
	.swd = &kitprog_swd,
	.init = kitprog_init,
	.quit = kitprog_quit
};
