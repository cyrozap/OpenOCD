/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   Copyright (C) 2011 by Andreas Fritiofson                              *
 *   andreas.fritiofson@gmail.com                                          *
 *                                                                         *
 *   Copyright (C) 2014 by Tomas Vanek (PSoC 4 support derived from STM32) *
 *   vanekt@fbl.cz                                                         *
 *                                                                         *
 *   Copyright (C) 2016 by Forest Crossman                                 *
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
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include <helper/binarybuffer.h>
#include <jtag/jtag.h>
#include <target/algorithm.h>
#include <target/armv7m.h>

/* device documets:

 PSoC(R) 5LP: CY8C58LP Family Datasheet
	Document Number: 001-84932 Rev. *J  Revised November 30, 2015

 PSoC 5LP Architecture TRM
	Document No. 001-78426 Rev. *D July 2, 2015

 PSoC 5LP Registers TRM
    Document No. 001-82120 Rev. *D June 30, 2015

 PSoC 5LP Device Programming Specifications
	Document No. 001-81290 Rev. *D June 29, 2015
*/

/* register locations */
#define PSOC5_SPC_CPU_DATA	0x40004720
#define PSOC5_SPC_STATUS	0x40004722
#define PSOC5_DEVICE_ID     0x4008001c

/* constants */
#define PSOC5_SPC_KEY1			0xb6
#define PSOC5_SPC_KEY2			0xd3

#define PSOC5_SPC_IDLE			(1 << 1)

#define PSOC5_CMD_LOAD_ROW		0x02
#define PSOC5_CMD_WRITE_ROW		0x05
#define PSOC5_CMD_PROGRAM_ROW		0x07
#define PSOC5_CMD_ERASE_ALL		0x09
#define PSOC5_CMD_READ_HIDDEN_ROW		0x0a
#define PSOC5_CMD_PROTECT		0x0b
#define PSOC5_CMD_CHECKSUM		0x0c

#define PSOC5_CHIP_PROT_VIRGIN		0x0
#define PSOC5_CHIP_PROT_OPEN		0x1
#define PSOC5_CHIP_PROT_PROTECTED	0x2
#define PSOC5_CHIP_PROT_KILL		0x4

#define PSOC5_KB_PER_ARRAY		64
#define PSOC5_BYTES_PER_ROW		256
#define PSOC5_ROWS_PER_ARRAY		(PSOC5_KB_PER_ARRAY * 1024)/PSOC5_BYTES_PER_ROW


struct psoc5_chip_details {
	uint16_t id;
	const char *type;
	const char *package;
	uint32_t flash_size_in_kb;
};

/* list of PSoC 5 chips
 * flash_size_in_kb is not necessary as it can be decoded from SPCIF_GEOMETRY
 */
const struct psoc5_chip_details psoc5_devices[] = {
	{ 0xe161, "CY8C5888LTI-LP097", "QFN-68", .flash_size_in_kb = 256 }, // 0x2e161069
};


struct psoc5_flash_bank {
	uint32_t row_size;
	uint32_t array_size;
	uint32_t user_bank_size;
	int probed;
	uint32_t silicon_id;
	uint8_t chip_protection;
	uint8_t cmd_program_row;
};


static const struct psoc5_chip_details *psoc5_details_by_id(uint32_t silicon_id)
{
	const struct psoc5_chip_details *p = psoc5_devices;
	unsigned int i;
	uint16_t id = (silicon_id >> 12) & 0xffff; /* ignore die revision */
	for (i = 0; i < sizeof(psoc5_devices)/sizeof(psoc5_devices[0]); i++, p++) {
		if (p->id == id)
			return p;
	}
	LOG_DEBUG("Unknown PSoC device silicon id 0x%08" PRIx32 ".", silicon_id);
	return NULL;
}

static const char *psoc5_decode_chip_protection(uint8_t protection)
{
	switch (protection) {
	case PSOC5_CHIP_PROT_VIRGIN:
		return "protection VIRGIN";
	case PSOC5_CHIP_PROT_OPEN:
		return "protection open";
	case PSOC5_CHIP_PROT_PROTECTED:
		return "PROTECTED";
	case PSOC5_CHIP_PROT_KILL:
		return "protection KILL";
	default:
		LOG_WARNING("Unknown protection state 0x%02" PRIx8 "", protection);
		return "";
	}
}


/* flash bank <name> psoc <base> <size> 0 0 <target#>
 */
FLASH_BANK_COMMAND_HANDLER(psoc5_flash_bank_command)
{
	struct psoc5_flash_bank *psoc5_info;

	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	psoc5_info = calloc(1, sizeof(struct psoc5_flash_bank));

	bank->driver_priv = psoc5_info;
	psoc5_info->user_bank_size = bank->size;

	return ERROR_OK;
}

static int psoc5_spc_command(struct target *target, uint8_t cmd, uint8_t *args,
		size_t num_args)
{
	int retval;

	LOG_DEBUG("SPC command: 0x%02x", cmd);

	do {
		retval = target_write_u8(target, PSOC5_SPC_CPU_DATA, PSOC5_SPC_KEY1);
		if (retval != ERROR_OK)
			break;

		retval = target_write_u8(target, PSOC5_SPC_CPU_DATA, PSOC5_SPC_KEY2 + cmd);
		if (retval != ERROR_OK)
			break;

		retval = target_write_u8(target, PSOC5_SPC_CPU_DATA, cmd);
		if (retval != ERROR_OK)
			break;

		for (unsigned int i = 0; i < num_args; i++) {
			retval = target_write_u8(target, PSOC5_SPC_CPU_DATA, args[i]);
			if (retval != ERROR_OK)
				break;
		}
	} while (0);

	if (retval != ERROR_OK)
		LOG_ERROR("SPC command failed");

	return retval;
}

static int psoc5_protect_check(struct flash_bank *bank)
{
	// struct target *target = bank->target;
	// struct psoc5_flash_bank *psoc5_info = bank->driver_priv;

	// uint32_t protection;
	// int i, s;
	// int num_bits;
	int retval = ERROR_OK;
	//
	// num_bits = bank->num_sectors;
	//
	// for (i = 0; i < num_bits; i += 32) {
	// 	retval = target_read_u32(target, prot_addr, &protection);
	// 	if (retval != ERROR_OK)
	// 		return retval;
	//
	// 	prot_addr += 4;
	//
	// 	for (s = 0; s < 32; s++) {
	// 		if (i + s >= num_bits)
	// 			break;
	// 		bank->sectors[i + s].is_protected = (protection & (1 << s)) ? 1 : 0;
	// 	}
	// }

	// retval = target_read_u32(target, 0, psoc5_info->chip_protection);
	// retval = ERROR_FAIL;
	return retval;
}


static int psoc5_mass_erase(struct flash_bank *bank)
{
	struct target *target = bank->target;
	int i;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* Call "Erase All" SPC command */
	int retval = psoc5_spc_command(target, PSOC5_CMD_ERASE_ALL, NULL, 0);
	if (retval != ERROR_OK) {
		return retval;
	}

	/* Read the status register to make sure the command succeeded */
	uint8_t status;
	do {
		retval = target_read_u8(target, PSOC5_SPC_STATUS, &status);
	} while (!(status & PSOC5_SPC_IDLE));

	/* set all sectors as erased */
	for (i = 0; i < bank->num_sectors; i++)
		bank->sectors[i].is_erased = 1;

	return ERROR_OK;
}


static int psoc5_erase(struct flash_bank *bank, int first, int last)
{
	// struct psoc5_flash_bank *psoc5_info = bank->driver_priv;
	// if (psoc5_info->cmd_program_row == PSOC5_CMD_WRITE_ROW) {
	// 	LOG_INFO("Autoerase enabled, erase command ignored");
	// 	return ERROR_OK;
	// }
	//
	// if ((first == 0) && (last == (bank->num_sectors - 1)))
	// 	return psoc5_mass_erase(bank);
	//
	// LOG_ERROR("Only mass erase available");

	return ERROR_FAIL;
}


static int psoc5_protect(struct flash_bank *bank, int set, int first, int last)
{
// 	struct target *target = bank->target;
// 	struct psoc5_flash_bank *psoc5_info = bank->driver_priv;
//
// 	if (psoc5_info->probed == 0)
// 		return ERROR_FAIL;
//
// 	if (target->state != TARGET_HALTED) {
// 		LOG_ERROR("Target not halted");
// 		return ERROR_TARGET_NOT_HALTED;
// 	}
//
// 	uint32_t *sysrq_buffer = NULL;
// 	int retval;
// 	int num_bits = bank->num_sectors;
// 	const int param_sz = 8;
// 	int prot_sz = num_bits / 8;
// 	int chip_prot = PSOC5_CHIP_PROT_OPEN;
// 	int flash_macro = 0; /* PSoC 42xx has only macro 0 */
// 	int i;
//
// 	sysrq_buffer = calloc(1, param_sz + prot_sz);
// 	if (sysrq_buffer == NULL) {
// 		LOG_ERROR("no memory for row buffer");
// 		return ERROR_FAIL;
// 	}
//
// 	for (i = first; i < num_bits && i <= last; i++)
// 		bank->sectors[i].is_protected = set;
//
// 	uint32_t *p = sysrq_buffer + 2;
// 	for (i = 0; i < num_bits; i++) {
// 		if (bank->sectors[i].is_protected)
// 			p[i / 32] |= 1 << (i % 32);
// 	}
//
// 	/* Call "Load Latch" system ROM API */
// 	sysrq_buffer[1] = prot_sz - 1;
// 	retval = psoc5_spc_command(target, PSOC5_CMD_LOAD_ROW,
// 			0,	/* Byte number in latch from what to write */
// 			sysrq_buffer, param_sz + psoc5_info->row_size);
// 	if (retval != ERROR_OK)
// 		goto cleanup;
//
// 	/* Call "Write Protection" system ROM API */
// 	retval = psoc5_spc_command(target, PSOC5_CMD_PROTECT,
// 			chip_prot | (flash_macro << 8), NULL, 0);
// cleanup:
// 	if (retval != ERROR_OK)
// 		psoc5_protect_check(bank);
//
// 	if (sysrq_buffer)
// 		free(sysrq_buffer);

	return ERROR_OK;
}


COMMAND_HANDLER(psoc5_handle_flash_autoerase_command)
{
	// if (CMD_ARGC < 1)
	// 	return ERROR_COMMAND_SYNTAX_ERROR;
	//
	// struct flash_bank *bank;
	// int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	// if (ERROR_OK != retval)
	// 	return retval;
	//
	// struct psoc5_flash_bank *psoc5_info = bank->driver_priv;
	// bool enable = psoc5_info->cmd_program_row == PSOC5_CMD_WRITE_ROW;
	//
	// if (CMD_ARGC >= 2)
	// 	COMMAND_PARSE_ON_OFF(CMD_ARGV[1], enable);
	//
	// if (enable) {
	// 	psoc5_info->cmd_program_row = PSOC5_CMD_WRITE_ROW;
	// 	LOG_INFO("Flash auto-erase enabled, non mass erase commands will be ignored.");
	// } else {
	// 	psoc5_info->cmd_program_row = PSOC5_CMD_PROGRAM_ROW;
	// 	LOG_INFO("Flash auto-erase disabled. Use psoc mass_erase before flash programming.");
	// }

	return ERROR_OK;
}


static int psoc5_write(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	// struct psoc5_flash_bank *psoc5_info = bank->driver_priv;
	// struct target *target = bank->target;
	// uint32_t *sysrq_buffer = NULL;
	int retval = ERROR_OK;
	// const int param_sz = 8;

// 	if (bank->target->state != TARGET_HALTED) {
// 		LOG_ERROR("Target not halted");
// 		return ERROR_TARGET_NOT_HALTED;
// 	}
//
// 	if (offset & 0x1) {
// 		LOG_ERROR("offset 0x%08" PRIx32 " breaks required 2-byte alignment", offset);
// 		return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
// 	}
//
// 	sysrq_buffer = malloc(param_sz + psoc5_info->row_size);
// 	if (sysrq_buffer == NULL) {
// 		LOG_ERROR("no memory for row buffer");
// 		return ERROR_FAIL;
// 	}
//
// 	uint8_t *row_buffer = (uint8_t *)sysrq_buffer + param_sz;
// 	uint32_t row_num = offset / psoc5_info->row_size;
// 	uint32_t row_offset = offset - row_num * psoc5_info->row_size;
// 	if (row_offset)
// 		memset(row_buffer, 0, row_offset);
//
// 	bool save_poll = jtag_poll_get_enabled();
// 	jtag_poll_set_enabled(false);
//
// 	while (count) {
// 		uint32_t chunk_size = psoc5_info->row_size - row_offset;
// 		if (chunk_size > count) {
// 			chunk_size = count;
// 			memset(row_buffer + chunk_size, 0, psoc5_info->row_size - chunk_size);
// 		}
// 		memcpy(row_buffer + row_offset, buffer, chunk_size);
// 		LOG_DEBUG("offset / row: 0x%08" PRIx32 " / %" PRIu32 ", size %" PRIu32 "",
// 				offset, row_offset, chunk_size);
//
// 		/* Call "Load Latch" system ROM API */
// 		sysrq_buffer[1] = psoc5_info->row_size - 1;
// 		retval = psoc5_spc_command(target, PSOC5_CMD_LOAD_ROW,
// 				0,	/* Byte number in latch from what to write */
// 				sysrq_buffer, param_sz + psoc5_info->row_size);
// 		if (retval != ERROR_OK)
// 			goto cleanup;
//
// 		/* Call "Program Row" or "Write Row" system ROM API */
// 		uint32_t sysrq_param;
// 		retval = psoc5_spc_command(target, psoc5_info->cmd_program_row,
// 				row_num & 0xffff,
// 				&sysrq_param, sizeof(sysrq_param));
// 		if (retval != ERROR_OK)
// 			goto cleanup;
//
// 		buffer += chunk_size;
// 		row_num++;
// 		row_offset = 0;
// 		count -= chunk_size;
// 	}
//
// cleanup:
// 	jtag_poll_set_enabled(save_poll);
//
// 	if (sysrq_buffer)
// 		free(sysrq_buffer);

	return retval;
}


static int psoc5_probe(struct flash_bank *bank)
{
	struct psoc5_flash_bank *psoc5_info = bank->driver_priv;
	struct target *target = bank->target;
	uint32_t flash_size_in_kb = 256;
	uint32_t max_flash_size_in_kb;
	// uint32_t cpu_id;
	uint32_t silicon_id;
	uint32_t row_size = 256;
	uint32_t base_address = 0x00000000;
	// uint8_t protection;
	int retval;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	psoc5_info->probed = 0;
	psoc5_info->cmd_program_row = PSOC5_CMD_PROGRAM_ROW;

	// /* Get the CPUID from the ARM Core
	//  * http://infocenter.arm.com/help/topic/com.arm.doc.ddi0432c/DDI0432C_cortex_m0_r0p0_trm.pdf 4.2.1 */
	// retval = target_read_u32(target, 0xE000ED00, &cpu_id);
	// if (retval != ERROR_OK)
	// 	return retval;
	//
	// LOG_DEBUG("cpu id = 0x%08" PRIx32 "", cpu_id);
	//
	// /* set page size, protection granularity and max flash size depending on family */
	// switch ((cpu_id >> 4) & 0xFFF) {
	// case 0xc23: /* M3 -> PSoC5 */
	// 	row_size = 256;
	// 	max_flash_size_in_kb = 256;
	// 	break;
	// default:
	// 	LOG_WARNING("Cannot identify target PSoC family.");
	// 	return ERROR_FAIL;
	// }

	/* Early revisions of ST-Link v2 have some problem reading PSOC5_SPCIF_GEOMETRY
		and an error is reported late. Dummy read gets this error. */
	uint32_t dummy;
	target_read_u32(target, PSOC5_SPC_CPU_DATA, &dummy);

	/* get silicon ID from target. */
	retval = target_read_u32(target, PSOC5_DEVICE_ID, &silicon_id);
	if (retval != ERROR_OK)
		return retval;

	const struct psoc5_chip_details *details = psoc5_details_by_id(silicon_id);
	if (details) {
		LOG_INFO("%s device detected.", details->type);
		if (flash_size_in_kb == 0)
			flash_size_in_kb = details->flash_size_in_kb;
		else if (flash_size_in_kb != details->flash_size_in_kb)
			LOG_ERROR("Flash size mismatch");
	}

	/* Get protection */

	psoc5_info->row_size = row_size;
	psoc5_info->silicon_id = silicon_id;
	// psoc5_info->chip_protection = protection;

	/* failed reading flash size or flash size invalid (early silicon),
	 * default to max target family */
	if (retval != ERROR_OK || flash_size_in_kb == 0xffff || flash_size_in_kb == 0) {
		LOG_WARNING("PSoC 4 flash size failed, probe inaccurate - assuming %" PRIu32 " k flash",
			max_flash_size_in_kb);
		flash_size_in_kb = max_flash_size_in_kb;
	}

	/* if the user sets the size manually then ignore the probed value
	 * this allows us to work around devices that have a invalid flash size register value */
	if (psoc5_info->user_bank_size) {
		LOG_INFO("ignoring flash probed value, using configured bank size");
		flash_size_in_kb = psoc5_info->user_bank_size / 1024;
	}

	LOG_INFO("flash size = %" PRIu32 " kbytes", flash_size_in_kb);

	/* did we assign flash size? */
	assert(flash_size_in_kb != 0xffff);

	/* calculate numbers of pages */
	uint32_t num_rows = flash_size_in_kb * 1024 / row_size;

	/* check that calculation result makes sense */
	assert(num_rows > 0);

	if (bank->sectors) {
		free(bank->sectors);
		bank->sectors = NULL;
	}

	bank->base = base_address;
	bank->size = num_rows * row_size;
	bank->num_sectors = num_rows;
	bank->sectors = malloc(sizeof(struct flash_sector) * num_rows);

	uint32_t i;
	for (i = 0; i < num_rows; i++) {
		bank->sectors[i].offset = i * row_size;
		bank->sectors[i].size = row_size;
		bank->sectors[i].is_erased = -1;
		bank->sectors[i].is_protected = 1;
	}

	LOG_INFO("flash bank set %" PRIu32 " rows", num_rows);
	psoc5_info->probed = 1;

	return ERROR_OK;
}

static int psoc5_auto_probe(struct flash_bank *bank)
{
	struct psoc5_flash_bank *psoc5_info = bank->driver_priv;
	if (psoc5_info->probed)
		return ERROR_OK;
	return psoc5_probe(bank);
}


static int get_psoc5_info(struct flash_bank *bank, char *buf, int buf_size)
{
	struct psoc5_flash_bank *psoc5_info = bank->driver_priv;
	int printed = 0;

	if (psoc5_info->probed == 0)
		return ERROR_FAIL;

	const struct psoc5_chip_details *details = psoc5_details_by_id(psoc5_info->silicon_id);

	if (details) {
		uint32_t chip_revision = psoc5_info->silicon_id & 0xffff;
		printed = snprintf(buf, buf_size, "PSoC 4 %s rev 0x%04" PRIx32 " package %s",
				details->type, chip_revision, details->package);
	} else
		printed = snprintf(buf, buf_size, "PSoC 4 silicon id 0x%08" PRIx32 "",
				psoc5_info->silicon_id);

	buf += printed;
	buf_size -= printed;

	const char *prot_txt = psoc5_decode_chip_protection(psoc5_info->chip_protection);
	uint32_t size_in_kb = bank->size / 1024;
	snprintf(buf, buf_size, " flash %" PRIu32 " kb %s", size_in_kb, prot_txt);
	return ERROR_OK;
}


COMMAND_HANDLER(psoc5_handle_mass_erase_command)
{
	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	retval = psoc5_mass_erase(bank);
	if (retval == ERROR_OK)
		command_print(CMD_CTX, "psoc mass erase complete");
	else
		command_print(CMD_CTX, "psoc mass erase failed");

	return ERROR_OK;
}


static const struct command_registration psoc5_exec_command_handlers[] = {
	{
		.name = "mass_erase",
		.handler = psoc5_handle_mass_erase_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Erase entire flash device.",
	},
	{
		.name = "flash_autoerase",
		.handler = psoc5_handle_flash_autoerase_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id on|off",
		.help = "Set autoerase mode for flash bank.",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration psoc5_command_handlers[] = {
	{
		.name = "psoc5",
		.mode = COMMAND_ANY,
		.help = "PSoC 5 flash command group",
		.usage = "",
		.chain = psoc5_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

struct flash_driver psoc5_flash = {
	.name = "psoc5",
	.commands = psoc5_command_handlers,
	.flash_bank_command = psoc5_flash_bank_command,
	.erase = psoc5_erase,
	.protect = psoc5_protect,
	.write = psoc5_write,
	.read = default_flash_read,
	.probe = psoc5_probe,
	.auto_probe = psoc5_auto_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = psoc5_protect_check,
	.info = get_psoc5_info,
};
