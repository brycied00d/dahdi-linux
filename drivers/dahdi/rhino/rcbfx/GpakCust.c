/*
 * Copyright (c) 2005, Adaptive Digital Technologies, Inc.
 *
 * File Name: GpakCust.c
 *
 * Description:
 *   This file contains host system dependent functions to support generic
 *   G.PAK API functions. The file is integrated into the host processor
 *   connected to C55x G.PAK DSPs via a Host Port Interface.
 *
 *   Note: This file needs to be modified by the G.PAK system integrator.
 *
 * Version: 1.0
 *
 * Revision History:
 *   06/15/05 - Initial release.
 *
 */

#include "GpakCust.h"
#include "rcbfx.h"
#include <linux/delay.h>

void rcb_card_wait_hpi(struct rcb_card_t *rcb_card, __u8 flags)
{
	volatile __u8 temp;
	int loops;

	if (!(rcb_card->hpi_fast)) {

		temp = (*(volatile __u8 *) (rcb_card->memaddr + RCB_HPIRDX_STAT));

		for (loops = 0; loops < 20; loops++) {

			temp = (*(volatile __u8 *) (rcb_card->memaddr + RCB_HPIRDX_STAT));
			if ((temp & flags) == flags) {
				return;
			}
		}
	}
	return;
}

void rcb_card_dsp_set(struct rcb_card_t *rcb_card, __u32 dsp_address, __u16 dsp_data)
{
	__u32 u_nib;

	if (rcb_card->dsp_type == DSP_5510) {
		u_nib = ((dsp_address & 0xf0000) >> 16);
		if (!(rcb_card->hpi_xadd == u_nib)) {
			rcb_card->hpi_xadd = u_nib;
			*(volatile __u32 *) (rcb_card->memaddr + RCB_HPIC) = (__u32) (RCB_XADD);
			rcb_card_wait_hpi(rcb_card, RCB_HRDY);
			*(volatile __u32 *) (rcb_card->memaddr + RCB_HPIA) = (__u32) (u_nib);
			rcb_card_wait_hpi(rcb_card, RCB_HRDY);
			*(volatile __u32 *) (rcb_card->memaddr + RCB_HPIC) = (__u32) (0);
			rcb_card_wait_hpi(rcb_card, RCB_HRDY);
		}
	}

	*(volatile __u32 *) (rcb_card->memaddr + RCB_HPIA) = (__u32) (dsp_address);
	rcb_card_wait_hpi(rcb_card, RCB_HRDY);

	*(volatile __u32 *) (rcb_card->memaddr + RCB_HPID) = (__u32) (dsp_data);
	rcb_card_wait_hpi(rcb_card, RCB_HRDY);

	return;
}

__u16 rcb_card_dsp_get(struct rcb_card_t * rcb_card, __u32 dsp_address)
{
	__u32 dsp_data;
	__u32 u_nib;

	if (rcb_card->dsp_type == DSP_5510) {

		u_nib = ((dsp_address & 0xf0000) >> 16);

		if (!(rcb_card->hpi_xadd == u_nib)) {
			rcb_card->hpi_xadd = u_nib;
			*(volatile __u32 *) (rcb_card->memaddr + RCB_HPIC) = (__u32) (RCB_XADD);
			rcb_card_wait_hpi(rcb_card, RCB_HRDY);
			*(volatile __u32 *) (rcb_card->memaddr + RCB_HPIA) = (__u32) (u_nib);
			rcb_card_wait_hpi(rcb_card, RCB_HRDY);
			*(volatile __u32 *) (rcb_card->memaddr + RCB_HPIC) = (__u32) (0);
			rcb_card_wait_hpi(rcb_card, RCB_HRDY);
		}
	}

	*(volatile __u32 *) (rcb_card->memaddr + RCB_HPIA) = (__u32) (dsp_address);
	rcb_card_wait_hpi(rcb_card, RCB_HRDY);

	dsp_data = (0xFFFF & *(volatile __u32 *) (rcb_card->memaddr + RCB_HPID));
	rcb_card_wait_hpi(rcb_card, RCB_HRDY);

	dsp_data = (0xFFFF & *(volatile __u32 *) (rcb_card->memaddr + RCB_HPIRDX));

	return (__u16) dsp_data;
}


/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
 * gpakReadDspMemory - Read DSP memory.
 *
 * FUNCTION
 *  This function reads a contiguous block of words from DSP memory starting at
 *  the specified address.
 *
 * RETURNS
 *  nothing
 *
 */
void gpakReadDspMemory(struct rcb_card_t *rcb_card,	/* Card containing the DSP */
					   unsigned short int DspId,	/* DSP Identifier (0 to MAX_DSP_CORES-1) */
					   DSP_ADDRESS DspAddress,	/* DSP's memory address of first word */
					   unsigned int NumWords,	/* number of contiguous words to read */
					   DSP_WORD * pWordValues	/* pointer to array of word values variable */
	)
{

	unsigned int word_num;

	if (!rcb_card) {
		printk("rcbfx: gpakReadDspMemory: No iface exists for DSP number %d\n", DspId);
		return;
	}

	/* read NumWords from auto increment data register */
	for (word_num = 0; word_num < NumWords; word_num++) {
		pWordValues[word_num] = rcb_card_dsp_get(rcb_card, DspAddress + word_num);
	}

	return;
}


/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
 * gpakWriteDspMemory - Write DSP memory.
 *
 * FUNCTION
 *  This function writes a contiguous block of words to DSP memory starting at
 *  the specified address.
 *
 * RETURNS
 *  nothing
 *
 */
void gpakWriteDspMemory(struct rcb_card_t *rcb_card,	/* Card containing the DSP */
						unsigned short int DspId,	/* DSP Identifier (0 to MAX_DSP_CORES-1) */
						DSP_ADDRESS DspAddress,	/* DSP's memory address of first word */
						unsigned int NumWords,	/* number of contiguous words to write */
						DSP_WORD * pWordValues	/* pointer to array of word values to write */
	)
{

	unsigned int word_num;

	if (!rcb_card) {
		printk("rcbfx: gpakWriteDspMemory: No iface exists for DSP number %d\n", DspId);
		return;
	}

	/* read NumWords from auto increment data register */
	for (word_num = 0; word_num < NumWords; word_num++) {
		rcb_card_dsp_set(rcb_card, DspAddress + word_num, pWordValues[word_num]);
	}

	return;
}


/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
 * gpakHostDelay - Delay for a fixed time interval.
 *
 * FUNCTION
 *  This function delays for a fixed time interval before returning. The time
 *  interval is the Host Port Interface sampling period when polling a DSP for
 *  replies to command messages.
 *
 * RETURNS
 *  nothing
 *
 */
void gpakHostDelay(void)
{
	msleep(5);
	return;
}


/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
 * gpakLockAccess - Lock access to the specified DSP.
 *
 * FUNCTION
 *  This function aquires exclusive access to the specified DSP.
 *
 * RETURNS
 *  nothing
 *
 */

void gpakLockAccess(struct rcb_card_t *rcb_card,	/* Card containing the DSP */
					unsigned short int DspId	/* DSP Identifier (0 to MAX_DSP_CORES-1) */
	)
{
	return;
}


/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
 * gpakUnlockAccess - Unlock access to the specified DSP.
 *
 * FUNCTION
 *  This function releases exclusive access to the specified DSP.
 *
 * RETURNS
 *  nothing
 *
 */
void gpakUnlockAccess(struct rcb_card_t *rcb_card,	/* Card containing the DSP */
					  unsigned short int DspId	/* DSP Identifier (0 to MAX_DSP_CORES-1) */
	)
{
	return;
}

extern const unsigned char _binary_DspLoader_fw_start[];
extern const unsigned int _binary_DspLoader_fw_size;
extern const unsigned char _binary_GpakDsp10_fw_start[];
extern const unsigned int _binary_GpakDsp10_fw_size;
extern const unsigned char _binary_GpakDsp0708_fw_start[];
extern const unsigned int _binary_GpakDsp0708_fw_size;
extern const unsigned char _binary_GpakDsp0704_fw_start[];
extern const unsigned int _binary_GpakDsp0704_fw_size;

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
 * gpakReadFile - Read a block of bytes from a G.PAK Download file.
 *
 * FUNCTION
 *  This function reads a contiguous block of bytes from a G.PAK Download file
 *  starting at the current file position.
 *
 * RETURNS
 *  The number of bytes read from the file.
 *   -1 indicates an error occurred.
 *    0 indicates all bytes have been read (end of file)
 *
 */


int gpakReadFile_5510(struct rcb_card_t *rcb_card,	/* Card containing the DSP */
					  GPAK_FILE_ID FileId,	/* G.PAK Download File Identifier */
					  unsigned char *pBuffer,	/* pointer to buffer for storing bytes */
					  unsigned int NumBytes	/* number of bytes to read */
	)
{
	static int stats = 0;
	int core_num;
	int byte_num = -1;
	int DspId;
	static unsigned int file_pos[MAX_DSP_CORES];
	static unsigned int file_size;
	if (!(stats)) {
		stats++;
		for (core_num = 0; core_num < MAX_DSP_CORES; core_num++) {
			file_pos[core_num] = 0;
		}

		file_size = (unsigned int) &_binary_GpakDsp10_fw_size;

		printk("rcbfx %d: G168 DSP App file size = %d %x\n", rcb_card->pos + 1,
			   (unsigned int) &_binary_GpakDsp10_fw_size,
			   (unsigned int) &_binary_GpakDsp10_fw_size);
	}

	DspId = rcb_card->pos;

	for (byte_num = 0; byte_num < NumBytes; byte_num++) {
		if ((file_size - 1) >= file_pos[DspId]) {
			pBuffer[byte_num] = _binary_GpakDsp10_fw_start[file_pos[DspId]++];
		}
	}

	return byte_num;
}

int gpakReadFile_5507(struct rcb_card_t *rcb_card,	/* Card containing the DSP */
					  GPAK_FILE_ID FileId,	/* G.PAK Download File Identifier */
					  unsigned char *pBuffer,	/* pointer to buffer for storing bytes */
					  unsigned int NumBytes	/* number of bytes to read */
	)
{
	static int stats = 0;
	int core_num;
	int byte_num = -1;
	int DspId;
	static unsigned int file_pos[num_files][MAX_DSP_CORES];
	static unsigned int file_size[num_files];
	if (!(stats)) {
		stats++;
		for (core_num = 0; core_num < MAX_DSP_CORES; core_num++) {
			file_pos[loader_file][core_num] = 0;
			file_pos[app_file][core_num] = 0;
		}

		file_size[loader_file] = (unsigned int) &_binary_DspLoader_fw_size;

		if (rcb_card->num_chans > 4)
			file_size[app_file] = (unsigned int) &_binary_GpakDsp0708_fw_size;
		else
			file_size[app_file] = (unsigned int) &_binary_GpakDsp0704_fw_size;

		if (rcb_card->num_chans > 4)
			printk("rcbfx %d: G168 07 08 DSP Loader file size = %d App file size = %d\n",
				   rcb_card->pos + 1, (unsigned int) &_binary_DspLoader_fw_size,
				   (unsigned int) &_binary_GpakDsp0708_fw_size);
		else
			printk("rcbfx %d: G168 07 04 DSP Loader file size = %d App file size = %d\n",
				   rcb_card->pos + 1, (unsigned int) &_binary_DspLoader_fw_size,
				   (unsigned int) &_binary_GpakDsp0704_fw_size);
	}

	DspId = rcb_card->pos;


	if (FileId == loader_file) {
		for (byte_num = 0; byte_num < NumBytes; byte_num++) {
			if ((file_size[loader_file] - 1) >= file_pos[loader_file][DspId]) {
				pBuffer[byte_num] =
					_binary_DspLoader_fw_start[file_pos[loader_file][DspId]++];
			}
		}
	}

	if (FileId == app_file) {
		for (byte_num = 0; byte_num < NumBytes; byte_num++) {
			if ((file_size[app_file] - 1) >= file_pos[app_file][DspId]) {
				if (rcb_card->num_chans > 4)
					pBuffer[byte_num] =
						_binary_GpakDsp0708_fw_start[file_pos[app_file][DspId]++];
				else
					pBuffer[byte_num] =
						_binary_GpakDsp0704_fw_start[file_pos[app_file][DspId]++];
			}
		}
	}

	return byte_num;
}
