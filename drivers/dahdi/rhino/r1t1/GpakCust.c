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
#include "r1t1.h"
#include <linux/delay.h>

void __r1t1_card_wait_hpi(struct r1t1 *rh, __u8 flags)
{
	__u8 temp;
	int loops;
	int bus_sel;

	bus_sel = (HPI_SEL | DSP_RST) & __r1t1_card_pci_in(rh, TARG_REGS + R1T1_HPIC);
	if (bus_sel != (HPI_SEL | DSP_RST))
		printk("r1t1: Locking Error %x\n", bus_sel);

	if (!(rh->hpi_fast)) {
		temp = (__u8) (__r1t1_card_pci_in(rh, TARG_REGS + R1T1_HPIRDX_STAT) >> 16);
		for (loops = 0; loops < 200; loops++) {
			temp = (__u8) (__r1t1_card_pci_in(rh, TARG_REGS + R1T1_HPIRDX_STAT) >> 16);

			if ((temp & flags) == flags) {
				return;
			}
		}
	}
	return;
}

void __r1t1_card_hpic_set(struct r1t1 *rh, __u16 hpic_data)
{
	__r1t1_card_pci_out(rh, R1T1_DSP_HPIC + TARG_REGS, (__u32) (hpic_data));
	__r1t1_card_wait_hpi(rh, R1T1_HRDY);
}

void r1t1_card_hpic_set(struct r1t1 *rh, __u16 hpic_data)
{
	unsigned int hpi_lock;
	hpi_lock = __r1t1_card_pci_in(rh, TARG_REGS + R1T1_HPIC);
	__r1t1_card_pci_out(rh, TARG_REGS + R1T1_HPIC,
						((hpi_lock & ~0x1F) | HPI_SEL | DSP_RST));
	__r1t1_card_hpic_set(rh, hpic_data);
	__r1t1_card_pci_out(rh, TARG_REGS + R1T1_HPIC, hpi_lock);
}

void r1t1_card_dsp_set(struct r1t1 *rh, __u32 dsp_address, __u16 dsp_data)
{
	__u32 u_nib;
	unsigned int hpi_lock;
	unsigned long flags;

	spin_lock_irqsave(&rh->lock, flags);
	hpi_lock = __r1t1_card_pci_in(rh, TARG_REGS + R1T1_HPIC);
	__r1t1_card_pci_out(rh, TARG_REGS + R1T1_HPIC,
						((hpi_lock & ~0x1F) | HPI_SEL | DSP_RST));

	if (rh->dsp_type == DSP_5510) {
		u_nib = ((dsp_address & 0xf0000) >> 16);
		if (!(rh->hpi_xadd == u_nib)) {

			rh->hpi_xadd = u_nib;

			__r1t1_card_pci_out(rh, R1T1_DSP_HPIC + TARG_REGS, (__u32) (R1T1_XADD));
			__r1t1_card_wait_hpi(rh, R1T1_HRDY);
			__r1t1_card_pci_out(rh, R1T1_HPIA + TARG_REGS, (__u32) (u_nib));
			__r1t1_card_wait_hpi(rh, R1T1_HRDY);
			__r1t1_card_pci_out(rh, R1T1_DSP_HPIC + TARG_REGS, (__u32) (0));
			__r1t1_card_wait_hpi(rh, R1T1_HRDY);

		}
	}

	__r1t1_card_pci_out(rh, R1T1_HPIA + TARG_REGS, (__u32) (dsp_address));
	__r1t1_card_wait_hpi(rh, R1T1_HRDY);

	__r1t1_card_pci_out(rh, R1T1_HPID + TARG_REGS, (__u32) (dsp_data));
	__r1t1_card_wait_hpi(rh, R1T1_HRDY);

	__r1t1_card_pci_out(rh, TARG_REGS + R1T1_HPIC, hpi_lock);

	spin_unlock_irqrestore(&rh->lock, flags);

	return;
}

__u16 r1t1_card_dsp_get(struct r1t1 * rh, __u32 dsp_address)
{
	__u32 dsp_data;
	__u32 u_nib;
	unsigned int hpi_lock;
	unsigned long flags;

	spin_lock_irqsave(&rh->lock, flags);

	hpi_lock = __r1t1_card_pci_in(rh, TARG_REGS + R1T1_HPIC);
	__r1t1_card_pci_out(rh, TARG_REGS + R1T1_HPIC,
						((hpi_lock & ~0x1F) | HPI_SEL | DSP_RST));

	if (rh->dsp_type == DSP_5510) {

		u_nib = ((dsp_address & 0xf0000) >> 16);

		if (!(rh->hpi_xadd == u_nib)) {
			rh->hpi_xadd = u_nib;

			__r1t1_card_pci_out(rh, R1T1_DSP_HPIC + TARG_REGS, (__u32) (R1T1_XADD));
			__r1t1_card_wait_hpi(rh, R1T1_HRDY);
			__r1t1_card_pci_out(rh, R1T1_HPIA + TARG_REGS, (__u32) (u_nib));
			__r1t1_card_wait_hpi(rh, R1T1_HRDY);
			__r1t1_card_pci_out(rh, R1T1_DSP_HPIC + TARG_REGS, (__u32) (0));
			__r1t1_card_wait_hpi(rh, R1T1_HRDY);

		}
	}

	__r1t1_card_pci_out(rh, R1T1_HPIA + TARG_REGS, (__u32) (dsp_address));
	__r1t1_card_wait_hpi(rh, R1T1_HRDY);
	dsp_data = __r1t1_card_pci_in(rh, TARG_REGS + R1T1_HPID);
	__r1t1_card_wait_hpi(rh, R1T1_HRDY);
	dsp_data = __r1t1_card_pci_in(rh, TARG_REGS + R1T1_HPIRDX);

	__r1t1_card_pci_out(rh, TARG_REGS + R1T1_HPIC, hpi_lock);

	spin_unlock_irqrestore(&rh->lock, flags);

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
void gpakReadDspMemory(struct r1t1 *rh,	/* Card containing the DSP */
					   unsigned short int DspId,	/* DSP Identifier (0 to MAX_DSP_CORES-1) */
					   DSP_ADDRESS DspAddress,	/* DSP's memory address of first word */
					   unsigned int NumWords,	/* number of contiguous words to read */
					   DSP_WORD * pWordValues	/* pointer to array of word values variable */
	)
{

	unsigned int word_num;

	if (!rh) {
		printk("r1t1: gpakReadDspMemory: No iface exists for DSP number %d\n", DspId);
		return;
	}

	/* read NumWords from auto increment data register */
	for (word_num = 0; word_num < NumWords; word_num++) {
		pWordValues[word_num] = r1t1_card_dsp_get(rh, DspAddress + word_num);
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
void gpakWriteDspMemory(struct r1t1 *rh,	/* Card containing the DSP */
						unsigned short int DspId,	/* DSP Identifier (0 to MAX_DSP_CORES-1) */
						DSP_ADDRESS DspAddress,	/* DSP's memory address of first word */
						unsigned int NumWords,	/* number of contiguous words to write */
						DSP_WORD * pWordValues	/* pointer to array of word values to write */
	)
{

	unsigned int word_num;

	if (!rh) {
		printk("r1t1: gpakWriteDspMemory: No iface exists for DSP number %d\n", DspId);
		return;
	}

	/* read NumWords from auto increment data register */
	for (word_num = 0; word_num < NumWords; word_num++) {
		r1t1_card_dsp_set(rh, DspAddress + word_num, pWordValues[word_num]);
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
void gpakLockAccess(struct r1t1 *rh,	/* Card containing the DSP */
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
void gpakUnlockAccess(struct r1t1 *rh,	/* Card containing the DSP */
					  unsigned short int DspId	/* DSP Identifier (0 to MAX_DSP_CORES-1) */
	)
{
	return;
}

extern const unsigned char _binary_GpakDsp_fw_start[];
extern const unsigned int _binary_GpakDsp_fw_size;

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


int gpakReadFile_5510(struct r1t1 *rh,	/* Card containing the DSP         */
					  GPAK_FILE_ID FileId,	/* G.PAK Download File Identifier  */
					  unsigned char *pBuffer,	/* pointer to buffer for storing bytes */
					  unsigned int NumBytes	/* number of bytes to read         */
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

		file_size = (unsigned int) &_binary_GpakDsp_fw_size;

		printk("r1t1: %d G168 DSP App file size = %d %x\n", rh->num + 1,
			   (unsigned int) &_binary_GpakDsp_fw_size,
			   (unsigned int) &_binary_GpakDsp_fw_size);
	}

	DspId = rh->num;

	for (byte_num = 0; byte_num < NumBytes; byte_num++) {
		if ((file_size - 1) >= file_pos[DspId]) {
			pBuffer[byte_num] = _binary_GpakDsp_fw_start[file_pos[DspId]++];
		}
	}

	return byte_num;
}
