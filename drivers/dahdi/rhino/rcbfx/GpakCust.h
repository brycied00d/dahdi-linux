/*
 * Copyright (c) 2005, Adaptive Digital Technologies, Inc.
 *
 * File Name: GpakCust.h
 *
 * Description:
 *   This file contains host system dependent definitions and prototypes of
 *   functions to support generic G.PAK API functions. The file is used when
 *   integrating G.PAK API functions in a specific host processor environment.
 *
 *   Note: This file may need to be modified by the G.PAK system integrator.
 *
 * Version: 1.0
 *
 * Revision History:
 *   06/15/05 - Initial release.
 *
 */


#ifndef _GPAKCUST_H				/* prevent multiple inclusion */
#define _GPAKCUST_H

#include "GpakApi.h"


/* Host and DSP system dependent related definitions. */
#define MAX_DSP_CORES 16		/* maximum number of DSP cores */
#define MAX_CHANNELS 48			/* maximum number of channels */
#define MAX_WAIT_LOOPS 50		/* max number of wait delay loops */
#define DSP_IFBLK_ADDRESS 0x0100	/* DSP address of I/F block pointer */
#define DOWNLOAD_BLOCK_SIZE 512	/* download block size (DSP words) */

#define loader_file 0			/* GPAK_FILE_ID for bootloader */
#define app_file 1				/* GPAK_FILE_ID for application */
#define num_files 2

extern void rcb_card_wait_hpi(struct rcb_card_t *rcb_card, __u8 flags);

extern void rcb_card_dsp_set(struct rcb_card_t *rcb_card, __u32 dsp_address,
							 __u16 dsp_data);

extern __u16 rcb_card_dsp_get(struct rcb_card_t *rcb_card, __u32 dsp_address);

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
extern void gpakReadDspMemory(struct rcb_card_t *rcb_card,	/* Card containing the DSP */
							  unsigned short int DspId,	/* DSP Identifier (0 to MAX_DSP_CORES-1) */
							  DSP_ADDRESS DspAddress,	/* DSP's memory address of first word */
							  unsigned int NumWords,	/* number of contiguous words to read */
							  DSP_WORD * pWordValues	/* pointer to array of word values variable */
	);


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
extern void gpakWriteDspMemory(struct rcb_card_t *rcb_card,	/* Card containing the DSP */
							   unsigned short int DspId,	/* DSP Identifier (0 to MAX_DSP_CORES-1) */
							   DSP_ADDRESS DspAddress,	/* DSP's memory address of first word */
							   unsigned int NumWords,	/* number of contiguous words to write */
							   DSP_WORD * pWordValues	/* pointer to array of word values to write */
	);


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
extern void gpakHostDelay(void);


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
extern void gpakLockAccess(struct rcb_card_t *rcb_card,	/* Card containing the DSP */
						   unsigned short int DspId	/* DSP Identifier (0 to MAX_DSP_CORES-1) */
	);


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
extern void gpakUnlockAccess(struct rcb_card_t *rcb_card,	/* Card containing the DSP */
							 unsigned short int DspId	/* DSP Identifier (0 to MAX_DSP_CORES-1) */
	);


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
extern int gpakReadFile_5510(struct rcb_card_t *rcb_card,	/* Card containing the DSP */
							 GPAK_FILE_ID FileId,	/* G.PAK Download File Identifier */
							 unsigned char *pBuffer,	/* pointer to buffer for storing bytes */
							 unsigned int NumBytes	/* number of bytes to read */
	);

extern int gpakReadFile_5507(struct rcb_card_t *rcb_card,	/* Card containing the DSP */
							 GPAK_FILE_ID FileId,	/* G.PAK Download File Identifier */
							 unsigned char *pBuffer,	/* pointer to buffer for storing bytes */
							 unsigned int NumBytes	/* number of bytes to read */
	);

#endif /* prevent multiple inclusion */
