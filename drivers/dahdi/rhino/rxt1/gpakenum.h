/*
 * Copyright (c) 2005, Adaptive Digital Technologies, Inc.
 *
 * File Name: gpakenum.h
 *
 * Description:
 *   This file contains common enumerations related to G.PAK application
 *   software.
 *
 * Version: 1.0
 *
 * Revision History:
 *   06/15/05 - Initial release.
 *
 */

#ifndef _GPAKENUM_H				/* prevent multiple inclusion */
#define _GPAKENUM_H

/* G.PAK Serial Port Word Size */
typedef enum {
	SerWordSize8 = 0,			// 8-bit seial word
	SerWordSize16 = 1			// 16-bit serial word
} GpakSerWordSize_t;

/* G.PAK Serial Port FrameSync Polarity */
typedef enum {
	FrameSyncActLow = 0,		// active low frame sync signal
	FrameSyncActHigh = 1		// active high frame sync signal
} GpakSerFrameSyncPol_t;

/* G.PAK Serial Port Clock Polarity */
typedef enum {
	SerClockActLow = 0,			// active low serial clock
	SerClockActHigh = 1			// active high serial clock
} GpakSerClockPol_t;

/* G.PAK Serial Port Data Delay */
typedef enum {
	DataDelay0 = 0,				// no data delay
	DataDelay1 = 1,				// 1-bit data delay
	DataDelay2 = 2				// 2-bit data delay
} GpakSerDataDelay_t;

/* G.PAK Serial Port Ids. */
typedef enum {
	SerialPortNull = 0,			// null serial port 
	SerialPort1 = 1,			// first PCM serial stream port (McBSP0) 
	SerialPort2 = 2,			// second PCM serial stream port (McBSP1) 
	SerialPort3 = 3				// third PCM serial stream port (McBSP2) 
} GpakSerialPort_t;

/* G.PAK serial port Slot Configuration selection codes. */
typedef enum {
	SlotCfgNone = 0,			// no time slots used 
	SlotCfg2Groups = 2,			// 2 groups of 16 time slots used, 32 Channels system
	SlotCfg8Groups = 8			// 8-partition mode for 128-channel system
} GpakSlotCfg_t;

/* G.PAK serial port Companding Mode codes. */
typedef enum {
	cmpPCMU = 0,				// u-Law 
	cmpPCMA = 1,				// A-Law 
	cmpNone = 2					// none 
} GpakCompandModes;

/* G.PAK Active/Inactive selection codes. */
typedef enum {
	Disabled = 0,				// Inactive 
	Enabled = 1					// Active 
} GpakActivation;

/* G.PAK Channel Type codes. */
typedef enum {
	inactive = 0,				// channel inactive 
	tdmToTdm = 1				// tdmToTdm
} GpakChanType;

/* G.PAK Algorithm control commands */
typedef enum {
	EnableEcanA = 0,			// Enable A side echo canceller
	BypassEcanA = 1,			// Bypass A side echo canceller
	ResetEcanA = 2,				// Reset A side echo canceller
	EnableEcanB = 3,			// Enable B side echo canceller
	BypassEcanB = 4,			// Bypass B side echo canceller
	ResetEcanB = 5,				// Reset B side echo canceller

	EnableMuLawSwCompanding = 6,	// Enable Mu-law Software companding
	EnableALawSwCompanding = 7,	// Enable Mu-law Software companding
	BypassSwCompanding = 8		// Bypass Software companding
} GpakAlgCtrl_t;

/* G.PAK Tone types. */
typedef enum {
	Null_tone = 0,				// no tone detection 
	DTMF_tone = 1				// DTMF tone 
} GpakToneTypes;

/* G.PAK direction. */
typedef enum {
	TDMAToB = 0,				// A to B 
	TDMBToA = 1					// B to A
} GpakTdmDirection;


typedef enum {
	rate1ms = 0,
	rate2ms = 1,
	rate10ms = 2
} GpakRate_t;

/* G.PAK Asynchronous Event Codes */
typedef enum {
	EventToneDetect = 0,		// Tone detection event
	EventDSPDebug = 7			// DSP debug data event
} GpakAsyncEventCode_t;

/* G.PAK MF Tone Code Indices */
typedef enum {
	DtmfDigit1 = 0,				// DTMF Digit 1 
	DtmfDigit2 = 1,				// DTMF Digit 2 
	DtmfDigit3 = 2,				// DTMF Digit 3 
	DtmfDigitA = 3,				// DTMF Digit A 
	DtmfDigit4 = 4,				// DTMF Digit 4 
	DtmfDigit5 = 5,				// DTMF Digit 5 
	DtmfDigit6 = 6,				// DTMF Digit 6 
	DtmfDigitB = 7,				// DTMF Digit B 
	DtmfDigit7 = 8,				// DTMF Digit 7 
	DtmfDigit8 = 9,				// DTMF Digit 8 
	DtmfDigit9 = 10,			// DTMF Digit 9 
	DtmfDigitC = 11,			// DTMF Digit C 
	DtmfDigitSt = 12,			// DTMF Digit * 
	DtmfDigit0 = 13,			// DTMF Digit 0 
	DtmfDigitPnd = 14,			// DTMF Digit # 
	DtmfDigitD = 15,			// DTMF Digit D 

	EndofMFDigit = 100,			// End of MF digit 
	EndofCngDigit = 101			// End of Cng Digit
} GpakToneCodes_t;

#endif // end multiple inclusion
