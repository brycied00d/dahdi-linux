/*
 * Copyright (c) 2001, Adaptive Digital Technologies, Inc.
 *
 * File Name: GpakExts.h
 *
 * Description:
 *   This file contains G.PAK external data and function declarations.
 *
 * Version: 1.0
 *
 * Revision History:
 *   11/13/01 - Initial release.
 *
 */

#ifndef _GPAKEXTS_H				/* prevent multiple inclusion */
#define _GPAKEXTS_H

#include "GpakEnum.h"
#include "sysconfig.h"


// Definition of System Configuration related constants and variables.
extern sysConfig_t sysConfig;	// System Configuration Info
extern struct chanInfo *chanTable[];	// pointers to Channel structures

// work input/output buffers
extern USHORT inWork_10msec[];
extern USHORT outWork_10msec[];
extern USHORT ECFarWork_10msec[];

extern USHORT inWork_2msec[];
extern USHORT outWork_2msec[];
extern USHORT ECFarWork_2msec[];

extern USHORT inWork_1msec[];
extern USHORT outWork_1msec[];
extern USHORT ECFarWork_1msec[];

// Echo Canceller scratch memory.
extern USHORT G168SAscratch_10ms[];	// G.168 SARAM scratch for 10 msec task
extern USHORT G168DAscratch_10ms[];	// G.168 DARAM scratch for 10 msec task

extern USHORT G168SAscratch_2ms[];	// G.168 SARAM scratch for 2 msec task
extern USHORT G168DAscratch_2ms[];	// G.168 DARAM scratch for 2 msec task

extern USHORT G168SAscratch_1ms[];	// G.168 SARAM scratch for 1 msec task
extern USHORT G168DAscratch_1ms[];	// G.168 DARAM scratch for 1 msec task


// Pointers to Echo Canceller data structures.
extern USHORT *pPcmEcDaState[];	// pntr to PCM EC DA States
extern USHORT *pPcmEcSaState[];	// pntr to PCM EC SA States
extern USHORT *pPcmEcEchoPath[];	// pntr to PCM EC Echo Paths
extern USHORT *pPcmEcBackEp[];	// pntr to PCM EC Background Echo Paths
extern USHORT *pPcmEcBackFar[];	// pntr to PCM EC Background Nears
extern USHORT *pPcmEcBackNear[];	// pntr to PCM EC Background Fars
extern USHORT *pPktEcDaState[];	// pntr to Pkt EC DA States
extern USHORT *pPktEcSaState[];	// pntr to Pkt EC SA States
extern USHORT *pPktEcEchoPath[];	// pntr to Pkt EC Echo Paths
extern USHORT *pPktEcBackEp[];	// pntr to Pkt EC Background Echo Paths
extern USHORT *pPktEcBackFar[];	// pntr to Pkt EC Background Nears
extern USHORT *pPktEcBackNear[];	// pntr to Pkt EC Background Fars

extern sysConfig_t sysConfig;
extern struct chanInfo *chanTable[];

extern USHORT DmaLoopback[];	// DMA Loopback flag for each port
extern USHORT NumCfgSlots[];	// num time slots configured on McBSP
extern USHORT McRcvChanEnabA[];	// McBSP's Rcv Chan Enable A reg val
extern USHORT McRcvChanEnabB[];	// McBSP's Rcv Chan Enable B reg val
extern USHORT McXmtChanEnabA[];	// McBSP's Xmt Chan Enable A reg val
extern USHORT McXmtChanEnabB[];	// McBSP's Xmt Chan Enable B reg val


extern USHORT BSP0DMA_TxBuffer[];	// DMA Transmit buffer for McBSP0
extern USHORT BSP0DMA_RxBuffer[];	// DMA Receive buffer for McBSP0
extern USHORT BSP1DMA_TxBuffer[];	// DMA Transmit buffer for McBSP1
extern USHORT BSP1DMA_RxBuffer[];	// DMA Receive buffer for McBSP1
extern USHORT BSP2DMA_TxBuffer[];	// DMA Transmit buffer for McBSP2
extern USHORT BSP2DMA_RxBuffer[];	// DMA Receive buffer for McBSP2
extern chanInfo_t GpakChanInstance[];




extern USHORT PcmInBufferPool[];
extern USHORT PcmOutBufferPool[];
extern USHORT PcmBInBufferPool[];
extern USHORT PcmBOutBufferPool[];
extern CircBufInfo_t PcmBOutCircInfo[];
extern CircBufInfo_t PcmBInCircInfo[];

// System status variables.
extern USHORT NumActiveChannels;	// number of active channels
// Echo Canceller management variables.
extern USHORT NumPcmEcansUsed;	// number of PCM Echo Cancellers in use
extern USHORT PcmEcInUse[];		// flag indicating PCM Echo Canceller in use

// Echo Canceller management variables.
extern USHORT *pPcmEcChan[];	// pointer to PCM Echo Canceller channels
extern USHORT *pPktEcChan[];	// pointer to Packet Echo Canceller channels

extern chanInfo_t GpakChanInstance[];
extern CPG_Instance_t ToneGenInstance[];
extern CPG_Params_t CPGParms[];
extern G168Params_t EcParmsA[];
extern G168Params_t EcParmsB[];

// Definition of function prototypes and their return values.

// Initialize G.PAK interface with host processor.
extern void InitGpakInterface(void);

// Initialize G.PAK PCM I/O.
extern void InitGpakPcm(void);

// Initialize G.PAK Framing task data.
extern void InitFrameTasks(void);

// Initialize a G.PAK channel's Channel structure.
extern struct chanInfo *initChanStruct(USHORT * saramBlock,	// pointer to channel's SARAM memory block
									   USHORT * daramBlock,	// pointer to channel's DARAM memory block
									   USHORT channelId	// channels Id
	);

// ConfigureGpakPcm return status.
typedef enum {
	SPCSuccess,					// serial ports configured successfully
	SPCTooManySlots1,			// too many slots selected for port 1
	SPCInvalidBlockCombo1,		// invalid combination of blocks for port 1
	SPCNoSlots1,				// no slots selected for port 1
	SPCInvalidSlots1,			// invalid slot (exceeds max) selected for port 1
	SPCTooManySlots2,			// too many slots selected for port 2
	SPCInvalidBlockCombo2,		// invalid combination of blocks for port 2
	SPCNoSlots2,				// no slots selected for port 2
	SPCInvalidSlots2,			// invalid slot (exceeds max) selected for port 2
	SPCTooManySlots3,			// too many slots selected for port 3
	SPCInvalidBlockCombo3,		// invalid combination of blocks for port 3
	SPCNoSlots3,				// no slots selected for port 3
	SPCInvalidSlots3			// invalid slot (exceeds max) selected for port 3
} SPCStatus_t;

// Configure G.PAK PCM I/O.
extern SPCStatus_t ConfigureGpakPcm(GpakSlotCfg_t SlotsSelect[],	// port's Slot selection
									USHORT BlockNumber1[],	// port's first group Block Number
									USHORT SlotMask1[],	// port's first group Slot Mask
									USHORT BlockNumber2[],	// port's second group Block Number
									USHORT SlotMask2[]	// port's second group Slot Mask
	);

// Get the address of PCM to Pkt framing rate queue head.
extern chanInfo_t **GetPcmToPktQueueHead(int SamplesPerFrame	// samples per frame (framing rate)
	);

// Get the address of Pkt to PCM framing rate queue head.
extern chanInfo_t **GetPktToPcmQueueHead(int SamplesPerFrame	// samples per frame (framing rate)
	);

// Determine the address of a framing rate phase counter.
extern USHORT *FrameRatePhaseCount(int SamplesPerFrame	// samples per frame (framing rate)
	);

// Adjust Far Echo and Bulk Delay buffer indices.
extern void AdjustFarBulkIndices(int WriteFrameSize,	// Bulk writer's frame size (samples per frame)
								 int ReadFrameSize,	// Far reader's frame size (samples per frame)
								 USHORT * pWrtPhaseCnt,	// pointer to Bulk writer's DMA phase count
								 USHORT * pReadPhaseCnt,	// pointer to Far reader's DMA phase count
								 USHORT BulkDelaySize,	// size of Bulk Delay buffer (samples)
								 USHORT * pBulkPutIndex,	// pointer to Bulk Delay buffer's Put index var
								 USHORT * pFarTakeIndex	// pointer to Far Echo buffer's Take index var
	);

// Enqueue a channel to it's framing rate or conference queues.
extern void EnqueueChannel(chanInfo_t * pChanInfo,	// pointer to channel's Channel Info
						   chanInfo_t ** pPcm2PktHead,	// pointer to PCM to Packet queue head
						   chanInfo_t ** pPkt2PcmHead	// pointer to Packet to PCM queue head
	);

// Dequeue a channel from it's framing rate/conference queues.
extern void DequeueChannel(chanInfo_t * pChanInfo,	// pointer to channel's Channel Info
						   chanInfo_t ** pPcm2PktHead,	// pointer to PCM to Packet queue head
						   chanInfo_t ** pPkt2PcmHead	// pointer to Packet to PCM queue head
	);

// ActivateGpakChannel return status.
typedef enum {
	ACSuccess,					// channel activated successfully
	ACInvalidInputPort1,		// invalid Input Port 1
	ACInvalidInputSlot1,		// invalid Input Slot 1
	ACBusyInputSlot1,			// busy Input Slot 1
	ACInvalidOutputPort1,		// invalid Output Port 1
	ACInvalidOutputSlot1,		// invalid Output Slot 1
	ACBusyOutputSlot1,			// busy Output Slot 1
	ACInvalidInputPort2,		// invalid Input Port 2
	ACInvalidInputSlot2,		// invalid Input Slot 2 
	ACBusyInputSlot2,			// busy Input Slot 2
	ACInvalidOutputPort2,		// invalid Output Port 2
	ACInvalidOutputSlot2,		// invalid Output Slot 2    
	ACBusyOutputSlot2			// busy Output Slot 2
} ACStatus_t;

// Activate a G.PAK Channel.
extern ACStatus_t ActivateGpakChannel(chanInfo_t * pChanInfo	// pointer to Channel Info
	);

// Deactivate a G.PAK Channel.
extern void DeactivateGpakChannel(chanInfo_t * pChanInfo	// pointer to Channel Info
	);

// Determine if the frame size is valid.
extern int ValidFrameSize(int FrameSize	// Frame Size
	);


// Copy linear buffer to circular buffer.
extern void copyLinearToCirc(USHORT * src, CircBufInfo_t * dest, USHORT len);

// Copy circular buffer to linear buffer.
extern void copyCircToLinear(CircBufInfo_t * src, USHORT * dest, USHORT len);

// Perform VAD, Tone Detect, and Encode functions.
extern void ProcessVadToneEncode(chanInfo_t * pChanInfo,	// pointer to Channel Info
								 USHORT * pInWork,	// pointer to input work buffer (contains data)
								 USHORT * pOutWork	// pointer to output work buffer
	);

// Initialize an Echo Canceller.
extern G168ChannelInstance_t *InitEchoCanceller(USHORT FrameSize,	// number of samples per frame
												G168Params_t * EcInitParms,	// Echo Canceller initialization parameters
												short int EcanIndex	// variable that stores ecan index
	);

/*
extern void ToneGenerate(
    short int           *pToneActive,
    short int           *pToneUpdate,
    CPG_Instance_t      *pToneGenPtr,
    CPG_Params_t        *pToneParms,
    short int           *pToneData,
    GpakToneGenCmd_t    ToneCmd,
    short int           FrameSize
	);
*/

extern void algorithmControl(chanInfo_t * pChan	// pointer to Channel structure
	);


extern void writeTransmitEnables(USHORT McBspId,	// McBSP Id
								 USHORT MaskA,	// A Block mask bits to be written
								 USHORT MaskB	// B Block mask bits to be written
	);

void ResetCpuUsageStats();

int validFrameRate(chanInfo_t * pChan, GpakRate_t frameRate);

#endif /* prevent multiple inclusion */
