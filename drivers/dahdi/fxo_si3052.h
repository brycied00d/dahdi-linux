#include <linux/spinlock.h>
#include <linux/dma-mapping.h>
#include <linux/time.h>
#include <linux/pci.h>
#include <linux/timer.h>
#include <linux/kernel.h>
#include <linux/workqueue.h>

#include <dahdi/kernel.h>

/* user-serviceable defines */
#define S3052_DRV_NAME "FXO-Si3052"

//#define S3052_DEBUG 1 /* enable debugging printks */
//#define S3052_BUFFDEBUG 1 /* enable buffer-related printks (warning: verbose; needs S3052_DEBUG) */

//#define S3052_BUFFER_TASKLETS 1 /* should buffers be serviced in tasklets? */

//#define S3052_AUTORESTART_DMA 1 /* should DMA be autorestarted by chip or by manually by driver? */

#define S3052_RXTXBUF_SIZE 2 /* in DAHDI_CHUNKSIZE units, minimum is 2, higher value means more latency but less jitter */

#define S3052_RINGER_CHECK_TIME 100 /* check for ringer end every x msec */
#define S3052_BATTERY_CHECK_TIME 50 /* check for battery every x msec */

/* no user-serviceable defines below */
#ifdef S3052_DEBUG
#define SENTER do { printk(KERN_DEBUG S3052_DRV_NAME ": enter %s\n", __FUNCTION__); } while (0);
#define SLEAVE do { printk(KERN_DEBUG S3052_DRV_NAME ": leave %s\n", __FUNCTION__); } while (0);
#define SDPRN(...) printk(KERN_DEBUG S3052_DRV_NAME ": " __VA_ARGS__);
#else
#define SENTER
#define SLEAVE
#define SDPRN(...)
#endif

#ifdef S3052_BUFFDEBUG
#define SBENTER do { printk(KERN_DEBUG S3052_DRV_NAME ": enter %s\n", __FUNCTION__); } while (0);
#define SBLEAVE do { printk(KERN_DEBUG S3052_DRV_NAME ": leave %s\n", __FUNCTION__); } while (0);
#define SBDPRN(...) SDPRN(__VA_ARGS__);
#else
#define SBENTER
#define SBLEAVE
#define SBDPRN(...)
#endif

			  /* G711x is 8 bits per sample but device expects 32-bit samples */
#define S3052_DMABUF_SIZE (S3052_RXTXBUF_SIZE * DAHDI_CHUNKSIZE * 4)
#define S3052_DMABUF_SPLIT ((S3052_RXTXBUF_SIZE / 2) * DAHDI_CHUNKSIZE * 4)
/* PCI side registers */
#define S3052_REG_DMAI 0x00
#define S3052_REG_DMAIS 0x04
#define S3052_REG_DMARSTART 0x08
#define S3052_REG_DMARSTOP 0x0C
#define S3052_REG_DMARINTERR 0x10
#define S3052_REG_DMARCURR 0x14
#define S3052_REG_DMAWSTART 0x18
#define S3052_REG_DMAWSTOP 0x1C
#define S3052_REG_DMAWINTERR 0x20
#define S3052_REG_DMAWCURR 0x24
#define S3052_REG_WATCH 0x28
/* DAA side registers */
#define S3052_REG_CTRL1 0x31
#define S3052_REG_CTRL2 0x32
#define S3052_REG_IM 0x33
#define S3052_REG_IS 0x34
#define S3052_REG_DAAC1 0x35
#define S3052_REG_DAAC2 0x36
#define S3052_REG_SRC 0x37
#define S3052_REG_DAAC3 0x3a
#define S3052_REG_SSR 0x3b
#define S3052_REG_LSS 0x3c
#define S3052_REG_TRG 0x3f
#define S3052_REG_IC1 0x40
#define S3052_REG_IC2 0x41
#define S3052_REG_IC3 0x42
#define S3052_REG_IC4 0x43
#define S3052_REG_RVC1 0x46
#define S3052_REG_RVC2 0x47
#define S3052_REG_RVC3 0x48
#define S3052_REG_RCAL 0x49
#define S3052_REG_DCT 0x4a
#define S3052_REG_DAAC4 0x4f
#define S3052_REG_SQ 0x6b /* last register */

#define S3052_DMAI_MTIE (1<<24)
#define S3052_DMAI_DIE (1<<23)
#define S3052_DMAI_WIE (1<<22)
#define S3052_DMAI_TAIE (1<<21)
#define S3052_DMAI_MAIE (1<<20)
#define S3052_DMAI_RBIE (1<<19)
#define S3052_DMAI_RAIE (1<<18)
#define S3052_DMAI_WAIE (1<<17)
#define S3052_DMAI_WBIE (1<<16)
#define S3052_DMAI_DM (3<<11)
#define S3052_DMAI_DM_DIRECT (0)
#define S3052_DMAI_DM_PAR (1<<11)
#define S3052_DMAI_DM_SERIAL_L (1<<12)
#define S3052_DMAI_DM_SERIAL_M (3<<11)
#define S3052_DMAI_DMAR (1<<9)
#define S3052_DMAI_DMAE (1<<8)
#define S3052_DMAI_DMAM (1<<7)
#define S3052_DMAI_DMAI (1<<6)
#define S3052_DMAI_DRST (1<<1)
#define S3052_DMAI_PRST (1<<0)

#define S3052_DMAIS_DAA (1<<23)
#define S3052_DMAIS_EER (1<<22)
#define S3052_DMAIS_WFU (1<<21)
#define S3052_DMAIS_WFF (1<<20)
#define S3052_DMAIS_WFE (1<<19)
#define S3052_DMAIS_RFO (1<<18)
#define S3052_DMAIS_RFF (1<<17)
#define S3052_DMAIS_RFE (1<<16)
#define S3052_DMAIS_MTO (1<<8)
#define S3052_DMAIS_DIS (1<<7)
#define S3052_DMAIS_WIS (1<<6)
#define S3052_DMAIS_PTA (1<<5)
#define S3052_DMAIS_PMA (1<<4)
#define S3052_DMAIS_DRB (1<<3)
#define S3052_DMAIS_DRA (1<<2)
#define S3052_DMAIS_DWA (1<<1)
#define S3052_DMAIS_DWB (1<<0)
#define S3052_DMAIS_ALL (S3052_DMAIS_DWB|S3052_DMAIS_DWA|S3052_DMAIS_DRA|S3052_DMAIS_DRB|S3052_DMAIS_PMA|S3052_DMAIS_PTA|S3052_DMAIS_WIS|S3052_DMAIS_DIS|S3052_DMAIS_MTO)

#define S3052_WATCH_XTAL (1<<24)
#define S3052_WATCH_WTC (1<<16)

#define S3052_CTRL1_DL (1<<1)
#define S3052_CTRL1_PWME (1<<3)
#define S3052_CTRL1_MAP (1<<6)
#define S3052_CTRL1_SR (1<<7)

#define S3052_CTRL2_RXE (1<<0)
#define S3052_CTRL2_HBE (1<<1)
#define S3052_CTRL2_RDM (1<<2)
#define S3052_CTRL2_AL (1<<3)
#define S3052_CTRL2_WDTE (1<<4)

#define S3052_IM_LCSM (1<<2)
#define S3052_IM_DODM (1<<3)
#define S3052_IM_BTDM (1<<4)
#define S3052_IM_FDTM (1<<5)
#define S3052_IM_ROVM (1<<6)
#define S3052_IM_RDTM (1<<7)

#define S3052_IS_LCSI (1<<2)
#define S3052_IS_DODI (1<<3)
#define S3052_IS_BTDI (1<<4)
#define S3052_IS_FDTI (1<<5)
#define S3052_IS_ROVI (1<<6)
#define S3052_IS_RDTI (1<<7)

#define S3052_DAAC1_OH (1<<0)
#define S3052_DAAC1_RDT (1<<2)
#define S3052_DAAC1_ONHM (1<<3)
#define S3052_DAAC1_RDTP (1<<5)
#define S3052_DAAC1_RDTN (1<<6)

#define S3052_DAAC2_PDN (1<<3)
#define S3052_DAAC2_PDL (1<<4)

#define S3052_SRC_SRC 0xf
#define S3052_SRC_SRC_8000 1
#define S3052_SRC_SRC_16000 9

#define S3052_DAAC3_DDL (1<<0)

#define S3052_SSR_LSID (0xf<<4)
#define S3052_SSR_LSID_3017 (0)
#define S3052_SSR_LSID_3018 (1<<4)
#define S3052_SSR_LSID_3011 (1<<6)

#define S3052_LSS_LCS (31<<0)
#define S3052_LSS_LCS_SHIFT (0)
#define S3052_LSS_LCS_BELOW_MIN (0)
#define S3052_LSS_LCS_MINIMUM (4)
#define S3052_LSS_FDT (1<<6)

#define S3052_TRG_ATX (7<<4)
#define S3052_TRG_ATX_0 (0)
#define S3052_TRG_ATX_3 (1<<4)
#define S3052_TRG_ATX_6 (1<<5)
#define S3052_TRG_ATX_9 (3<<4)
#define S3052_TRG_ATX_12 (1<<6)
#define S3052_TRG_ARX (7<<0)
#define S3052_TRG_ARX_3 (1<<4)
#define S3052_TRG_ARX_6 (1<<5)
#define S3052_TRG_ARX_9 (3<<4)
#define S3052_TRG_ARX_12 (1<<6)
#define S3052_TRG_ARX_0 (0)

#define S3052_IC1_RT (1<<0)
#define S3052_IC1_RZ (1<<1)
#define S3052_IC1_IIRE (1<<4)
#define S3052_IC1_ACT (1<<5)
#define S3052_IC1_OHS (1<<6)
#define S3052_IC1_ACT2 (1<<7)

#define S3052_IC2_ROV (1<<1)
#define S3052_IC2_BTE (1<<2)
#define S3052_IC2_OPE (1<<3)
#define S3052_IC2_CALD (1<<5)
#define S3052_IC2_CALZ (1<<7)

#define S3052_IC3_RFWE (1<<1)

#define S3052_IC4_OVL (1<<2)
#define S3052_IC4_OPD (1<<0)

#define S3052_RVC1_RMX (63<<0)
#define S3052_RVC1_RMX_SHIFT (0)
#define S3052_RVC1_RDLY (3<<6)
#define S3052_RVC1_RDLY_SHIFT (6)

#define S3052_RVC2_RCC (7<<0)
#define S3052_RVC2_RCC_1024 (7<<0)
#define S3052_RVC2_RCC_640 (6<<0)
#define S3052_RVC2_RCC_512 (5<<0)
#define S3052_RVC2_RCC_384 (4<<0)
#define S3052_RVC2_RCC_256 (3<<0)
#define S3052_RVC2_RCC_200 (2<<0)
#define S3052_RVC2_RCC_150 (1<<0)
#define S3052_RVC2_RCC_100 (0<<0)
#define S3052_RVC2_RTO (15<<3)
#define S3052_RVC2_RTO_SHIFT (3)
#define S3052_RVC2_RDLY2 (1<<7)

#define S3052_RVC3_RNGV (1<<7)
#define S3052_RVC3_RAS (63<<0)
#define S3052_RVC3_RAS_SHIFT (0)

#define S3052_RCAL_RCALD (1<<5)

#define S3052_DCT_DCR (1<<0)
#define S3052_DCT_ILIM (1<<1)
#define S3052_DCT_MINI (3<<4)
#define S3052_DCT_MINI_SHIFT (4)
#define S3052_DCT_MINI_10 (0)
#define S3052_DCT_MINI_12 (1<<4)
#define S3052_DCT_MINI_14 (1<<5)
#define S3052_DCT_MINI_16 (3<<4)
#define S3052_DCT_DCV (3<<6)
#define S3052_DCT_DCV_SHIFT (6)
#define S3052_DCT_DCV_31 (0)
#define S3052_DCT_DCV_32 (1<<6)
#define S3052_DCT_DCV_335 (1<<7)
#define S3052_DCT_DCV_35 (3<<6)

#define S3052_DAAC4_OHS2 (1<<3)
#define S3052_DAAC4_FOH (3<<5)
#define S3052_DAAC4_FOH_512 (0)
#define S3052_DAAC4_FOH_128 (1<<5)
#define S3052_DAAC4_FOH_64 (1<<6)
#define S3052_DAAC4_FOH_8 (3<<5)

#define S3052_SQ_SQ (5<<4)
#define S3052_SQ_NORM (0)
#define S3052_SQ_AUSTRALIA (5<<4)

typedef struct {
    int devno;
    int onhook;
    struct pci_dev *pdev;
    spinlock_t lock;
    u8 *rxbuf, *txbuf;
    dma_addr_t rxbuf_h, txbuf_h;
    u8 txori[S3052_RXTXBUF_SIZE * DAHDI_CHUNKSIZE];
    unsigned int rxbuf_state, txbuf_state;
    int enable_tx;
    unsigned long mmio;
    void *mmio_ptr;
    unsigned int daa_type;
    u8 daa_is;
    struct dahdi_span span;
    struct dahdi_chan chan;
    struct dahdi_chan *chans;
    struct tasklet_struct dma_restart_task;
#ifdef S3052_BUFFER_TASKLETS
    struct tasklet_struct dma_buffers_task;
#endif
    struct work_struct daa_irq_work;
    struct timer_list onhook_timer, ringer_timer, battery_timer;
    unsigned long lastbatttime;
    int lastbatt_signaledstate;
} si3052_dev;

/* *_state field values */
#define S3052_BUF_EMPTY 0 /* buffer is empty[tx]/full[rx] */ 
#define S3052_BUF_FIRST 1 /* first part of buffer is valid */
#define S3052_BUF_SECOND 2 /* second part of buffer is valid */

/* daa_type field values */
#define S3052_DAA_3011 0
#define S3052_DAA_3017 1
#define S3052_DAA_3018 2
