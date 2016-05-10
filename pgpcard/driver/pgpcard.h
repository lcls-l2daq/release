//---------------------------------------------------------------------------------
// Title         : Kernel Module For PGP To PCI Bridge Card
// Project       : PGP To PCI-E Bridge Card
//---------------------------------------------------------------------------------
// File          : pgpcard.h
// Author        : Ryan Herbst, rherbst@slac.stanford.edu
// Created       : 05/18/2010
//---------------------------------------------------------------------------------
//
//---------------------------------------------------------------------------------
// Copyright (c) 2010 by SLAC National Accelerator Laboratory. All rights reserved.
//---------------------------------------------------------------------------------
// Modification history:
// 05/18/2010: created.
//---------------------------------------------------------------------------------
#include <linux/init.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <linux/cdev.h>
#include <asm/uaccess.h>
#include <linux/types.h>
#include <linux/spinlock.h>
#include <asm/spinlock.h>

// DMA Buffer Size, Bytes
#define DEF_RX_BUF_SIZE 2097152
#define DEF_TX_BUF_SIZE 2097152

#define NUMBER_OF_LANES 4

// Number of RX & TX Buffers
#define NUMBER_OF_RX_BUFFERS 512
#define MINIMUM_FIRMWARE_BUFFER_COUNT_THRESHOLD 32
#define MAXIMUM_RX_CLIENT_BUFFERS (NUMBER_OF_RX_BUFFERS-MINIMUM_FIRMWARE_BUFFER_COUNT_THRESHOLD-1)
#define NUMBER_OF_RX_CLIENT_BUFFERS 768
#define NUMBER_OF_TX_BUFFERS 64
#define DEF_TX_QUEUE_CNT (NUMBER_OF_TX_BUFFERS)

// PCI IDs
#define PCI_VENDOR_ID_XILINX      0x10ee
#define PCI_DEVICE_ID_XILINX_PCIE 0x0007
#define PCI_VENDOR_ID_SLAC           0x1a4a
#define PCI_DEVICE_ID_SLAC_PGPCARD   0x2000

// Max number of devices to support
#define MAX_PCI_DEVICES 8

// Module Name
#define MOD_NAME "pgpcard"
#define PGPCARD_VERSION "pgpcard driver v02.00.02"

// Number of minor devices =  number of combinations where one and
//  only one group of contiguous bits is high is a four bit value.
#define NUMBER_OF_MINOR_DEVICES (17) // 1 + 2*3 + 3*2 + 4*1
#define ALL_LANES_MASK (0xf)
#define MAX_NUMBER_OPEN_MINOR_DEVICES (NUMBER_OF_LANES + 1) // One for the back door

enum MODELS {SmallMemoryModel=4, LargeMemoryModel=8};

struct PgpCardTXReg {
    __u32 txLWr0;     // 0x800
    __u32 txLWr1;     // 0x804
};

// Address Map, offset from base
struct PgpCardReg {
   __u32               version;                     // 0x000
   __u32               scratch;                     // 0x004
   __u32               irq;                         // 0x008
   __u32               control;                     // 0x00C   // bit 1 is reset
   __u32               __spare0[12];                // 0x010 - 0x03C
   __u32               pgpStat[NUMBER_OF_LANES];    // 0x040 - 0x04C
   __u32               __spare1[12];                // 0x050 - 0x07C
   __u32               pciStat[NUMBER_OF_LANES];    // 0x080 - 0x08C
   __u32               __spare2[220];               // 0x090 - 0x3FC
   __u32               rxFree;                      // 0x400   // head of free list in firmware, buffers added by writing address to this reg
   __u32               rxMaxFrame;                  // 0x404
   __u32               rxStatus;                    // 0x408
   __u32               rxCount;                     // 0x40C
   __u32               __spare3[4];                 // 0x410 - 0x41C
   __u32               rxRead0;                     // 0x420   // RxBuffer struct fields from the firmware
   __u32               rxRead1;                     // 0x424   // DMA pointer with "next valid" field in bit 1
                                                               //  from the firmware with auto advance on read
   __u32               __spare4[246];               // 0x428 - 0x7FC
   struct PgpCardTXReg txLWr[NUMBER_OF_LANES];      // 0x800 - 0x828
   __u32               txStatus;                    // 0x820
   __u32               txReturn;                      // 0x824
   __u32               txCount;                     // 0x828
};

// Structure for TX buffers
struct TxBuffer {
   dma_addr_t  dma;
   unchar*     buffer;
   __u32       lane;
   __u32       vc;
   __u32       length;
   __u32       allocated;
};

// Structure for RX buffers
struct RxBuffer {
   dma_addr_t  dma;
   unchar*     buffer;
   __u32       lengthError;
   __u32       fifoError;
   __u32       eofe;
   __u32       lane;
   __u32       vc;
   __u32       length;
};

// Minor structure
struct Minor {
    __u32             mask;
    struct file*      fp;
    struct inode*     inode;
    // Queues
    wait_queue_head_t inq;
    wait_queue_head_t outq;
};

// Device structure
struct PgpDevice {

   // PCI address regions
   ulong             baseHdwr;
   ulong             baseLen;
   struct PgpCardReg *reg;

   // Device structure
   int          major;
   struct Minor minor[MAX_NUMBER_OPEN_MINOR_DEVICES];
   struct cdev  cdev;

   // Async queue
   struct fasync_struct *async_queue;

   __u32 isOpen;    // minor device ports open mask
   __u32 openCount;

   // Debug flag
   __u32 debug;

   // IRQ
   int irq;

   // Top pointer for rx queue
   struct RxBuffer** rxBuffer;
   struct RxBuffer** rxQueue[NUMBER_OF_LANES];
   __u32            rxRead[NUMBER_OF_LANES];
   __u32            rxWrite[NUMBER_OF_LANES];
   __u32            rxBufferCount;
   __u32*           rxHisto;
   __u32*           rxLoopHisto;
   __u32            rxLaneHisto[NUMBER_OF_LANES];
   __u32            rxBuffersHisto[NUMBER_OF_RX_BUFFERS<<1];
   __u32            rxTotalBufferCount;
   __u32			rxCopyToUserPrintCount;
   __u32            rxTossedBuffers[NUMBER_OF_LANES];
   __u32            rxTotalTossedBuffers;

   // Top pointer for tx queue
   __u32             txBufferCount;
   struct TxBuffer** txBuffer;
   __u32             txRead;
   spinlock_t        txLock;
   spinlock_t        txLockIrq;
   spinlock_t        rxLock;
   spinlock_t        readLock[NUMBER_OF_LANES];
   spinlock_t        ioctlLock;
   spinlock_t        releaseLock;
   spinlock_t        pollLock;
   __u32             goingDown;
   __u32             pollEnabled;
   __u32*            txHisto;
   __u32             interruptNesting;
   __u32             noClientPacketCount[NUMBER_OF_LANES];
   __u32             noClientPacketMax;
};

// TX32 Structure
typedef struct {
    // Data
    __u32 model; // large=8, small=4
    __u32 cmd; // ioctl commands
    __u32 data;

    // Lane & VC
   __u32 pgpLane;
   __u32 pgpVc;

   __u32   size;  // dwords

} PgpCardTx32;

// RX32 Structure
typedef struct {
    __u32   model; // large=8, small=4
    __u32   maxSize; // dwords
    __u32   data;

   // Lane & VC
   __u32    pgpLane;
   __u32    pgpVc;

   // Data
   __u32    rxSize;  // dwords

   // Error flags
   __u32   eofe;
   __u32   fifoErr;
   __u32   lengthErr;

} PgpCardRx32;

// Function prototypes
int PgpCard_Open(struct inode *inode, struct file *filp);
int PgpCard_Release(struct inode *inode, struct file *filp);
ssize_t PgpCard_Write(struct file *filp, const char *buf, size_t count, loff_t *f_pos);
ssize_t PgpCard_Read(struct file *filp, char *buf, size_t count, loff_t *f_pos);
int PgpCard_Ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg);
int my_Ioctl(struct file *filp, __u32 cmd, __u64 argument);
unsigned countRXFirmwareBuffers(struct PgpDevice*, __u32);
unsigned countRxBuffers(struct PgpDevice*, __u32);
unsigned countTxBuffers(struct PgpDevice*);
static irqreturn_t PgpCard_IRQHandler(int irq, void *dev_id, struct pt_regs *regs);
static unsigned int PgpCard_Poll(struct file *filp, poll_table *wait );
static int PgpCard_Probe(struct pci_dev *pcidev, const struct pci_device_id *dev_id);
static void PgpCard_Remove(struct pci_dev *pcidev);
static int PgpCard_Init(void);
static void PgpCard_Exit(void);
int dumpWarning(struct PgpDevice *);
int PgpCard_Mmap(struct file *filp, struct vm_area_struct *vma);
int PgpCard_Fasync(int fd, struct file *filp, int mode);
void PgpCard_VmOpen(struct vm_area_struct *vma);
void PgpCard_VmClose(struct vm_area_struct *vma);

// PCI device IDs
static struct pci_device_id PgpCard_Ids[] = {
   { PCI_DEVICE(PCI_VENDOR_ID_XILINX,PCI_DEVICE_ID_XILINX_PCIE) },
   { PCI_DEVICE(PCI_VENDOR_ID_SLAC,   PCI_DEVICE_ID_SLAC_PGPCARD)   },
   { 0, }
};

// PCI driver structure
static struct pci_driver PgpCardDriver = {
  .name     = MOD_NAME,
  .id_table = PgpCard_Ids,
  .probe    = PgpCard_Probe,
  .remove   = PgpCard_Remove,
};

// Define interface routines
struct file_operations PgpCard_Intf = {
   read:    PgpCard_Read,
   write:   PgpCard_Write,
   ioctl:   PgpCard_Ioctl,
   open:    PgpCard_Open,
   release: PgpCard_Release,
   poll:    PgpCard_Poll,
   fasync:  PgpCard_Fasync,
   mmap:    PgpCard_Mmap,
};

// Virtual memory operations
static struct vm_operations_struct PgpCard_VmOps = {
  open:  PgpCard_VmOpen,
  close: PgpCard_VmClose,
};

