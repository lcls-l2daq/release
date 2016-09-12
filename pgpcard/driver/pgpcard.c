//---------------------------------------------------------------------------------
// Title         : Kernel Module For PGP To PCI Bridge Card
// Project       : PGP To PCI-E Bridge Card
//---------------------------------------------------------------------------------
// File          : pgpcard.c
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
#include <linux/compat.h>
#include <asm/uaccess.h>
#include <linux/cdev.h>
#include "pgpcard.h"
#include "../include/PgpCardMod.h"
#include "../include/PgpCardStatus.h"
#include <linux/types.h>

MODULE_LICENSE("GPL");
MODULE_DEVICE_TABLE(pci, PgpCard_Ids);
module_init(PgpCard_Init);
module_exit(PgpCard_Exit);

// Global Variable
struct PgpDevice gPgpDevices[MAX_PCI_DEVICES];


// Open Returns 0 on success, error code on failure
int PgpCard_Open(struct inode *inode, struct file *filp) {
  int i;
  struct PgpDevice *pgpDevice;
  int requestedMinor;
  unsigned mi;
  unsigned startLooking = 0;
  unsigned myVcMask = 0xf;
  unsigned vcm = 0;
  unsigned clients = 0;
  int ret = SUCCESS;

  // Extract structure for card
  pgpDevice = container_of(inode->i_cdev, struct PgpDevice, cdev);
  filp->private_data = pgpDevice;
  requestedMinor = iminor(inode);
//  printk(KERN_DEBUG"%s: Maj %u Open major %u client %u\n", MOD_NAME, pgpDevice->major, imajor(inode), iminor(inode));
//  if (requestedMinor == 0) { requestedMinor = 15; }
  if (requestedMinor > ALL_LANES_MASK) startLooking = NUMBER_OF_LANE_CLIENTS;

  // Conflict found?
  spin_lock_irq(pgpDevice->releaseLock);
  if ( pgpDevice->isOpen & requestedMinor ) {
    // find the client with the conflict
 	  for (i=0; i<NUMBER_OF_LANE_CLIENTS; i++) {
	    if (requestedMinor & pgpDevice->client[i].mask) {
	      vcm |= pgpDevice->client[i].vcMask;
	      clients += 1;
	    }
	  }
    if (vcm != 0xf) {
      myVcMask = vcm ^ 0xf;
    }
	  if ((myVcMask == 0xf) || (clients > 1)) {
	    printk(KERN_WARNING"%s: Open: module open failed. Device port conflict. Maj=%i, request 0x%x, already opened 0x%x found %u clients vcm(%x) myVcMask(%x)\n",
	        MOD_NAME, pgpDevice->major, (unsigned)requestedMinor, (unsigned)pgpDevice->isOpen, clients, vcm, myVcMask);
	    ret = ERROR;
	  }
  }
  if ((ret == SUCCESS) && (requestedMinor < NUMBER_OF_MINOR_DEVICES)) {
    pgpDevice->goingDown &= ~(requestedMinor&0xf);
    printk(KERN_DEBUG"%s: Maj %u cleared %x going down %x\n", MOD_NAME, pgpDevice->major, requestedMinor & 0xf, pgpDevice->goingDown);
    for(mi=startLooking; mi<MAX_NUMBER_OPEN_CLIENTS; mi++) {
      if (pgpDevice->client[mi].fp == 0) break;
    }
    if (mi<MAX_NUMBER_OPEN_CLIENTS) {
      pgpDevice->isOpen |= (requestedMinor & ALL_LANES_MASK);
      pgpDevice->client[mi].mask = (__u32)requestedMinor & 0xf;
      pgpDevice->client[mi].vcMask = 0;
      pgpDevice->client[mi].fp = filp;
      pgpDevice->client[mi].inode = inode;
      if (mi < NUMBER_OF_LANE_CLIENTS) {
        pgpDevice->client[mi].vcMask = myVcMask;
        init_waitqueue_head(&pgpDevice->client[mi].inq);
        init_waitqueue_head(&pgpDevice->client[mi].outq);
        pgpDevice->rxTossedBuffers[mi] = 0;
      }
      printk(KERN_DEBUG"%s: Maj %u Opened client %u, mask=0x%x  vcMask=0x%x pollEnabled=%u\n", MOD_NAME, pgpDevice->major,
          mi, pgpDevice->client[mi].mask, pgpDevice->client[mi].vcMask, pgpDevice->pollEnabled);
      if ((pgpDevice->openCount == 0) &&  (pgpDevice->client[mi].mask != 0)){
        pgpDevice->pollEnabled = 1;
        printk(KERN_DEBUG"%s: Maj %u polling enabled\n", MOD_NAME, pgpDevice->major);
      }
      pgpDevice->openCount += 1;
    } else {
      printk(KERN_WARNING "%s: No more clients available, %u already open.\n", MOD_NAME, pgpDevice->openCount);
      ret =  ERROR;
    }
  } else {
    printk(KERN_WARNING "%s: Requested lane number %u is too high or found VC conflict above\n", MOD_NAME, requestedMinor);
    ret =  ERROR;  //just in case the former
  }
  spin_unlock_irq(pgpDevice->releaseLock);
  return ret;
}


// PgpCard_Release
// Called when the device is closed
// Returns 0 on success, error code on failure
int PgpCard_Release(struct inode *inode, struct file *filp) {
  struct PgpDevice *pgpDevice = (struct PgpDevice *)filp->private_data;
  unsigned mi;
  unsigned i = MAX_NUMBER_OPEN_CLIENTS;
  unsigned found = 0;
  unsigned count = 0;
  unsigned useAllVcs = 1;
  spin_lock_irq(pgpDevice->releaseLock);
  for (mi=0; mi<MAX_NUMBER_OPEN_CLIENTS; mi++) {
    if ((!found) && (pgpDevice->client[mi].fp == filp)) {
      printk(KERN_DEBUG"%s: Maj %u Closing client %u, mask 0x%x vcMask 0x%x\n",
          MOD_NAME, pgpDevice->major, mi, pgpDevice->client[mi].mask, pgpDevice->client[mi].vcMask);
      if (pgpDevice->client[mi].mask) {
        if (pgpDevice->client[mi].vcMask != 0xf) {
          useAllVcs = 0;
          for (i=0; i<MAX_NUMBER_OPEN_CLIENTS; i++) {
            if (pgpDevice->client[mi].mask & pgpDevice->client[i].mask) {
              if (i!=mi) {
                break;
              }
            }
          }
        }
        // if we did not find a match or are using all VCs ...
        if ((i==MAX_NUMBER_OPEN_CLIENTS) || (useAllVcs==1)) {
          pgpDevice->goingDown |= pgpDevice->client[mi].mask;
          printk(KERN_DEBUG"%s: Maj %u set %x going down %x\n", MOD_NAME, pgpDevice->major, pgpDevice->client[mi].mask, pgpDevice->goingDown);
          pgpDevice->isOpen &= ~(pgpDevice->client[mi].mask);
        }
      }
      found = 1;
      pgpDevice->client[mi].fp = 0;
      pgpDevice->client[mi].mask = 0;
      pgpDevice->client[mi].vcMask = 0;
      if (mi < NUMBER_OF_LANE_CLIENTS) {
        init_waitqueue_head(&pgpDevice->client[mi].inq);
        init_waitqueue_head(&pgpDevice->client[mi].outq);
        count = 0;
        while (pgpDevice->rxRead[mi] != pgpDevice->rxWrite[mi]) {
          pgpDevice->reg->rxFree = pgpDevice->rxQueue[mi][pgpDevice->rxRead[mi]]->dma;
          pgpDevice->rxRead[mi] = (pgpDevice->rxRead[mi] + 1) % (NUMBER_OF_RX_CLIENT_BUFFERS);
          count += 1;
        }
        if (count) {
          printk(KERN_WARNING "%s: PgpCard_Release reclaimed %u buffer%s for client %u\n",
              MOD_NAME, count, count>1 ? "s" : "", mi);
        }
        if (pgpDevice->rxTossedBuffers[mi]) {
          printk(KERN_WARNING"%s: Maj %u client %u has discarded %u buffers\n",
              MOD_NAME, pgpDevice->major, mi, pgpDevice->rxTossedBuffers[mi]);
        }
      }
    }
  }
  if (!found) {
    // File is not open
    spin_unlock_irq(pgpDevice->releaseLock);
    printk(KERN_WARNING"%s: Release: module close failed. Device is not open. Maj=%i, Min=0x%x, Open=0x%x\n",
        MOD_NAME,pgpDevice->major, MINOR(inode->i_cdev->dev), pgpDevice->isOpen);
    return ERROR;
  }
  pgpDevice->openCount -= 1;
  if (pgpDevice->openCount == 0) {
    pgpDevice->pollEnabled = 0;
    pgpDevice->goingDown = 0;
    printk(KERN_DEBUG"%s: Maj %u polling disabled goingDown %x\n", MOD_NAME, pgpDevice->major, pgpDevice->goingDown);
    spin_unlock_irq(pgpDevice->releaseLock);
    dumpWarning(pgpDevice);
  } else {
    printk(KERN_DEBUG"%s: Maj %u polling %s, %u clients\n", MOD_NAME, pgpDevice->major,
        pgpDevice->pollEnabled ? "left enabled" : "now close, must be a panic!", pgpDevice->openCount);
    spin_unlock_irq(pgpDevice->releaseLock);
  }
  return SUCCESS;
}


// PgpCard_Write
// Called when the device is written to
// Returns write count on success. Error code on failure.
ssize_t PgpCard_Write(struct file *filp, const char* buffer, size_t count, loff_t* f_pos) {
  int         i;
  __u32       descA;
  __u32       descB;
  PgpCardTx*  pgpCardTx;
  PgpCardTx   myPgpCardTx;
  __u32       buf[count / sizeof(__u32)];
  __u32       theRightWriteSize = sizeof(PgpCardTx);
  __u32       largeMemoryModel, smallMemoryModel;
  __u32       found = 0;
  __u32       mi = 0;  // minor index into open client list
  __u32       j;

  struct PgpDevice* pgpDevice = (struct PgpDevice *)filp->private_data;

  // Copy command stucture from user space
  if ( copy_from_user(buf, buffer, count) ) {
    printk(KERN_WARNING "%s: Write: failed to copy command structure from user(%p) space. Maj=%i\n",
        MOD_NAME, buffer, pgpDevice->major);
    return ERROR;
  }

  largeMemoryModel = buf[0] == LargeMemoryModel;
  smallMemoryModel = buf[0] == SmallMemoryModel;

  if (smallMemoryModel) {
    PgpCardTx32* p = (PgpCardTx32*) buf;
    pgpCardTx      = &myPgpCardTx;
    pgpCardTx->cmd     = p->cmd;
    pgpCardTx->pgpLane = p->pgpLane;
    pgpCardTx->pgpVc   = p->pgpVc;
    pgpCardTx->size    = p->size;
    pgpCardTx->data    = (__u32*)(0LL | p->data);
    theRightWriteSize  = sizeof(PgpCardTx32);
    //       printk(KERN_WARNING "%s: Write: diddling 32->64 (0x%x)->(0x%p)\n", MOD_NAME, p->data, pgpCardTx->data);
  } else if (largeMemoryModel) {
    pgpCardTx = (PgpCardTx*) buf;
  } else {
    printk(KERN_WARNING "%s: Write: failed because Bad Memory Model %u. Maj=%i\n",
            MOD_NAME, buf[0], pgpDevice->major);
    return ERROR;
  }

  if ( pgpDevice->debug & 0x100 ) printk(KERN_DEBUG "%s: cmd 0x%x, data 0x%p\n", MOD_NAME, pgpCardTx->cmd, pgpCardTx->data);
  switch (pgpCardTx->cmd) {
    case IOCTL_Normal_Write :
      for (i=0; i<MAX_NUMBER_OPEN_CLIENTS; i++) {
        if (pgpDevice->client[i].fp == filp) {
          if (pgpDevice->client[i].mask & (1 << pgpCardTx->pgpLane)) {
            found = 1;
            mi = i;
          } else {
            printk(KERN_WARNING "%s: Write: failed because this client's (%d) mask 0x%x does not have lane %u opened\n",
                MOD_NAME, i, pgpDevice->client[i].mask, pgpCardTx->pgpLane);
            return ERROR;
          }
        }
      }
      if (!(pgpDevice->client[mi].vcMask & (1 << pgpCardTx->pgpVc))) {
        printk(KERN_WARNING "%s: Write: failed because this this client's (%d) vcMask 0x%x does not have VC %u opened\n",
            MOD_NAME, mi, pgpDevice->client[mi].vcMask, pgpCardTx->pgpVc);
        return ERROR;
      }
      if (!found) {
        printk(KERN_WARNING "%s: Write: failed because this file pointer is not opened\n", MOD_NAME);
        return ERROR;
      }

      if (count != theRightWriteSize) {
        printk(KERN_WARNING "%s: Write(%u) passed size is not expected(%u) size(%u). Maj=%i\n",
            MOD_NAME,
            pgpCardTx->cmd,
            (unsigned)sizeof(PgpCardTx),
            (unsigned)count, pgpDevice->major);
      }
      if ( (pgpCardTx->size*sizeof(__u32)) > DEF_TX_BUF_SIZE ) {
        printk(KERN_WARNING"%s: Write: passed size is too large for TX buffer. Maj=%i\n",MOD_NAME,pgpDevice->major);
        return(ERROR);
      }
      // Are buffers available
      if ( pgpDevice->debug & 0x10 ) {
        printk(KERN_DEBUG"%s: PgpCard_Write() test for buffers avail for lane=%u txRead=%u\n",
            MOD_NAME, pgpCardTx->pgpLane, pgpDevice->txRead);
      }
      if ( pgpDevice->debug & 0x20 ) {
        printk(KERN_DEBUG "-Lock-%u", pgpCardTx->pgpLane);
      }
//      printk(KERN_DEBUG "%s: write: buffCount %u numBuffs %u\n", MOD_NAME, countTxBuffers(pgpDevice), NUMBER_OF_TX_BUFFERS);
      while ( (pgpDevice->txBufferCount = countTxBuffers(pgpDevice)) >= NUMBER_OF_TX_BUFFERS ) {
        if ( filp->f_flags & O_NONBLOCK ) {
          if ( pgpDevice->debug & 0x20 ) {
            printk(KERN_DEBUG "___\n");
          }
          return(-EAGAIN);
        }
        if ( pgpDevice->debug & 4 ) {
          printk(KERN_DEBUG"%s: Write: going to sleep. Maj=%i\n",MOD_NAME,pgpDevice->major);
        }
        if (wait_event_interruptible(pgpDevice->client[mi].outq, (pgpDevice->txBufferCount < NUMBER_OF_TX_BUFFERS))) {
          if ( pgpDevice->debug & 0x20 ) {
            printk(KERN_DEBUG "___\n");
          }
          return (-ERESTARTSYS);
        }
        if ( pgpDevice->debug & 4 ) {
          printk(KERN_DEBUG"%s: Write: woke up. Maj=%i\n",MOD_NAME,pgpDevice->major);
        }
      }

      j = 0;
      spin_lock(pgpDevice->txLock);
      do {
        pgpDevice->txRead += 1;
        pgpDevice->txRead %= NUMBER_OF_TX_BUFFERS;
        if (pgpDevice->txBuffer[pgpDevice->txRead]->allocated != 0) {
          if ((pgpDevice->txBuffer[pgpDevice->txRead]->allocated)++ > 3) {
            pgpDevice->txBuffer[pgpDevice->txRead]->allocated = 0;
            spin_lock_irq(pgpDevice->txLockIrq);
            pgpDevice->txBufferCount = countTxBuffers(pgpDevice);
            spin_unlock_irq(pgpDevice->txLockIrq);
            printk(KERN_DEBUG "%s: Write reclaimed buffer lane %u, vc %u, txRead %u\n", MOD_NAME,
                pgpDevice->txBuffer[pgpDevice->txRead]->lane,
                pgpDevice->txBuffer[pgpDevice->txRead]->vc,
                pgpDevice->txRead);
          }
        }
      } while ((j++ < NUMBER_OF_TX_BUFFERS) && (pgpDevice->txBuffer[pgpDevice->txRead]->allocated != 0));

      if (j >= NUMBER_OF_TX_BUFFERS) {  // should never happen
        spin_unlock(pgpDevice->txLock);
        return (-ERROR);
      }

      pgpDevice->txBuffer[pgpDevice->txRead]->allocated = 1;
      spin_lock_irq(pgpDevice->txLockIrq);
      pgpDevice->txBufferCount = countTxBuffers(pgpDevice);
      pgpDevice->txHisto[pgpDevice->txBufferCount] += 1;
      spin_unlock_irq(pgpDevice->txLockIrq);

      if ( pgpDevice->debug & 0x10 ) {
        printk(KERN_DEBUG"%s: copy_from_user( %p, %p, %u\n", MOD_NAME,
            pgpDevice->txBuffer[pgpDevice->txRead]->buffer, pgpCardTx->data, (unsigned int)(long unsigned int)(pgpCardTx->size*sizeof(__u32)));
      }

      // Copy data from user space
      if ( copy_from_user(pgpDevice->txBuffer[pgpDevice->txRead]->buffer,pgpCardTx->data,(pgpCardTx->size*sizeof(__u32))) ) {
        printk(KERN_WARNING "%s: Write: failed to copy from user(%p) space. Maj=%i\n",
            MOD_NAME,
            pgpCardTx->data,
            pgpDevice->major);
        if ( pgpDevice->debug & 0x20 ) printk(KERN_DEBUG "___\n");
        spin_unlock(pgpDevice->txLock);
        return ERROR;
      }

      if ( pgpDevice->debug & 0x10 ) {
        printk(KERN_DEBUG"%s: Fields for tracking purpose\n", MOD_NAME);
      }
      // Fields for tracking purpose
      pgpDevice->txBuffer[pgpDevice->txRead]->lane   = pgpCardTx->pgpLane;
      pgpDevice->txBuffer[pgpDevice->txRead]->vc     = pgpCardTx->pgpVc;
      pgpDevice->txBuffer[pgpDevice->txRead]->length = pgpCardTx->size;

      // Generate Tx descriptor
      descA  = (pgpCardTx->pgpLane << 30) & 0xC0000000; // Bits 31:30 = Lane
      descA += (pgpCardTx->pgpVc   << 28) & 0x30000000; // Bits 29:28 = VC
      descA += (pgpCardTx->size         ) & 0x00FFFFFF; // Bits 23:0 = Length
      descB = pgpDevice->txBuffer[pgpDevice->txRead]->dma;
      pgpDevice->txHistoLV[pgpCardTx->pgpLane * NUMBER_OF_VC + pgpCardTx->pgpVc] += 1;

      // Debug
      if ( pgpDevice->debug & 2 ) {
        printk(KERN_DEBUG"%s: Write: Words=%i, Lane=%i, VC=%i, Addr=%p, DMA=%p. Maj=%d\n",
            MOD_NAME, pgpCardTx->size, pgpCardTx->pgpLane, pgpCardTx->pgpVc,
            (pgpDevice->txBuffer[pgpDevice->txRead]->buffer), (void*)(pgpDevice->txBuffer[pgpDevice->txRead]->dma),
            pgpDevice->major);
      }

      if ( pgpDevice->debug & 0x10 ) {
        printk(KERN_DEBUG"%s: Write descriptor\n", MOD_NAME);
      }
      // Write descriptor
      pgpDevice->reg->txLWr[pgpCardTx->pgpLane & 3].txLWr0 = descA;
      pgpDevice->reg->txLWr[pgpCardTx->pgpLane & 3].txLWr1 = descB;

      if ( pgpDevice->debug & 0x20 ) printk(KERN_DEBUG "unlock\n");
      spin_unlock(pgpDevice->txLock);
      return(pgpCardTx->size);
      break;
    default :
      if ((pgpCardTx->cmd > IOCTL_Normal_Write) && (pgpCardTx->cmd <= IOCTL_End_Of_List)) {
        printk(KERN_DEBUG "%s: IOCTL cmd %d, data 0x%p\n", MOD_NAME, pgpCardTx->cmd, pgpCardTx->data);
      }
      return my_Ioctl(filp, pgpCardTx->cmd, (__u64)pgpCardTx->data);
      break;
  }
}


// PgpCard_Read
// Called when the device is read from
// Returns read count on success. Error code on failure.
ssize_t PgpCard_Read(struct file *filp, char *buffer, size_t count, loff_t *f_pos) {
  int        ret, i;
  __u32        buf[count / sizeof(__u32)];
  PgpCardRx*    p64 = (PgpCardRx *)buf;
  PgpCardRx32*  p32 = (PgpCardRx32*)buf;
  __u32 __user * dp;
  __u32       maxSize;
  __u32       copyLength;
  __u32       largeMemoryModel, smallMemoryModel;
  __u32       found = 0;
  __u32       mi = 0;  // minor index into open client list

  struct PgpDevice *pgpDevice = (struct PgpDevice *)filp->private_data;

  if ((count != sizeof(PgpCardRx32)) && (count != sizeof(PgpCardRx))) {
    printk(KERN_WARNING"%s: Read: passed size is not expected(%u) Maj=%i\n",MOD_NAME, (unsigned)count, pgpDevice->major);
          return(ERROR);
  }

  // Copy command structure from user space
  if ( copy_from_user(buf, buffer, count) ) {

    printk(KERN_WARNING "%s: Read: failed to copy command structure from user(%p) space. Maj=%i\n",
        MOD_NAME,
        buffer,
        pgpDevice->major);
    return ERROR;
  }

  largeMemoryModel = buf[0] == LargeMemoryModel;
  smallMemoryModel = buf[0] == SmallMemoryModel;

  // Verify that size of passed structure and get variables from the correct structure.
  if ( smallMemoryModel ) {
    // small memory model
    if ( count != sizeof(PgpCardRx32) ) {
      printk(KERN_WARNING"%s: Read: passed size is not expected(%u) size(%u). Maj=%i\n",MOD_NAME, (unsigned)sizeof(PgpCardRx32), (unsigned)count, pgpDevice->major);
      return(ERROR);
    }
    dp      = (__u32*)(0LL | p32->data);
    maxSize = p32->maxSize;
  } else if ( largeMemoryModel ) {
    // large memory model
    if ( count != sizeof(PgpCardRx) ) {
      printk(KERN_WARNING"%s: Read: passed size is not expected(%u) size(%u). Maj=%i\n",MOD_NAME, (unsigned)sizeof(PgpCardRx), (unsigned)count, pgpDevice->major);
      return(ERROR);
    } else {
      dp      = p64->data;
      maxSize = p64->maxSize;
    }
  } else {
    printk(KERN_WARNING"%s: Read: failed, bad memory model %d\n", MOD_NAME, buf[0]);
    return(ERROR);
  }

  for (i=0; i<NUMBER_OF_LANE_CLIENTS; i++) {
    if (!found && (pgpDevice->client[i].fp == filp)) {
      found = 1;
      mi = i;
    }
  }
  if (!found) {
    printk(KERN_WARNING "%s: Read: failed because this file pointer is not opened\n", MOD_NAME);
    return ERROR;
  }

  spin_lock(&(pgpDevice->readLock[mi]));

  if ( pgpDevice->debug & 2 ) {
    printk(KERN_DEBUG "%s: Read: mi(%u) rxRead(%u) rxWrite(%u)\n", MOD_NAME, mi, pgpDevice->rxRead[mi], pgpDevice->rxWrite[mi]);
  }

  // No data is ready
  while ( pgpDevice->rxRead[mi] == pgpDevice->rxWrite[mi] ) {
    if ( filp->f_flags & O_NONBLOCK ) {
      spin_unlock(&(pgpDevice->readLock[mi]));
      return(-EAGAIN);
    }
    if ( pgpDevice->debug & 4 ) printk(KERN_DEBUG"%s: Read: going to sleep. Maj=%i\n", MOD_NAME, pgpDevice->major);
    if (wait_event_interruptible(pgpDevice->client[mi].inq, (pgpDevice->rxRead[mi] != pgpDevice->rxWrite[mi]))) {
      spin_unlock(&(pgpDevice->readLock[mi]));
      return (-EAGAIN);
    }
    if ( pgpDevice->debug & 4 ) printk(KERN_DEBUG"%s: Read: woke up. Maj=%i\n", MOD_NAME,pgpDevice->major);
  }

  if ( pgpDevice->debug & 2 ) {
    printk(KERN_DEBUG "%s: Read: Check for frame errors\n", MOD_NAME);
  }

  if (pgpDevice->rxQueue[mi][pgpDevice->rxRead[mi]]->eofe |
      pgpDevice->rxQueue[mi][pgpDevice->rxRead[mi]]->fifoError |
      pgpDevice->rxQueue[mi][pgpDevice->rxRead[mi]]->lengthError) {
    printk(KERN_WARNING "%s: Read: error encountered  eofe(%u), fifoError(%u), lengthError(%u) lane(%u) vc(%u)\n",
        MOD_NAME,
        pgpDevice->rxQueue[mi][pgpDevice->rxRead[mi]]->eofe,
        pgpDevice->rxQueue[mi][pgpDevice->rxRead[mi]]->fifoError,
        pgpDevice->rxQueue[mi][pgpDevice->rxRead[mi]]->lengthError,
        pgpDevice->rxQueue[mi][pgpDevice->rxRead[mi]]->lane,
        pgpDevice->rxQueue[mi][pgpDevice->rxRead[mi]]->vc);
  }

  if ( pgpDevice->debug & 2 ) {
    printk(KERN_DEBUG "%s: Read: Check if user buffer too short\n", MOD_NAME);
  }

  // User buffer is short
  if ( maxSize < pgpDevice->rxQueue[mi][pgpDevice->rxRead[mi]]->length ) {
    printk(KERN_WARNING"%s: Read: user buffer is too small. Rx=%i, User=%i. Maj=%i\n",
        MOD_NAME, pgpDevice->rxQueue[mi][pgpDevice->rxRead[mi]]->length, maxSize, pgpDevice->major);
    copyLength = maxSize;
    pgpDevice->rxQueue[mi][pgpDevice->rxRead[mi]]->lengthError |= 1;
  }
  else copyLength = pgpDevice->rxQueue[mi][pgpDevice->rxRead[mi]]->length;

  // Copy to user
  if ( (i=copy_to_user(dp, pgpDevice->rxQueue[mi][pgpDevice->rxRead[mi]]->buffer, copyLength*sizeof(__u32))) ) {
    if (pgpDevice->rxCopyToUserPrintCount++ < 12) printk(KERN_WARNING"%s: Read: failed to copy %d out of %u to user. Maj=%i\n",
        MOD_NAME, i, (unsigned int)(copyLength*sizeof(__u32)), pgpDevice->major);
    ret =  ERROR;
  }
  else {
    ret = copyLength;

    // Copy associated data
    if (largeMemoryModel) {
      p64->rxSize    = pgpDevice->rxQueue[mi][pgpDevice->rxRead[mi]]->length;
      p64->eofe      = pgpDevice->rxQueue[mi][pgpDevice->rxRead[mi]]->eofe;
      p64->fifoErr   = pgpDevice->rxQueue[mi][pgpDevice->rxRead[mi]]->fifoError;
      p64->lengthErr = pgpDevice->rxQueue[mi][pgpDevice->rxRead[mi]]->lengthError;
      p64->pgpLane   = pgpDevice->rxQueue[mi][pgpDevice->rxRead[mi]]->lane;
      p64->pgpVc     = pgpDevice->rxQueue[mi][pgpDevice->rxRead[mi]]->vc;
      if ( pgpDevice->debug & 2 ) {
        printk(KERN_DEBUG"%s: Read: Words=%i, Lane=%i, VC=%i, Eofe=%i, FifoErr=%i, LengthErr=%i, Addr=%p, Map=%p, Maj=%i\n",
            MOD_NAME, p64->rxSize, p64->pgpLane, p64->pgpVc, p64->eofe,
            p64->fifoErr, p64->lengthErr, (pgpDevice->rxQueue[mi][pgpDevice->rxRead[mi]]->buffer),
            (void*)(pgpDevice->rxQueue[mi][pgpDevice->rxRead[mi]]->dma),(unsigned)pgpDevice->major);
      }
    } else {
      p32->rxSize    = pgpDevice->rxQueue[mi][pgpDevice->rxRead[mi]]->length;
      p32->eofe      = pgpDevice->rxQueue[mi][pgpDevice->rxRead[mi]]->eofe;
      p32->fifoErr   = pgpDevice->rxQueue[mi][pgpDevice->rxRead[mi]]->fifoError;
      p32->lengthErr = pgpDevice->rxQueue[mi][pgpDevice->rxRead[mi]]->lengthError;
      p32->pgpLane   = pgpDevice->rxQueue[mi][pgpDevice->rxRead[mi]]->lane;
      p32->pgpVc     = pgpDevice->rxQueue[mi][pgpDevice->rxRead[mi]]->vc;
      if ( pgpDevice->debug & 2 ) {
        printk(KERN_DEBUG"%s: Read: Words=%i, Lane=%i, VC=%i, Eofe=%i, FifoErr=%i, LengthErr=%i, Addr=%p, Map=%p, Maj=%i\n",
            MOD_NAME, p32->rxSize, p32->pgpLane, p32->pgpVc, p32->eofe,
            p32->fifoErr, p32->lengthErr, (pgpDevice->rxQueue[mi][pgpDevice->rxRead[mi]]->buffer),
            (void*)(pgpDevice->rxQueue[mi][pgpDevice->rxRead[mi]]->dma),(unsigned)pgpDevice->major);
      }
    }

    // Copy command structure to user space
    if ( (i=copy_to_user(buffer, buf, count)) ) {
      if (pgpDevice->rxCopyToUserPrintCount++ < 12) printk(KERN_WARNING "%s: Read: failed to copy %u bytes of command structure to user(%p) space. Maj=%i\n",
          MOD_NAME,
          i,
          buffer,
          pgpDevice->major);
      ret =  ERROR;
    }
  }

  spin_lock_irq(pgpDevice->rxLock);
  countRxBuffers(pgpDevice, 1);
  // Return entry to RX queue
   pgpDevice->reg->rxFree = pgpDevice->rxQueue[mi][pgpDevice->rxRead[mi]]->dma;
  // Increment read pointer
  pgpDevice->rxRead[mi] = (pgpDevice->rxRead[mi] + 1) % (NUMBER_OF_RX_CLIENT_BUFFERS);
  spin_unlock_irq(pgpDevice->rxLock);

  if ( pgpDevice->debug & 2 ) {
    printk(KERN_DEBUG"%s: Read: Added buffer %.8x to RX queue. Maj=%i\n",
      MOD_NAME,(__u32)(pgpDevice->rxQueue[mi][pgpDevice->rxRead[mi]]->dma),pgpDevice->major);
  }

  spin_unlock(&(pgpDevice->readLock[mi]));
  return(ret);
}


// IRQ Handler
static irqreturn_t PgpCard_IRQHandler(int irq, void *dev_id, struct pt_regs *regs) {
  __u32        stat;
  __u32        descA;
  __u32        descB;
  __u32        idx;
  __u32        next;
  __u32        lane;
  __u32        vc;
  __u32        bfcnt;
  __u32        i;
  __u32        mi;  // minor index into open client list
  __u32        rxLoopCount;
  irqreturn_t ret;

  struct PgpDevice *pgpDevice = (struct PgpDevice *)dev_id;
  // Read IRQ Status
  stat = pgpDevice->reg->irq;

  // Is this the source
  if ( (stat & 0x2) != 0 ) {
    if ( pgpDevice->debug & 1 ) {
      printk(KERN_DEBUG"%s: Irq: IRQ Called. Maj=%i\n", MOD_NAME,pgpDevice->major);
    }
    // Disable interrupts
    pgpDevice->reg->irq = 0;
    if (pgpDevice->interruptNesting) {
      printk(KERN_WARNING "%s: Irq: Nested interrupt detection!!!!! %u\n", MOD_NAME, pgpDevice->interruptNesting);
    } else {
      pgpDevice->interruptNesting += 1;
    }
    // Read Tx completion status
    stat = pgpDevice->reg->txStatus;
    // Tx Data is ready
    if ( (stat & 0x00000400) != 0 ) {
      do {
        // Read dma value
        stat = pgpDevice->reg->txReturn;
        if ( pgpDevice->debug & 1 ) {
          printk(KERN_DEBUG"%s: Irq: Return TX Status Value %.8x. Maj=%i\n", MOD_NAME, stat, pgpDevice->major);
        }
        // Find TX buffer entry
        for ( idx=0; idx < NUMBER_OF_TX_BUFFERS; idx++ ) {
          if ( pgpDevice->txBuffer[idx]->dma == (stat & 0xFFFFFFFC) ) break;
        }
        // Entry was found

        if ( idx < NUMBER_OF_TX_BUFFERS ) {
          // Return to queue
          pgpDevice->txBuffer[idx]->allocated = 0;
          pgpDevice->txBufferCount = countTxBuffers(pgpDevice);
          // Wake up any writers
          mi = 0;
          while (mi < MAX_NUMBER_OPEN_CLIENTS) {
            if ((pgpDevice->goingDown & pgpDevice->client[mi].mask) == 0) {
              if (pgpDevice->client[mi].fp != 0) {
                wake_up_interruptible(&(pgpDevice->client[mi].outq));
              }
            }
            mi += 1;
          }
          if ( pgpDevice->debug & 0x10 ) {
            printk(KERN_DEBUG"%s: IRQ: unallocated buffer %u\n", MOD_NAME, idx);
          }
        } else {
          printk(KERN_WARNING"%s: Irq: Failed to locate TX descriptor %.8x. Maj=%i\n", MOD_NAME, (__u32)(stat&0xFFFFFFFC), pgpDevice->major);
        }
        // Repeat while next valid flag is set
      } while ( (stat & 0x2) != 0 );
    }

    // Read Rx completion status
    stat = pgpDevice->reg->rxStatus;
    // Data is ready
    if ( (stat & 0x00000400) != 0 ) {
      rxLoopCount = 0;
      do {
        // Read descriptor
        descA = pgpDevice->reg->rxRead0;   // RxBuffer struct fields from the firmware
        descB = pgpDevice->reg->rxRead1;   // DMA pointer with next valid field in bit 1 from the firmware
        lane = (descA >> 30) & 0x3;
        vc   = (descA >> 28) & 0x3;
        spin_lock_irq(pgpDevice->rxLock);
        bfcnt = countRxBuffers(pgpDevice, 0);
        spin_unlock_irq(pgpDevice->rxLock);
        countRXFirmwareBuffers(pgpDevice, 1);
        rxLoopCount += 1;
        pgpDevice->rxLaneHisto[lane][vc] += 1;
        // Find RX buffer entry
        for ( idx=0; idx < NUMBER_OF_RX_CLIENT_BUFFERS; idx++ ) {
          if ( pgpDevice->rxBuffer[idx]->dma == (descB & 0xFFFFFFFC) ) break;
        }
        // Entry was found
        if ( idx < NUMBER_OF_RX_CLIENT_BUFFERS ) {
          mi = 0;
          while (mi < NUMBER_OF_LANE_CLIENTS) {
            if ((pgpDevice->client[mi].mask & (1 << lane)) &&
                (pgpDevice->client[mi].vcMask & (1 << vc))) {
              break;
            }
            mi++;
          }
          // If device is open ...
          if ( mi < NUMBER_OF_LANE_CLIENTS ) {
            // Drop data if too few buffers left
            if (bfcnt > MAXIMUM_RX_CLIENT_BUFFERS) {
              pgpDevice->reg->rxFree = (descB & 0xFFFFFFFC);
              pgpDevice->rxTossedBuffers[mi] += 1;
              pgpDevice->rxTotalTossedBuffers += 1;
            } else {
              next = (pgpDevice->rxWrite[mi]+1) % (NUMBER_OF_RX_CLIENT_BUFFERS);
              if ( next == pgpDevice->rxRead[mi] ) {
                printk(KERN_WARNING"%s: Irq: Rx queue pointer collision, discarding input. Maj=%i lane=%u client=%u\n",
                    MOD_NAME, pgpDevice->major, lane, mi);
                pgpDevice->reg->rxFree = (descB & 0xFFFFFFFC);
              } else {
                // Set descriptor into rxBuffer
                pgpDevice->rxBuffer[idx]->lane        = lane;
                pgpDevice->rxBuffer[idx]->vc          = vc;
                pgpDevice->rxBuffer[idx]->lengthError = (descA >> 26) & 0x1;
                pgpDevice->rxBuffer[idx]->fifoError   = (descA >> 25) & 0x1;
                pgpDevice->rxBuffer[idx]->eofe        = (descA >> 24) & 0x1;
                pgpDevice->rxBuffer[idx]->length      = descA & 0x00FFFFFF;

                if ( pgpDevice->debug & 1 ) {
                  printk(KERN_DEBUG "%s: IRQ: mi=%u, Rx Words=%i, Lane=%i, VC=%i, Eofe=%i, FifoErr=%i, LengthErr=%i, Addr=%p, Map=%p\n",
                      MOD_NAME, mi, pgpDevice->rxBuffer[idx]->length, pgpDevice->rxBuffer[idx]->lane, pgpDevice->rxBuffer[idx]->vc,
                      pgpDevice->rxBuffer[idx]->eofe, pgpDevice->rxBuffer[idx]->fifoError, pgpDevice->rxBuffer[idx]->lengthError,
                      (pgpDevice->rxBuffer[idx]->buffer), (void*)(pgpDevice->rxBuffer[idx]->dma));
                  printk(KERN_DEBUG "%s: Irq: rxWrite=%u\n", MOD_NAME, pgpDevice->rxWrite[mi]);
                  printk(KERN_DEBUG "%s: Irq: inq=%p\n", MOD_NAME, (void*)&(pgpDevice->client[mi].inq));
                  printk(KERN_DEBUG "%s: Irq: rxQ=%p\n", MOD_NAME, (void*)pgpDevice->rxQueue);
                  printk(KERN_DEBUG "%s: Irq: rxQ[mi]=%p\n", MOD_NAME, (void*)pgpDevice->rxQueue[mi]);
                  if (pgpDevice->rxQueue[mi] != 0) {
                    printk(KERN_DEBUG "%s: Irq: rxQ[mi][rxW]=%p\n", MOD_NAME, (void*)pgpDevice->rxQueue[mi][pgpDevice->rxWrite[mi]]);
                  }
                }
                if (pgpDevice->rxQueue[mi] != 0) {
                  // Store to Queue
                  pgpDevice->rxQueue[mi][pgpDevice->rxWrite[mi]] = pgpDevice->rxBuffer[idx];
                  spin_lock_irq(pgpDevice->rxLock);
                  pgpDevice->rxWrite[mi] = next;
                  spin_unlock_irq(pgpDevice->rxLock);
                  // Wake up the correct reader
                  if ( &(pgpDevice->client[mi].inq) ) {
                    if ((pgpDevice->goingDown & pgpDevice->client[mi].mask)  == 0) {
                      wake_up_interruptible(&(pgpDevice->client[mi].inq));
                    } else {
                      printk(KERN_WARNING "%s: Irq: not waking, going down %u mi=%u\n", MOD_NAME, pgpDevice->goingDown, mi);
                      pgpDevice->reg->rxFree = (descB & 0xFFFFFFFC);
                    }
                  } else {
                    printk(KERN_WARNING "%s: Irq: poll queue zero!! mi=%u\n", MOD_NAME, mi);
                    pgpDevice->reg->rxFree = (descB & 0xFFFFFFFC);
                  }
                } else {
                  printk(KERN_WARNING "%s: Irq: rxQ[%u] is ZERO!\n", MOD_NAME, mi);
                  pgpDevice->reg->rxFree = (descB & 0xFFFFFFFC);
                }
              }
            }
          } else {
            if ((((i=(++(pgpDevice->noClientPacketCount[lane])))%1000) == 0) &&
                (i > pgpDevice->noClientPacketMax) ) {
              pgpDevice->noClientPacketMax = i;
              printk(KERN_WARNING"%s: Irq: packet count for device not open in lane order:%8u%8u%8u%8u\n",
                  MOD_NAME,pgpDevice->noClientPacketCount[0],pgpDevice->noClientPacketCount[1],
                  pgpDevice->noClientPacketCount[2],pgpDevice->noClientPacketCount[3]);
            }
            pgpDevice->reg->rxFree = (descB & 0xFFFFFFFC);
          }
        } else {
          printk(KERN_WARNING "%s: Irq: Failed to locate RX descriptor %.8x. Maj=%i\n",MOD_NAME,(__u32)(descA&0xFFFFFFFC),pgpDevice->major);
          pgpDevice->reg->rxFree = (descB & 0xFFFFFFFC);
        }
        // Repeat while next valid flag is set
      } while ((descB & 0x2) != 0);
      if (rxLoopCount > 1) {
        if (--rxLoopCount < NUMBER_OF_RX_BUFFERS ) pgpDevice->rxLoopHisto[rxLoopCount] += 1;
        else pgpDevice->rxLoopHisto[NUMBER_OF_RX_BUFFERS-1] += 1;
        if (rxLoopCount >= NUMBER_OF_RX_BUFFERS-1) {
          printk(KERN_WARNING"%s: Irq: rxLoopCount %u\n", MOD_NAME, rxLoopCount);
        }
      }
    }
    // Enable interrupts
    if ( pgpDevice->debug & 1 ) {
      printk(KERN_DEBUG"%s: Irq: Done. Maj=%i\n", MOD_NAME,pgpDevice->major);
    }
    pgpDevice->interruptNesting -= 1;
    pgpDevice->reg->irq = 1;
    ret = IRQ_HANDLED;
  }
  else ret = IRQ_NONE;
  return(ret);
}


// Poll/Select
static __u32 PgpCard_Poll(struct file *filp, poll_table *wait ) {
  int i;
  __u32 mask    = 0;
  __u32 readOk  = 0;
  __u32 writeOk = 0;
  __u32 found = 0;
  __u32 mi = 0;  // minor index into open client list

  struct PgpDevice *pgpDevice = (struct PgpDevice *)filp->private_data;

  spin_lock(pgpDevice->pollLock);
  if (pgpDevice->pollEnabled) {
    for (i=0; i<MAX_NUMBER_OPEN_CLIENTS; i++) {
      if (pgpDevice->client[i].fp == filp) {
        found = 1;
        mi = i;
        break;
      }
    }

    if (found) {
      if ( pgpDevice->debug & 8 ) {
        printk(KERN_DEBUG"%s: Poll: Maj=%i Min=%u ", MOD_NAME, pgpDevice->major, pgpDevice->client[mi].mask);
      }
      poll_wait(filp, &(pgpDevice->client[mi].inq), wait);
      poll_wait(filp, &(pgpDevice->client[mi].outq), wait);

      if ( pgpDevice->rxWrite[mi] != pgpDevice->rxRead[mi] ) {
        mask |= POLLIN | POLLRDNORM; // Readable
        readOk = 1;
      }
      spin_lock_irq(pgpDevice->txLockIrq);
      if ( (pgpDevice->txBufferCount = countTxBuffers(pgpDevice)) < NUMBER_OF_TX_BUFFERS ) {
        mask |= POLLOUT | POLLWRNORM; // Writable
        writeOk = 1;
      }
      spin_unlock_irq(pgpDevice->txLockIrq);
    } else {
      printk(KERN_WARNING "%s: Poll: FAILED because this file pointer was not found to be open\n",
          MOD_NAME);
    }
    if ( pgpDevice->debug & 8 ) {
      printk(KERN_DEBUG" ReadOk=%i, WriteOk=%i\n", readOk, writeOk);
    }
  } else {
    mask |= POLLOUT | POLLWRNORM;
  }
  spin_unlock(pgpDevice->pollLock);
  return(mask);
}


// Probe device
static int PgpCard_Probe(struct pci_dev *pcidev, const struct pci_device_id *dev_id) {
  int i, j, res, idx, ret;
  dev_t chrdev = 0;
  struct PgpDevice *pgpDevice;
  struct pci_device_id *id = (struct pci_device_id *) dev_id;

  // We keep device instance number in id->driver_data
  id->driver_data = -1;

  // Find empty structure
  for (i = 0; i < MAX_PCI_DEVICES; i++) {
    if (gPgpDevices[i].baseHdwr == 0) {
      id->driver_data = i;
      break;
    }
  }

  // Overflow
  if (id->driver_data < 0) {
    printk(KERN_WARNING "%s: Probe: Too Many Devices.\n", MOD_NAME);
    return -EMFILE;
  }
  pgpDevice = &gPgpDevices[id->driver_data];

  // Allocate device numbers for character device.
  res = alloc_chrdev_region(&chrdev, 0, NUMBER_OF_MINOR_DEVICES, MOD_NAME);
  if (res < 0) {
    printk(KERN_WARNING "%s: Probe: Cannot register char device\n", MOD_NAME);
    return res;
  }

  // Init device
  cdev_init(&pgpDevice->cdev, &PgpCard_Intf);

  // Initialize device structure
  pgpDevice->major = MAJOR(chrdev);
  pgpDevice->readLock = vmalloc(NUMBER_OF_LANE_CLIENTS * sizeof(spinlock_t));
  for (i=0; i<MAX_NUMBER_OPEN_CLIENTS; i++) {
    pgpDevice->client[i].mask = 0;
    pgpDevice->client[i].fp = 0;
    if (i<NUMBER_OF_LANE_CLIENTS) {
      pgpDevice->rxTossedBuffers[i] = 0;
      spin_lock_init(&(pgpDevice->readLock[i]));
    }
  }
  pgpDevice->isOpen = 0;
  pgpDevice->openCount = 0;
  pgpDevice->cdev.owner    = THIS_MODULE;
  pgpDevice->cdev.ops      = &PgpCard_Intf;
  pgpDevice->debug         = 0;
  pgpDevice->rxTotalBufferCount = 0;
  pgpDevice->rxTotalTossedBuffers = 0;
  pgpDevice->rxCopyToUserPrintCount = 0;

  pgpDevice->status = (PgpCardStatus*) vmalloc(sizeof(PgpCardStatus));

  // Add device
  if ( cdev_add(&pgpDevice->cdev, chrdev, NUMBER_OF_MINOR_DEVICES) )
    printk(KERN_WARNING "%s: Probe: Error cdev_adding device Maj=%i with %u devices\n", MOD_NAME,pgpDevice->major, NUMBER_OF_MINOR_DEVICES);

  // Enable devices
  ret = pci_enable_device(pcidev);
  if (ret) {
    printk(KERN_WARNING "%s: pci_enable_device() returned %d, Maj %i\n", MOD_NAME, ret, pgpDevice->major);
  }

  // Get Base Address of registers from pci structure.
  pgpDevice->baseHdwr = pci_resource_start (pcidev, 0);
  pgpDevice->baseLen  = pci_resource_len (pcidev, 0);

  // Remap the I/O register block so that it can be safely accessed.
  pgpDevice->reg = (struct PgpCardReg *)ioremap_nocache(pgpDevice->baseHdwr, pgpDevice->baseLen);
  if (! pgpDevice->reg ) {
    printk(KERN_WARNING"%s: Init: Could not remap memory Maj=%i.\n", MOD_NAME,pgpDevice->major);
    return (ERROR);
  }

  // Try to gain exclusive control of memory
  if (check_mem_region(pgpDevice->baseHdwr, pgpDevice->baseLen) < 0 ) {
    printk(KERN_WARNING"%s: Init: Memory in use Maj=%i.\n", MOD_NAME,pgpDevice->major);
    return (ERROR);
  }

  // Remove card reset, bit 1 of control register
  pgpDevice->reg->control &= 0xFFFFFFFD;

  request_mem_region(pgpDevice->baseHdwr, pgpDevice->baseLen, MOD_NAME);
  printk(KERN_INFO "%s: Probe: Found card. Version=0x%x, Maj=%i\n", MOD_NAME,pgpDevice->reg->version,pgpDevice->major);

  // Get IRQ from pci_dev structure.
  pgpDevice->irq = pcidev->irq;
  printk(KERN_INFO "%s: Init: IRQ %d Maj=%i\n", MOD_NAME, pgpDevice->irq,pgpDevice->major);

  // Request IRQ from OS.
  if (request_irq( pgpDevice->irq, PgpCard_IRQHandler, IRQF_SHARED, MOD_NAME, (void*)pgpDevice) < 0 ) {
    printk(KERN_WARNING"%s: Init: Unable to allocate IRQ. Maj=%i",MOD_NAME,pgpDevice->major);
    return (ERROR);
  }

  // Init TX Buffers
  pgpDevice->txBuffer   = (struct TxBuffer **) vmalloc(NUMBER_OF_TX_BUFFERS * sizeof(struct TxBuffer *));

  for ( idx=0; idx < NUMBER_OF_TX_BUFFERS; idx++ ) {
    pgpDevice->txBuffer[idx] = (struct TxBuffer *) vmalloc(sizeof(struct TxBuffer ));
    pgpDevice->txBuffer[idx]->allocated = 0;
    if ((pgpDevice->txBuffer[idx]->buffer = pci_alloc_consistent(pcidev, DEF_TX_BUF_SIZE, &(pgpDevice->txBuffer[idx]->dma))) == NULL ) {
      printk(KERN_WARNING"%s: Init: unable to allocate tx buffer. Maj=%i\n",MOD_NAME,pgpDevice->major);
      return ERROR;
    }
  }
  pgpDevice->txHisto = (__u32*)vmalloc(NUMBER_OF_TX_BUFFERS * sizeof(__u32));
  for ( idx=0; idx < NUMBER_OF_TX_BUFFERS; idx++ ) {
    pgpDevice->txHisto[idx] = 0;
  }
  pgpDevice->txHistoLV = (__u32*)vmalloc(NUMBER_OF_LANES*NUMBER_OF_VC*sizeof(__u32));
  for ( idx=0; idx < NUMBER_OF_LANES; idx++ ) {
    for (i= 0; i < NUMBER_OF_VC; i++) {
      pgpDevice->txHistoLV[idx*NUMBER_OF_VC+i] = 0;
    }
  }
  printk(KERN_DEBUG"%s: size of pgpDevice %u\n", MOD_NAME, (unsigned int)sizeof(struct PgpDevice));
  pgpDevice->txRead  = 0;
  pgpDevice->txBufferCount = 0;
  pgpDevice->interruptNesting = 0;
  pgpDevice->rxBufferCount = 0;
  for (i=0; i<NUMBER_OF_LANES; i++) {
    pgpDevice->noClientPacketCount[i] = 0;
  }
  pgpDevice->noClientPacketMax = 0;

  // Set max frame size, clear rx buffer reset
  pgpDevice->reg->rxMaxFrame = DEF_RX_BUF_SIZE | 0x80000000;

  // allocate variousn Buffers
  pgpDevice->rxBuffer   = (struct RxBuffer **) vmalloc(NUMBER_OF_RX_BUFFERS * sizeof(struct RxBuffer *));
  pgpDevice->rxBuffersHisto = (__u32*)vmalloc((NUMBER_OF_RX_BUFFERS<<1) * sizeof(__u32));
  pgpDevice->rxHisto = (__u32*)vmalloc(NUMBER_OF_RX_CLIENT_BUFFERS * sizeof(__u32));
  pgpDevice->rxLoopHisto = (__u32*)vmalloc(NUMBER_OF_RX_BUFFERS * sizeof(__u32));

  for ( i=0; i < NUMBER_OF_RX_BUFFERS; i++ ) {
    pgpDevice->rxLoopHisto[i] = 0;
  }
  for ( i=0; i < NUMBER_OF_LANES; i++ ) {
    for (j=0; j < NUMBER_OF_VC; j++) {
      pgpDevice->rxLaneHisto[i][j] = 0;
    }
  }
  for ( i=0; i < NUMBER_OF_RX_BUFFERS<<1; i++ ) {
    pgpDevice->rxBuffersHisto[i] = 0;
  }
  for ( idx=0; idx < NUMBER_OF_RX_CLIENT_BUFFERS; idx++ ) {
    pgpDevice->rxHisto[idx] = 0;
  }
  for ( idx=0; idx < NUMBER_OF_RX_BUFFERS; idx++ ) {
    pgpDevice->rxBuffer[idx] = (struct RxBuffer *) vmalloc(sizeof(struct RxBuffer ));
    if ((pgpDevice->rxBuffer[idx]->buffer = pci_alloc_consistent(pcidev, DEF_RX_BUF_SIZE, &(pgpDevice->rxBuffer[idx]->dma))) == NULL ) {
      printk(KERN_WARNING"%s: Init: unable to allocate rx buffer. Maj=%i\n",MOD_NAME,pgpDevice->major);
      return ERROR;
    };

    // Add to RX queue
    pgpDevice->reg->rxFree = pgpDevice->rxBuffer[idx]->dma;
  }

  // Init queues
  for (i=0; i<NUMBER_OF_LANE_CLIENTS; i++) {
    pgpDevice->rxQueue[i]    = (struct RxBuffer **) vmalloc((NUMBER_OF_RX_CLIENT_BUFFERS) * sizeof(struct RxBuffer *));
    if ( pgpDevice->debug & 1 ) {
      printk(KERN_DEBUG "%s: Probe: rxQ[%u]=%p\n", MOD_NAME, i, (void*)pgpDevice->rxQueue[i]);
    }
    pgpDevice->rxRead[i]  = 0;
    pgpDevice->rxWrite[i] = 0;
    init_waitqueue_head(&pgpDevice->client[i].inq);
    init_waitqueue_head(&pgpDevice->client[i].outq);
    pgpDevice->client[i].mask = 0;
    if (pgpDevice->debug & 0x40) {
      printk(KERN_DEBUG "%s: Probe: client %u inq=%p outq=%p\n", MOD_NAME, i, &pgpDevice->client[i].inq, &pgpDevice->client[i].outq);
    }
  }
  // Write scratchpad
  pgpDevice->reg->scratch = SPAD_WRITE;

  if (pgpDevice->debug & 0x40) {
    printk(KERN_DEBUG "%s: Probe: sizeof(spinlock_t)=%u\n", MOD_NAME, (unsigned int)(long unsigned int)sizeof(spinlock_t));
  }
  pgpDevice->rxLock      = vmalloc(sizeof(spinlock_t));
  pgpDevice->txLock      = vmalloc(sizeof(spinlock_t));
  pgpDevice->txLockIrq   = vmalloc(sizeof(spinlock_t));
  pgpDevice->ioctlLock   = vmalloc(sizeof(spinlock_t));
  pgpDevice->releaseLock = vmalloc(sizeof(spinlock_t));
  pgpDevice->pollLock    = vmalloc(sizeof(spinlock_t));
  spin_lock_init(pgpDevice->rxLock);
  spin_lock_init(pgpDevice->txLock);
  spin_lock_init(pgpDevice->txLockIrq);
  spin_lock_init(pgpDevice->ioctlLock);
  spin_lock_init(pgpDevice->releaseLock);
  spin_lock_init(pgpDevice->pollLock);

  // Enable interrupts
  pgpDevice->reg->irq = 1;

  printk(KERN_INFO"%s: Init: Driver is loaded. Maj=%i %s\n", MOD_NAME,pgpDevice->major, PGPCARD_VERSION);
  return SUCCESS;
}


// Remove
static void PgpCard_Remove(struct pci_dev *pcidev) {
  __u32 idx;
  int  i;
  struct PgpDevice *pgpDevice = NULL;

  // Look for matching device
  for (i = 0; i < MAX_PCI_DEVICES; i++) {
    if ( gPgpDevices[i].baseHdwr == pci_resource_start(pcidev, 0)) {
      pgpDevice = &gPgpDevices[i];
      break;
    }
  }

  // Device not found
  if (pgpDevice == NULL) {
    printk(KERN_WARNING "%s: Remove: Device Not Found.\n", MOD_NAME);
  }
  else {

    // Disable interrupts
    pgpDevice->reg->irq = 0;

    // Clear RX buffer
    pgpDevice->reg->rxMaxFrame = 0;

    if (pgpDevice->status) {
      vfree(pgpDevice->status);
    }

    // Free TX Buffers
    for ( idx=0; idx < NUMBER_OF_TX_BUFFERS; idx++ ) {
      pci_free_consistent( pcidev,DEF_TX_BUF_SIZE, pgpDevice->txBuffer[idx]->buffer, pgpDevice->txBuffer[idx]->dma);
      if (pgpDevice->txBuffer[idx]) {
        vfree(pgpDevice->txBuffer[idx]);
      }
    }
    if (pgpDevice->txBuffer) {
      vfree(pgpDevice->txBuffer);
    }
    if (pgpDevice->txHisto) {
      vfree(pgpDevice->txHisto);
    }

    // Free RX Buffers
    if (pgpDevice->rxHisto) {
      vfree(pgpDevice->rxHisto);
    }
    for ( idx=0; idx < NUMBER_OF_RX_BUFFERS; idx++ ) {
      pci_free_consistent( pcidev, DEF_RX_BUF_SIZE, pgpDevice->rxBuffer[idx]->buffer, pgpDevice->rxBuffer[idx]->dma);
      if (pgpDevice->rxBuffer[idx]) {
        vfree(pgpDevice->rxBuffer[idx]);
      }
    }
    if (pgpDevice->rxBuffer) {
      vfree(pgpDevice->rxBuffer);
    }
    for (i=0; i<NUMBER_OF_LANE_CLIENTS; i++) {
      if (pgpDevice->rxQueue[i]) {
        vfree(pgpDevice->rxQueue[i]);
      }
    }
    if (pgpDevice->rxLoopHisto) {
      vfree(pgpDevice->rxLoopHisto);
    }
    if (pgpDevice->txHistoLV) {
      vfree(pgpDevice->txHistoLV);
    }

    // Set card reset, bit 1 of control register
    pgpDevice->reg->control |= 0x00000002;

    // Release memory region
    release_mem_region(pgpDevice->baseHdwr, pgpDevice->baseLen);

    // Release IRQ
    free_irq(pgpDevice->irq, pgpDevice);

    // Unmap
    iounmap(pgpDevice->reg);

    // Unregister Device Driver
    cdev_del(&pgpDevice->cdev);
    unregister_chrdev_region(MKDEV(pgpDevice->major,0), NUMBER_OF_MINOR_DEVICES);

    // Disable device
    pci_disable_device(pcidev);
    pgpDevice->baseHdwr = 0;
    printk(KERN_INFO"%s: Remove: %s is unloaded. Maj=%i\n", MOD_NAME, PGPCARD_VERSION, pgpDevice->major);
  }
}


// Init Kernel Module
static int PgpCard_Init(void) {
  int ret = 0;
  /* Allocate and clear memory for all devices. */
  memset(gPgpDevices, 0, sizeof(struct PgpDevice)*MAX_PCI_DEVICES);

  // Register driver
  ret = pci_register_driver(&PgpCardDriver);

  printk(KERN_INFO"%s: Init: Register driver returned %d\n", MOD_NAME, ret);

  return(ret);
}


// Exit Kernel Module
static void PgpCard_Exit(void) {
  printk(KERN_INFO"%s: Exit.\n", MOD_NAME);
  pci_unregister_driver(&PgpCardDriver);
}

unsigned countRXFirmwareBuffers(struct PgpDevice* pgpDevice, __u32 update) {
  unsigned hbcnt = 0;
  unsigned tmp;
  tmp = pgpDevice->reg->rxStatus;
  hbcnt = (tmp >> 16)&0x3FF;
  if ( (tmp >> 29)&0x1 ) hbcnt++;
  hbcnt += (tmp & 0x3FF);
  if ( (tmp >> 10)&1 ) hbcnt++;
  if (update) {
    if ((hbcnt) < NUMBER_OF_RX_BUFFERS<<1) {
      pgpDevice->rxBuffersHisto[hbcnt] += 1;
    } else {
      pgpDevice->rxBuffersHisto[(NUMBER_OF_RX_BUFFERS<<1)-1] += 1;
    }
  }
  return hbcnt;
}

unsigned countRxBuffers(struct PgpDevice* pgpDevice, __u32 update) {
  unsigned mi;
  unsigned bcnt = 0;
  for (mi=0; mi<NUMBER_OF_LANE_CLIENTS; mi++) {
    if ( pgpDevice->rxRead[mi] > pgpDevice->rxWrite[mi] )
      bcnt += (__u32)((NUMBER_OF_RX_CLIENT_BUFFERS + pgpDevice->rxWrite[mi]) - pgpDevice->rxRead[mi]);
    else {
      bcnt += (pgpDevice->rxWrite[mi] - pgpDevice->rxRead[mi]);
    }
  }
  if (update) {
    pgpDevice->rxBufferCount = bcnt;
    if (bcnt < NUMBER_OF_RX_CLIENT_BUFFERS) {
        pgpDevice->rxHisto[bcnt] += 1;
     } else {
        pgpDevice->rxHisto[NUMBER_OF_RX_CLIENT_BUFFERS-1] += 1;
     }
  }
  return bcnt;
}

unsigned countTxBuffers(struct PgpDevice* pgpDevice) {
  unsigned bcnt = 0;
  unsigned mi;
  for (mi=0; mi<NUMBER_OF_TX_BUFFERS; mi++) {
    if (pgpDevice->txBuffer[mi]->allocated) {
      bcnt += 1;
    }
  }
  pgpDevice->txBufferCount = bcnt;
  return bcnt;
}

// PgpCard_Ioctl
// Called when ioctl is called on the device
// Returns success.
int PgpCard_Ioctl(struct inode *inode, struct file *filp, __u32 cmd, unsigned long arg) {
  printk(KERN_WARNING "%s: warning Ioctl is deprecated and no longer supported\n", MOD_NAME);
  return SUCCESS;
}

int my_Ioctl(struct file *filp, __u32 cmd, __u64 argument) {
  int i, j;
  unsigned mi;
  PgpCardStatus *stat = 0;
  __u32          tmp;
  __u32          tmp1;
  __u32          mask;
  __u32          lane;
  __u32          vcMask;
  unsigned       bfcnt;
  __u32          read = 0;
  __u32          arg = argument & 0xffffffffLL;
  __u32          txSum = 0;
  __u32          rxSum = 0;
  __u32          rem;
  unsigned       upperLimit = 0;
  char           s1[40];

  struct PgpDevice *pgpDevice = (struct PgpDevice *)filp->private_data;
  /*if (pgpDevice->debug & 2)*/ printk(KERN_DEBUG "%s: entering my_Ioctl, cmd %u, arg(0x%llx)\n", MOD_NAME, cmd, argument);
  stat = pgpDevice->status;
  // Determine command
  spin_lock(pgpDevice->ioctlLock);
  switch ( cmd ) {

    // Write scratchpad
    case IOCTL_Write_Scratch:
      pgpDevice->reg->scratch = arg;
      printk(KERN_WARNING "%s: Scratch set to 0x%x\n", MOD_NAME, arg);
      spin_unlock(pgpDevice->ioctlLock);
      return(SUCCESS);
      break;

    // Set Debug
    case IOCTL_Set_Debug:
      pgpDevice->debug = arg;
      printk(KERN_WARNING "%s: debug set to 0x%x\n", MOD_NAME, arg);
      spin_unlock(pgpDevice->ioctlLock);
      return(SUCCESS);
      break;

      // Count Reset
    case IOCTL_Count_Reset:
      pgpDevice->reg->control |= 0x1;
      pgpDevice->reg->control &= 0xFFFFFFFE;
      for ( i=0; i < NUMBER_OF_RX_CLIENT_BUFFERS; i++ ) {
        pgpDevice->rxHisto[i] = 0;
      }
      for ( i=0; i < NUMBER_OF_RX_BUFFERS; i++ ) {
        pgpDevice->rxLoopHisto[i] = 0;
      }
      for ( i=0; i < NUMBER_OF_TX_BUFFERS; i++ ) {
        pgpDevice->txHisto[i] = 0;
      }
      for (i=0; i < NUMBER_OF_LANES; i++) {
        for (j=0; j < NUMBER_OF_VC; j++) {
          pgpDevice->txHistoLV[i*NUMBER_OF_VC+j] = 0;
        }
      }
      spin_unlock(pgpDevice->ioctlLock);
      if (pgpDevice->debug & 1) printk(KERN_DEBUG "%s: Count reset\n", MOD_NAME);
      return(SUCCESS);
      break;

      // Set Loopback
    case IOCTL_Set_Loop:
      pgpDevice->reg->control |= ((0x10 << arg) & 0xF0);
      spin_unlock(pgpDevice->ioctlLock);
      if (pgpDevice->debug & 1) printk(KERN_DEBUG "%s: Set loopback for %u\n", MOD_NAME, arg);
      return(SUCCESS);
      break;

      // Clr Loopback
    case IOCTL_Clr_Loop:
      mask = 0xFFFFFFFF ^ ((0x10 << arg) & 0xF0);
      pgpDevice->reg->control &= mask;
      spin_unlock(pgpDevice->ioctlLock);
      if (pgpDevice->debug & 1) printk(KERN_DEBUG "%s: Clr loopback for %u\n", MOD_NAME, arg);
      return(SUCCESS);
      break;

      // Set RX reset
    case IOCTL_Set_Rx_Reset:
      pgpDevice->reg->control |= ((0x100 << arg) & 0xF00);
      spin_unlock(pgpDevice->ioctlLock);
      if (pgpDevice->debug & 1) printk(KERN_DEBUG "%s: Rx reset set for %u\n", MOD_NAME, arg);
      return(SUCCESS);
      break;

      // Clr RX reset
    case IOCTL_Clr_Rx_Reset:
      mask = 0xFFFFFFFF ^ ((0x100 << arg) & 0xF00);
      pgpDevice->reg->control &= mask;
      spin_unlock(pgpDevice->ioctlLock);
      if (pgpDevice->debug & 1) printk(KERN_DEBUG "%s: Rx reset clr for %u\n", MOD_NAME, arg);
      return(SUCCESS);
      break;

      // Set TX reset
    case IOCTL_Set_Tx_Reset:
      pgpDevice->reg->control |= ((0x1000 << arg) & 0xF000);
      spin_unlock(pgpDevice->ioctlLock);
      if (pgpDevice->debug & 1) printk(KERN_DEBUG "%s: Tx reset set for %u\n", MOD_NAME, arg);
      return(SUCCESS);
      break;

      // Clr TX reset
    case IOCTL_Clr_Tx_Reset:
      mask = 0xFFFFFFFF ^ ((0x1000 << arg) & 0xF000);
      pgpDevice->reg->control &= mask;
      spin_unlock(pgpDevice->ioctlLock);
      if (pgpDevice->debug & 1) printk(KERN_DEBUG "%s: Tx reset clr for %u\n", MOD_NAME, arg);
      return(SUCCESS);
      break;

    case IOCTL_Set_VC_Mask:
      i = SUCCESS;
      lane = arg & 3;
      vcMask = (arg & 0xf00) >> 8;
      for (mi=0; mi<NUMBER_OF_LANE_CLIENTS; mi++) {
        if (pgpDevice->client[mi].mask & (1<<lane)) {
          if (pgpDevice->client[mi].vcMask & vcMask) {
            pgpDevice->client[mi].vcMask = vcMask;
            break;
          }
        }
      }
      spin_unlock(pgpDevice->ioctlLock);
      if (mi == NUMBER_OF_LANE_CLIENTS) {
        printk(KERN_WARNING "%s: IOCTL_Set_VC_Mask unable to find matching client lane %u\n", MOD_NAME, lane);
        i = ERROR;
      }
      if (i != ERROR) printk(KERN_DEBUG "%s: set VC mask for client %u on lane %u to 0x%x\n", MOD_NAME, mi, lane, vcMask);
      return(i);
      break;

    case IOCTL_Show_Version:
      printk(KERN_WARNING "%s: %s\n",  MOD_NAME, PGPCARD_VERSION);
      spin_unlock(pgpDevice->ioctlLock);
      return(SUCCESS);
      break;

      // Status read
    case IOCTL_Read_Status:
      if (pgpDevice->debug & 1) printk(KERN_DEBUG "%s IOCTL_ReadStatus 1\n", MOD_NAME);

      // Read Values
      stat->Version = pgpDevice->reg->version;
      stat->ScratchPad = pgpDevice->reg->scratch;

      tmp = pgpDevice->reg->pciStat[0];
      stat->PciCommand = (tmp >> 16)&0xFFFF;
      stat->PciStatus  = tmp & 0xFFFF;

      tmp = pgpDevice->reg->pciStat[1];
      stat->PciDCommand = (tmp >> 16)&0xFFFF;
      stat->PciDStatus  = tmp & 0xFFFF;

      tmp = pgpDevice->reg->pciStat[2];
      stat->PciLCommand = (tmp >> 16)&0xFFFF;
      stat->PciLStatus  = tmp & 0xFFFF;

      tmp = pgpDevice->reg->pciStat[3];
      stat->PciLinkState = (tmp >> 24)&0x7;
      stat->PciFunction  = (tmp >> 16)&0x3;
      stat->PciDevice    = (tmp >>  8)&0xF;
      stat->PciBus       = tmp&0xFF;

      tmp = pgpDevice->reg->control;
      for (i=0; i<NUMBER_OF_LANES; i++) {
        tmp1 = pgpDevice->reg->pgpStat[i];
        stat->PgpLink[i].PgpLoopBack     = (tmp >> ( 4+i)) & 0x1;
        stat->PgpLink[i].PgpRxReset      = (tmp >> ( 8+i)) & 0x1;
        stat->PgpLink[i].PgpTxReset      = (tmp >> (12+i)) & 0x1;
        stat->PgpLink[i].PgpLocLinkReady = (tmp1      )    & 0x1;
        stat->PgpLink[i].PgpRemLinkReady = (tmp1 >>  1)    & 0x1;
        stat->PgpLink[i].PgpRxReady      = (tmp1 >>  2)    & 0x1;
        stat->PgpLink[i].PgpTxReady      = (tmp1 >>  3)    & 0x1;
        stat->PgpLink[i].PgpRxCount      = (tmp1 >>  4)    & 0xF;
        stat->PgpLink[i].PgpCellErrCnt   = (tmp1 >>  8)    & 0xF;
        stat->PgpLink[i].PgpLinkDownCnt  = (tmp1 >> 12)    & 0xF;
        stat->PgpLink[i].PgpLinkErrCnt   = (tmp1 >> 16)    & 0xF;
        stat->PgpLink[i].PgpFifoErr      = (tmp1 >> 20)    & 0x1;
      }
      for (i=0; i<NUMBER_OF_LANE_CLIENTS; i++) {
        stat->RxWrite[i]     = pgpDevice->rxWrite[i];
        stat->RxRead[i]      = pgpDevice->rxRead[i];
      }

      tmp = pgpDevice->reg->txStatus;
      stat->TxDma3AFull    = (tmp >> 31)&0x1;
      stat->TxDma2AFull    = (tmp >> 30)&0x1;
      stat->TxDma1AFull    = (tmp >> 29)&0x1;
      stat->TxDma0AFull    = (tmp >> 28)&0x1;
      stat->TxReadReady    = (tmp >> 10)&0x1;
      stat->TxRetFifoCount = tmp&0x3FF;

      stat->TxRead         = pgpDevice->txRead;
      stat->TxCount        = pgpDevice->reg->txCount;
      stat->TxBufferCount  = pgpDevice->txBufferCount;

      tmp = pgpDevice->reg->rxStatus;
      stat->RxFreeEmpty     = (tmp >> 31)&0x1;
      stat->RxFreeFull      = (tmp >> 30)&0x1;
      stat->RxFreeValid     = (tmp >> 29)&0x1;
      stat->RxFreeFifoCount = (tmp >> 16)&0x3FF;
      stat->RxReadReady     = (tmp >> 10)&0x1;
      stat->RxRetFifoCount  = tmp&0x3FF;

      stat->RxCount = pgpDevice->reg->rxCount;
      stat->RxBufferCount = pgpDevice->rxBufferCount;

      // Copy to user
     if (pgpDevice->debug & 1)  {
       printk(KERN_DEBUG "%s IOCTL_ReadStatus copy to user %p, %p, %d\n",
          MOD_NAME, (__u32*)argument, stat, (unsigned)sizeof(PgpCardStatus));
     }
      if ((read = copy_to_user((__u32*)argument, stat, sizeof(PgpCardStatus)))) {
        printk(KERN_WARNING "%s: Read Status: failed to copy %u to user. Maj=%i\n",
            MOD_NAME,
            read,
            pgpDevice->major);
        spin_unlock(pgpDevice->ioctlLock);
        return ERROR;
      }

      spin_unlock(pgpDevice->ioctlLock);
      if (pgpDevice->debug & 1) printk(KERN_DEBUG "%s IOCTL_ReadStatus 4\n", MOD_NAME);
      return(SUCCESS);
      break;

      // Dump Debug
    case IOCTL_Dump_Debug:

      printk(KERN_DEBUG "%s: IOCTL_Dump_Debug %s\n", MOD_NAME, PGPCARD_VERSION);

      printk(KERN_DEBUG "%s: Ioctl: Open Clients ...\n", MOD_NAME);

      for (i=0; i<MAX_NUMBER_OPEN_CLIENTS; i++) {
        if (pgpDevice->client[i].fp) {
          printk(KERN_DEBUG "%s: \t%d: mask(0x%x) vcMask(0x%x)\n",
              MOD_NAME, i, pgpDevice->client[i].mask, pgpDevice->client[i].vcMask);
        }
      }

      // Rx Buffers
      printk(KERN_DEBUG"%s: Ioctl: Rx Queue for all lanes contain %i out of %i buffers. Maj=%i.\n",
          MOD_NAME, pgpDevice->rxBufferCount, NUMBER_OF_RX_CLIENT_BUFFERS, pgpDevice->major);

      // Rx Firmware Fifo
      spin_lock_irq(pgpDevice->rxLock);
      bfcnt = countRXFirmwareBuffers(pgpDevice, 0);
      spin_unlock_irq(pgpDevice->rxLock);
      printk(KERN_DEBUG"%s: Ioctl: Rx Firmware Fifo contains %i out of %i buffers. Maj=%i.\n",
          MOD_NAME, bfcnt, NUMBER_OF_RX_BUFFERS, pgpDevice->major);

      // Tx Buffers
      printk(KERN_DEBUG "%s: Ioctl: %u txBuffers are allocated\n", MOD_NAME, countTxBuffers(pgpDevice));
      printk(KERN_DEBUG "%s: Ioctl:           Tx Histo    Rx Histo\n", MOD_NAME);
      for (mi=0; mi<NUMBER_OF_TX_BUFFERS; mi++) {
        if (mi < 10) {
          if (pgpDevice->txHisto[mi] || pgpDevice->rxHisto[mi]) {
            printk(KERN_DEBUG "%s:\t%3u     %10u  %10u\n", MOD_NAME, mi, pgpDevice->txHisto[mi], pgpDevice->rxHisto[mi]);
          }
        } else if (mi % 10 == 0) {
          if (mi != 10) {
            if (txSum) {
              sprintf(s1, "%8u.%1u", txSum/10, txSum%10);
            } else {
              sprintf(s1, "          ");
            }
            if (txSum || rxSum) {
              printk(KERN_DEBUG "%s:\t%3u-%3u   %s  %8u.%u\n", MOD_NAME, mi-10, mi-1, s1, rxSum/10, rxSum%10);
            }
          }
          txSum = pgpDevice->txHisto[mi];
          rxSum = pgpDevice->rxHisto[mi];
        } else {
          txSum += pgpDevice->txHisto[mi];
          rxSum += pgpDevice->rxHisto[mi];
        }
      }
      if (txSum) {
        rem = NUMBER_OF_TX_BUFFERS%10;
        printk(KERN_DEBUG "%s:\t%3u-%3u   %8u.%1u\n", MOD_NAME, (mi/10)*10, mi-1, txSum/rem, ((txSum%rem)*10)/rem );
      }
      if (NUMBER_OF_RX_CLIENT_BUFFERS > NUMBER_OF_TX_BUFFERS) {
        for (mi=NUMBER_OF_TX_BUFFERS; mi<NUMBER_OF_RX_CLIENT_BUFFERS; mi++) {
          if ((mi<10)
              || ((mi>=(NUMBER_OF_RX_BUFFERS/10)*10)&&(mi<((NUMBER_OF_RX_BUFFERS/10)+1)*10))
              || (mi>=(NUMBER_OF_RX_CLIENT_BUFFERS/10)*10)) {
            if (pgpDevice->rxHisto[mi]) printk(KERN_DEBUG "%s:\t%3u                 %10u\n", MOD_NAME, mi, pgpDevice->rxHisto[mi]);
          }
          if (mi % 10 == 0) {
            if (rxSum) {
              printk(KERN_DEBUG "%s:\t%3u-%3u               %8u.%u\n", MOD_NAME, mi-10, mi-1, rxSum/10, rxSum%10);
            }
            rxSum = pgpDevice->rxHisto[mi];
          } else {
            rxSum += pgpDevice->rxHisto[mi];
          }
//          if (((NUMBER_OF_RX_BUFFERS - mi) < (NUMBER_OF_RX_BUFFERS % 10)) && pgpDevice->rxHisto[mi]) {
//            printk(KERN_DEBUG "%s:\t%3u                 %10u\n", MOD_NAME, mi, pgpDevice->rxHisto[mi]);
//          }
        }
      }

      printk(KERN_DEBUG "%s: RX Buffers Count Histo:\n", MOD_NAME);
      upperLimit = ((NUMBER_OF_RX_BUFFERS / 10) -1) * 10;
      rxSum = 0;
      for ( i=0; i < NUMBER_OF_RX_BUFFERS<<1; i++ ) {
        if ((i<40) || (i>upperLimit)) {
          if (pgpDevice->rxBuffersHisto[i]) {
            printk(KERN_DEBUG "%s: RX\t%3u     %10u\n", MOD_NAME, i, pgpDevice->rxBuffersHisto[i]);
          }
        } else {
          if (i%10 == 0) {
            if (rxSum) {
              printk(KERN_DEBUG "%s: RX\t%3u-%3u %10u.%u\n", MOD_NAME, i-10, i-1, rxSum/10, rxSum%10);
            }
            rxSum = pgpDevice->rxBuffersHisto[i];
          } else {
            rxSum += pgpDevice->rxBuffersHisto[i];
          }
        }
        if (i == upperLimit) {
          printk(KERN_DEBUG "%s: RX\t%3u     %10u\n", MOD_NAME, i, pgpDevice->rxBuffersHisto[i]);
        }
      }
      printk(KERN_DEBUG "%s:pgpDevice->goingDown(%x)\n", MOD_NAME, pgpDevice->goingDown);
      for ( i=0; i < NUMBER_OF_LANE_CLIENTS; i++ ) {
        if ( pgpDevice->client[i].fp != 0 ) {
          printk(KERN_DEBUG "%s:\tClient %u has lanes 0x%x and had to toss %u buffers\n",
              MOD_NAME, i, pgpDevice->client[i].mask, pgpDevice->rxTossedBuffers[i]);
        }
      }
      printk(KERN_DEBUG"%s: Irq: packet count for device not open in lane order:%8u%8u%8u%8u\n",
          MOD_NAME,pgpDevice->noClientPacketCount[0],pgpDevice->noClientPacketCount[1],
          pgpDevice->noClientPacketCount[2],pgpDevice->noClientPacketCount[3]);
      printk(KERN_DEBUG "%s:Total Tossed Buffers %u\n", MOD_NAME, pgpDevice->rxTotalTossedBuffers);
      printk(KERN_DEBUG "%s:Lane Rx Histo:\n", MOD_NAME);
      for ( i=0; i < NUMBER_OF_LANES; i++ ) {
        printk(KERN_DEBUG "%s:\tLane %u - %9u %u %u %u\n", MOD_NAME, i,
            pgpDevice->rxLaneHisto[i][0],
            pgpDevice->rxLaneHisto[i][1],
            pgpDevice->rxLaneHisto[i][2],
            pgpDevice->rxLaneHisto[i][3]);
      }
      printk(KERN_DEBUG "%s:Lane Tx Histo:\n", MOD_NAME);
      for ( i=0; i < NUMBER_OF_LANES; i++) {
        printk(KERN_DEBUG "%s:\tLane %u - %9u %u %u %u\n", MOD_NAME, i,
            pgpDevice->txHistoLV[i*NUMBER_OF_VC+0],
            pgpDevice->txHistoLV[i*NUMBER_OF_VC+1],
            pgpDevice->txHistoLV[i*NUMBER_OF_VC+2],
            pgpDevice->txHistoLV[i*NUMBER_OF_VC+3]);
      }
      printk(KERN_DEBUG "%s: IRQ Loop Count Histo:\n", MOD_NAME);
      for ( i=0; i < NUMBER_OF_RX_BUFFERS; i++ ) {
        if (pgpDevice->rxLoopHisto[i]) {
          printk(KERN_DEBUG "%s: IRQ\t%3u %10u\n", MOD_NAME, i, pgpDevice->rxLoopHisto[i]);
        }
      }

      spin_unlock(pgpDevice->ioctlLock);
      return(SUCCESS);
      break;

    case IOCTL_Clear_Open_Clients:
      printk(KERN_DEBUG "%s: IOCTL Clearing %u open clients 0x%x\n", MOD_NAME, pgpDevice->openCount-1, pgpDevice->isOpen);
      for (i=0; i<MAX_NUMBER_OPEN_CLIENTS; i++) {
        if (pgpDevice->client[i].fp  && (pgpDevice->client[i].fp != filp)) {
          PgpCard_Release(pgpDevice->client[i].inode, pgpDevice->client[i].fp);
        }
      }
      spin_unlock(pgpDevice->ioctlLock);
      return(SUCCESS);
      break;

    case IOCTL_Clear_Polling:
      pgpDevice->pollEnabled = 0;
      spin_unlock(pgpDevice->ioctlLock);
      return(SUCCESS);
      break;


    default:
      spin_unlock(pgpDevice->ioctlLock);
      return(ERROR);
      break;
  }
  return ERROR;
}

int dumpWarning(struct PgpDevice *pgpDevice) {

  int            i;
  unsigned       mi;
  __u32          txSum = 0;
  __u32          rxSum = 0;
  __u32          rem;
  __u32          bfcnt;
  unsigned       upperLimit = 0;
  char           s1[40];

  spin_lock(pgpDevice->ioctlLock);
  printk(KERN_WARNING "%s Driver State on closing all clients\n", MOD_NAME);

  // Rx Buffers
  printk(KERN_WARNING"%s: Ioctl: Rx Queue for all lanes contain %i out of %i buffers. Maj=%i.\n",
      MOD_NAME, pgpDevice->rxBufferCount, NUMBER_OF_RX_CLIENT_BUFFERS, pgpDevice->major);

  // Rx Firmware Fifo
  spin_lock(pgpDevice->rxLock);
  bfcnt = countRXFirmwareBuffers(pgpDevice, 0);
  spin_unlock(pgpDevice->rxLock);
  printk(KERN_WARNING"%s: Ioctl: Rx Firmware Fifo contains %i out of %i buffers. Maj=%i.\n",
      MOD_NAME, bfcnt, NUMBER_OF_RX_BUFFERS, pgpDevice->major);

  // Tx Buffers
  printk(KERN_WARNING "%s: Ioctl: %u txBuffers are allocated\n", MOD_NAME, countTxBuffers(pgpDevice));
  printk(KERN_WARNING "%s: Ioctl:           Tx Histo    Rx Histo\n", MOD_NAME);
  for (mi=0; mi<NUMBER_OF_TX_BUFFERS; mi++) {
    if (mi < 10) {
      if (pgpDevice->txHisto[mi] || pgpDevice->rxHisto[mi]) {
        printk(KERN_WARNING "%s:\t%3u     %10u  %10u\n", MOD_NAME, mi, pgpDevice->txHisto[mi], pgpDevice->rxHisto[mi]);
      }
    } else if (mi % 10 == 0) {
      if (mi != 10) {
        if (txSum) {
          sprintf(s1, "%8u.%1u", txSum/10, txSum%10);
        } else {
          sprintf(s1, "          ");
        }
        if (txSum || rxSum) {
          printk(KERN_WARNING "%s:\t%3u-%3u   %s  %8u.%u\n", MOD_NAME, mi-10, mi-1, s1, rxSum/10, rxSum%10);
        }
      }
      txSum = pgpDevice->txHisto[mi];
      rxSum = pgpDevice->rxHisto[mi];
    } else {
      txSum += pgpDevice->txHisto[mi];
      rxSum += pgpDevice->rxHisto[mi];
    }
  }
  if (txSum) {
    rem = NUMBER_OF_TX_BUFFERS%10;
    printk(KERN_WARNING "%s:\t%3u-%3u   %8u.%1u\n", MOD_NAME, (mi/10)*10, mi-1, txSum/rem, ((txSum%rem)*10)/rem );
  }
  if (NUMBER_OF_RX_CLIENT_BUFFERS > NUMBER_OF_TX_BUFFERS) {
    for (mi=NUMBER_OF_TX_BUFFERS; mi<NUMBER_OF_RX_CLIENT_BUFFERS; mi++) {
      if ((mi<10)
          || ((mi>=(NUMBER_OF_RX_BUFFERS/10)*10)&&(mi<((NUMBER_OF_RX_BUFFERS/10)+1)*10))
          || (mi>=(NUMBER_OF_RX_CLIENT_BUFFERS/10)*10)) {
        if (pgpDevice->rxHisto[mi]) printk(KERN_WARNING "%s:\t%3u                 %10u\n", MOD_NAME, mi, pgpDevice->rxHisto[mi]);
      }
      if (mi % 10 == 0) {
        if (rxSum) {
          printk(KERN_WARNING "%s:\t%3u-%3u               %8u.%u\n", MOD_NAME, mi-10, mi-1, rxSum/10, rxSum%10);
        }
        rxSum = pgpDevice->rxHisto[mi];
      } else {
        rxSum += pgpDevice->rxHisto[mi];
      }
      //          if (((NUMBER_OF_RX_BUFFERS - mi) < (NUMBER_OF_RX_BUFFERS % 10)) && pgpDevice->rxHisto[mi]) {
      //            printk(KERN_WARNING "%s:\t%3u                 %10u\n", MOD_NAME, mi, pgpDevice->rxHisto[mi]);
      //          }
    }
  }

  printk(KERN_WARNING "%s: RX Buffers Count Histo:\n", MOD_NAME);
  upperLimit = ((NUMBER_OF_RX_BUFFERS / 10) -1) * 10;
  rxSum = 0;
  for ( i=0; i < NUMBER_OF_RX_BUFFERS<<1; i++ ) {
    if ((i<40) || (i>upperLimit)) {
      if (pgpDevice->rxBuffersHisto[i]) {
        printk(KERN_WARNING "%s: RX\t%3u     %10u\n", MOD_NAME, i, pgpDevice->rxBuffersHisto[i]);
      }
    } else {
      if (i%10 == 0) {
        if (rxSum) {
          printk(KERN_WARNING "%s: RX\t%3u-%3u %10u.%u\n", MOD_NAME, i-10, i-1, rxSum/10, rxSum%10);
        }
        rxSum = pgpDevice->rxBuffersHisto[i];
      } else {
        rxSum += pgpDevice->rxBuffersHisto[i];
      }
    }
    if (i == upperLimit) {
      printk(KERN_WARNING "%s: RX\t%3u     %10u\n", MOD_NAME, i, pgpDevice->rxBuffersHisto[i]);
    }
  }
  printk(KERN_WARNING "%s:pgpDevice->goingDown(%x)\n", MOD_NAME, pgpDevice->goingDown);
  for ( i=0; i < NUMBER_OF_LANE_CLIENTS; i++ ) {
    if ( pgpDevice->client[i].fp != 0 ) {
      printk(KERN_WARNING "%s:\tClient %u has lanes 0x%x and had to toss %u buffers\n",
          MOD_NAME, i, pgpDevice->client[i].mask, pgpDevice->rxTossedBuffers[i]);
    }
  }
  printk(KERN_WARNING"%s: Irq: packet count for device not open in lane order:%8u%8u%8u%8u\n",
      MOD_NAME,pgpDevice->noClientPacketCount[0],pgpDevice->noClientPacketCount[1],
      pgpDevice->noClientPacketCount[2],pgpDevice->noClientPacketCount[3]);
  printk(KERN_WARNING "%s:Total Tossed Buffers %u\n", MOD_NAME, pgpDevice->rxTotalTossedBuffers);
  printk(KERN_WARNING "%s:Lane Rx Histo:\n", MOD_NAME);
  for ( i=0; i < NUMBER_OF_LANES; i++ ) {
    printk(KERN_WARNING "%s:\tLane %u - %9u %u %u %u\n", MOD_NAME, i,
        pgpDevice->rxLaneHisto[i][0],
        pgpDevice->rxLaneHisto[i][1],
        pgpDevice->rxLaneHisto[i][2],
        pgpDevice->rxLaneHisto[i][3]);
  }
  printk(KERN_DEBUG "%s:Lane Tx Histo:\n", MOD_NAME);
  for ( i=0; i < NUMBER_OF_LANES; i++) {
    printk(KERN_DEBUG "%s:\tLane %u - %9u %u %u %u\n", MOD_NAME, i,
        pgpDevice->txHistoLV[i*NUMBER_OF_VC+0],
        pgpDevice->txHistoLV[i*NUMBER_OF_VC+1],
        pgpDevice->txHistoLV[i*NUMBER_OF_VC+2],
        pgpDevice->txHistoLV[i*NUMBER_OF_VC+3]);
  }
  printk(KERN_WARNING "%s: IRQ Loop Count Histo:\n", MOD_NAME);
  for ( i=0; i < NUMBER_OF_RX_BUFFERS; i++ ) {
    if (pgpDevice->rxLoopHisto[i]) {
      printk(KERN_WARNING "%s: IRQ\t%3u %10u\n", MOD_NAME, i, pgpDevice->rxLoopHisto[i]);
    }
  }

  spin_unlock(pgpDevice->ioctlLock);
  return(SUCCESS);
}

// Memory map
int PgpCard_Mmap(struct file *filp, struct vm_area_struct *vma) {

   struct PgpDevice *pgpDevice = (struct PgpDevice *)filp->private_data;

   unsigned long offset = vma->vm_pgoff << PAGE_SHIFT;
   unsigned long physical = ((unsigned long) pgpDevice->baseHdwr) + offset;
   unsigned long vsize = vma->vm_end - vma->vm_start;
   int result;

   // Check bounds of memory map
   if (vsize > pgpDevice->baseLen) {
      printk(KERN_WARNING"%s: Mmap: mmap vsize %08x, baseLen %08x. Maj=%i\n", MOD_NAME,
         (unsigned int) vsize, (unsigned int) pgpDevice->baseLen,pgpDevice->major);
      return -EINVAL;
   }

   result = io_remap_pfn_range(vma, vma->vm_start, physical >> PAGE_SHIFT,
            vsize, vma->vm_page_prot);
//   result = io_remap_page_range(vma, vma->vm_start, physical, vsize,
//            vma->vm_page_prot);

   if (result) return -EAGAIN;

   vma->vm_ops = &PgpCard_VmOps;
   PgpCard_VmOpen(vma);
   return 0;
}


void PgpCard_VmOpen(struct vm_area_struct *vma) { }


void PgpCard_VmClose(struct vm_area_struct *vma) { }


// Flush queue
int PgpCard_Fasync(int fd, struct file *filp, int mode) {
   struct PgpDevice *pgpDevice = (struct PgpDevice *)filp->private_data;
   return fasync_helper(fd, filp, mode, &(pgpDevice->async_queue));
}


