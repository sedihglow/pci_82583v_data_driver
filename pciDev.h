/******************************************************************************
 * filename: pciDev.h
 *
 * written by: James Ross
 ******************************************************************************/

#ifndef _PCI_DEV_H
#define _PCI_DEV_H

#include <linux/init.h>
#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/kdev_t.h>
#include <linux/types.h>
#include <linux/export.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/pci.h>
#include <linux/ioport.h>
#include <linux/io.h>
#include <asm/io.h>
#include <linux/vmalloc.h>
#include <linux/dma-mapping.h>

#include "ledTimer.h"

                  /* kmalloc allocation macros */
#define VZALLOC(type)            ((type*)vzalloc(sizeof(type))
#define VZALLOC_VOID(size)       ((void*)vzalloc(size)
#define VZALLOC_ARRAY(type, num) ((type*)vzalloc(sizeof(type)*(num)))

#define KZALLOC(type, flag)      ((type*)kzalloc(sizeof(type), flag))
#define KZALLOC_VOID(size, flag) ((void*)kzalloc(size, flag))
#define KZALLOC_ARRAY(type, num, flag)                                         \
    ((type*)kzalloc(sizeof(type)*(num), flag))

                    /* register definitions */
#define HWREG(bar, offset) ((bar)+(offset))

/* LED ctrl offset */
#define LED_OFFSET 0xE00
/* led numbers */
#define RIGHT_GRN 0x1 /* right green is LED0 */
#define LEFT_GRN  0x2 /* left green is LED1 */
/* led ctrl mode values */
#define LED_ON     0x4E   /* includes changing polarity bit */
#define LED_OFF    0xF
/* led mode locations: regVal |= (LED_MODE_ON << LEFT_AMBER_SHIFT) */
#define RIGHT_GRN_SHIFT 0 /* LED0 */
#define LEFT_GRN_SHIFT  8 /* LED1 */

/* device Control Register */
#define DEVCTL_OFFSET 0x0   
#define SWRST_BIT     0x4000000 /* software reset bit in */

/* interrupt regs */
#define ICR_OFFSET 0xC0       /* Interrupt Cause Read */
#define ICS_OFFSET 0xC8       /* Interrupt Cause Set  */
#define IMS_OFFSET 0xD0       /* Interrupt Mask Set/Read */
#define LSC        0x2        /* Link Status Change bit 2 */
#define RXQ0       0x100000   /* Receive Queue 0 interrupt bit 20 */

/* Receive Control Register offset */
#define RXCTL_OFFSET 0x100 
#define REC_EN       0x2 /* RCTL enable bit */

/* Receive Descriptor control */
#define RXDCTL_OFFSET 0x2828
#define WTHRESH_SHIFT 16
#define RXD_WTHRESH_MASK(toSet) ((toSet)<<(WTHRESH_SHIFT))
#define RXD_GRAIN 0x1000000     /* descriptor grainulatiry bit 24 */

/* Recieve descriptor length reg */
#define RDLEN_OFFSET 0x2808
#define RDL_SHIFT 7
#define RDL_SET(toSet) ((toSet)<<(RDL_SHIFT))

/* Recieve Descriptor Bar, Must be 16byte aligned */
#define RDBAL_OFFSET 0x2800 /* Base Address Low bits */
#define RDBAH_OFFSET 0x2804 /* Base Address High bits */

/* Head and Tail of Rx descriptor ring */
#define RX_HEAD_OFFSET 0x2818
#define RX_TAIL_OFFSET 0x2810

                    /* general definitions */
#define SUCCESS 0
#define FAILURE -1
#define HIGH 1
#define LOW  0
#define DEV_NAME "pciDev"

#define MINOR_CNT   1
#define MINOR_STRT  0
#define NUM_DEVICES 1

#define PCI_DEV_ID  0x150C /* 82583V */
#define PCI_VEND_ID 0x8086 /* intel */

/* gets the descriptor from void *desc. 
 * Ring, Desc num, struct type */
#define GBE38V_DESC_BUFF(R, i, type)                                           \
    (((struct (type)*)((R)->descBuff)) + (i))

#define ONE_PAGE    4096              /* Page size of system */
#define DESC_CNT    16                /* num of desc in ring */
#define DBUF_SIZE   (sizeof(u8)*2048) /* 2k bytes */

#define WQ_SLEEP    500 /* miliseconds, 0.5seconds */

/* rx legacy descriptor */
struct gbe38v_rx_desc{
    __le64 bufferAddr;
    union{
        __le32 data;
        struct{
            __le16 length;
            __le16 checkSum;
        }flags;
    }lower;
    union{
        __le32 data;
        struct{
            u8 status;
            u8 errors;
            __le16 vlan;
        }fields;
    }upper;
};

/* info buffer for legecy descriptor
 * 1 buffer info for each descriptor, described meta data for descriptor */
struct gbe38v_buffer{ 
    dma_addr_t dma; /* pinned address for buffer used for desc bufferAddr */
    void *desc;     /* kernel address of buffer used for desc bufferAddr */
    unsigned int size;
};

struct gbe38v_ring{
	struct gbe38v_dev *myDev;	/* back pointer to adapter */
	void *desc;			        /* pointer to ring memory  */
    void *descBuff[DESC_CNT];   /* buffer address's for desc */
	dma_addr_t dma;			    /* phys address of ring    */
	unsigned int size;		    /* length of ring in bytes */
	unsigned int count;		    /* number of desc. in ring */

	u16 useNext;                /* next descriptor to use */
	u16 cleanNext;              /* next used descriptor to clean */

	void __iomem *head;         /* (NAME) head register adderess */
	void __iomem *tail;         /* (NAME) tail register address */

	/* array of buffer information structs */
	struct gbe38v_buffer *bufferInfo;
};

struct gbe38v_dev{
    dev_t devNum;
    void __iomem *hwAddr;
    struct cdev cdev;
    struct device *device;
    struct class *pciClass;
    struct pci_dev *pdev;

    struct gbe38v_ring *rxRing;
    dma_addr_t rxdma; /* physical address of rxRing */

    struct work_struct workq;

    struct gbe38v_timer ledTimer;
};

                /* Function prototypes */
int gbe38v_open(struct inode *inode, struct file *file);

int gbe38v_release(struct inode *inode, struct file *file);

ssize_t gbe38v_read(struct file *filep, char __user *buff, size_t count, 
                  loff_t *offp);

ssize_t gbe38v_write(struct file *filep, const char __user *buff, size_t count,
                   loff_t *offp);

int gbe38v_probe(struct pci_dev *pdev, const struct pci_device_id *ent);

void gbe38v_remove(struct pci_dev *pdev);
#endif
/****************** EOF ***************/
