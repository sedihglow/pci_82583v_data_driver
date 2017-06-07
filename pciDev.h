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
#include <linux/delay.h>
#include <linux/interrupt.h>

                  /* kmalloc allocation macros */
#define VZALLOC(type)            ((type*)vzalloc(sizeof(type))
#define VZALLOC_VOID(size)       ((void*)vzalloc(size)
#define VZALLOC_ARRAY(type, num) ((type*)vzalloc(sizeof(type)*(num)))

#define KZALLOC(type, flag)      ((type*)kzalloc(sizeof(type), flag))
#define KZALLOC_VOID(size, flag) ((void*)kzalloc(size, flag))
#define KZALLOC_ARRAY(type, num, flag)                                         \
    ((type*)kzalloc(sizeof(type)*(num), flag))

/* descriptor count and size */
#define DESC_CNT    16                /* num of desc in ring */
#define DBUF_SIZE   (sizeof(u8)*2048) /* 2k bytes */

                    /* register definitions */
#define HWREG(bar, offset) ((bar)+(offset))

/* LED ctrl offset */
#define LED_OFFSET 0xE00 /* LED control register */
#define INV_BIT    0x00000040
/* led numbers */
#define RIGHT_GRN  0x1 /* right green is LED1 */
#define LEFT_GRN   0x2 /* left green is LED2 */
#define LEFT_AMBER 0x4 /* left amber is LED1 */
/* led ctrl mode values */
#define LED_ON     0xE /* always asserted */
#define LED_OFF    0xF
#define DISABLE_LEDS 0xF0F0F
/* led mode locations: regVal |= (LED_MODE_ON << LEFT_AMBER_SHIFT) */
#define RIGHT_GRN_SHIFT  8  /* LED0 */
#define LEFT_GRN_SHIFT   16  /* LED1 */
#define LEFT_AMBER_SHIFT 0 /* LED2 */
#define CLEAR_MODE(shift) (0xFFFFFFFF^(0xF<<(shift)))

/* device Control Register */
#define DEVCTL_OFFSET 0x0   
#define PHY_RST_BIT   0x80000000 /* PHY_RST bit */
#define MAC_RST_BIT   0x4000000  /* RST bit 27 */

/* Medium Dependant Interface Port Register */
#define MDIC_OFFSET 0x20 /* MDI control register */
#define PHY_INIT    0x1831AF08 /* needed for a forced PHY setup */
/* TODO: I want more information about what this INIT is doing and how to do it
 *       without hand holding */

/* 3GIO control registers */
#define GCR_OFFSET  0x5B00
#define GCR_INIT    0x400000 /* required bit to set during initialization */
#define GCR2_OFFSET 0x5B64
#define GCR2_INIT   0x1      /* required bit to set during initalization */

/* interrupt regs */
#define ICR_OFFSET   0xC0       /* Interrupt Cause Read */
#define ICS_OFFSET   0xC8       /* Interrupt Cause Set  */
#define IMS_OFFSET   0xD0       /* Interrupt Mask Set/Read */
#define IMC_OFFSET   0xD8       /* Interrupt Mask Clear Regiser */
#define LSC          0x4        /* Link Status Change bit 2 */
#define RXQ0         0x100000   /* Receive Queue 0 interrupt bit 20 */
#define RXT          0x80       /* receiver timer interrupt */
#define RXO          0x40       /* receiver overrun */
#define RXDMT        0x10       /* receive desc min threshold hit */
#define INT_MASK     0x100004 //(LSC | RXO| RXQ0 | RXDMT)
#define DISABLE_INTS 0x15382D7 /* Mask for IMC to disable all interrupts */

/* Receive Control Register offset */
#define RXCTL_OFFSET 0x100 
#define RX_EN        0x2    /* RCTL enable bit */
#define RX_UNI_P     0x8    /* unicast promiscuous bit */
#define RX_MULTI_P   0x10   /* multicast promiscuous bit */
#define RX_BAM       0x8000 /* Breadcast Accept mode bit */
#define RXCTL_INIT (RX_EN | RX_UNI_P | RX_BAM | RX_MULTI_P)

/* Receive Descriptor control */
#define RXDCTL_OFFSET 0x2828
#define WTHRESH_SHIFT 16
#define RXD_WTHRESH_MASK(toSet) ((toSet)<<(WTHRESH_SHIFT))
#define RXD_GRAIN 0x1000000     /* descriptor grainulatiry bit 24 */

/* Receive descriptor length reg */
#define RDLEN_OFFSET 0x2808
#define RDL_LEN   256  //(sizeof(struct gbe38v_rx_desc)*DESC_CNT)
#define RDL_SHIFT 7
#define RDL_SET(toSet) ((toSet)<<(RDL_SHIFT))

/* Receive Descriptor Bar, Must be 16byte aligned */
#define RDBAH_OFFSET 0x2804 /* Base Address High bits */
#define RDBAL_OFFSET 0x2800 /* Base Address Low bits */

/* Head and Tail of Rx descriptor ring */
#define RX_HEAD_OFFSET 0x2818
#define HEAD_INIT      0x0
#define RX_TAIL_OFFSET 0x2810
#define TAIL_INIT      15
/* status register */
#define STAT_OFFSET 0x8

#define FLUSH(myDev) (ioread32((myDev->hwAddr+STAT_OFFSET)))
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
#define GBE38V_DESC(R, i, type)                                                \
    (((struct type *)((R)->desc)) + (i))

#define ONE_PAGE    4096 /* Page size of system */
#define WQ_SLEEP    500  /* miliseconds, 0.5seconds */

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
