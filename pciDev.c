/*******************************************************************************
 * filename: picDev.c
 *
 * Implements the functions defined in pciDev.h
 *
 * Written by: James Ross
 ******************************************************************************/
#include "pciDev.h"

#define REG_SIZE   sizeof(int)
#define ONE_BYTE   1
#define MM_BAR0    0 /* memory map IO on 82583 is bar0 */
#define CLEAR_MASK 0xF

/* TODO MAYBE: static function prototypes */

            /* global structs */
static struct file_operations fops = {
    .owner   = THIS_MODULE,
    .open    = gbe38v_open,
    .read    = gbe38v_read,
    .write   = gbe38v_write,
	.release = gbe38v_release
};

/*******************************************************************************
 *                      led functionality
 ******************************************************************************/
static int set_led(void __iomem *ledReg, int ledNums, bool setTo)
{
    u32 toWrite;
    u32 setMode = (setTo == HIGH ? LED_ON : LED_OFF);

    toWrite = ioread32(ledReg);

    if(ledNums & RIGHT_GRN){
        toWrite &= ~(CLEAR_MASK << RIGHT_GRN_SHIFT); /* clear mode bits */
        toWrite |= setMode << RIGHT_GRN_SHIFT;
    }
    else if(ledNums & LEFT_GRN){
        toWrite &= ~(CLEAR_MASK << LEFT_GRN_SHIFT); /* clear mode bits */
        toWrite |= setMode << LEFT_GRN_SHIFT;
    }
    else{ /* amber LED not in use */
        /* TODO: print warning */
        return -EINVAL;
    }
    
    iowrite32(toWrite, ledReg);
    return SUCCESS;
}/* end set_led */

/*******************************************************************************
 *                          head/tail Managment
 ******************************************************************************/
static int gbe3v_rx_update_tail(struct gbe38v_ring *rxRing)
{
    u16 useNext = rxRing->useNext;
    iowrite32(useNext, rxRing->tail);
    /* TODO: make error checking here to ensure the tail pointer
     * is proper after write before continueing */
    

    /* TODO: make it add and error check a desc "send num" to send so we can
     *       later make it send and clean more than 1 desc at a time */
    ++useNext; 
    (useNext < rxRing->count) ? (++rxRing->useNext) : (rxRing->useNext = 0);

    return SUCCESS;
}

static int gbe38v_rx_clean_desc(struct gbe38v_ring *rxRing)
{
    /* NOTE: see TODO in rx_update */
    struct gbe38v_rx_desc *toClean;
    struct gbe38v_buffer *bufferInfo;
    u16 cleanNext = rxRing->cleanNext;

    /* TODO: Should i just use buffer info? When i free, i could just
     * use my buffer array instead of buffer info's alias... Should i just
     * make buffer info my buffer array? 
     * Also, should i be pinning and unpinning these addresses as they are
     * needed and cleaned? */
    toClean = GBE38V_DESC_BUFF(rxRing, cleanNext, gbe38v_rx_desc);
    
    /* clear DD bit in status feild */
    /* TODO: Should i be checking EOF bit, also checksums not checked... 
     *       Implemented this as function for furutre additions,
     *       Also note in update rx about pinning and unpinning the descriptors
     *       on clean and tail bump */
    toClean->upper.fields.status = 0; /* Reset status field */ 
    
    return SUCCESS;
}

/*******************************************************************************
 *                          Workqueue
 ******************************************************************************/
static void rx_wq_task(struct work_struct *work)
{
    struct gbe38v_dev  *myDev  = container_of(work, struct gbe38v_dev, workq);
    struct gbe38v_ring *rxRing = myDev->rxRing;
    void __iomem *devBar  = myDev->hwAddr;
    void __iomem *ledReg  = HWREG(devBar, LED_OFFSET);

    /* sleep .5 sec */
    msleep(WQ_SLEEP);

    /* turn off both green LED's */
    set_led(ledReg, RIGHT_GRN, LOW);
    set_led(ledReg, LEFT_GRN, LOW);
    
    /* bump tail */
    gbe3v_rx_update_tail(rxRing);
    gbe38v_rx_clean_desc(rxRing);
}

/*******************************************************************************
 *                          irq interrupts
 ******************************************************************************/
static irqreturn_t gbe38v_rx_handle(int irq, void *dev_id)
{
    struct gbe38v_dev *myDev = dev_id;
    void __iomem *ledReg = (myDev->hwAddr)+LED_OFFSET;

    /* read cause of IRQ and make sure it is for us */
    /* TODO: tune the interupts we check and fire. No different action so far
     *       for different interrupts */
    if(!(ioread32(HWREG(myDev->hwAddr,ICR_OFFSET)) & (LSC|RXQ0)))
        return IRQ_NONE;

    /* turn both green LED's on */
    set_led(ledReg, RIGHT_GRN, HIGH);
    set_led(ledReg, LEFT_GRN, HIGH);

    /* submit workqueue thread to run */
    schedule_work(&myDev->workq);

    /* rearm the interrupt */
    enable_irq(irq);

    return IRQ_HANDLED;
}

static int gbe38v_setup_interrupts(struct gbe38v_dev *myDev)
{
    struct pci_dev *pdev = myDev->pdev;
    void __iomem *maskReg = HWREG(myDev->hwAddr, IMS_OFFSET);

    /* enable LSC and RXQ0 interrupts */
    iowrite32((LSC|RXQ0), maskReg);

    //pci_enable_msi(pdev);
    request_irq(pdev->irq, gbe38v_rx_handle, 0, DEV_NAME, myDev);
    INIT_WORK(&myDev->workq, rx_wq_task);
    return SUCCESS;
}

/*******************************************************************************
 *                      dma allocations
 ******************************************************************************/
static int gbe38v_rx_alloc_dma(struct gbe38v_dev *myDev)
{
    myDev->rxRing = dma_alloc_coherent(&myDev->pdev->dev, 
                                       sizeof(struct gbe38v_ring),
                                       &myDev->rxdma, GFP_KERNEL);
    if(!myDev->rxRing)
        return -ENOMEM;

    myDev->rxRing->myDev = myDev;
    myDev->rxRing->count = DESC_CNT;

    return SUCCESS;
}

static int gbe38v_alloc_dma_ring(struct pci_dev *pdev, struct gbe38v_ring *ring)
{
    /* allocate physical ring and pin it */
    ring->desc = dma_alloc_coherent(&pdev->dev, ring->size, 
                                    &ring->dma, GFP_KERNEL);
    if(!ring->desc)
        return -ENOMEM;

    return SUCCESS;
}

static int gbe38v_setup_rx_resources(struct gbe38v_ring *rxRing)
{
    int i = 0;
    int err;
    void **descBuff = rxRing->descBuff; /* descBuff allocated as *descBuff[] */
    struct gbe38v_buffer *bufferInfo; 
    struct pci_dev *pdev = rxRing->myDev->pdev;
    struct gbe38v_rx_desc *descRing; 
    descRing = GBE38V_DESC_RING(rxRing, 0, gbe38v_rx_desc); /* ptr of first i */

    rxRing->size = sizeof(struct gbe38v_rx_desc)*rxRing->count;
	rxRing->size = ALIGN(rxRing->size, ONE_PAGE);
    
    /* allocate descriptors */
	err = gbe38v_alloc_dma_ring(pdev, rxRing);
	if(err)
	    goto dma_ring_fail;

	rxRing->useNext   = 0;
	rxRing->cleanNext = 0;

    /* allocate buffer info */
    rxRing->bufferInfo = VZALLOC_ARRAY(struct gbe38v_buffer, rxRing->count);
    bufferInfo = rxRing->bufferInfo;
    if(!rxRing->bufferInfo)
        goto alloc_fail;

    /* allocate buffer addresses and pin them */
    for(i=0; i < rxRing->count; ++i){
        descBuff[i] = KZALLOC_VOID(DBUF_SIZE, GFP_KERNEL);
        if(!descBuff[i]){
            printk(KERN_ERR DEV_NAME ": Failed to allocate descriptor address "
                   "buffer %d\n",i);
            goto alloc_fail;
        }


        bufferInfo[i].desc = descBuff[i];
        bufferInfo[i].size = DBUF_SIZE;
        bufferInfo[i].dma = dma_map_single(&pdev->dev, descBuff[i], DBUF_SIZE, 
                                            DMA_BIDIRECTIONAL);
        if(!bufferInfo[i].dma){
            printk(KERN_ERR DEV_NAME ": Failed to DMA buffer address %d\n",i);
            goto alloc_fail;
        }

        /* Fill descriptor addresses */
        descRing[i].bufferAddr = cpu_to_le64(bufferInfo[i].dma);
        descRing[i].lower.flags.length = cpu_to_le16(DBUF_SIZE);
        descRing[i].lower.flags.checkSum = 0; /* TODO: checksum stuff */
        descRing[i].upper.data = 0;
    }
    
    rxRing->head = HWREG(rxRing->myDev->hwAddr, RX_HEAD_OFFSET);
    rxRing->tail = HWREG(rxRing->myDev->hwAddr, RX_TAIL_OFFSET);

    return SUCCESS;

/* error handling */
alloc_fail:
    err = -ENOMEM;
    /* unpin and free all descriptors allocated */
    kfree(descBuff[i]); /* if first buff failed, this is nop */
    for(--i; i >= 0; --i){
        kfree(descBuff[i]);
        dma_unmap_single(&pdev->dev, bufferInfo[i].dma, DBUF_SIZE, 
                         DMA_BIDIRECTIONAL);
    }
dma_ring_fail:
    return err;
}

static int gbe38v_setup_dma(struct gbe38v_dev *myDev)
{
    int err;
    err = gbe38v_rx_alloc_dma(myDev); /* allocate rxRing */
    if(err)
        return err;

    /* initalize dma ring resources */
    err = gbe38v_setup_rx_resources(myDev->rxRing);
    if(err)
        return err;

    return SUCCESS;
}

/*******************************************************************************
 *                      pci device functions
 ******************************************************************************/

static void gbe38v_init_rx_reg(struct gbe38v_dev *myDev)
{
    struct gbe38v_ring *rxRing = myDev->rxRing;
    void __iomem *hwAddr = myDev->hwAddr;
    void __iomem *workReg;
    u32 toWrite;

    /* set high and low addr of Rx descriptor ring */
    workReg = HWREG(hwAddr, RDBAH_OFFSET);
    toWrite = (rxRing->dma >> 32) & 0xFFFFFFFF; /* high 32 bits */
    iowrite32(toWrite, workReg);

    workReg = HWREG(hwAddr, RDBAL_OFFSET);
    toWrite = rxRing->dma & 0xFFFFFFFF; /* low 32 bits */
    iowrite32(toWrite, workReg);

    /* set Rx decriptor ring length */
    workReg = HWREG(hwAddr, RDLEN_OFFSET);
    toWrite = RDL_SET(RDL_LEN);

    /* set Rx head */
    workReg = HWREG(hwAddr, RX_HEAD_OFFSET);
    iowrite32(HEAD_INIT, workReg);

    /* set Rx tail "one descriptor beyone the end" */
    workReg = HWREG(hwAddr, RX_TAIL_OFFSET);
    iowrite32(TAIL_INIT, workReg);
}

static void init_device_reg(struct gbe38v_dev *myDev)
{
    /* NOTE: RXDCTL.WTHRESH have been used, they are written
     *       back, regardless of cahche line allignment. It is
     *       therefore recommended that WTHRESH be a multiple of cache line
     *       size. Currently only operating on 1 desc at a time. 
     *       RXDCTL.GRAIN = 0 by default, cahche line grainularity.
     *TODO:? Might need to = 1 for descriptor grainulatiry of WTHRESH */
     void __iomem *hwAddr = myDev->hwAddr;
     void __iomem *workReg;
     u32 toWrite;
    
    workReg = HWREG(hwAddr, DEVCTL_OFFSET);
    /* PHY reset, effects load NVM and datapath */
    toWrite = ioread32(workReg) | PHY_RST_BIT;
    iowrite32(toWrite, workReg);
    while((ioread32(workReg) & PHY_RST_BIT) != 0);

    /* Reset mac function */
    toWrite = ioread32(workReg) | MAC_RST_BIT;
    iowrite32(toWrite, workReg);
    while((ioread32(workReg) & MAC_RST_BIT) != 0);

    /* Disable all interrupts */
    workReg = HWREG(hwAddr, IMC_OFFSET);
    iowrite32(DISABLE_INTS, workReg);

    /* Set GCR bit 22 as requested by init sequence in datasheet on page 53 */
    workReg = HWREG(hwAddr, GCR_OFFSET);
    toWrite = ioread32(workReg) | GCR_INIT; 
    iowrite32(toWrite, workReg);

    /* Set GCR2 bit 1. Said in datasheet in register information, cant find it
     * requesting the bit in chapter 4 of datasheet on initialization. */
    workReg = HWREG(hwAddr, GCR2_OFFSET);
    toWrite = ioread32(workReg) | GCR2_INIT; 
    iowrite32(toWrite, workReg);

    /* PHY setup */
    workReg = HWREG(hwAddr, MDIC_OFFSET);
    iowrite32(PHY_INIT, workReg);

    gbe38v_init_rx_reg(myDev);

    /* set promiscuous mode, enable recieve */
    workReg = HWREG(hwAddr, RXDCTL_OFFSET);
    toWrite = ioread32(workReg) | RXCTL_INIT;
    iowrite32(toWrite, workReg);

    /* Disable all interrupts */
    workReg = HWREG(hwAddr, IMC_OFFSET);
    iowrite32(DISABLE_INTS, workReg);
}

static int myDev_init(struct gbe38v_dev *myDev)
{
    int err;

    /* TODO: Should i be doing this in all my functions? Its not an API so 
     * maybe not?
     * obviously myDev will be passed in, but it could prevent a kernel panic
     * for someone altering my code....?
     * 
     * note on checking passed pointers to functions that could be NULL.
     */
    if(unlikely(!myDev)){
        printk(KERN_ERR DEV_NAME ": myDev_init, null pointer, EINVAL.\n");
        return -EINVAL;
    }

    /* get major and minor */
    err = alloc_chrdev_region(&(myDev->devNum), MINOR_STRT, MINOR_CNT, 
                                 DEV_NAME);
    if(unlikely(err != SUCCESS)){
        printk(KERN_ERR DEV_NAME ": Unable to allocate major %d\n", 
               MAJOR(myDev->devNum));
        return err;
    }
    
    cdev_init(&(myDev->cdev), &fops);
    myDev->cdev.owner = THIS_MODULE;
    myDev->cdev.ops   = &fops;
    
    err = cdev_add(&(myDev->cdev), myDev->devNum, NUM_DEVICES);
    if(unlikely(err != SUCCESS)){
        printk(KERN_ERR DEV_NAME ": Unable to add cdev, err: %d\n", err);
        goto cdev_add_fail;
    }

    /* place module in /dev, requires a GPL liscense */
    myDev->pciClass = class_create(THIS_MODULE, DEV_NAME);
    if(IS_ERR(myDev->pciClass)){
        err = PTR_ERR(myDev->pciClass);
        printk(KERN_ERR DEV_NAME ": Unable to add cdev, err: %d\n", err);
        goto class_create_fail;
    }
     
    /* No parent device, No additional data */
    myDev->device = device_create(myDev->pciClass, NULL, myDev->devNum, NULL, 
                                  DEV_NAME);
    if(IS_ERR(myDev->device)){
        printk(KERN_ERR DEV_NAME ": Failed to create device. %d\n", err);
        err = PTR_ERR(myDev->device);
        goto device_create_fail;
    }

    return SUCCESS;

/* error handling */
device_create_fail: 
    class_destroy(myDev->pciClass);
class_create_fail :
cdev_add_fail     : 
    unregister_chrdev_region(myDev->devNum, MINOR_CNT);
    return err; 
}/* end mydev_init */

int gbe38v_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
{
    int err;
    struct gbe38v_dev *myDev;
    resource_size_t mmioStart, mmioLen;

    myDev = KZALLOC(struct gbe38v_dev, GFP_KERNEL);
    if(!myDev){
        err = -ENOMEM;
        printk(KERN_ERR DEV_NAME ": Unable to allocate drive_dev,"
               "error: ENOMEM\n");
        return err;
    }
    
    /* init device */
    err = myDev_init(myDev);
    if(err){
        dev_err(&pdev->dev, DEV_NAME ": Unable to allocate cdev, NULL ret\n");
        goto dev_init_fail;
    }

    err = pci_enable_device_mem(pdev);
    if(unlikely(err != SUCCESS)){
        dev_err(&pdev->dev, DEV_NAME ": Failed to enable device mem region, "
                "err: %d", err);
        goto pci_enable_fail;
    }

    /* ensure device can support DMA operations */
    err = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(64));
    if(err){
        dev_err(&pdev->dev, "requested DMA not supported\n");
        goto dma_set_mask_fail;
    }
    
    /* set bar registers */
    err = pci_request_selected_regions(pdev, 
                                         pci_select_bars(pdev, IORESOURCE_MEM),
                                         DEV_NAME);
    if(unlikely(err)){
        dev_err(&pdev->dev, DEV_NAME ": Failed to PCI request bar mem region, "
                "err: %d", err);
        goto pci_request_region_fail;
    }
    
    /* get master on bus for communication */
    pci_set_master(pdev);

    /* store driver struct */
    myDev->pdev = pdev;
    pci_set_drvdata(pdev, myDev);

    /* set memory map io start/finish for physical address */ 
    mmioStart = pci_resource_start(pdev, MM_BAR0);
    mmioLen   = pci_resource_len(pdev, MM_BAR0);

    /* get physical space addr */
    myDev->hwAddr = ioremap(mmioStart, mmioLen);
    if(unlikely(!myDev->hwAddr)){
        dev_err(&pdev->dev, DEV_NAME ": Failed to remap io, err: %d", err);
        err = -EIO;
        goto ioremap_fail;
    }


    /* set up dma */
    err = gbe38v_setup_dma(myDev);
    if(err){
        /* TODO: print statment */
        goto dma_setup_fail;
    }

    init_device_reg(myDev); /* TODO: Error checking depending on the funct */

    /* set up interupt handler */
    err = gbe38v_setup_interrupts(myDev);
    if(err){
        /* TODO: print statment */
        goto int_setup_fail;
    }

    /* chicken bits */

    /* force uplink */

    /* device initialization */


    return SUCCESS;

/* error handling */
int_setup_fail:
    /* TODO: unpin and deallocate everything set in setup_dma() */
dma_setup_fail:
    iounmap(myDev->hwAddr);
ioremap_fail: 
    pci_release_selected_regions(pdev, pci_select_bars(pdev, IORESOURCE_MEM));
pci_request_region_fail:
dma_set_mask_fail:
    pci_disable_device(pdev);
pci_enable_fail:
dev_init_fail:
    kfree(myDev);
    return err;
}/* end gbe38v_probe */

void gbe38v_remove(struct pci_dev *pdev)
{
    struct gbe38v_dev *myDev = pci_get_drvdata(pdev);
    iounmap(myDev->hwAddr);
    pci_release_selected_regions(pdev, pci_select_bars(pdev, IORESOURCE_MEM));
    pci_disable_device(pdev);
    device_destroy(myDev->pciClass, myDev->devNum);
    class_destroy(myDev->pciClass);
    unregister_chrdev_region(myDev->devNum, MINOR_CNT);
    cdev_del(&(myDev->cdev));
}/* end gbe38v_remove */

/*******************************************************************************
 *                      fops functions
 ******************************************************************************/
int gbe38v_open(struct inode *inode, struct file *file)
{
    struct gbe38v_dev *myDev;
    myDev = container_of(inode->i_cdev, struct gbe38v_dev, cdev);
    file->private_data = myDev;

    /* have hardware address, can initialize the timer for pci device LED */
    if(gbe38v_init_led_timer((myDev->hwAddr)+LED_OFFSET, &(myDev->ledTimer)) 
      != SUCCESS){
        printk(KERN_ERR DEV_NAME ":_open: Invalid parameter. Change parameter "
        "from negetive value and try opening again.\n");
        return -EINVAL;
    }

    return SUCCESS;
}/* end gbe38v_open */

int gbe38v_release(struct inode *inode, struct file *file)
{
    struct gbe38v_dev *myDev;
    myDev = container_of(inode->i_cdev, struct gbe38v_dev, cdev);
    file->private_data = myDev;

    gbe38v_remove_led_timer(&(myDev->ledTimer));
    printk(KERN_INFO DEV_NAME ": Device closed\n");
    return SUCCESS;
}/* end gbe38v_release */

ssize_t gbe38v_read(struct file *filep, char __user *buff, size_t count, 
                  loff_t *offp)
{
    unsigned long blinkRate;
    unsigned long retVal;

    /* read 0 bytes */
    if(unlikely(count == 0))
        return 0;

    if(count != sizeof(int))
        printk(KERN_ERR DEV_NAME ": Count is not sizeof(int), read will be.\n");
    
    if(unlikely(buff  == NULL)){
        printk(KERN_ERR DEV_NAME ": NULL __user buffer\n");
        return -EINVAL;
    } /* NULL buffer */

    blinkRate = gbe38v_timer_blink_rate();

    retVal = copy_to_user((int*)buff, &blinkRate, count);
    if(unlikely(retVal != SUCCESS)){
        if(retVal > SUCCESS)
            printk(KERN_ERR DEV_NAME ": read: copy_to_user(), partial copy\n");
        else /* error */
            return retVal;
    }

    return count - retVal;
}/* end gbe_read */

ssize_t gbe38v_write(struct file *filep, const char __user *buff, size_t count,
                   loff_t *offp)
{
    unsigned long retVal;
    unsigned long toWrite;

    if(unlikely(count == 0))/* write 0 bytes */
        return SUCCESS;

    if(unlikely(buff  == NULL)){
        printk(KERN_ERR DEV_NAME ": NULL __user buffer\n");
        return -EINVAL;
    } 

    if(count != sizeof(int))
        printk(KERN_ERR DEV_NAME ": Count is not sizeof(int), write will be.\n");
    
    retVal = copy_from_user(&toWrite, (int*)buff, count);
    if(unlikely(retVal != SUCCESS)){
        if(retVal > SUCCESS){
            printk(KERN_ERR DEV_NAME ": write: copy_from_user(), "
                   "partial copy, no write to register.");
            return -EIO;
        }
        else /* error */
            return retVal;
    }

    if(toWrite < 0){
        printk(KERN_ERR DEV_NAME ": blink rate cannot be negetive.\n");
        return -EINVAL;
    }
    
    gbe38v_set_timer_blink_rate(toWrite);

    return SUCCESS;
}/* end gbe_write */
/****************** EOF **************/
