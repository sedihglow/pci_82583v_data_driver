/*******************************************************************************
 * filename: picDev.c
 *
 * Implements the functions defined in pciDev.h
 *
 * Written by: James Ross
 ******************************************************************************/
#include "pciDev.h"

#define DEBUG

#define MM_BAR0    0 /* memory map IO on 82583 is bar0 */
#define CLEAR_MASK 0xF

            /* global structs */
static struct file_operations fops = {
    .owner   = THIS_MODULE,
    .open    = gbe38v_open,
    .read    = gbe38v_read,
    .write   = gbe38v_write,
	.release = gbe38v_release
};

static struct pci_device_id gbe83v_ids[] = {
    { PCI_DEVICE(PCI_VEND_ID, PCI_DEV_ID) },
    {0,} /* end: all zeros */
};
MODULE_DEVICE_TABLE(pci, gbe83v_ids);

static struct pci_driver gbe83v = {
    .name     = DEV_NAME,
    .id_table = gbe83v_ids,
    .probe    = gbe38v_probe, 
    .remove   = gbe38v_remove
};

/*******************************************************************************
 *                      led functionality
 ******************************************************************************/
static void set_led(void __iomem *ledReg, int ledNums, bool setTo)
{
    u32 toWrite;
    u32 setMode = ((setTo) ? LED_ON : LED_OFF);

    toWrite = ioread32(ledReg);

    if(ledNums & RIGHT_GRN){
        if(setTo == LOW){
            /* clear parity bit if setMode is low */
            toWrite &= ~(INV_BIT);
        }

        toWrite &= CLEAR_MODE(RIGHT_GRN_SHIFT); /* clear mode bits */
        toWrite |= (setMode << RIGHT_GRN_SHIFT);
    }
    else if(ledNums & LEFT_GRN){
        toWrite &= CLEAR_MODE(LEFT_GRN_SHIFT); /* clear mode bits */
        toWrite |= (setMode << LEFT_GRN_SHIFT);
    }
    else{ /* amber LED */
        toWrite &= CLEAR_MODE(LEFT_AMBER_SHIFT); /* clear mode bits */
        toWrite |= (setMode << LEFT_AMBER_SHIFT);
    }
    
    iowrite32(toWrite, ledReg);
}/* end set_led */

static void disable_all_leds(struct gbe38v_dev *myDev)
{
    void __iomem *workReg = HWREG(myDev->hwAddr, LED_OFFSET);
    iowrite32(0x0, workReg);
    set_led(workReg, LEFT_AMBER, LOW);
    set_led(workReg, RIGHT_GRN,  LOW);
    set_led(workReg, LEFT_GRN,   LOW);
}
/*******************************************************************************
 *                          Head and Tail managment
 ******************************************************************************/
static void gbe38v_handle_desc(struct gbe38v_ring *rxRing)
{
    struct gbe38v_rx_desc *desc;
    int ddRet;
    u16 cleanNext = rxRing->cleanNext;

	cleanNext = rxRing->cleanNext;
	desc = GBE38V_DESC(rxRing, cleanNext, gbe38v_rx_desc);

    ddRet = desc->upper.fields.status & DD_BIT;
    if(!ddRet)
        return;

    /* go through each descriptor and clean the status if needed */
    do{
		desc->upper.fields.status = 0;
        
        /* bump tail */
		cleanNext++;
		if (cleanNext >= rxRing->count)
			cleanNext = 0;

		/* get the next descriptor in the ring */
        desc = GBE38V_DESC(rxRing, cleanNext, gbe38v_rx_desc);
	}while(desc->upper.fields.status & DD_BIT);

    iowrite16(cleanNext, rxRing->tail);

    rxRing->cleanNext = cleanNext;
}

/*******************************************************************************
 *                          Workqueue
 ******************************************************************************/
static void print_desc_info(struct gbe38v_ring *rxRing)
{
    void __iomem *workReg;
    struct gbe38v_rx_desc *desc;
    u16 start;
    u16 end;

    /* for each descriptor between HEAD and TAIL, read the descriptor from the
     * ring. */
    workReg = HWREG(rxRing->myDev->hwAddr, RX_HEAD_OFFSET);
    start = ioread16(workReg);

    workReg = HWREG(rxRing->myDev->hwAddr, RX_TAIL_OFFSET);
    end = ioread16(workReg);

    if(start == end)
        return;

    for(; start < end; ++start){
        /* get descriptor */
        desc = GBE38V_DESC(rxRing, start, gbe38v_rx_desc);

        /* print length and status */
        printk(KERN_INFO DEV_NAME ":desc %hu, status %X, length %X\n",
                                                    start,
                                                    desc->upper.fields.status,
                                                    desc->lower.flags.length);
    }
}


static void rx_wq_task(struct work_struct *work)
{
    struct gbe38v_dev  *myDev  = container_of(work, struct gbe38v_dev, workq);
    struct gbe38v_ring *rxRing = myDev->rxRing;
    void __iomem *ledReg  = HWREG(myDev->hwAddr, LED_OFFSET);

    /* sleep .5 sec */
    mdelay(WQ_SLEEP);

    /* turn off both green LED's */
    set_led(ledReg, LEFT_GRN, LOW);
    set_led(ledReg, RIGHT_GRN, LOW);

    print_desc_info(myDev->rxRing);

    gbe38v_handle_desc(rxRing);
   
    /* enable LSC and RXQ0 interrupts */
    iowrite32(INT_MASK, myDev->hwAddr + IMS_OFFSET);
    FLUSH(myDev);
}

/*******************************************************************************
 *                          irq interrupts
 ******************************************************************************/
static irqreturn_t gbe38v_rx_handle(int irq, void *dev_id)
{
    struct gbe38v_dev *myDev = dev_id;
    void __iomem *workReg;
    u32 icr;
    
    workReg = HWREG(myDev->hwAddr,ICR_OFFSET);
    icr = ioread32(workReg);

    /* Disable interrupts */
    workReg = HWREG(myDev->hwAddr, IMC_OFFSET);
    iowrite32(DISABLE_INTS, workReg);
    FLUSH(myDev);
    

    if(!(icr & INT_ASSERTED)){
        iowrite32(INT_MASK, myDev->hwAddr + IMS_OFFSET);
        return IRQ_NONE;
    }

    if(icr & LSC){
        iowrite32(INT_MASK, myDev->hwAddr + IMS_OFFSET);
        return IRQ_HANDLED;
    }

    /* turn both green LED's on */
    workReg = HWREG(myDev->hwAddr, LED_OFFSET);
    set_led(workReg, LEFT_GRN, HIGH);
    set_led(workReg, RIGHT_GRN, HIGH);

    /* submit workqueue thread to run */
    schedule_work(&myDev->workq);


    return IRQ_HANDLED;
}

static int gbe38v_setup_interrupts(struct gbe38v_dev *myDev)
{
    struct pci_dev *pdev = myDev->pdev;
    void __iomem *maskReg = HWREG(myDev->hwAddr, IMS_OFFSET);
    int err;

    err = request_irq(pdev->irq, gbe38v_rx_handle, IRQF_SHARED, DEV_NAME, myDev);
    if(err){
        return err;
    }

    /* enable LSC and RXQ0 interrupts */
    iowrite32(INT_MASK, maskReg);
    return SUCCESS;
}

/*******************************************************************************
 *                      dma allocations
 ******************************************************************************/
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
    struct gbe38v_buffer *bufferInfo; 
    struct pci_dev *pdev = rxRing->myDev->pdev;
    struct gbe38v_rx_desc *descRing; 

    rxRing->size = sizeof(struct gbe38v_rx_desc) * rxRing->count;
	rxRing->size = ALIGN(rxRing->size, ONE_PAGE);
    
    /* allocate descriptors */
	err = gbe38v_alloc_dma_ring(pdev, rxRing);
	if(err)
	    goto dma_ring_fail;

	rxRing->cleanNext = 0;

    /* allocate buffer info */
    rxRing->bufferInfo = VZALLOC_ARRAY(struct gbe38v_buffer, rxRing->count);
    bufferInfo = rxRing->bufferInfo;
    if(!rxRing->bufferInfo){
        printk(KERN_ERR DEV_NAME ":buffer info failed to allocate\n");
        goto buffInfo_alloc_fail;
    }

    /* allocate buffer addresses and pin them */
    for(i=0; i < rxRing->count; ++i){
        bufferInfo[i].desc = KZALLOC_VOID(DBUF_SIZE, GFP_KERNEL);
        if(!bufferInfo[i].desc){
            printk(KERN_ERR DEV_NAME ": Failed to allocate descriptor address "
                   "buffer %d\n",i);
            goto alloc_fail;
        }

        bufferInfo[i].size = DBUF_SIZE;
        bufferInfo[i].dma = dma_map_single(&pdev->dev, bufferInfo[i].desc, 
                                           DBUF_SIZE, DMA_BIDIRECTIONAL);
        if(!bufferInfo[i].dma){
            printk(KERN_ERR DEV_NAME ": Failed to DMA buffer address %d\n",i);
            goto alloc_fail;
        }

        /* Fill descriptor addresses */
        descRing = GBE38V_DESC(rxRing, i, gbe38v_rx_desc);
        //((struct gbe38v_rx_desc *)(bufferInfo[i].desc));
        descRing->bufferAddr = cpu_to_le64(bufferInfo[i].dma);
        descRing->lower.flags.length = cpu_to_le16(DBUF_SIZE);
        descRing->upper.data = 0;
    }
    
    rxRing->head = HWREG(rxRing->myDev->hwAddr, RX_HEAD_OFFSET);
    rxRing->tail = HWREG(rxRing->myDev->hwAddr, RX_TAIL_OFFSET);

    return SUCCESS;

/* error handling */
buffInfo_alloc_fail:
    dma_free_coherent(&pdev->dev, rxRing->size, rxRing->desc, rxRing->dma);
alloc_fail:
    err = -ENOMEM;
    /* unpin and free all descriptors allocated */
    kfree(bufferInfo[i].desc); /* if first buff failed, this is nop */
    for(--i; i >= 0; --i){
        dma_unmap_single(&pdev->dev, bufferInfo[i].dma, DBUF_SIZE, 
                         DMA_BIDIRECTIONAL);
        kfree(bufferInfo[i].desc);
    }
    vfree(bufferInfo);
dma_ring_fail:
    return err;
}

static void gbe38v_free_rx_resources(struct gbe38v_dev *myDev)
{
    struct pci_dev *pdev = myDev->pdev;
    struct gbe38v_ring *rxRing = myDev->rxRing;
    struct gbe38v_buffer *bufferInfo = rxRing->bufferInfo;
    int i;

    for(i=0; i < rxRing->count; ++i){
        /* unpin descriptors in buffer */
        dma_unmap_single(&pdev->dev, bufferInfo[i].dma, bufferInfo[i].size,
                         DMA_BIDIRECTIONAL);
        /* kfree descriptors in buffer */
        kfree(bufferInfo[i].desc);
    }
    /* free buffer */
    vfree(rxRing->bufferInfo);

    /* unpin and free desc ring */
    dma_free_coherent(&pdev->dev, rxRing->size, rxRing->desc, rxRing->dma);

    /* unpin gbe38v_ring data */
    kfree(myDev->rxRing);
}

static int gbe38v_setup_dma(struct gbe38v_dev *myDev)
{
    int err;
    myDev->rxRing = KZALLOC(struct gbe38v_ring, GFP_KERNEL);
    if(!myDev->rxRing){
        printk(KERN_ERR DEV_NAME ": Rxring failed to alloc");
        return -ENOMEM;
    }

    myDev->rxRing->myDev = myDev;
    myDev->rxRing->count = DESC_CNT;
    
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

    /* set Rx head */
    workReg = HWREG(hwAddr, RX_HEAD_OFFSET);
    iowrite32(HEAD_INIT, workReg);
    FLUSH(myDev);

    /* set Rx tail "one descriptor beyone the end" */
    workReg = HWREG(hwAddr, RX_TAIL_OFFSET);
    iowrite32(TAIL_INIT, workReg);
    FLUSH(myDev);

    /* set Rx decriptor ring length */
    workReg = HWREG(hwAddr, RDLEN_OFFSET);
    toWrite = RDL_SET(RDL_LEN);
    iowrite32(toWrite, workReg);
    FLUSH(myDev);

    /* set high and low addr of Rx descriptor ring */
    workReg = HWREG(hwAddr, RDBAH_OFFSET);
    toWrite = (rxRing->dma >> 32) & 0xFFFFFFFF; /* high 32 bits */
    iowrite32(toWrite, workReg);
    FLUSH(myDev);

    workReg = HWREG(hwAddr, RDBAL_OFFSET);
    toWrite = rxRing->dma & 0xFFFFFFFF; /* low 32 bits */
    iowrite32(toWrite, workReg);
    FLUSH(myDev);
    msleep(100);
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

    /* Disable all interrupts */
    workReg = HWREG(hwAddr, IMC_OFFSET);
    iowrite32(DISABLE_INTS, workReg);
    FLUSH(myDev);

    /* Reset mac function */
    workReg = HWREG(hwAddr, DEVCTL_OFFSET);
    toWrite = ioread32(workReg) | MAC_RST_BIT;
    iowrite32(toWrite, workReg);
    //while((ioread32(workReg) & MAC_RST_BIT) != 0);
    FLUSH(myDev);
    msleep(25);

    /* Disable all interrupts */
    workReg = HWREG(hwAddr, IMC_OFFSET);
    iowrite32(DISABLE_INTS, workReg);
    FLUSH(myDev);

    /* Set GCR bit 22 as requested by init sequence in datasheet on page 53 */
    workReg = HWREG(hwAddr, GCR_OFFSET);
    toWrite = ioread32(workReg) | GCR_INIT; 
    iowrite32(toWrite, workReg);
    FLUSH(myDev);

    /* Set GCR2 bit 1. Said in datasheet in register information, cant find it
     * requesting the bit in chapter 4 of datasheet on initialization. */
    workReg = HWREG(hwAddr, GCR2_OFFSET);
    toWrite = ioread32(workReg) | GCR2_INIT; 
    iowrite32(toWrite, workReg);
    FLUSH(myDev);

    /* PHY setup */
    workReg = HWREG(hwAddr, MDIC_OFFSET);
    iowrite32(PHY_INIT, workReg);
    FLUSH(myDev);
    msleep(100);

    /* init rx registers */
    gbe38v_init_rx_reg(myDev);

    /* set promiscuous mode, enable recieve and BAM */
    workReg = HWREG(myDev->hwAddr, RXCTL_OFFSET);
    toWrite = ioread32(workReg) | RXCTL_INIT;
    iowrite32(toWrite, workReg);
    FLUSH(myDev);
    msleep(100);

    /* force link up  */
    workReg = HWREG(myDev->hwAddr, DEVCTL_OFFSET);
    toWrite = ioread32(workReg) | SLU_BIT;
    iowrite32(toWrite, workReg);
    FLUSH(myDev);
    msleep(50);

}

static int myDev_init(struct gbe38v_dev *myDev)
{
    int err;

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
    err = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
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
        dev_err(&pdev->dev, DEV_NAME ": failed to setup dma, err: %d", err);
        goto dma_setup_fail;
    }

    INIT_WORK(&myDev->workq, rx_wq_task);

    init_device_reg(myDev); 

	/* disable LEDs */ 
	disable_all_leds(myDev);

    /* set up interupt handler */
    err = gbe38v_setup_interrupts(myDev);
    if(err){
        /* TODO: print statment */
        goto int_setup_fail;
    }

    return SUCCESS;

/* error handling */
int_setup_fail:
    gbe38v_free_rx_resources(myDev);
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

    free_irq(pdev->irq, myDev);
    gbe38v_free_rx_resources(myDev);
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

    return SUCCESS;
}/* end gbe38v_open */

int gbe38v_release(struct inode *inode, struct file *file)
{
    struct gbe38v_dev *myDev;
    myDev = container_of(inode->i_cdev, struct gbe38v_dev, cdev);
    file->private_data = myDev;

    return SUCCESS;
}/* end gbe38v_release */

ssize_t gbe38v_read(struct file *filep, char __user *buff, size_t count, 
                  loff_t *offp)
{
    struct gbe38v_dev *myDev = filep->private_data;
    unsigned long retVal;
    void __iomem *workReg;
    u32 sendBuff;
    u32 head;
    u16 tail;
    
    /* read 0 bytes */
    if(unlikely(count == 0))
        return 0;

    if(count != sizeof(u32))
        printk(KERN_ERR DEV_NAME ": Count is not sizeof(u32), read will be.\n");
    
    if(unlikely(buff  == NULL)){
        printk(KERN_ERR DEV_NAME ": NULL __user buffer\n");
        return -EINVAL;
    } /* NULL buffer */
   
    /* get head and tail */
    workReg = HWREG(myDev->hwAddr, RX_HEAD_OFFSET); 
    head = ioread16(workReg);

    workReg = HWREG(myDev->hwAddr, RX_TAIL_OFFSET);
    tail = ioread16(workReg);

    /* pack into 64 bits */
    sendBuff  = tail;
    sendBuff |= (head << 16);

    retVal = copy_to_user((u32*)buff, &sendBuff, count);
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
    return SUCCESS;
}/* end gbe_write */

static int __init pci_dev_init(void)
{
    int errRet;
    
    /* register pci device */
    errRet = pci_register_driver(&gbe83v);
    if(unlikely(errRet != SUCCESS)){
        printk(KERN_ERR DEV_NAME ": Unable to reg pci, err: %d\n", errRet);
        return errRet;
    }
    
    return SUCCESS;

}

static void __exit pci_dev_exit(void)
{
    pci_unregister_driver(&gbe83v);
}

module_init(pci_dev_init);
module_exit(pci_dev_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("James Ross");
MODULE_DESCRIPTION();

MODULE_VERSION("0.1"); /* Look for convention in module.h */
/********************* EOF *******************/
