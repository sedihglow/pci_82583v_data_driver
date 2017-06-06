/******************************************************************************
 * filename: pciInt.c
 *
 * written by: James Ross
 ******************************************************************************/

#include "pciInt.h"

static irqreturn_t gbe38v_rx_wq_handle(int irq, void *dev_id, struct pt_regs *regs)
{
     

    return IRQ_HANDLED;
}

int gbe38v_setup_interrupts(struct gbe38v_dev *myDev)
{
    struct pci_dev pdev = myDev->pdev;
    pci_enable_msi(pdev);
    request_irq(pdev->irq, gbe38v_rx_int_handle, 0, DEV_NAME, myDev);
    return SUCCESS;
}
