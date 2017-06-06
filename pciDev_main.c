/******************************************************************************
 * filename: pci_ledDrivers.c
 *
 * This driver will make the LED blink through the PCI bus on the ethernet
 * gbe 82583v
 *
 * Written by: James Ross
 ******************************************************************************/

#include "pciDev.h"

static struct pci_device_id gbe83v_ids[] = {
    { PCI_DEVICE(PCI_VEND_ID, PCI_DEV_ID) },
    {0,} /* end: all zeros */
};
//MODULE_DEVICE_TABLE(pci, gbe88v_ids);

static struct pci_driver gbe83v = {
    .name     = DEV_NAME,
    .id_table = gbe83v_ids,
    .probe    = gbe38v_probe, 
    .remove   = gbe38v_remove
};

                    /* functions */
static int __init pci_led_init(void)
{
    int errRet;

    /* register pci device */
    errRet = pci_register_driver(&gbe83v);
    if(unlikely(errRet != SUCCESS)){
        printk(KERN_WARNING DEV_NAME ": Unable to reg pci, err: %d\n", errRet);
        return errRet;
    }
    
    return SUCCESS;

}/* end pci_led_init */

static void __exit pci_led_exit(void)
{
    pci_unregister_driver(&gbe83v);
}/* end pci_led_exit */

module_init(pci_led_init);
module_exit(pci_led_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("James Ross");
MODULE_DESCRIPTION();

MODULE_VERSION("0.1"); /* Look for convention in module.h */
/********************* EOF *******************/

