/******************************************************************************
 * filename: pciInt.h
 *
 * written by: James Ross
 ******************************************************************************/

#ifndef _PCI_INT_H_
#define _PCI_INT_H_

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

#define DEV_NAME "pciDev"
int gbe38v_setup_interrupts(struct gbe38v_dev *myDev);

#endif
