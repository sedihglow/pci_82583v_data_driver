#include <linux/module.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

MODULE_INFO(vermagic, VERMAGIC_STRING);

__visible struct module __this_module
__attribute__((section(".gnu.linkonce.this_module"))) = {
	.name = KBUILD_MODNAME,
	.init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
	.exit = cleanup_module,
#endif
	.arch = MODULE_ARCH_INIT,
};

static const struct modversion_info ____versions[]
__used
__attribute__((section("__versions"))) = {
	{ 0xe37a5a9c, __VMLINUX_SYMBOL_STR(module_layout) },
	{ 0xcf2b6910, __VMLINUX_SYMBOL_STR(pci_unregister_driver) },
	{ 0xb87504ad, __VMLINUX_SYMBOL_STR(__pci_register_driver) },
	{ 0xcbafac0a, __VMLINUX_SYMBOL_STR(x86_dma_fallback_dev) },
	{ 0x2072ee9b, __VMLINUX_SYMBOL_STR(request_threaded_irq) },
	{ 0xa72e91e3, __VMLINUX_SYMBOL_STR(pci_enable_msi_block) },
	{ 0x4c9d28b0, __VMLINUX_SYMBOL_STR(phys_base) },
	{ 0x40a9b349, __VMLINUX_SYMBOL_STR(vzalloc) },
	{ 0x42c8de35, __VMLINUX_SYMBOL_STR(ioremap_nocache) },
	{ 0x5fe56825, __VMLINUX_SYMBOL_STR(dev_set_drvdata) },
	{ 0x369b7e7c, __VMLINUX_SYMBOL_STR(pci_set_master) },
	{ 0x1d130b31, __VMLINUX_SYMBOL_STR(pci_request_selected_regions) },
	{ 0xd0ffc1f4, __VMLINUX_SYMBOL_STR(dma_supported) },
	{ 0x5d48e83a, __VMLINUX_SYMBOL_STR(dev_err) },
	{ 0xd0089522, __VMLINUX_SYMBOL_STR(dma_set_mask) },
	{ 0x629e4e99, __VMLINUX_SYMBOL_STR(pci_enable_device_mem) },
	{ 0x9a1acca6, __VMLINUX_SYMBOL_STR(device_create) },
	{ 0x5dd312bb, __VMLINUX_SYMBOL_STR(__class_create) },
	{ 0x7480084f, __VMLINUX_SYMBOL_STR(cdev_add) },
	{ 0x78340d32, __VMLINUX_SYMBOL_STR(cdev_init) },
	{ 0x29537c9e, __VMLINUX_SYMBOL_STR(alloc_chrdev_region) },
	{ 0xa8758b20, __VMLINUX_SYMBOL_STR(kmem_cache_alloc_trace) },
	{ 0xf510c4, __VMLINUX_SYMBOL_STR(kmalloc_caches) },
	{ 0x204e31ab, __VMLINUX_SYMBOL_STR(cdev_del) },
	{ 0x7485e15e, __VMLINUX_SYMBOL_STR(unregister_chrdev_region) },
	{ 0x5af89e6e, __VMLINUX_SYMBOL_STR(class_destroy) },
	{ 0x70c8bfb, __VMLINUX_SYMBOL_STR(device_destroy) },
	{ 0x659987df, __VMLINUX_SYMBOL_STR(pci_disable_device) },
	{ 0x702b08d, __VMLINUX_SYMBOL_STR(pci_release_selected_regions) },
	{ 0xe519c070, __VMLINUX_SYMBOL_STR(pci_select_bars) },
	{ 0xedc03953, __VMLINUX_SYMBOL_STR(iounmap) },
	{ 0xa2e8d51e, __VMLINUX_SYMBOL_STR(pci_disable_msi) },
	{ 0xf20dabd8, __VMLINUX_SYMBOL_STR(free_irq) },
	{ 0xb749ae89, __VMLINUX_SYMBOL_STR(dev_get_drvdata) },
	{ 0x999e8297, __VMLINUX_SYMBOL_STR(vfree) },
	{ 0x1ab42d89, __VMLINUX_SYMBOL_STR(dma_ops) },
	{ 0x37a0cba, __VMLINUX_SYMBOL_STR(kfree) },
	{ 0x2e0d2f7f, __VMLINUX_SYMBOL_STR(queue_work_on) },
	{ 0x2d3385d3, __VMLINUX_SYMBOL_STR(system_wq) },
	{ 0xf9a482f9, __VMLINUX_SYMBOL_STR(msleep) },
	{ 0x4f8b5ddb, __VMLINUX_SYMBOL_STR(_copy_to_user) },
	{ 0xc5534d64, __VMLINUX_SYMBOL_STR(ioread16) },
	{ 0x436c2179, __VMLINUX_SYMBOL_STR(iowrite32) },
	{ 0xe484e35f, __VMLINUX_SYMBOL_STR(ioread32) },
	{ 0x27e1a049, __VMLINUX_SYMBOL_STR(printk) },
	{ 0xbdfb6dbb, __VMLINUX_SYMBOL_STR(__fentry__) },
};

static const char __module_depends[]
__used
__attribute__((section(".modinfo"))) =
"depends=";


MODULE_INFO(srcversion, "6B55EB902426A4A91658793");
