#include <linux/module.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

MODULE_INFO(vermagic, VERMAGIC_STRING);

struct module __this_module
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
	{ 0x741d9962, "struct_module" },
	{ 0xf3349a4d, "remap_pfn_range" },
	{ 0x8e092f2a, "dma_alloc_coherent" },
	{ 0xd6ee688f, "vmalloc" },
	{ 0x1fc91fb2, "request_irq" },
	{ 0x4c3af445, "__request_region" },
	{ 0xda73a1d0, "__check_region" },
	{ 0xfbed20d8, "ioremap_nocache" },
	{ 0xd50d2291, "pci_enable_device" },
	{ 0x565d8bb0, "cdev_add" },
	{ 0x1653339d, "cdev_init" },
	{ 0x29537c9e, "alloc_chrdev_region" },
	{ 0xc64c75d, "pci_disable_device" },
	{ 0x7485e15e, "unregister_chrdev_region" },
	{ 0xa4ca496c, "cdev_del" },
	{ 0xedc03953, "iounmap" },
	{ 0xf20dabd8, "free_irq" },
	{ 0x8bb33e7d, "__release_region" },
	{ 0xdc3eaf70, "iomem_resource" },
	{ 0x999e8297, "vfree" },
	{ 0xae9addc8, "dma_free_coherent" },
	{ 0x994e1983, "__wake_up" },
	{ 0x5878e0d0, "finish_wait" },
	{ 0x1000e51, "schedule" },
	{ 0x2caa52dc, "prepare_to_wait" },
	{ 0xc8b57c27, "autoremove_wake_function" },
	{ 0x3302b500, "copy_from_user" },
	{ 0xe914e41e, "strcpy" },
	{ 0x1d26aa98, "sprintf" },
	{ 0x85f8a266, "copy_to_user" },
	{ 0x900e0d6, "_spin_lock" },
	{ 0x6665e783, "fasync_helper" },
	{ 0x8a38fe0c, "__pci_register_driver" },
	{ 0xde0bdcff, "memset" },
	{ 0xffd3c7, "init_waitqueue_head" },
	{ 0x57f46cef, "_spin_lock_irq" },
	{ 0xb906bb64, "pci_unregister_driver" },
	{ 0xdd132261, "printk" },
};

static const char __module_depends[]
__used
__attribute__((section(".modinfo"))) =
"depends=";

MODULE_ALIAS("pci:v000010EEd00000007sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v00001A4Ad00002020sv*sd*bc*sc*i*");

MODULE_INFO(srcversion, "471F9CF226EA807AC05E3E3");
