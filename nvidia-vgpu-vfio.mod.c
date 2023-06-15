#include <linux/module.h>
#define INCLUDE_VERMAGIC
#include <linux/build-salt.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

BUILD_SALT;

MODULE_INFO(vermagic, VERMAGIC_STRING);
MODULE_INFO(name, KBUILD_MODNAME);

__visible struct module __this_module
__section(".gnu.linkonce.this_module") = {
	.name = KBUILD_MODNAME,
	.init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
	.exit = cleanup_module,
#endif
	.arch = MODULE_ARCH_INIT,
};

#ifdef CONFIG_RETPOLINE
MODULE_INFO(retpoline, "Y");
#endif

static const struct modversion_info ____versions[]
__used __section("__versions") = {
	{ 0xe32130cf, "module_layout" },
	{ 0xe6881b4b, "dma_map_sg_attrs" },
	{ 0xb4ff84a7, "cdev_del" },
	{ 0x9c4befaf, "kmalloc_caches" },
	{ 0xf61b8642, "pci_write_config_dword" },
	{ 0xeb233a45, "__kmalloc" },
	{ 0xecc5c1dd, "cdev_init" },
	{ 0xf9a482f9, "msleep" },
	{ 0x7edd7413, "pci_free_irq_vectors" },
	{ 0xa3f3529d, "pci_write_config_word" },
	{ 0x7aa1756e, "kvfree" },
	{ 0xdf0f75c6, "eventfd_signal" },
	{ 0x754d539c, "strlen" },
	{ 0xda02a56b, "pci_read_config_byte" },
	{ 0x46cf10eb, "cachemode2protval" },
	{ 0x4762ae61, "remap_vmalloc_range" },
	{ 0x1ca1159d, "dma_unmap_sg_attrs" },
	{ 0x747b0368, "mdev_set_drvdata" },
	{ 0x36c748e4, "dma_set_mask" },
	{ 0x5367b4b4, "boot_cpu_data" },
	{ 0x7b02e154, "pci_disable_device" },
	{ 0x20000329, "simple_strtoul" },
	{ 0xe79f875d, "vfio_unregister_notifier" },
	{ 0xf7322bce, "mdev_uuid" },
	{ 0x8f3ec4de, "pcie_capability_read_dword" },
	{ 0x93801aef, "mdev_unregister_device" },
	{ 0x87b8798d, "sg_next" },
	{ 0xf4a71226, "mdev_dev" },
	{ 0x7b699356, "pci_write_config_byte" },
	{ 0xd6e34ad6, "pcie_capability_clear_and_set_word" },
	{ 0x409bcb62, "mutex_unlock" },
	{ 0x6091b333, "unregister_chrdev_region" },
	{ 0x999e8297, "vfree" },
	{ 0x125b23f4, "mdev_register_device" },
	{ 0x97651e6c, "vmemmap_base" },
	{ 0x3c3ff9fd, "sprintf" },
	{ 0xdc4c58b0, "mdev_get_drvdata" },
	{ 0x6c28be5a, "vfio_info_add_capability" },
	{ 0xeaae65e, "dma_set_coherent_mask" },
	{ 0xf1d9b346, "vfio_register_notifier" },
	{ 0xece784c2, "rb_first" },
	{ 0xd9a5ea54, "__init_waitqueue_head" },
	{ 0x6b10bee1, "_copy_to_user" },
	{ 0x5b8239ca, "__x86_return_thunk" },
	{ 0x3609e70d, "pci_set_master" },
	{ 0x7abe0dc0, "pci_alloc_irq_vectors_affinity" },
	{ 0xfb578fc5, "memset" },
	{ 0x86a52ae0, "pci_restore_state" },
	{ 0xd35cce70, "_raw_spin_unlock_irqrestore" },
	{ 0x1a0ed64, "current_task" },
	{ 0xc5850110, "printk" },
	{ 0xe1537255, "__list_del_entry_valid" },
	{ 0xd67364f7, "eventfd_ctx_fdget" },
	{ 0x4c9d28b0, "phys_base" },
	{ 0x531b604e, "__virt_addr_valid" },
	{ 0x4d9b652b, "rb_erase" },
	{ 0x79bc98e5, "pci_read_config_word" },
	{ 0x2ab7989d, "mutex_lock" },
	{ 0x78a5c41d, "pci_wait_for_pending_transaction" },
	{ 0x94048899, "vfio_unpin_pages" },
	{ 0x1e6d26a8, "strstr" },
	{ 0x6626afca, "down" },
	{ 0xf9471b9f, "mdev_parent_dev" },
	{ 0x92d5838e, "request_threaded_irq" },
	{ 0x68f31cbd, "__list_add_valid" },
	{ 0xfe487975, "init_wait_entry" },
	{ 0x868784cb, "__symbol_get" },
	{ 0xa2f381b5, "cdev_add" },
	{ 0x599fb41c, "kvmalloc_node" },
	{ 0x7cd8d75e, "page_offset_base" },
	{ 0xfc177fae, "module_put" },
	{ 0xd8ee461d, "mdev_set_iommu_device" },
	{ 0x6a5cb5ee, "__get_free_pages" },
	{ 0x5635a60a, "vmalloc_user" },
	{ 0xc959d152, "__stack_chk_fail" },
	{ 0x8ddd8aad, "schedule_timeout" },
	{ 0x2ea2c95c, "__x86_indirect_thunk_rax" },
	{ 0xabc0f726, "pci_read_config_dword" },
	{ 0xaf54fefd, "dev_driver_string" },
	{ 0xbdfb6dbb, "__fentry__" },
	{ 0x5562b026, "kmem_cache_alloc_trace" },
	{ 0x34db050b, "_raw_spin_lock_irqsave" },
	{ 0xa5526619, "rb_insert_color" },
	{ 0x146d5510, "pci_irq_vector" },
	{ 0x4302d0eb, "free_pages" },
	{ 0x3eeb2322, "__wake_up" },
	{ 0x29941ef8, "pci_set_power_state" },
	{ 0x8c26d495, "prepare_to_wait_event" },
	{ 0xae04012c, "__vmalloc" },
	{ 0x37a0cba, "kfree" },
	{ 0x1f99333, "remap_pfn_range" },
	{ 0x6323a2f8, "unmap_mapping_range" },
	{ 0x69acdf38, "memcpy" },
	{ 0xbf0cc2ad, "send_sig_info" },
	{ 0xcf2a6966, "up" },
	{ 0xe616e2df, "mdev_from_dev" },
	{ 0xfd301ed, "sg_alloc_table_from_pages" },
	{ 0x92540fbf, "finish_wait" },
	{ 0x7f5b4fe4, "sg_free_table" },
	{ 0x6e9dd606, "__symbol_put" },
	{ 0x9291cd3b, "memdup_user" },
	{ 0x656e4a6e, "snprintf" },
	{ 0xadc044b7, "vfio_set_irqs_validate_and_prepare" },
	{ 0x941f2aaa, "eventfd_ctx_put" },
	{ 0x19567d06, "vfio_info_cap_shift" },
	{ 0xfbf07e72, "vfio_pin_pages" },
	{ 0x73303297, "pci_enable_device" },
	{ 0x13c49cc2, "_copy_from_user" },
	{ 0x88db9f48, "__check_object_size" },
	{ 0xe3ec2f2b, "alloc_chrdev_region" },
	{ 0xffb8d0cf, "try_module_get" },
	{ 0xc1514a3b, "free_irq" },
	{ 0x46a2eecb, "pci_save_state" },
	{ 0x8a35b432, "sme_me_mask" },
};

MODULE_INFO(depends, "mdev,vfio");

MODULE_ALIAS("pci:v000010DEd*sv*sd*bc03sc00i00*");
MODULE_ALIAS("pci:v000010DEd*sv*sd*bc03sc02i00*");
MODULE_ALIAS("pci:v000010DEd*sv*sd*bc06sc80i00*");

MODULE_INFO(srcversion, "949918549C9D51D59FDE846");
