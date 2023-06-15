/*
 * Copyright (c) 2018-2021, NVIDIA CORPORATION. All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#ifndef _NVIDIA_VGPU_VFIO_H_
#define _NVIDIA_VGPU_VFIO_H_

#include "conftest.h"
#include "nvstatus.h"
#include "nv-vgpu-ioctl.h"
#include "nv-hypervisor.h"

#if defined(NV_VGPU_KVM_BUILD)
#include "nvmisc.h"
#include <linux/uuid.h>
#include <linux/vfio.h>
#include <linux/iommu.h>
#include <linux/mdev.h>
#include <linux/list.h>
#include <linux/pci.h>
#include <linux/sched.h>
#include <linux/eventfd.h>
#include <linux/anon_inodes.h>
#include "nv-vgpu-vfio-interface.h"
#if defined(NV_USE_VFIO_PCI_CORE)
#include <linux/vfio_pci_core.h>
#endif

struct vgpu_dev_s;
struct phys_dev_s;
struct mapping_node_s;

static NV_STATUS  nv_vgpu_invalidate_guest_mmio(NvU64, NvU64, NvBool, NvU64, void *);
NV_STATUS  nv_vgpu_inject_interrupt(void *);
NV_STATUS  nv_vgpu_unpin_pages(NvU64 *, NvU32, void *, VGPU_ADDR_TYPE_T);
NV_STATUS  nv_vgpu_translate_gfn_to_pfn(NvU64 *, NvU64 *, NvU32, void *, VGPU_ADDR_TYPE_T);
NV_STATUS  nv_vgpu_update_mapping(void *, NvU64, NvU64, NvU64, NvBool, NvBool);
NV_STATUS  nv_vgpu_init_pci_config(NvU8 *, struct vgpu_dev_s *);
NV_STATUS  nv_get_vgpu_type_id(struct kobject *, struct device *, NvU32 *, void *);

static NV_STATUS  nv_vgpu_probe(struct pci_dev *dev, NvU32, NvU32 *);
static NV_STATUS  nv_vgpu_vfio_validate_map_request(struct vgpu_dev_s *, loff_t, NvU64 *,
                                                    NvU64 *, NvU64 *, pgprot_t *, NvBool *);
static void       nv_vgpu_remove(struct pci_dev *);
int        nv_alloc_vgpu_dev(struct vgpu_dev_s **);
void       nv_free_vgpu_dev(struct vgpu_dev_s *);
int        nv_vgpu_vfio_create(struct vgpu_dev_s *, struct pci_dev *, NvU32);
int        nv_vgpu_vfio_destroy(struct vgpu_dev_s *, struct device *);
int        nv_vgpu_vfio_open(struct vgpu_dev_s *);
void       nv_vgpu_vfio_close(struct vgpu_dev_s *);
int        nv_vgpu_vfio_mmap(struct vgpu_dev_s *, struct vm_area_struct *);
long       nv_vgpu_vfio_ioctl(struct vgpu_dev_s *, unsigned int,
                                     unsigned long);
ssize_t    nv_vgpu_vfio_read(struct vgpu_dev_s *, char __user *, size_t,
                                    loff_t *);
ssize_t    nv_vgpu_vfio_write(struct vgpu_dev_s *, const char __user *,
                                     size_t, loff_t *);
static ssize_t    nv_vgpu_vfio_access(struct vgpu_dev_s *, char *, size_t, loff_t,
                                      NvBool);
static void       nv_destroy_dma_mappings(struct vgpu_dev_s *,struct mapping_node_s *);
NV_STATUS nv_vfio_vgpu_vf_reg_access_from_plugin(struct pci_dev *pdev, VGPU_EMUL_SPACE_T emulSpace,
                                                 NvU64 offset, NvU32 width, NvU8* data, NvBool isWrite);
NV_STATUS  nv_vgpu_set_mdev_fops(struct phys_dev_s *);
void       nv_vgpu_free_mdev_fops(struct phys_dev_s *);
void       nv_vgpu_vfio_dma_unmap(struct vgpu_dev_s *, NvU64, NvU64);

#if defined(NV_VFIO_DEVICE_MIG_STATE_PRESENT)
struct file *nv_vgpu_set_device_state(struct vfio_device *core_vdev, struct vgpu_dev_s *vgpu_dev,
                                      enum vfio_device_mig_state new_state);
#endif

#if defined(NV_USE_VFIO_PCI_CORE)
NV_STATUS   nv_vfio_pci_core_init(struct phys_dev_s *);
void        nv_vfio_pci_core_uninit(struct phys_dev_s *);
#else
/* Stub functions */
static inline NV_STATUS   nv_vfio_pci_core_init(struct phys_dev_s * phys_dev)
{
    return NV_OK;
}
static inline void nv_vfio_pci_core_uninit(struct phys_dev_s *phys_dev) { }
#endif

#if defined(NV_MDEV_PARENT_OPS_HAS_DEVICE_DRIVER) || defined(NV_MDEV_DRIVER_HAS_SUPPORTED_TYPE_GROUPS)
int nv_vfio_mdev_register_driver(void);
void nv_vfio_mdev_unregister_driver(void);
#endif
extern const struct attribute_group *vgpu_dev_groups[];

#define GET_VGPU_DEV_NAME(vgpu_dev, name)               \
do                                                      \
{                                                       \
    if (vgpu_dev->mdev)                                 \
        name = NV_GET_MDEV_UUID(vgpu_dev->mdev);        \
    else                                                \
        name = dev_name(vgpu_dev->dev);                 \
} while(0)

#if defined(NV_MDEV_UUID_PRESENT)
  #if defined(NV_MDEV_UUID_RETURN_GUID_PTR)
    #define NV_GET_MDEV_UUID(mdev)              \
        ({                                      \
             const guid_t *__temp;              \
             __temp = mdev_uuid(mdev);          \
             __temp->b;                         \
        })
  #else
    #define NV_GET_MDEV_UUID(mdev)      \
        ({                              \
             uuid_le __temp;            \
             __temp = mdev_uuid(mdev);  \
             __temp.b;                  \
        })
  #endif
#else
#define NV_GET_MDEV_UUID(mdev) (mdev)->uuid.b
#endif

#if defined(NV_MDEV_DEV_PRESENT)
#define NV_GET_MDEV_DEV(mdev) mdev_dev(mdev)
#else
#define NV_GET_MDEV_DEV(mdev) &(mdev)->dev
#endif

#if defined(NV_MDEV_PARENT_DEV_PRESENT)
#define NV_GET_MDEV_PARENT(mdev) mdev_parent_dev(mdev)
#else
#define NV_GET_MDEV_PARENT(mdev) (mdev)->parent->dev
#endif

#if defined(NV_MDEV_FROM_DEV_PRESENT)
#define NV_GET_MDEV_FROM_DEV(dev) mdev_from_dev(dev)
#else
#define NV_GET_MDEV_FROM_DEV(dev) to_mdev_device(dev)
#endif

#define NV_DMA_ALLOC_SG_TABLE(sgt, pages, cnt)          \
    sg_alloc_table_from_pages(&sgt, pages, cnt,         \
                              0, cnt * PAGE_SIZE,       \
                              NV_GFP_KERNEL)

#define NV_DMA_FREE_SG_TABLE(sgt) sg_free_table(&sgt)

#endif /* NV_VGPU_KVM_BUILD */

typedef enum
{
    VGPU_INFO = 0,
    VGPU_ERR = 1
} VGPU_VFIO_LOG_LEVEL_T;

static inline VGPU_VFIO_LOG_LEVEL_T get_vgpu_vfio_log_level(void)
{
#if defined(DEBUG)
    return VGPU_INFO;
#else
    return VGPU_ERR;
#endif
}

#define NV_VGPU_STRING_BUFFER_SIZE 32

#define NV_VGPU_LOG(lvl, fmt, ...)                                   \
    if (lvl >= get_vgpu_vfio_log_level())                            \
        printk(KERN_ERR NV_VGPU_VFIO_LOG_PREFIX fmt, ##__VA_ARGS__)  \

#define NV_VGPU_DEV_LOG(lvl, vgpu_dev, fmt, ...)                     \
    if (lvl >= get_vgpu_vfio_log_level())                            \
       printk(KERN_ERR NV_VGPU_VFIO_LOG_PREFIX                       \
              "%s: " fmt, dev_name(vgpu_dev->dev), ##__VA_ARGS__) \

#define NV_VGPU_VFIO_PGOFF_PCI_OFFSET    39:0
#define NV_VGPU_VFIO_PGOFF_PCI_INDEX     43:40
#define NV_VGPU_VFIO_PGOFF_UNIQUE_ID     59:44

#define NV_VGPU_VFIO_LOG_PREFIX "[nvidia-vgpu-vfio] "

#define PCI_EXTENDED_CONFIG_SPACE_SIZE 4096
#define VGPU_UUID_SIZE 16
#define VGPU_CONFIG_PARAMS_MAX_LENGTH 1024
#define VM_NAME_SIZE 128
#define GFN_PINNED ((NvU64)~0)

// RM can only do 1MB of copy_from_user().
// So, a single RmControl can pin this much pages.
#define NV_VGPU_MAX_PAGE_COUNT ((1024 * 1024) / sizeof(NvU64))

// Mapping cache size for 1GB of system memory.
// Also, in power of 2 to use with NV_ALIGN_UP macro.
#define NV_INITIAL_CACHE_SIZE (256 * 1024)

#define INVALID_IRQ 0xBEEF

#if defined(NV_VFIO_DEVICE_MIGRATION_HAS_START_PFN)
#define NV_VGPU_VFIO_MIGRATION_REGION   (VFIO_PCI_NUM_REGIONS + 0)
#define NV_VGPU_VFIO_REGIONS_MAX        (NV_VGPU_VFIO_MIGRATION_REGION + 1)
#else
#define NV_VGPU_VFIO_REGIONS_MAX        (VFIO_PCI_NUM_REGIONS + 0)
#endif
/* Console region should be always last region */
#define NV_VGPU_VFIO_CONSOLE_REGION     (NV_VGPU_VFIO_REGIONS_MAX)

typedef struct
{
    void *surface;
    NvU64 surface_size;
    NV_VFIO_VGPU_CONSOLE_SURFACE_PARAMS surface_params;
} vgpu_console_t;

typedef struct
{
    NvU64 start;
    NvU64 phys_start;
    NvU64 size;
} region_info_t;

struct addr_desc {
    unsigned long start;
    unsigned long size;
    struct list_head next;
};

struct mdev_phys_mapping
{
    struct address_space *mapping;
    struct list_head addr_desc_list;
    struct semaphore addr_lock;
    NvBool bar1_munmapped;
    struct vm_area_struct *bar1_vma;
};

typedef struct
{
    wait_queue_head_t wait;
    NvU64 offset;
    NvU8 *data;
    NvU32 count;
    VGPU_EMUL_SPACE_T emul_space;
    NvS32 status;
    NV_STATUS cfg_access_status;
    NvBool is_write;
} reg_access_t;

typedef struct
{
    struct rb_node node;
    unsigned long gpfn;
    atomic_t ref_count;
} gpfn_node_t;

typedef struct
{
    struct rb_node node;
    NvU64 guest_addr;
    NvU64 host_addr;
    NvU64 size;
    NvBool is_validated;
    NvBool is_dummy;
} bar1_node_t;

typedef struct mapping_node_s
{
    NvU64 iova;
    NvU32 pfn_count;
#if !defined(NV_KVMALLOC_PRESENT)
    NvBool is_vmalloc;
#endif
    NvBool base_gpfn_pinned;
    struct page *page_buffer[0];
} mapping_node_t;

typedef struct
{
    wait_queue_head_t wait;
    nv_spinlock_t lock;
    NvU32 type;
    NvBool pending;
} vgpu_event_t;

typedef struct
{
    struct list_head next;
    vgpu_event_t *event;
    struct vgpu_dev_s *vgpu_dev;
} vgpu_file_private_t;

typedef struct
{
    struct eventfd_ctx *intx_trigger;
    struct eventfd_ctx *msi_trigger;
    int index;
    NvBool ignore_interrupts;
    NvU32 max_num_vectors;
    NvU32 *allocated_irq;
    NvU32 num_ctx;
#if defined(NV_VGPU_KVM_BUILD)
    struct eventfd_ctx **msix_trigger;
#endif
} intr_info_t;

typedef struct
{
    NvU64 pending;
    NvU64 written;
    NvU64 buffer_size;
    NvU32 read_pending;
} vgpu_mig_data_bytes_t;

typedef struct
{
    struct list_head next;
    NvU64 start_pfn;
    NvU64 page_size;
    NvU64 total_pfns;
    NvU64 copied_pfns;
    NvU64 counter;
} vgpu_dirty_pfn_t;

typedef struct
{
    vgpu_dirty_pfn_t dirty_pfns;
    struct list_head dirty_pfn_reported_list;
    vgpu_mig_data_bytes_t bytes;
#if defined(NV_VFIO_DEVICE_MIG_STATE_PRESENT) && defined(NV_VGPU_KVM_BUILD)
    enum vfio_device_mig_state vfio_state;
#endif
    struct file *save_fp;
    struct file *resume_fp;
    NvU8 *staging_buffer;
    NvU32 migration_state;
    NvU8 *qemu_buffer;
    NvU32 vfio_device_state;
#if defined(DEBUG)
    NvU64 bytes_transferred;    /* Tracks bytes transfered during migration */
    NvU32 dirty_pfn_count;      /* Tracks pinned pages for migration statistic */
#endif

} migration_info_t;

typedef struct vgpu_dev_s
{
    struct list_head next;
    struct mdev_phys_mapping phys_mappings;
    struct rb_root gpfn_list;
    struct rb_root bar1_list;
    wait_queue_head_t wait_queue;
    struct list_head file_private_list;
    struct semaphore ops_lock;
    struct semaphore dev_lock;
    reg_access_t reg_info;
    intr_info_t intr_info;
    nv_spinlock_t intr_info_lock; /* used to protect intr_info_t */
    atomic_t usage_count;
#if defined(NV_VGPU_KVM_BUILD)
    region_info_t region_info[NV_VGPU_VFIO_REGIONS_MAX];
    struct vfio_device_info vfio_info;
    struct vfio_region_info_cap_sparse_mmap *sparse;
    struct notifier_block nb;
    struct mdev_device *mdev;
    struct cdev cdev;
#endif
    migration_info_t migration_info;
    NvU8 config_params[VGPU_CONFIG_PARAMS_MAX_LENGTH];
    NvU8 vm_name[VM_NAME_SIZE];
    struct task_struct *vgpu_task;
    NvU8 *vconfig;
    NvU64 *offsets;
    NvU64 *sizes;
    mapping_node_t **mapping_cache;
#if defined(NV_MDEV_PARENT_OPS_HAS_DEVICE_DRIVER) || defined(NV_MDEV_DRIVER_HAS_SUPPORTED_TYPE_GROUPS)
    struct vfio_device *vdev;
#endif
    NvU64 vfio_max_gpfn;
    NvU64 dummy_phys_addr;
    NvU64 dummy_virt_addr;
    int qemu_pid;
    NvS32 return_status;
    NvU32 num_areas;
    NvU32 vgpu_type_id;
    NvU16 vgpu_id;
    NvBool dummy_page_allocated;
    NvU32 gpu_pci_bdf;
    NvU32 gpu_pci_id;
    NvU32 instance_id;
    VGPU_DEVICE_STATE device_state;
    vgpu_console_t console;
    NvBool migration_enabled;
    NvBool is_driver_vm;
    struct device *dev;
} vgpu_dev_t;

typedef struct phys_dev_s
{
    struct list_head         next;
    struct pci_dev          *dev;
    NvBool                   is_virtfn;
#if defined(NV_USE_VFIO_PCI_CORE)
    struct vfio_pci_core_device vpcdev;
    vgpu_dev_t              *vgpu_dev;
#endif
    struct attribute_group **vgpu_type_groups;
    int                      num_vgpu_types;
    NvU32                   *vgpu_type_ids;
    struct semaphore         ops_lock;
/* vgpu fops to be registed to mdev module */
#if defined(NV_MDEV_DRIVER_HAS_SUPPORTED_TYPE_GROUPS)
    struct mdev_driver      *vgpu_fops;
#elif defined(NV_MDEV_PARENT_OPS_STRUCT_PRESENT)
    struct mdev_parent_ops  *vgpu_fops;
#else
    struct parent_ops       *vgpu_fops;
#endif
} phys_dev_t;

#define INVALID_VGPU_INSTANCE_ID 0xFFFFFFFF

/* list to maintain all vGPU devices */
struct vgpu_devs
{
    struct list_head vgpu_dev_list;
    struct semaphore start_lock;
    struct semaphore vgpu_dev_list_lock;
};

extern struct vgpu_devs vgpu_devices;

/* list to maintain all physical GPUs */
struct phys_devs
{
    struct list_head phys_dev_list;
    struct semaphore phys_dev_list_lock;
};

extern struct phys_devs phys_devices;

#if defined(NV_VGPU_KVM_BUILD)

static inline int nv_vfio_info_add_capability(struct vfio_info_cap *caps,
                                              struct vfio_info_cap_header *header,
                                              void *cap_type,
                                              size_t size)
{
#if defined(NV_VFIO_INFO_ADD_CAPABILITY_HAS_CAP_TYPE_ID_ARGS)
    return vfio_info_add_capability(caps, header->id, cap_type);
#else
    return vfio_info_add_capability(caps, header, size);
#endif
}

static inline int nv_vfio_register_notifier(vgpu_dev_t *vgpu_dev)
{
    int ret = 0;
#if !defined(NV_VFIO_DEVICE_OPS_HAS_DMA_UNMAP)

#if (NV_VFIO_NOTIFIER_ARGUMENT_COUNT == 4)
        unsigned long events = VFIO_IOMMU_NOTIFY_DMA_UNMAP;
#if defined(NV_VFIO_PIN_PAGES_HAS_VFIO_DEVICE_ARG)
        ret = vfio_register_notifier(vgpu_dev->vdev,
#else
        ret = vfio_register_notifier(vgpu_dev->dev,
#endif
                                     VFIO_IOMMU_NOTIFY, &events,
                                     &vgpu_dev->nb);
#else
        ret = vfio_register_notifier(vgpu_dev->dev, &vgpu_dev->nb);
#endif /* NV_VFIO_NOTIFIER_ARGUMENT_COUNT */

#endif /* NV_VFIO_DEVICE_OPS_HAS_DMA_UNMAP */
    return ret;
}

static inline int nv_vfio_unregister_notifier(vgpu_dev_t *vgpu_dev)
{
    int ret = 0;
#if !defined(NV_VFIO_DEVICE_OPS_HAS_DMA_UNMAP)

#if (NV_VFIO_NOTIFIER_ARGUMENT_COUNT == 4)
#if defined(NV_VFIO_PIN_PAGES_HAS_VFIO_DEVICE_ARG)
    ret = vfio_unregister_notifier(vgpu_dev->vdev, VFIO_IOMMU_NOTIFY, &vgpu_dev->nb);
#else
    ret = vfio_unregister_notifier(vgpu_dev->dev, VFIO_IOMMU_NOTIFY, &vgpu_dev->nb);
#endif
#else
    ret = vfio_unregister_notifier(vgpu_dev->dev, &vgpu_dev->nb);
#endif /* NV_VFIO_NOTIFIER_ARGUMENT_COUNT */

#endif /* NV_VFIO_DEVICE_OPS_HAS_DMA_UNMAP */

    return ret;
}

struct device *nv_get_device(vgpu_dev_t *);
#endif

#define IS_SRIOV(pdev, vgpu_dev)    (pdev->is_virtfn || vgpu_dev->is_driver_vm)
#define IS_LEGACY(pdev, vgpu_dev)   (!pdev->is_virtfn && !vgpu_dev->is_driver_vm)


#endif /* _NVIDIA_VGPU_VFIO_H_ */
