/*
 * Copyright (c) 2017-2022, NVIDIA CORPORATION. All rights reserved.
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/err.h>
#include "nvstatus.h"
#include "nv-linux.h"
#include "nv-vgpu-vfio-interface.h"
#include "vgpu-devices.h"

#if defined(NV_USE_VFIO_PCI_CORE)

extern rm_vgpu_vfio_ops_t rm_vgpu_vfio_ops;

static inline struct pci_dev *get_pdev_from_vdev(struct vfio_device *core_vdev)
{
    struct vfio_pci_core_device *vpcdev =
                    container_of(core_vdev, struct vfio_pci_core_device, vdev);
    struct pci_dev *pdev = vpcdev->pdev;

    return pdev;
}

static vgpu_dev_t *nv_vfio_pci_get_vgpu(struct vfio_device *core_vdev)
{
    struct vfio_pci_core_device *vpcdev = NULL;
    phys_dev_t *phys_dev = NULL;
    vgpu_dev_t *vgpu_dev = NULL;

    vpcdev   = container_of(core_vdev, struct vfio_pci_core_device, vdev);
    phys_dev = container_of(vpcdev, phys_dev_t, vpcdev);

    down(&phys_dev->ops_lock);
    vgpu_dev = phys_dev->vgpu_dev;
    up(&phys_dev->ops_lock);

    if (!vgpu_dev)
    {
        struct pci_dev *pdev = get_pdev_from_vdev(core_vdev);
        NV_VGPU_LOG(VGPU_ERR, "No vGPU device found for VF %04x:%02x:%02x.%x \n",
                    NV_PCI_DOMAIN_NUMBER(pdev), NV_PCI_BUS_NUMBER(pdev),
                    NV_PCI_SLOT_NUMBER(pdev), PCI_FUNC(pdev->devfn));
    }

    return vgpu_dev;
}

static void nv_set_phys_dev_vgpu_dev(struct device *dev, vgpu_dev_t *vgpu_dev)
{
    struct vfio_pci_core_device *vpcdev = NULL;
    phys_dev_t *phys_dev;

    vpcdev = dev_get_drvdata(dev);
    if (!vpcdev)
    {
        NV_VGPU_LOG(VGPU_ERR, "vfio_pci_core_device not found on %s\n", dev_name(dev));
        return;
    }

    phys_dev = container_of(vpcdev, phys_dev_t, vpcdev);

    down(&phys_dev->ops_lock);
    phys_dev->vgpu_dev = vgpu_dev;
    up(&phys_dev->ops_lock);
}

static vgpu_dev_t *nv_get_phys_dev_vgpu_dev(struct device *dev)
{
    vgpu_dev_t *vgpu_dev = NULL;
    struct vfio_pci_core_device *vpcdev = NULL;
    phys_dev_t *phys_dev;

    vpcdev = dev_get_drvdata(dev);
    if (!vpcdev)
    {
        NV_VGPU_LOG(VGPU_ERR, "vfio_pci_core_device not found on %s\n", dev_name(dev));
        return NULL;
    }

    phys_dev = container_of(vpcdev, phys_dev_t, vpcdev);

    down(&phys_dev->ops_lock);
    vgpu_dev = phys_dev->vgpu_dev;
    up(&phys_dev->ops_lock);

    return vgpu_dev;
}

static int nv_vfio_pci_remove(struct device *dev)
{
    vgpu_dev_t *vgpu_dev = NULL;
    int ret = 0;

    if (!dev)
        return -EINVAL;

    vgpu_dev = nv_get_phys_dev_vgpu_dev(dev);
    if (!vgpu_dev)
    {
        NV_VGPU_LOG(VGPU_ERR, "Failed to remove vGPU. No vGPU exists on current device\n");
        return -EPERM;
    }

    ret = nv_vgpu_vfio_destroy(vgpu_dev, dev);
    if (ret)
        return ret;

    nv_free_vgpu_dev(vgpu_dev);
    nv_set_phys_dev_vgpu_dev(dev, NULL);

    return 0;
}

static int nv_vfio_pci_create(struct device *dev, NvU32 vgpu_type_id)
{
    vgpu_dev_t *vgpu_dev = NULL;
    struct pci_dev *pdev = NULL;
    struct vfio_pci_core_device *vpcdev = NULL;
    int ret = 0;

    if (!dev)
        return -EINVAL;

    vgpu_dev = nv_get_phys_dev_vgpu_dev(dev);
    if (vgpu_dev)
    {
        NV_VGPU_DEV_LOG(VGPU_ERR, vgpu_dev, "vGPU device already created with type ID: %d\n",
                        vgpu_dev->vgpu_type_id);
        return -EPERM;
    }

    ret = nv_alloc_vgpu_dev(&vgpu_dev);
    if (ret)
        return ret;

    pdev = to_pci_dev(dev);
    if (!pdev)
        return -ENODEV;

    vgpu_dev->dev = dev;
    vgpu_dev->mdev = NULL;

    vpcdev = dev_get_drvdata(dev);
    if (vpcdev == NULL)
    {
        NV_VGPU_LOG(VGPU_ERR, "Failed to get vfio_pci_core device\n");
        return -EINVAL;
    }

    vgpu_dev->vdev = &vpcdev->vdev;

    nv_set_phys_dev_vgpu_dev(dev, vgpu_dev);

    ret = nv_vgpu_vfio_create(vgpu_dev, pdev, vgpu_type_id);
    if (ret)
    {
        nv_free_vgpu_dev(vgpu_dev);
        nv_set_phys_dev_vgpu_dev(dev, NULL);

        return ret;
    }

    return 0;
}

static int nv_vfio_pci_open_device(struct vfio_device *core_vdev)
{
    vgpu_dev_t *vgpu_dev = nv_vfio_pci_get_vgpu(core_vdev);
    int ret;

    if (!vgpu_dev)
    {
        NV_VGPU_LOG(VGPU_ERR, "current_vgpu_type of VF not configured\n");
        return -ENODEV;
    }

    ret = nv_vgpu_vfio_open(vgpu_dev);
    if (ret)
        return ret;

    return ret;
}

static void nv_vfio_pci_close_device(struct vfio_device *core_vdev)
{
    vgpu_dev_t *vgpu_dev = nv_vfio_pci_get_vgpu(core_vdev);
    nv_vgpu_vfio_close(vgpu_dev);
}

static long nv_vfio_pci_ioctl(struct vfio_device *core_vdev, unsigned int cmd,
                unsigned long arg)
{
    vgpu_dev_t *vgpu_dev = nv_vfio_pci_get_vgpu(core_vdev);
    return nv_vgpu_vfio_ioctl(vgpu_dev, cmd, arg);
}

static ssize_t nv_vfio_pci_read(struct vfio_device *core_vdev, char __user *buf,
                size_t count, loff_t *ppos)
{
    vgpu_dev_t *vgpu_dev = nv_vfio_pci_get_vgpu(core_vdev);
    return nv_vgpu_vfio_read(vgpu_dev, buf, count, ppos);
}

static ssize_t nv_vfio_pci_write(struct vfio_device *core_vdev, const char __user *buf,
                size_t count, loff_t *ppos)
{
    vgpu_dev_t *vgpu_dev = nv_vfio_pci_get_vgpu(core_vdev);
    return nv_vgpu_vfio_write(vgpu_dev, buf, count, ppos);
}

static int nv_vfio_pci_mmap(struct vfio_device *core_vdev, struct vm_area_struct *vma)
{
    vgpu_dev_t *vgpu_dev = nv_vfio_pci_get_vgpu(core_vdev);
    return nv_vgpu_vfio_mmap(vgpu_dev, vma);
}

static int nv_vfio_pci_match(struct vfio_device *core_vdev, char *buf)
{
    struct pci_dev *pdev = get_pdev_from_vdev(core_vdev);

    if (strncmp(pci_name(pdev), buf, strlen(pci_name(pdev))))
        return 0; /* No match */

    return 1;
}

#if defined(NV_VFIO_DEVICE_MIG_STATE_PRESENT)
static struct file *
nv_vfio_pci_set_device_state(struct vfio_device *core_vdev,
                         enum vfio_device_mig_state new_state)
{
    vgpu_dev_t *vgpu_dev = nv_vfio_pci_get_vgpu(core_vdev);
    struct file *fp = NULL;

    if (!vgpu_dev)
        return ERR_PTR(-EINVAL);

    fp = nv_vgpu_set_device_state(core_vdev, vgpu_dev, new_state);

    return fp;
}

int nv_vfio_pci_get_device_state(struct vfio_device *core_vdev,
                             enum vfio_device_mig_state *curr_state)
{
    vgpu_dev_t *vgpu_dev = nv_vfio_pci_get_vgpu(core_vdev);
    if (!vgpu_dev)
        return -EINVAL;

    *curr_state = vgpu_dev->migration_info.vfio_device_state;

    return 0;
}
#endif

#if defined(NV_VFIO_DEVICE_OPS_HAS_DMA_UNMAP)
static void nv_vfio_pci_dma_unmap(struct vfio_device *core_vdev,
                                   u64 iova, u64 size)
{
    vgpu_dev_t *vgpu_dev = nv_vfio_pci_get_vgpu(core_vdev);
    if (!vgpu_dev)
        return;

    nv_vgpu_vfio_dma_unmap(vgpu_dev, iova, size);
}
#endif

static ssize_t
creatable_vgpu_types_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    NvU32 vgpu_type_ids[VFIO_MAX_VGPU_TYPES_PER_PGPU] = {0};
    NvU32 num_vgpu_types = 0;
    NV_STATUS status = 0;
    struct pci_dev *pdev;
    ssize_t ret = 0;
    char vgpu_name[NV_VGPU_STRING_BUFFER_SIZE];
    int i;

    if (!dev || !buf)
        return -EINVAL;

    pdev = to_pci_dev(dev);

    status = rm_vgpu_vfio_ops.get_types(pdev, vgpu_type_ids, &num_vgpu_types);
    if (status != NV_OK)
    {
        NV_VGPU_LOG(VGPU_ERR, "Failed to get vGPU types: 0x%x\n", status);
        return -EINVAL;
    }

    ret += sprintf(buf, "ID    : vGPU Name\n");

    for (i = 0; i < num_vgpu_types; i++)
    {
        status = rm_vgpu_vfio_ops.get_name(pdev, vgpu_type_ids[i], vgpu_name);
        if (status != NV_OK)
        {
            NV_VGPU_LOG(VGPU_ERR, "Failed to get vGPU name of ID %d, status: 0x%x\n",
                        vgpu_type_ids[i], status);
            continue;
        }

        ret += sprintf(buf + ret, "%-5d : %s", vgpu_type_ids[i], vgpu_name);
    }

    return ret;
}

static DEVICE_ATTR_RO(creatable_vgpu_types);

static ssize_t
current_vgpu_type_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    vgpu_dev_t *vgpu_dev = NULL;
    NvU32 type_id = 0;

    vgpu_dev = nv_get_phys_dev_vgpu_dev(dev);
    if (!vgpu_dev)
    {
        NV_VGPU_LOG(VGPU_INFO, "vGPU device not created on %s yet\n", dev_name(dev));
        type_id = 0;
    }
    else
    {
        type_id = vgpu_dev->vgpu_type_id;
    }

    return sprintf(buf, "%d\n", type_id);
}

static ssize_t
current_vgpu_type_store(struct device *dev, struct device_attribute *attr,
                        const char *buf, size_t count)
{
    long l_vgpu_type_id = -1;
    int ret = 0;

    if (!dev || !buf)
        return -EINVAL;

    ret = kstrtol(buf, 10, &l_vgpu_type_id);
    if (ret)
        return ret;

    if (l_vgpu_type_id == 0)
        ret = nv_vfio_pci_remove(dev);
    else
        ret = nv_vfio_pci_create(dev, (NvU32) l_vgpu_type_id);

    if (ret)
        return ret;

    return count;
}

static DEVICE_ATTR_RW(current_vgpu_type);

static struct attribute *vf_dev_attrs[] = {
    &dev_attr_creatable_vgpu_types.attr,
    &dev_attr_current_vgpu_type.attr,
    NULL,
};

static const struct attribute_group vf_dev_group = {
    .name  = "nvidia",
    .attrs = vf_dev_attrs,
};

const struct attribute_group *vf_dev_groups[] = {
    &vf_dev_group,
    NULL,
};

#if defined(NV_VFIO_MIGRATION_OPS_PRESENT)
static const struct vfio_migration_ops nv_vfio_pci_migration_ops = {
    .migration_set_state = nv_vfio_pci_set_device_state,
    .migration_get_state = nv_vfio_pci_get_device_state,
};
#endif

static const struct vfio_device_ops nv_vfio_pci_ops = {
    .name           = "nvidia-vfio-pci",
    .open_device    = nv_vfio_pci_open_device,
    .close_device   = nv_vfio_pci_close_device,
    .ioctl          = nv_vfio_pci_ioctl,
    .read           = nv_vfio_pci_read,
    .write          = nv_vfio_pci_write,
    .mmap           = nv_vfio_pci_mmap,
    .match          = nv_vfio_pci_match,
#if defined(NV_VFIO_DEVICE_OPS_HAS_DMA_UNMAP)
    .dma_unmap      = nv_vfio_pci_dma_unmap,
#endif
#if defined(NV_VFIO_DEVICE_MIG_STATE_PRESENT) && !defined(NV_VFIO_MIGRATION_OPS_PRESENT)
    .migration_set_state = nv_vfio_pci_set_device_state,
    .migration_get_state = nv_vfio_pci_get_device_state,
#endif
};

NV_STATUS nv_vfio_pci_core_init(phys_dev_t *phys_dev)
{
    NV_STATUS status = NV_OK;

    if (phys_dev->dev->is_virtfn) {
        int ret;

        vfio_pci_core_init_device(&phys_dev->vpcdev, phys_dev->dev, &nv_vfio_pci_ops);

        /*
         * As per below commit, set vfio_pci_core_device in drvdata
         * 91be0bd6c6cf21328017e990d3ceeb00f03821fd vfio/pci: Have all VFIO PCI
         * drivers store the vfio_pci_core_device in drvdata
         */
        dev_set_drvdata(&phys_dev->dev->dev, &phys_dev->vpcdev);

#if defined(NV_VFIO_DEVICE_MIG_STATE_PRESENT)
        phys_dev->vpcdev.vdev.migration_flags = VFIO_MIGRATION_STOP_COPY;
#endif

#if defined(NV_VFIO_MIGRATION_OPS_PRESENT)
        phys_dev->vpcdev.vdev.mig_ops = &nv_vfio_pci_migration_ops;
#endif

        ret = vfio_pci_core_register_device(&phys_dev->vpcdev);
        if (ret) {
            NV_VGPU_LOG(VGPU_ERR, "vfio_pci_core_register_device failed err: %d\n", ret);
            vfio_pci_core_uninit_device(&phys_dev->vpcdev);
            phys_dev->vpcdev.pdev = NULL;
            return NV_ERR_OPERATING_SYSTEM;
        }

        ret = sysfs_create_groups(&phys_dev->dev->dev.kobj, vf_dev_groups);
        if (ret) {
            NV_VGPU_LOG(VGPU_ERR, "Failed to create sysfs for vGPU types %d\n", ret);
            vfio_pci_core_unregister_device(&phys_dev->vpcdev);
            vfio_pci_core_uninit_device(&phys_dev->vpcdev);
            phys_dev->vpcdev.pdev = NULL;
            return NV_ERR_OPERATING_SYSTEM;
        }
    }
    return status;
}

void nv_vfio_pci_core_uninit(phys_dev_t *phys_dev)
{
    vfio_pci_core_unregister_device(&phys_dev->vpcdev);
    dev_set_drvdata(&phys_dev->dev->dev, NULL);
    vfio_pci_core_uninit_device(&phys_dev->vpcdev);
    phys_dev->vpcdev.pdev = NULL;
    sysfs_remove_groups(&phys_dev->dev->dev.kobj, vf_dev_groups);
}

#endif /* NV_USE_VFIO_PCI_CORE */

