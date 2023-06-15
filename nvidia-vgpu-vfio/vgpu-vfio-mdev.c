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

#include <linux/err.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/vfio.h>

#include "nvstatus.h"
#include "nv-linux.h"
#include "nv-vgpu-vfio-interface.h"
#include "vgpu-devices.h"

#if defined(NV_VGPU_KVM_BUILD)

/* Common code used by both modern and older mdev */

static void nv_init_vgpu_dev(vgpu_dev_t *vgpu_dev, struct mdev_device *mdev)
{
    vgpu_dev->mdev = mdev;
    vgpu_dev->dev = NV_GET_MDEV_DEV(mdev);
}
int nv_vgpu_vfio_mdev_destroy(vgpu_dev_t *vgpu_dev)
{
    struct mdev_device *mdev = NULL;
    struct device *dev = NULL;
    int ret = 0;

    mdev = vgpu_dev->mdev;
    dev  = NV_GET_MDEV_DEV(mdev);

    ret = nv_vgpu_vfio_destroy(vgpu_dev, dev);
    if (ret)
        return ret;

    return 0;
}

#if defined(NV_MDEV_GET_TYPE_GROUP_ID_PRESENT)
int nv_vgpu_vfio_mdev_create(vgpu_dev_t *vgpu_dev)
{
    struct kobject *kobj = NULL;
#else
int nv_vgpu_vfio_mdev_create(struct kobject *kobj, vgpu_dev_t *vgpu_dev)
{
#endif
    struct pci_dev *pdev;
    void *mtype = NULL;
    NvU32 vgpu_type_id;
    int ret = 0;

    pdev = to_pci_dev(NV_GET_MDEV_PARENT(vgpu_dev->mdev));
    if (!pdev)
        return -EINVAL;

#if defined(NV_MDEV_GET_TYPE_GROUP_ID_PRESENT)
    mtype = vgpu_dev->mdev->type;
#endif

    if (nv_get_vgpu_type_id(kobj, NV_GET_MDEV_PARENT(vgpu_dev->mdev), &vgpu_type_id, mtype)
        != NV_OK)
    {
        NV_VGPU_DEV_LOG(VGPU_ERR, vgpu_dev, "vGPU creation failed, %d\n", ret);
        return -EINVAL;
    }

    ret = nv_vgpu_vfio_create(vgpu_dev, pdev, vgpu_type_id);
    if (ret)
        return ret;

    if (pdev->is_virtfn)
    {
#if defined(NV_MDEV_SET_IOMMU_DEVICE_PRESENT)

#if defined(NV_MDEV_GET_TYPE_GROUP_ID_PRESENT)
        mdev_set_iommu_device(vgpu_dev->mdev, NV_GET_MDEV_PARENT(vgpu_dev->mdev));
#else
        ret = mdev_set_iommu_device(NV_GET_MDEV_DEV(vgpu_dev->mdev),
                                    NV_GET_MDEV_PARENT(vgpu_dev->mdev));
#endif
        if (ret != 0)
        {
            NV_VGPU_DEV_LOG(VGPU_ERR, vgpu_dev, "Failed to set IOMMU device. ret: %d \n", ret);
            nv_vgpu_vfio_mdev_destroy(vgpu_dev);
        }
#endif
    }

    return 0;
}

#if defined(NV_MDEV_PARENT_OPS_HAS_DEVICE_DRIVER) || defined(NV_MDEV_DRIVER_HAS_SUPPORTED_TYPE_GROUPS)

/* Modern mdev codepath for kernel > v5.14 */

static vgpu_dev_t *nv_get_vgpu_from_vdev(struct vfio_device *core_vdev)
{
    vgpu_dev_t *vgpu_dev= NULL;

    if (!core_vdev || !core_vdev->dev)
    {
        NV_VGPU_LOG(VGPU_ERR, "No vfio device found\n");
        return NULL;
    }

    vgpu_dev = dev_get_drvdata(core_vdev->dev);
    if (!vgpu_dev)
        return NULL;

    return vgpu_dev;
}

static int nv_vfio_mdev_open(struct vfio_device *core_vdev)
{
    vgpu_dev_t *vgpu_dev = nv_get_vgpu_from_vdev(core_vdev);
    int ret;

    ret = nv_vgpu_vfio_open(vgpu_dev);
    if (ret)
        return ret;

    return ret;
}

static void nv_vfio_mdev_release(struct vfio_device *core_vdev)
{
    vgpu_dev_t *vgpu_dev = nv_get_vgpu_from_vdev(core_vdev);
    nv_vgpu_vfio_close(vgpu_dev);
}

static long nv_vfio_mdev_unlocked_ioctl(struct vfio_device *core_vdev,
                     unsigned int cmd, unsigned long arg)
{
    vgpu_dev_t *vgpu_dev = nv_get_vgpu_from_vdev(core_vdev);
    return nv_vgpu_vfio_ioctl(vgpu_dev, cmd, arg);
}

static ssize_t nv_vfio_mdev_read(struct vfio_device *core_vdev, char __user *buf,
                  size_t count, loff_t *ppos)
{
    vgpu_dev_t *vgpu_dev = nv_get_vgpu_from_vdev(core_vdev);
    return nv_vgpu_vfio_read(vgpu_dev, buf, count, ppos);
}

static ssize_t nv_vfio_mdev_write(struct vfio_device *core_vdev,
                   const char __user *buf, size_t count,
                   loff_t *ppos)
{
    vgpu_dev_t *vgpu_dev = nv_get_vgpu_from_vdev(core_vdev);
    return nv_vgpu_vfio_write(vgpu_dev, buf, count, ppos);
}

static int nv_vfio_mdev_mmap(struct vfio_device *core_vdev,
              struct vm_area_struct *vma)
{
    vgpu_dev_t *vgpu_dev = nv_get_vgpu_from_vdev(core_vdev);
    return nv_vgpu_vfio_mmap(vgpu_dev, vma);
}

static void nv_vfio_mdev_request(struct vfio_device *core_vdev, unsigned int count)
{
    vgpu_dev_t *vgpu_dev = nv_get_vgpu_from_vdev(core_vdev);
    if (!vgpu_dev)
        return;

    if (count == 0)
        NV_VGPU_DEV_LOG(VGPU_ERR, vgpu_dev,
                    "No mdev vendor driver request callback support, blocked until released by user\n");
}

#if defined(NV_VFIO_DEVICE_MIG_STATE_PRESENT)
static int nv_vfio_mdev_get_device_state(struct vfio_device *core_vdev,
                                    enum vfio_device_mig_state *curr_state)
{
    vgpu_dev_t *vgpu_dev = nv_get_vgpu_from_vdev(core_vdev);
    if (!vgpu_dev)
        return -EINVAL;

    *curr_state = vgpu_dev->migration_info.vfio_state;

    return 0;
}

static struct file *
nv_vfio_mdev_set_device_state(struct vfio_device *core_vdev,
                         enum vfio_device_mig_state new_state)
{
    vgpu_dev_t *vgpu_dev = nv_get_vgpu_from_vdev(core_vdev);
    struct file *fp = NULL;

    if (!vgpu_dev)
        return ERR_PTR(-EINVAL);

    fp = nv_vgpu_set_device_state(core_vdev, vgpu_dev, new_state);

    return fp;
}
#endif

#if defined(NV_VFIO_DEVICE_OPS_HAS_DMA_UNMAP)
static void nv_vfio_mdev_dma_unmap(struct vfio_device *core_vdev,
                                   u64 iova, u64 size)
{
    vgpu_dev_t *vgpu_dev = nv_get_vgpu_from_vdev(core_vdev);
    if (!vgpu_dev)
        return;

    nv_vgpu_vfio_dma_unmap(vgpu_dev, iova, size);
}
#endif

#if defined(NV_VFIO_MIGRATION_OPS_PRESENT)
static const struct vfio_migration_ops nv_vfio_mdev_migration_ops = {
    .migration_set_state = nv_vfio_mdev_set_device_state,
    .migration_get_state = nv_vfio_mdev_get_device_state,
};
#endif

static const struct vfio_device_ops nv_vfio_mdev_dev_ops = {
    .name           = "nvidia-vgpu-vfio",
#if defined(NV_MDEV_PARENT_OPS_HAS_OPEN_DEVICE) || defined(NV_MDEV_DRIVER_HAS_SUPPORTED_TYPE_GROUPS)
    .open_device    = nv_vfio_mdev_open,
    .close_device   = nv_vfio_mdev_release,
#else
    .open           = nv_vfio_mdev_open,
    .release        = nv_vfio_mdev_release,
#endif
    .ioctl          = nv_vfio_mdev_unlocked_ioctl,
    .read           = nv_vfio_mdev_read,
    .write          = nv_vfio_mdev_write,
    .mmap           = nv_vfio_mdev_mmap,
    .request        = nv_vfio_mdev_request,
#if defined(NV_VFIO_DEVICE_OPS_HAS_DMA_UNMAP)
    .dma_unmap      = nv_vfio_mdev_dma_unmap,
#endif
#if defined(NV_VFIO_DEVICE_MIG_STATE_PRESENT) && !defined(NV_VFIO_MIGRATION_OPS_PRESENT)
    .migration_set_state = nv_vfio_mdev_set_device_state,
    .migration_get_state = nv_vfio_mdev_get_device_state,
#endif
};

static int nv_vfio_mdev_probe(struct mdev_device *mdev)
{
    struct vfio_device *vdev;
    vgpu_dev_t *vgpu_dev = NULL;
    int ret = 0;

    if (!mdev)
        return -EINVAL;

    ret = nv_alloc_vgpu_dev(&vgpu_dev);
    if (ret)
        return ret;

    nv_init_vgpu_dev(vgpu_dev, mdev);

    vdev = kzalloc(sizeof(*vdev), GFP_KERNEL);
    if (!vdev)
        return -ENOMEM;

    vfio_init_group_dev(vdev, &mdev->dev, &nv_vfio_mdev_dev_ops);

#if defined(NV_VFIO_DEVICE_MIG_STATE_PRESENT)
    vdev->migration_flags = VFIO_MIGRATION_STOP_COPY;
#endif

#if defined(NV_VFIO_MIGRATION_OPS_PRESENT)
    vdev->mig_ops = &nv_vfio_mdev_migration_ops;
#endif

#if defined(NV_VFIO_REGISTER_EMULATED_IOMMU_DEV_PRESENT)
    ret = vfio_register_emulated_iommu_dev(vdev);
#else
    ret = vfio_register_group_dev(vdev);
#endif
    if (ret)
        goto out_err;

    ret = nv_vgpu_vfio_mdev_create(vgpu_dev);
    if (ret)
    {
        vfio_unregister_group_dev(vdev);
        goto out_err;
    }

    dev_set_drvdata(&mdev->dev, vgpu_dev);
    vgpu_dev->vdev = vdev;
    return 0;

out_err:
    NV_VGPU_DEV_LOG(VGPU_ERR, vgpu_dev, "Failed to probe mdev device, ret: %d\n", ret);
#if defined(NV_VFIO_UNINIT_GROUP_DEV_PRESENT)
    vfio_uninit_group_dev(vdev);
#endif /* NV_VFIO_UNINIT_GROUP_DEV_PRESENT */
    kfree(vdev);
    nv_free_vgpu_dev(vgpu_dev);
    return ret;
}

static void nv_vfio_mdev_remove(struct mdev_device *mdev)
{
    int ret;
    struct vfio_device *vdev = NULL;
    vgpu_dev_t *vgpu_dev = NULL;

    if (!mdev)
        return;

    vgpu_dev = dev_get_drvdata(&mdev->dev);
    if (!vgpu_dev)
        return;

    ret = nv_vgpu_vfio_mdev_destroy(vgpu_dev);
    if (ret)
        NV_VGPU_DEV_LOG(VGPU_ERR, vgpu_dev, "Failed to destroy vGPU device, ret: %d\n", ret);

    vdev = vgpu_dev->vdev;
    if (!vdev) {
        NV_VGPU_DEV_LOG(VGPU_ERR, vgpu_dev, "Failed to get device driver data\n");
        return;
    }

    dev_set_drvdata(&mdev->dev, NULL);
    vfio_unregister_group_dev(vdev);
#if defined(NV_VFIO_UNINIT_GROUP_DEV_PRESENT)
    vfio_uninit_group_dev(vdev);
#endif /* NV_VFIO_UNINIT_GROUP_DEV_PRESENT */
    kfree(vdev);
    nv_free_vgpu_dev(vgpu_dev);
}

struct mdev_driver nv_vfio_mdev_driver = {
    .driver = {
        .name       = "nvidia-vgpu-vfio",
        .owner      = THIS_MODULE,
        .mod_name   = KBUILD_MODNAME,
        .dev_groups = vgpu_dev_groups,
    },
    .probe  = nv_vfio_mdev_probe,
    .remove = nv_vfio_mdev_remove,
};

NV_STATUS nv_vgpu_set_mdev_fops(struct phys_dev_s *phys_dev)
{
#if defined(NV_MDEV_DRIVER_HAS_SUPPORTED_TYPE_GROUPS)
    NV_KMALLOC(phys_dev->vgpu_fops, sizeof(struct mdev_driver));
#elif defined(NV_MDEV_PARENT_OPS_STRUCT_PRESENT)
    NV_KMALLOC(phys_dev->vgpu_fops, sizeof(struct mdev_parent_ops));
#else
    NV_KMALLOC(phys_dev->vgpu_fops, sizeof(struct parent_ops));
#endif

    if (phys_dev->vgpu_fops == NULL)
        return NV_ERR_NO_MEMORY;

#if defined(NV_MDEV_DRIVER_HAS_SUPPORTED_TYPE_GROUPS)
    memset(phys_dev->vgpu_fops, 0, sizeof(struct mdev_driver));
#elif defined(NV_MDEV_PARENT_OPS_STRUCT_PRESENT)
    memset(phys_dev->vgpu_fops, 0, sizeof(struct mdev_parent_ops));
#else
    memset(phys_dev->vgpu_fops, 0, sizeof(struct parent_ops));
#endif

#if defined(NV_MDEV_DRIVER_HAS_SUPPORTED_TYPE_GROUPS)
    memcpy((void *)phys_dev->vgpu_fops, &nv_vfio_mdev_driver, sizeof(struct mdev_driver));
#else
    phys_dev->vgpu_fops->device_driver = &nv_vfio_mdev_driver;
#endif

#if !defined(NV_MDEV_DRIVER_HAS_SUPPORTED_TYPE_GROUPS)
    /*
     * On kernel >= 5.14 with modern mdev, the .owner, .supported_type_groups,
     * and .device_driver vgpu fops need to be registered with the mdev module
     */
    phys_dev->vgpu_fops->owner                 = THIS_MODULE;
#endif

    phys_dev->vgpu_fops->supported_type_groups = phys_dev->vgpu_type_groups;
    return NV_OK;
}

int nv_vfio_mdev_register_driver()
{
    return mdev_register_driver(&nv_vfio_mdev_driver);
}

void nv_vfio_mdev_unregister_driver()
{
    mdev_unregister_driver(&nv_vfio_mdev_driver);
}

#else /* NV_MDEV_PARENT_OPS_HAS_DEVICE_DRIVER || NV_MDEV_DRIVER_HAS_SUPPORTED_TYPE_GROUPS */

/* Old mdev codepath for kernel < v5.14 */

static vgpu_dev_t *nv_get_vgpu_from_mdev(struct mdev_device *mdev)
{
    vgpu_dev_t *vgpu_dev = NULL;

    if (!mdev)
    {
        NV_VGPU_LOG(VGPU_ERR, "No mdev device found\n");
        return NULL;
    }

    vgpu_dev = mdev_get_drvdata(mdev);
    if (!vgpu_dev)
        NV_VGPU_LOG(VGPU_ERR, "%s No vGPU device found\n", dev_name(NV_GET_MDEV_DEV(mdev)));

    return vgpu_dev;
}

static int nv_vfio_mdev_open(struct mdev_device *mdev)
{
    vgpu_dev_t *vgpu_dev = nv_get_vgpu_from_mdev(mdev);
    return nv_vgpu_vfio_open(vgpu_dev);
}

static void nv_vfio_mdev_close(struct mdev_device *mdev)
{
    vgpu_dev_t *vgpu_dev = nv_get_vgpu_from_mdev(mdev);
    nv_vgpu_vfio_close(vgpu_dev);
}

static int nv_vfio_mdev_mmap(struct mdev_device *mdev, struct vm_area_struct *vma)
{
    vgpu_dev_t *vgpu_dev = nv_get_vgpu_from_mdev(mdev);
    return nv_vgpu_vfio_mmap(vgpu_dev, vma);
}

static long nv_vfio_mdev_ioctl(struct mdev_device *mdev, unsigned int cmd,
                               unsigned long arg)
{
    vgpu_dev_t *vgpu_dev = nv_get_vgpu_from_mdev(mdev);
    return nv_vgpu_vfio_ioctl(vgpu_dev, cmd, arg);
}

static ssize_t nv_vfio_mdev_read(struct mdev_device *mdev, char __user *buf,
                                 size_t count, loff_t *ppos)
{
    vgpu_dev_t *vgpu_dev = nv_get_vgpu_from_mdev(mdev);
    return nv_vgpu_vfio_read(vgpu_dev, buf, count, ppos);
}

static ssize_t nv_vfio_mdev_write(struct mdev_device *mdev, const char __user *buf,
                                  size_t count, loff_t *ppos)
{
    vgpu_dev_t *vgpu_dev = nv_get_vgpu_from_mdev(mdev);
    return nv_vgpu_vfio_write(vgpu_dev, buf, count, ppos);
}

#if defined(NV_MDEV_GET_TYPE_GROUP_ID_PRESENT)
static int nv_vfio_mdev_create(struct mdev_device *mdev)
#else
static int nv_vfio_mdev_create(struct kobject *kobj, struct mdev_device *mdev)
#endif
{
    vgpu_dev_t *vgpu_dev = NULL;
    int ret = 0;

    if (!mdev)
        return -EINVAL;

    ret = nv_alloc_vgpu_dev(&vgpu_dev);
    if (ret)
        return ret;

    nv_init_vgpu_dev(vgpu_dev, mdev);

#if defined(NV_MDEV_GET_TYPE_GROUP_ID_PRESENT)
    ret = nv_vgpu_vfio_mdev_create(vgpu_dev);
#else
    ret = nv_vgpu_vfio_mdev_create(kobj, vgpu_dev);
#endif
    if (ret)
        nv_free_vgpu_dev(vgpu_dev);

    mdev_set_drvdata(vgpu_dev->mdev, vgpu_dev);
    return ret;
}

static int nv_vfio_mdev_destroy(struct mdev_device *mdev)
{
    vgpu_dev_t *vgpu_dev = nv_get_vgpu_from_mdev(mdev);
    int ret = 0;

    if (!vgpu_dev)
        return -EINVAL;

    ret = nv_vgpu_vfio_mdev_destroy(vgpu_dev);

    mdev_set_drvdata(mdev, NULL);
    nv_free_vgpu_dev(vgpu_dev);

    return ret;
}

NV_STATUS nv_vgpu_set_mdev_fops(struct phys_dev_s *phys_dev)
{
#if defined(NV_MDEV_PARENT_OPS_STRUCT_PRESENT)
    NV_KMALLOC(phys_dev->vgpu_fops, sizeof(struct mdev_parent_ops));
#else
    NV_KMALLOC(phys_dev->vgpu_fops, sizeof(struct parent_ops));
#endif
    if (phys_dev->vgpu_fops == NULL)
        return NV_ERR_NO_MEMORY;

#if defined(NV_MDEV_PARENT_OPS_STRUCT_PRESENT)
    memset(phys_dev->vgpu_fops, 0, sizeof(struct mdev_parent_ops));
#else
    memset(phys_dev->vgpu_fops, 0, sizeof(struct parent_ops));
#endif

    /* vgpu fops to be registed to mdev module */
    phys_dev->vgpu_fops->owner                 = THIS_MODULE;
    phys_dev->vgpu_fops->supported_type_groups = phys_dev->vgpu_type_groups;
    phys_dev->vgpu_fops->create                = nv_vfio_mdev_create;
    phys_dev->vgpu_fops->remove                = nv_vfio_mdev_destroy;
    phys_dev->vgpu_fops->mdev_attr_groups      = vgpu_dev_groups;
    phys_dev->vgpu_fops->read                  = nv_vfio_mdev_read;
    phys_dev->vgpu_fops->write                 = nv_vfio_mdev_write;
    phys_dev->vgpu_fops->ioctl                 = nv_vfio_mdev_ioctl;
    phys_dev->vgpu_fops->mmap                  = nv_vfio_mdev_mmap;

#if defined(NV_MDEV_PARENT_OPS_HAS_OPEN_DEVICE)
    phys_dev->vgpu_fops->open_device           = nv_vfio_mdev_open;
    phys_dev->vgpu_fops->close_device          = nv_vfio_mdev_close;
#else
    phys_dev->vgpu_fops->open                  = nv_vfio_mdev_open;
    phys_dev->vgpu_fops->release               = nv_vfio_mdev_close;
#endif

    return NV_OK;
}

#endif /* NV_MDEV_PARENT_OPS_HAS_DEVICE_DRIVER || NV_MDEV_DRIVER_HAS_SUPPORTED_TYPE_GROUPS */

void nv_vgpu_free_mdev_fops(struct phys_dev_s *phys_dev)
{
#if defined(NV_MDEV_DRIVER_HAS_SUPPORTED_TYPE_GROUPS)
    NV_KFREE(phys_dev->vgpu_fops, sizeof(struct mdev_driver));
#elif defined(NV_MDEV_PARENT_OPS_STRUCT_PRESENT)
    NV_KFREE(phys_dev->vgpu_fops, sizeof(struct mdev_parent_ops));
#else
    NV_KFREE(phys_dev->vgpu_fops, sizeof(struct parent_ops));
#endif
}

#endif /* NV_VGPU_KVM_BUILD */
