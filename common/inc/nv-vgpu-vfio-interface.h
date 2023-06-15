/*
 * SPDX-FileCopyrightText: Copyright (c) 2016-2022 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: MIT
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#ifndef _NV_VGPU_VFIO_INTERFACE_H_
#define _NV_VGPU_VFIO_INTERFACE_H_

#include <linux/pci.h>
#include <linux/device.h>
#include "nvstatus.h"
#include "nv-hypervisor.h"

#if defined(NV_VGPU_KVM_BUILD) && defined(NV_VFIO_PCI_CORE_PRESENT) && \
    !defined(NV_MDEV_SET_IOMMU_DEVICE_PRESENT)
#define NV_USE_VFIO_PCI_CORE
#else
#undef NV_USE_VFIO_PCI_CORE
#endif

#define VFIO_MAX_VGPU_TYPES_PER_PGPU 64

typedef NV_STATUS (*vgpu_vfio_probe_t)(struct pci_dev *, NvU32, NvU32 *);
typedef void (*vgpu_vfio_remove_t)(struct pci_dev *);
typedef NV_STATUS (*vgpu_vfio_inject_interrupt_t)(void *);

/*
 * structure to be registered to RM using which it
 * will call into nvidia-vgpu-vfio module.
 * RM will use this structure to call into
 * nvidia-vgpu-vfio module
 */
typedef struct
{
    vgpu_vfio_probe_t probe;
    vgpu_vfio_remove_t  remove;
    vgpu_vfio_inject_interrupt_t inject_interrupt;
} vgpu_vfio_ops_t;

typedef NV_STATUS (*vgpu_vfio_vgpu_create_t) (struct pci_dev *dev, const NvU8 *uuid,
                                              NvU32 vgpu_type_id, NvU16 *vgpu_id,
                                              NvU32 *gpu_pci_id, NvU32 gpu_pci_bdf, NvBool *is_driver_vm);
typedef NV_STATUS (*vgpu_vfio_vgpu_destroy_t) (const NvU8 *uuid, NvU16 vgpu_id);
typedef NV_STATUS (*vgpu_vfio_start_t) (const NvU8 *uuid, void *wait_queue, NvS32 *return_status,
                                        NvU8 *vm_name, NvU32 qemu_pid);
typedef NV_STATUS (*vgpu_vfio_bar_info_t) (struct pci_dev *dev, const NvU8 *, NvU64 *,
                                           NvU32 bar_index, void *);
typedef NV_STATUS (*vgpu_vfio_get_description) (struct pci_dev *, NvU32, char *);
typedef NV_STATUS (*vgpu_vfio_get_name) (struct pci_dev *, NvU32, char *);
typedef NV_STATUS (*vgpu_vfio_get_types) (struct pci_dev *, NvU32 *, NvU32 *);
typedef NV_STATUS (*vgpu_vfio_get_instance) (struct pci_dev *, NvU32, char *);
typedef NV_STATUS (*vgpu_vfio_sparse_mmap_t) (struct pci_dev *, const NvU8 *, NvU64 **, NvU64 **, NvU32 *);
typedef void (*vgpu_vfio_update_request_t) (const NvU8 *, NvU64 *, NvU64 *, VGPU_DEVICE_STATE, const char *);

/*
 * nvidia-vgpu-vfio module uses this structure to
 * call into RM
 */
typedef struct
{
    const char *version_string;
    vgpu_vfio_vgpu_create_t vgpu_create;
    vgpu_vfio_vgpu_destroy_t vgpu_delete;
    vgpu_vfio_bar_info_t vgpu_bar_info;
    vgpu_vfio_start_t vgpu_start;
    vgpu_vfio_get_description get_description;
    vgpu_vfio_get_instance get_instances;
    vgpu_vfio_get_name get_name;
    vgpu_vfio_get_types get_types;
    vgpu_vfio_sparse_mmap_t get_sparse_mmap;
    vgpu_vfio_update_request_t update_request;
} rm_vgpu_vfio_ops_t;

/*
 * function exposed by RM and called by nvidia-vgpu-vfio
 * module to initialize vgpu_vfio_ops_t strucure
 */
NV_STATUS nvidia_vgpu_vfio_set_ops(vgpu_vfio_ops_t *vgpu_vfio_ops);

/*
 * function exposed by RM and called by nvidia-vgpu-vfio
 * module to initialize rm_vgpu_vfio_ops_t strucure
 */
NV_STATUS nvidia_vgpu_vfio_get_ops(rm_vgpu_vfio_ops_t *ops);

NV_STATUS nvidia_vgpu_vfio_probe(struct pci_dev *);
void nvidia_vgpu_vfio_remove(struct pci_dev *, NvBool);

#endif /* _NV_VGPU_VFIO_INTERFACE_H_ */
