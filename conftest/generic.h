#undef NV_DOM0_KERNEL_PRESENT
#define NV_VGPU_KVM_BUILD
#undef NV_GRID_BUILD
#undef NV_GRID_BUILD_CSP
#undef NV_GET_USER_PAGES_HAS_ARGS_WRITE_FORCE
#undef NV_GET_USER_PAGES_HAS_ARGS_TSK_WRITE_FORCE
#undef NV_GET_USER_PAGES_HAS_ARGS_TSK_FLAGS
#define NV_GET_USER_PAGES_HAS_ARGS_FLAGS
#define NV_GET_USER_PAGES_REMOTE_PRESENT
#undef NV_GET_USER_PAGES_REMOTE_HAS_ARGS_TSK_WRITE_FORCE
#undef NV_GET_USER_PAGES_REMOTE_HAS_ARGS_TSK_FLAGS
#undef NV_GET_USER_PAGES_REMOTE_HAS_ARGS_TSK_FLAGS_LOCKED
#define NV_GET_USER_PAGES_REMOTE_HAS_ARGS_FLAGS_LOCKED
#define NV_PIN_USER_PAGES_PRESENT
#define NV_PIN_USER_PAGES_REMOTE_PRESENT
#undef NV_PIN_USER_PAGES_REMOTE_HAS_ARGS_TSK
#define NV_PM_RUNTIME_AVAILABLE
#define NV_VM_FAULT_T_IS_PRESENT
#define NV_PCI_CLASS_MULTIMEDIA_HD_AUDIO_PRESENT
#define NV_DRM_AVAILABLE
#undef NV_VFIO_PCI_CORE_PRESENT
// Kernel version:             "5.10.0-136.12.0.86.oe2203sp1.x86_64"
