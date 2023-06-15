#include "conftest/headers.h"
    #if defined(NV_LINUX_KCONFIG_H_PRESENT)
    #include <linux/kconfig.h>
    #endif
    #if defined(NV_GENERATED_AUTOCONF_H_PRESENT)
    #include <generated/autoconf.h>
    #else
    #include <linux/autoconf.h>
    #endif
    #if defined(CONFIG_XEN) &&         defined(CONFIG_XEN_INTERFACE_VERSION) &&  !defined(__XEN_INTERFACE_VERSION__)
    #define __XEN_INTERFACE_VERSION__ CONFIG_XEN_INTERFACE_VERSION
    #endif
    #if defined(CONFIG_KASAN) && defined(CONFIG_ARM64)
    #if defined(CONFIG_KASAN_SW_TAGS)
    #define KASAN_SHADOW_SCALE_SHIFT 4
    #else
    #define KASAN_SHADOW_SCALE_SHIFT 3
    #endif
    #endif
    
            #if defined NV_SOC_TEGRA_TEGRA_BPMP_H_PRESENT
            #include <soc/tegra/tegra_bpmp.h>
            #endif
            int conftest_tegra_bpmp_send_receive(
                    int mrq,
                    void *ob_data,
                    int ob_sz,
                    void *ib_data,
                    int ib_sz) {
                return tegra_bpmp_send_receive(mrq, ob_data, ob_sz, ib_data, ib_sz);
            }
            
