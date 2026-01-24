#ifndef HW_ARM_VIRT_PCI_RESOURCE_H
#define HW_ARM_VIRT_PCI_RESOURCE_H

#include "hw/pci-host/gpex.h"
#include "hw/arm/virt.h"

void pci_fixed_bar_allocator(struct GPEXConfig *cfg, VirtMachineState *vms);

#endif
