#ifndef HW_ARM_VIRT_PCI_ENUMERATE_H
#define HW_ARM_VIRT_PCI_ENUMERATE_H

#include "hw/pci/pci_bus.h"

void virt_pci_enumerate_bus(PCIBus *root_bus);

#endif
