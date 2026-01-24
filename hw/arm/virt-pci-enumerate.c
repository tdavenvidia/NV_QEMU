/*
 * ARM virt: PCI bus number enumeration
 * Moved from virt-acpi-build.c
 */

#include "qemu/osdep.h"
#include "qemu/error-report.h"
#include "hw/pci/pci.h"
#include "hw/pci/pci_bridge.h"
#include "hw/pci/pci_bus.h"
#include "hw/arm/virt-pci-enumerate.h"

/* Forward declaration for recursion */
static uint8_t virt_pci_bridge_program_bus_numbers_sub(PCIBus *bus,
                                                        uint8_t current_bus_num,
                                                        uint8_t *next_bus_num);

/* Find the child bus whose parent bridge is @dev on @bus. Returns NULL if @dev has no child. */
static PCIBus *virt_pci_find_child_bus(PCIBus *bus, PCIDevice *dev)
{
    PCIBus *child_bus;

    if (!dev) {
        return NULL;
    }
    QLIST_FOREACH(child_bus, &bus->child, sibling) {
        if (child_bus->parent_dev == dev) {
            return child_bus;
        }
    }
    return NULL;
}

/* Program one bridge's P/S/S and recurse into its child bus. Returns max subordinate.
 * Uses pci_default_write_config so the bridge's config_write runs and internal state
 * (e.g. pci_bridge_update_mappings) stays in sync. Also sets Memory Space Enable. */
static uint8_t virt_pci_bridge_program_one(PCIBus *bus, uint8_t current_bus_num,
                                           PCIDevice *dev, PCIBus *child_bus,
                                           uint8_t *next_bus_num)
{
    uint8_t secondary, max_child, max_subordinate = current_bus_num;
    uint16_t cmd;

    if (*next_bus_num == 0) {
        warn_report("virt_pci_bridge_program_bus_numbers: bus number overflow");
        return max_subordinate;
    }
    secondary = *next_bus_num;
    (*next_bus_num)++;

    /* Use pci_default_write_config so bridge config_write runs and state stays in sync */
    pci_default_write_config(dev, PCI_PRIMARY_BUS, current_bus_num, 1);
    pci_default_write_config(dev, PCI_SECONDARY_BUS, secondary, 1);
    pci_default_write_config(dev, PCI_SUBORDINATE_BUS, secondary, 1);  /* temporary */

    /* Set Memory Space Enable so the bridge decodes memory transactions */
    cmd = pci_get_word(dev->config + PCI_COMMAND);
    if (!(cmd & PCI_COMMAND_MEMORY)) {
        pci_default_write_config(dev, PCI_COMMAND, cmd | PCI_COMMAND_MEMORY, 2);
    }

    max_child = virt_pci_bridge_program_bus_numbers_sub(child_bus, secondary,
                                                        next_bus_num);
    pci_default_write_config(dev, PCI_SUBORDINATE_BUS, max_child, 1);
    if (max_child > max_subordinate) {
        max_subordinate = max_child;
    }
    warn_report("virt_pci_bridge: %02x:%02x.%x P=%u S=%u Sub=%u",
                (unsigned)current_bus_num,
                (unsigned)PCI_SLOT(dev->devfn),
                (unsigned)PCI_FUNC(dev->devfn),
                (unsigned)current_bus_num,
                (unsigned)secondary,
                (unsigned)max_child);
    return max_subordinate;
}

/* Pair (bridge dev, child bus) for programming P/S/S. */
typedef struct {
    PCIDevice *dev;
    PCIBus *child_bus;
} VirtPciBridgePair;

static int virt_pci_bridge_compare_pairs(gconstpointer a, gconstpointer b)
{
    const VirtPciBridgePair *pa = (const VirtPciBridgePair *)a;
    const VirtPciBridgePair *pb = (const VirtPciBridgePair *)b;
    int da = pa->dev ? (int)pa->dev->devfn : 0x100;
    int db = pb->dev ? (int)pb->dev->devfn : 0x100;
    return da - db;
}

/* Compare host-child buses by bus number so we process PXB bus 1 before PXB bus 9 (match EDK2). */
static int virt_pci_compare_host_child_bus_num(gconstpointer a, gconstpointer b)
{
    PCIBus *ba = *(PCIBus * const *)a;
    PCIBus *bb = *(PCIBus * const *)b;
    return pci_bus_num(ba) - pci_bus_num(bb);
}

/* Walk like allocator (bus->child) so we see every child bus including PXB.
 * Root has two PXB buses (00:01.0, 00:04.0) — they are not IS_PCI_BRIDGE, so
 * we must recurse via bus->child. For each child: if parent is bridge, program
 * P/S/S and recurse; if parent is PXB (or similar), recurse with pci_bus_num(child).
 * Also scan devices on this bus so we never miss a bridge (e.g. 09:00.0) whose
 * child might not be in bus->child. Children processed in devfn order. */
static uint8_t virt_pci_bridge_program_bus_numbers_sub(PCIBus *bus,
                                                        uint8_t current_bus_num,
                                                        uint8_t *next_bus_num)
{
    uint8_t max_subordinate = current_bus_num;
    PCIBus *child_bus;
    GArray *bridge_pairs = g_array_new(FALSE, FALSE, sizeof(VirtPciBridgePair));
    GArray *host_child_buses = g_array_new(FALSE, FALSE, sizeof(PCIBus *));
    GHashTable *seen_child = g_hash_table_new(g_direct_hash, g_direct_equal);
    VirtPciBridgePair pair;
    guint i;
    int devfn;

    /* (1) From bus->child — same list as allocator; includes PXB buses */
    QLIST_FOREACH(child_bus, &bus->child, sibling) {
        PCIDevice *parent_dev = child_bus->parent_dev;
        if (!parent_dev) {
            continue;
        }
        if (IS_PCI_BRIDGE(parent_dev)) {
            pair.dev = parent_dev;
            pair.child_bus = child_bus;
            g_array_append_val(bridge_pairs, pair);
            g_hash_table_insert(seen_child, child_bus, GINT_TO_POINTER(1));
        } else {
            /* PXB or similar host bridge: recurse with bus's own number */
            g_array_append_val(host_child_buses, child_bus);
        }
    }

    /* (2) From device iteration — bridges whose child not already in (1) */
    for (devfn = 0; devfn < ARRAY_SIZE(bus->devices); devfn++) {
        PCIDevice *dev = bus->devices[devfn];
        if (!dev || !IS_PCI_BRIDGE(dev)) {
            continue;
        }
        child_bus = pci_bridge_get_sec_bus(PCI_BRIDGE(dev));
        if (!child_bus || g_hash_table_contains(seen_child, child_bus)) {
            continue;
        }
        pair.dev = dev;
        pair.child_bus = child_bus;
        g_array_append_val(bridge_pairs, pair);
        g_hash_table_insert(seen_child, child_bus, GINT_TO_POINTER(1));
    }
    g_hash_table_destroy(seen_child);

    /* Sort bridge pairs by devfn for stable order (like EDK2) */
    if (bridge_pairs->len > 1) {
        g_array_sort(bridge_pairs, virt_pci_bridge_compare_pairs);
    }

    /* Sort host-child buses by bus number so PXB bus 1 is processed before PXB bus 9 (match EDK2) */
    if (host_child_buses->len > 1) {
        g_array_sort(host_child_buses, virt_pci_compare_host_child_bus_num);
    }

    /* Recurse into host-bridge children (PXB) — use pci_bus_num(child) */
    for (i = 0; i < host_child_buses->len; i++) {
        child_bus = g_array_index(host_child_buses, PCIBus *, i);
        uint8_t child_num = (uint8_t)pci_bus_num(child_bus);
        uint8_t one_max;
        if (child_num + 1 > *next_bus_num) {
            *next_bus_num = child_num + 1;
        }
        one_max = virt_pci_bridge_program_bus_numbers_sub(child_bus, child_num,
                                                          next_bus_num);
        if (one_max > max_subordinate) {
            max_subordinate = one_max;
        }
    }
    g_array_free(host_child_buses, TRUE);

    /* Program bridge children (assign next_bus_num, write P/S/S, recurse) */
    for (i = 0; i < bridge_pairs->len; i++) {
        pair = g_array_index(bridge_pairs, VirtPciBridgePair, i);
        uint8_t one_max = virt_pci_bridge_program_one(bus, current_bus_num,
                                                      pair.dev, pair.child_bus,
                                                      next_bus_num);
        if (one_max > max_subordinate) {
            max_subordinate = one_max;
        }
    }
    g_array_free(bridge_pairs, TRUE);
    return max_subordinate;
}

/* Print each PCI device once with its BDF (bus:slot.func) after bus number programming. */
static void virt_pci_print_bdfs_on_bus(PCIBus *bus, GHashTable *printed)
{
    int bus_num = pci_bus_num(bus);
    int devfn;

    for (devfn = 0; devfn < ARRAY_SIZE(bus->devices); devfn++) {
        PCIDevice *dev = bus->devices[devfn];
        if (dev && !g_hash_table_contains(printed, dev)) {
            g_hash_table_add(printed, dev);
            warn_report("virt_pci: %02x:%02x.%x",
                        (unsigned)bus_num,
                        (unsigned)PCI_SLOT(dev->devfn),
                        (unsigned)PCI_FUNC(dev->devfn));
        }
    }
}

static void virt_pci_print_all_bdfs_sub(PCIBus *bus, GHashTable *printed)
{
    int devfn;
    PCIBus *child_bus;

    virt_pci_print_bdfs_on_bus(bus, printed);
    /* Recurse into host-bridge children (PXB) — same as bus number programming */
    QLIST_FOREACH(child_bus, &bus->child, sibling) {
        if (child_bus->parent_dev && !IS_PCI_BRIDGE(child_bus->parent_dev)) {
            virt_pci_print_all_bdfs_sub(child_bus, printed);
        }
    }
    /* Recurse into bridge children (devfn order); use same fallback */
    for (devfn = 0; devfn < ARRAY_SIZE(bus->devices); devfn++) {
        PCIDevice *dev = bus->devices[devfn];
        child_bus = NULL;

        if (!dev || !IS_PCI_BRIDGE(dev)) {
            continue;
        }
        child_bus = virt_pci_find_child_bus(bus, dev);
        if (!child_bus) {
            child_bus = pci_bridge_get_sec_bus(PCI_BRIDGE(dev));
        }
        if (child_bus) {
            virt_pci_print_all_bdfs_sub(child_bus, printed);
        }
    }
}

static void virt_pci_print_all_bdfs(PCIBus *root_bus)
{
    GHashTable *printed;

    if (!root_bus) {
        return;
    }
    printed = g_hash_table_new(g_direct_hash, g_direct_equal);
    virt_pci_print_all_bdfs_sub(root_bus, printed);
    g_hash_table_destroy(printed);
}

/* Program Primary/Secondary/Subordinate bus numbers for the entire PCI tree.
 * Called after the allocator so firmware (e.g. EDK2) can discover all buses. */
static void virt_pci_bridge_program_bus_numbers(PCIBus *root_bus)
{
    uint8_t next_bus_num = 1;
    int num_children = 0;
    PCIBus *child_bus;

    if (!root_bus) {
        warn_report("virt_pci_bridge_program_bus_numbers: root_bus is NULL");
        return;
    }
    QLIST_FOREACH(child_bus, &root_bus->child, sibling) {
        num_children++;
    }
    warn_report("virt_pci_bridge_program_bus_numbers: root bus has %d child bus(es)",
                num_children);

    virt_pci_bridge_program_bus_numbers_sub(root_bus, 0, &next_bus_num);
    warn_report("virt_pci_bridge_program_bus_numbers: programmed bus numbers (1..%u)",
                (unsigned)(next_bus_num > 1 ? next_bus_num - 1 : 0));
    virt_pci_print_all_bdfs(root_bus);
}

/* Re-apply bus number programming after cold reset (reset zeros bridge config). */
void virt_pci_enumerate_bus(PCIBus *root_bus)
{
    if (!root_bus) {
        return;
    }
    virt_pci_bridge_program_bus_numbers(root_bus);
}
