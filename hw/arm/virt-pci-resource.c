/*
 * ARM virt: PCI BAR allocation (fixed BARs, bridge windows, FDT ranges)
 * Moved from virt-acpi-build.c
 */

#include "qemu/osdep.h"
#include "qemu/error-report.h"
#include "qemu/bitops.h"
#include "qemu/range.h"
#include "hw/pci/pci.h"
#include "hw/pci/pci_bridge.h"
#include "hw/pci/pci_bus.h"
#include "hw/pci/pci_host.h"
#include "hw/pci-host/gpex.h"
#include "hw/arm/virt.h"
#include "hw/arm/virt-pci-resource.h"
#include "system/device_tree.h"
#include "libfdt.h"

typedef struct {
    uint64_t addr;
    uint64_t end;
    uint64_t flags;
} PhysBAR;

typedef struct {
    uint64_t wbase;
    uint64_t wlimit;
    uint64_t wbase64;
    uint64_t wlimit64;
    uint64_t rbase;
    uint64_t rlimit;
    uint64_t rsize;
    uint64_t piobase;
    bool     available;
    bool     search_mmio64;
    PCIDevice *dev;
    PCIBus *bus;
    struct GPEXConfig *cfg;
    bool debug;
} VirtPciAllocCfg;

#define IORESOURCE_PREFETCH     0x00002000    /* No side effects */
#define IORESOURCE_MEM_64       0x00100000

/* Global list of claimed fixed 64-bit prefetchable BAR ranges (first-win) */
typedef struct FixedClaim {
    uint64_t start;
    uint64_t end;
    PCIDevice *owner;
    int bar;
} FixedClaim;
static GArray *virt_fixed_claims;

static void virt_fixed_claims_reset(void)
{
    if (virt_fixed_claims) {
        g_array_free(virt_fixed_claims, true);
        virt_fixed_claims = NULL;
    }
    virt_fixed_claims = g_array_new(false, true, sizeof(FixedClaim));
}

static bool virt_fixed_claims_conflicts(uint64_t start, uint64_t end,
                                        uint64_t wbase64, uint64_t wlimit64,
                                        uint64_t *conflict_end)
{
    /* Hard guard: out-of-window ranges are invalid input */
    if (start < wbase64 || end > wlimit64) {
        error_report("acpi/mmio64: placement [0x%"PRIx64"..0x%"PRIx64"] out of window "
                     "[0x%"PRIx64"..0x%"PRIx64"]",
                     start, end, wbase64, wlimit64);
        exit(1);
    }
    if (!virt_fixed_claims) {
        return false;
    }
    for (guint i = 0; i < virt_fixed_claims->len; i++) {
        FixedClaim *c = &g_array_index(virt_fixed_claims, FixedClaim, i);
        if (ranges_overlap(start, end - start + 1, c->start, c->end - c->start + 1)) {
            if (conflict_end) {
                *conflict_end = c->end;
            }
            return true;
        }
    }
    return false;
}

static void virt_fixed_claims_add(uint64_t start, uint64_t end, PCIDevice *dev, int bar)
{
    FixedClaim cl = { .start = start, .end = end, .owner = dev, .bar = bar };
    g_array_append_val(virt_fixed_claims, cl);
    warn_report("acpi/mmio64: claim-add [%02x:%02x.%x] BAR%d "
                "=[0x%"PRIx64"..0x%"PRIx64"]",
                pci_dev_bus_num(dev), PCI_SLOT(dev->devfn), PCI_FUNC(dev->devfn),
                bar, start, end);
}

static void pci_validate_fixed_bar(PCIDevice *dev,
                                        int bar_index,
                                        uint64_t addr,
                                        uint64_t size,
                                        uint64_t wbase64,
                                        uint64_t wlimit64)
{
    PCIIORegion *r = &dev->io_regions[bar_index];
    if (!r->size || !(r->type & PCI_BASE_ADDRESS_MEM_TYPE_64)) {
        error_report("acpi/mmio64: invalid pci-boot-config for %s [%02x:%02x.%x] BAR%d: "
                     "BAR not 64-bit or size=0 (type=0x%x size=0x%"PRIx64")",
                     dev->name, pci_dev_bus_num(dev),
                     PCI_SLOT(dev->devfn), PCI_FUNC(dev->devfn),
                     bar_index, r->type, (uint64_t)r->size);
        exit(1);
    }
    /* Guard: 64-bit non-prefetchable BARs must not be placed behind bridges. */
    if (!(r->type & PCI_BASE_ADDRESS_MEM_PREFETCH) &&
        !pci_bus_is_root(pci_get_bus(dev))) {
        error_report("acpi/mmio64: invalid pci-boot-config for %s [%02x:%02x.%x] BAR%d: "
                     "64-bit non-prefetchable BAR cannot be assigned behind a PCIe bridge",
                     dev->name, pci_dev_bus_num(dev),
                     PCI_SLOT(dev->devfn), PCI_FUNC(dev->devfn), bar_index);
        exit(1);
    }
    uint64_t end = addr + size - 1;
    if (addr & (size - 1)) {
        error_report("acpi/mmio64: invalid pci-boot-config alignment for %s [%02x:%02x.%x] "
                     "BAR%d: addr=0x%"PRIx64" size=0x%"PRIx64,
                     dev->name, pci_dev_bus_num(dev),
                     PCI_SLOT(dev->devfn), PCI_FUNC(dev->devfn),
                     bar_index, addr, size);
        exit(1);
    }
    if (addr < wbase64 || end > wlimit64) {
        error_report("acpi/mmio64: pci-boot-config out of window for %s [%02x:%02x.%x] BAR%d "
                     "range=[0x%"PRIx64"..0x%"PRIx64"] window=[0x%"PRIx64"..0x%"PRIx64"]",
                     dev->name, pci_dev_bus_num(dev),
                     PCI_SLOT(dev->devfn), PCI_FUNC(dev->devfn),
                     bar_index, addr, end, wbase64, wlimit64);
        exit(1);
    }
}


static void pci_check_fixed_bar_overlap(PCIDevice *dev, PhysBAR *pbars)
{
    for (int i = 0; i < PCI_ROM_SLOT; i++) {
        if (!(pbars[i].flags & IORESOURCE_PREFETCH)) {
            continue;
        }
        for (int j = i + 1; j < PCI_ROM_SLOT; j++) {
            if (!(pbars[j].flags & IORESOURCE_PREFETCH)) {
                continue;
            }
            if (ranges_overlap(pbars[i].addr, dev->io_regions[i].size,
                               pbars[j].addr, dev->io_regions[j].size)) {
                error_report("acpi/mmio64: invalid pci-boot-config — fixed BAR overlap on %s [%02x:%02x.%x]: "
                             "BAR%d [0x%lx..0x%lx] vs BAR%d [0x%lx..0x%lx]",
                             dev->name, pci_dev_bus_num(dev),
                             PCI_SLOT(dev->devfn), PCI_FUNC(dev->devfn),
                             i, pbars[i].addr, pbars[i].addr + dev->io_regions[i].size - 1,
                             j, pbars[j].addr, pbars[j].addr + dev->io_regions[j].size - 1);
                exit(1);
            }
        }
    }
}

static void pci_get_bridge_window(PCIBus *bus, void *opaque)
{
    PCIDevice *bridge = pci_bridge_get_device(bus);
    VirtPciAllocCfg *ncfg = (VirtPciAllocCfg *)opaque;
    struct GPEXConfig *cfg = ncfg->cfg;

    if (!bridge) {
        ncfg->wbase = cfg->mmio32.base;
        ncfg->wlimit = cfg->mmio32.base + cfg->mmio32.size - 1;
        ncfg->wbase64 = cfg->mmio64.base;
        ncfg->wlimit64 = cfg->mmio64.base + cfg->mmio64.size - 1;
    } else {
        ncfg->wbase = pci_bridge_get_base(bridge, PCI_BASE_ADDRESS_MEM_TYPE_32);
        ncfg->wlimit = pci_bridge_get_limit(bridge, PCI_BASE_ADDRESS_MEM_TYPE_32);
        ncfg->wbase64 = pci_bridge_get_base(bridge, PCI_BASE_ADDRESS_MEM_PREFETCH);
        ncfg->wlimit64 = pci_bridge_get_limit(bridge, PCI_BASE_ADDRESS_MEM_PREFETCH);
    }
}

static void pci_update_prefetch_window(PCIBus *bus, uint64_t base, uint64_t limit)
{
    PCIDevice *bridge = pci_bridge_get_device(bus);
    uint32_t value0, value1;

    assert(bridge);

    value0 = (uint32_t)(extract64(base, 20, 12) << 4);
    value1 = (uint32_t)(extract64(limit, 20, 12) << 4);
    pci_host_config_write_common(bridge,
                                 PCI_PREF_MEMORY_BASE,
                                 pci_config_size(bridge),
                                 value0 | PCI_PREF_RANGE_TYPE_64,
                                 2);
    pci_host_config_write_common(bridge,
                                 PCI_PREF_BASE_UPPER32,
                                 pci_config_size(bridge),
                                 (uint32_t)(base >> 32),
                                 4);
    pci_host_config_write_common(bridge,
                                 PCI_PREF_MEMORY_LIMIT,
                                 pci_config_size(bridge),
                                 value1 | PCI_PREF_RANGE_TYPE_64,
                                 2);
    pci_host_config_write_common(bridge,
                                 PCI_PREF_LIMIT_UPPER32,
                                 pci_config_size(bridge),
                                 (uint32_t)(limit >> 32),
                                 4);
}


/* Helper: program a set of packed prefetchable 64-bit BARs */
static void pci_program_pbars(PCIDevice *dev, PhysBAR *pbars, struct GPEXConfig *cfg)
{
    int idx;
    uint32_t laddr;

    for (idx = 0; idx < PCI_ROM_SLOT; idx++) {
        PhysBAR *pbar = &pbars[idx];
        
        if (!(pbar->flags & IORESOURCE_PREFETCH)) {
            continue;
        }
        laddr = pbar->addr & PCI_BASE_ADDRESS_MEM_MASK;
        laddr |= PCI_BASE_ADDRESS_MEM_TYPE_64;
        /* Set PREFETCH bit only if the BAR itself is prefetchable */
        if (dev->io_regions[idx].type & PCI_BASE_ADDRESS_MEM_PREFETCH) {
            laddr |= PCI_BASE_ADDRESS_MEM_PREFETCH;
        }
        
        /* Write to physical device config space */
        pci_host_config_write_common(dev,
                                     PCI_BASE_ADDRESS_0 + (idx * 4),
                                     pci_config_size(dev),
                                     laddr,
                                     4);
        pci_host_config_write_common(dev,
                                     PCI_BASE_ADDRESS_0 + (idx * 4) + 4,
                                     pci_config_size(dev),
                                     (uint32_t)(pbar->addr >> 32),
                                     4);
        
        warn_report("acpi/mmio64: programmed %s [%02x:%02x.%x] BAR%d -> 0x%lx",
                    dev->name, pci_dev_bus_num(dev),
                    PCI_SLOT(dev->devfn), PCI_FUNC(dev->devfn),
                    idx, pbar->addr);
        cfg->preserve_config = true;
    }
}

typedef struct BusDemandCtx {
    GHashTable *bus_to_demand; /* key: PCIBus* , value: uint64_t* bytes */
} BusDemandCtx;

static void pci_bus_add_demand_bytes(PCIBus *bus, uint64_t bytes, BusDemandCtx *bdc)
{
    uint64_t *p = g_hash_table_lookup(bdc->bus_to_demand, bus);
    if (!p) {
        p = g_new0(uint64_t, 1);
        g_hash_table_insert(bdc->bus_to_demand, bus, p);
    }
    *p += bytes;
}

/* Helper: check if a BAR is 64-bit prefetchable (what we allocate) */
static inline bool is_64bit_pref_bar(PCIIORegion *r)
{
    if (!r->size) {
        return false;
    }
    if (r->type & PCI_BASE_ADDRESS_SPACE_IO) {
        return false;
    }
    if (!(r->type & PCI_BASE_ADDRESS_MEM_TYPE_64)) {
        return false;
    }
    if (!(r->type & PCI_BASE_ADDRESS_MEM_PREFETCH)) {
        return false;
    }
    return true;
}

static void pci_collect_bus_pref64_demand_dev(PCIBus *bus, PCIDevice *dev, void *opaque)
{
    BusDemandCtx *bdc = (BusDemandCtx *)opaque;
    for (int idx = 0; idx < PCI_ROM_SLOT; idx++) {
        PCIIORegion *r = &dev->io_regions[idx];
        if (!is_64bit_pref_bar(r)) {
            continue;
        }
        /* If this BAR was fixed by pci-boot-config, it's already placed (no demand). */
        if (dev->fixed_bar_pci_boot_config &&
            dev->fixed_bar_pci_boot_config_addr[idx] != PCI_BAR_UNMAPPED) {
            continue;
        }
        pci_bus_add_demand_bytes(bus, r->size, bdc);
    }
}

static void pci_collect_bus_pref64_demand_bus(PCIBus *bus, void *opaque)
{
    BusDemandCtx *bdc = (BusDemandCtx *)opaque;
    pci_for_each_device_under_bus(bus, pci_collect_bus_pref64_demand_dev, bdc);
}

static void pci_debug_print_bus_pref64_demand(PCIBus *root)
{
    BusDemandCtx bdc = {
        .bus_to_demand = g_hash_table_new(g_direct_hash, g_direct_equal),
    };
    pci_for_each_bus(root, pci_collect_bus_pref64_demand_bus, &bdc);

    GHashTableIter iter;
    gpointer key, value;
    g_hash_table_iter_init(&iter, bdc.bus_to_demand);
    while (g_hash_table_iter_next(&iter, &key, &value)) {
        PCIBus *bus = (PCIBus *)key;
        uint64_t *bytes = (uint64_t *)value;
        if (*bytes) {
            warn_report("acpi/mmio64: bus [%02x] demand_pref64=0x%"PRIx64" bytes",
                        pci_bus_num(bus), *bytes);
        }
    }
    g_hash_table_destroy(bdc.bus_to_demand);
}




/* Address interval for hole/free region calculations */
typedef struct {
    uint64_t start;
    uint64_t end;
} AddressInterval;

/* Comparison function for sorting intervals by start address */
static int compare_intervals(gconstpointer a, gconstpointer b)
{
    const AddressInterval *ia = (const AddressInterval *)a;
    const AddressInterval *ib = (const AddressInterval *)b;
    if (ia->start < ib->start) return -1;
    if (ia->start > ib->start) return 1;
    return 0;
}

/* BAR entry for Phase 2/3 packing */
typedef struct {
    PCIDevice *dev;
    int bar_idx;
    uint64_t size;
} BarEntry;

/* Comparison function for sorting BARs by descending size */
static int compare_bar_size_desc(gconstpointer a, gconstpointer b)
{
    const BarEntry *ea = (const BarEntry *)a;
    const BarEntry *eb = (const BarEntry *)b;
    if (ea->size > eb->size) return -1;
    if (ea->size < eb->size) return 1;
    return 0;
}

#if 0  /* Unused after Phase 2 refactoring - kept for reference */
/* Comparison function for sorting claims by start address */
static int compare_claim_start(gconstpointer a, gconstpointer b)
{
    const FixedClaim *ca = (const FixedClaim *)a;
    const FixedClaim *cb = (const FixedClaim *)b;
    if (ca->start < cb->start) return -1;
    if (ca->start > cb->start) return 1;
    return 0;
}
#endif  /* Unused after Phase 2 refactoring */

/* Categorized holes relative to anchors */
typedef struct {
    int leftmost_hole;      /* Index of hole before first anchor, or -1 */
    GArray *middle_holes;   /* Array of hole indices between anchors */
    int rightmost_hole;     /* Index of hole after last anchor, or -1 */
} CategorizedHoles;

/* Helper: categorize holes relative to anchors */
static CategorizedHoles categorize_holes(GArray *holes, GArray *fixed_bars)
{
    CategorizedHoles result = {
        .leftmost_hole = -1,
        .middle_holes = g_array_new(false, false, sizeof(int)),
        .rightmost_hole = -1
    };
    
    /* Get anchor boundaries */
    uint64_t first_anchor_start = g_array_index(fixed_bars, AddressInterval, 0).start;
    uint64_t last_anchor_end = g_array_index(fixed_bars, AddressInterval,
                                               fixed_bars->len - 1).end;
    
    /* Categorize each hole */
    for (guint h = 0; h < holes->len; h++) {
        AddressInterval *hole = &g_array_index(holes, AddressInterval, h);
        
        if (hole->end < first_anchor_start) {
            result.leftmost_hole = h;  /* Before all anchors */
        } else if (hole->start > last_anchor_end) {
            result.rightmost_hole = h;  /* After all anchors */
        } else {
            g_array_append_val(result.middle_holes, h);  /* Between anchors */
        }
    }
    
    return result;
}

/* Helper: compute REAL holes considering both local anchors and global claims
 * This returns actual free space that can be used for packing.
 * Strategy: Collect all obstacles (local fixed BARs + global claims from other buses),
 * then compute gaps between them.
 */
static GArray* compute_real_holes(GArray *fixed_bars, uint64_t mmio_start, uint64_t mmio_end)
{
    GArray *holes = g_array_new(false, false, sizeof(AddressInterval));
    GArray *claimed_regions = g_array_new(false, false, sizeof(AddressInterval));
    
    /* Add local fixed BARs (anchors) as claimed regions */
    for (guint i = 0; i < fixed_bars->len; i++) {
        AddressInterval *anchor = &g_array_index(fixed_bars, AddressInterval, i);
        g_array_append_val(claimed_regions, *anchor);
    }
    
    /* Add global claims from ALL buses (including other buses) */
    if (virt_fixed_claims) {
        for (guint i = 0; i < virt_fixed_claims->len; i++) {
            FixedClaim *claim = &g_array_index(virt_fixed_claims, FixedClaim, i);
            /* Only consider claims within our MMIO window */
            if (claim->start <= mmio_end && claim->end >= mmio_start) {
                AddressInterval region = {
                    .start = claim->start,
                    .end = claim->end
                };
                g_array_append_val(claimed_regions, region);
            }
        }
    }
    
    /* Handle case with no claimed regions */
    if (claimed_regions->len == 0) {
        AddressInterval hole = { .start = mmio_start, .end = mmio_end };
        g_array_append_val(holes, hole);
        g_array_free(claimed_regions, true);
        return holes;
    }
    
    /* Sort claimed regions by start address */
    g_array_sort(claimed_regions, compare_intervals);
    
    /* Compute holes between all claimed regions */
    uint64_t scan = mmio_start;
    
    for (guint i = 0; i < claimed_regions->len; i++) {
        AddressInterval *claimed = &g_array_index(claimed_regions, AddressInterval, i);
        
        /* Free space before this claimed region */
        if (scan < claimed->start) {
            AddressInterval hole = { .start = scan, .end = claimed->start - 1 };
            g_array_append_val(holes, hole);
        }
        
        /* Move scan cursor past this claimed region */
        scan = MAX(scan, claimed->end + 1);
    }
    
    /* Free space after last claimed region */
    if (scan <= mmio_end) {
        AddressInterval hole = { .start = scan, .end = mmio_end };
        g_array_append_val(holes, hole);
    }
    
    g_array_free(claimed_regions, true);
    return holes;
}


/* Helper: pack BARs into a given region and return window bounds */
static bool pack_bars_into_region(GArray *bars, uint64_t pack_start, uint64_t pack_end,
                                   struct GPEXConfig *cfg,
                                   uint64_t *out_min_addr, uint64_t *out_max_addr)
{
    uint64_t pack_cursor = pack_start;
    uint64_t min_addr = UINT64_MAX;
    uint64_t max_addr = 0;

    for (guint i = 0; i < bars->len; i++) {
        BarEntry *e = &g_array_index(bars, BarEntry, i);
        PCIIORegion *r = &e->dev->io_regions[e->bar_idx];

        uint64_t aligned_addr = ROUND_UP(pack_cursor, r->size);
        uint64_t bar_start = aligned_addr;
        uint64_t bar_end = bar_start + r->size - 1;

        if (bar_end > pack_end) {
            return false; /* Doesn't fit */
        }

        PhysBAR pbars_array[PCI_ROM_SLOT];
        memset(pbars_array, 0, sizeof(pbars_array));
        pbars_array[e->bar_idx].addr = bar_start;
        pbars_array[e->bar_idx].end = bar_end;
        pbars_array[e->bar_idx].flags = IORESOURCE_PREFETCH;

        pci_program_pbars(e->dev, pbars_array, cfg);

        min_addr = MIN(min_addr, bar_start);
        max_addr = MAX(max_addr, bar_end);
        pack_cursor = bar_end + 1;
    }

    *out_min_addr = min_addr;
    *out_max_addr = max_addr;
    return true;
}

/* Helper: finalize bridge window by programming and claiming */
static void finalize_bridge_window(PCIBus *bus, uint64_t min_addr, uint64_t max_addr,
                                    const char *phase_name)
{
    PCIDevice *bridge_dev = pci_bridge_get_device(bus);

    if (bridge_dev) {
        virt_fixed_claims_add(min_addr, max_addr, bridge_dev, -1);
        pci_update_prefetch_window(bus, min_addr, max_addr);

        warn_report("acpi/mmio64: %s bus [%02x] claimed bridge window: "
                    "[0x%"PRIx64"..0x%"PRIx64"] size=0x%"PRIx64,
                    phase_name, pci_bus_num(bus), min_addr, max_addr,
                    max_addr - min_addr + 1);
        warn_report("acpi/mmio64: %s bus [%02x] programmed bridge [%02x:%02x.%x] PREF window",
                    phase_name, pci_bus_num(bus), pci_dev_bus_num(bridge_dev),
                    PCI_SLOT(bridge_dev->devfn), PCI_FUNC(bridge_dev->devfn));
    }
}

/* Three-phase programming context */
typedef enum {
    PCI_PHASE_CLAIM_AND_PROGRAM_FIXED_BARS = 0,
    PCI_PHASE_PACK_BARS_FOR_FIXED_BAR_DEVICES = 1,
    PCI_PHASE_PACK_BARS_FOR_NON_FIXED_DEVICES = 2,
} VirtPciPhase;

typedef struct {
    struct GPEXConfig *cfg;
    VirtPciPhase phase;
    GHashTable *had_fixed; /* set of PCIDevice* that had at least one fixed BAR */
} VirtPciProgramCtx;

static void pci_dev_program_bars_phase(PCIBus *bus, PCIDevice *dev, void *opaque)
{
    VirtPciProgramCtx *pctx = (VirtPciProgramCtx *)opaque;
    struct GPEXConfig *cfg = pctx->cfg;
    PhysBAR *pbar, pbars[PCI_ROM_SLOT];
    int idx;
    bool had_any_fixed = false;

    pbar = pbars;
    memset(pbar, 0, sizeof(pbars));

    switch (pctx->phase) {
    case PCI_PHASE_CLAIM_AND_PROGRAM_FIXED_BARS: {
        if (!dev->fixed_bar_pci_boot_config) {
            return;
        }
        warn_report("acpi/mmio64: phase1 (fixed) dev [%02x:%02x.%x] %s",
                    pci_dev_bus_num(dev), PCI_SLOT(dev->devfn), PCI_FUNC(dev->devfn),
                    dev->name);
        /* Place fixed bars and program them */
        for (idx = 0; idx < PCI_ROM_SLOT; idx++) {
            PCIIORegion *r = &dev->io_regions[idx];
            if (dev->fixed_bar_pci_boot_config_addr[idx] == PCI_BAR_UNMAPPED) {
                continue;
            }
            pci_validate_fixed_bar(dev, idx,
                                        dev->fixed_bar_pci_boot_config_addr[idx],
                                        r->size,
                                        cfg->mmio64.base,
                                        cfg->mmio64.base + cfg->mmio64.size - 1);
            /* cross-device first-win against existing claims */
            {
                uint64_t start = dev->fixed_bar_pci_boot_config_addr[idx];
                uint64_t end = start + r->size - 1;
                if (virt_fixed_claims_conflicts(start, end,
                                                cfg->mmio64.base,
                                                cfg->mmio64.base + cfg->mmio64.size - 1,
                                                NULL)) {
                    error_report("acpi/mmio64: invalid pci-boot-config — fixed BAR for %s [%02x:%02x.%x] "
                                 "BAR%d [0x%"PRIx64"..0x%"PRIx64"] overlaps an existing fixed range",
                                 dev->name, pci_dev_bus_num(dev),
                                 PCI_SLOT(dev->devfn), PCI_FUNC(dev->devfn),
                                 idx, start, end);
                    exit(1);
                }
                virt_fixed_claims_add(start, end, dev, idx);
            }
            pbars[idx].addr = dev->fixed_bar_pci_boot_config_addr[idx];
            pbars[idx].end = pbars[idx].addr + r->size - 1;
            pbars[idx].flags = IORESOURCE_PREFETCH;
            had_any_fixed = true;
        }
        if (had_any_fixed) {
            g_hash_table_insert(pctx->had_fixed, dev, dev);
        }
        /* Abort if intra-device fixed overlap */
        pci_check_fixed_bar_overlap(dev, pbars);
        /* Program fixed BARs now */
        pci_program_pbars(dev, pbars, cfg);
        break;
    }
    case PCI_PHASE_PACK_BARS_FOR_FIXED_BAR_DEVICES: {
        if (!g_hash_table_contains(pctx->had_fixed, dev)) {
            return;
        }
        warn_report("acpi/mmio64: phase2 (pack-fixed-dev) dev [%02x:%02x.%x] %s",
                    pci_dev_bus_num(dev), PCI_SLOT(dev->devfn), PCI_FUNC(dev->devfn),
                    dev->name);
        
        PCIBus *this_bus = pci_get_bus(dev);
        uint64_t mmio_start = cfg->mmio64.base;
        uint64_t mmio_end = cfg->mmio64.base + cfg->mmio64.size - 1;
        
        /* Single pass: collect both fixed and remaining BARs */
        GArray *fixed_bars = g_array_new(false, false, sizeof(AddressInterval));
        GArray *remaining_bars = g_array_new(false, false, sizeof(BarEntry));

        for (int devfn = 0; devfn < ARRAY_SIZE(this_bus->devices); devfn++) {
            PCIDevice *d = this_bus->devices[devfn];
            if (!d) {
                continue;
            }

            /* Collect fixed BARs only from devices that have pci-boot-config */
            bool device_has_fixed = g_hash_table_contains(pctx->had_fixed, d);

            for (int i = 0; i < PCI_ROM_SLOT; i++) {
                PCIIORegion *r = &d->io_regions[i];
                if (!is_64bit_pref_bar(r)) {
                    continue;
                }

                if (device_has_fixed &&
                    d->fixed_bar_pci_boot_config &&
                    d->fixed_bar_pci_boot_config_addr[i] != PCI_BAR_UNMAPPED) {
                    /* Fixed BAR: collect for hole computation */
                    AddressInterval interval = {
                        .start = d->fixed_bar_pci_boot_config_addr[i],
                        .end = d->fixed_bar_pci_boot_config_addr[i] + r->size - 1
                    };
                    g_array_append_val(fixed_bars, interval);
                } else {
                    /* Remaining BAR: collect for packing (from ALL devices on this bus) */
                    BarEntry entry = { .dev = d, .bar_idx = i, .size = r->size };
                    g_array_append_val(remaining_bars, entry);
                }
            }
        }
        
        if (remaining_bars->len == 0) {
            /* No remaining BARs to pack; still set bridge window from fixed BARs only */
            if (fixed_bars->len > 0) {
                g_array_sort(fixed_bars, compare_intervals);
                uint64_t bus_min_addr = g_array_index(fixed_bars, AddressInterval, 0).start;
                uint64_t bus_max_addr = g_array_index(fixed_bars, AddressInterval,
                                                      fixed_bars->len - 1).end;
                finalize_bridge_window(this_bus, bus_min_addr, bus_max_addr, "phase2");
            }
            g_array_free(fixed_bars, true);
            g_array_free(remaining_bars, true);
            break;
        }
        
        /* Sort fixed BARs by address (for hole computation) */
        g_array_sort(fixed_bars, compare_intervals);
        
        /* Sort remaining BARs by size descending (for packing) */
        g_array_sort(remaining_bars, compare_bar_size_desc);
        
        /* Calculate remaining demand */
        uint64_t remaining_demand = 0;
        for (guint i = 0; i < remaining_bars->len; i++) {
            BarEntry *e = &g_array_index(remaining_bars, BarEntry, i);
            remaining_demand += e->size;
        }
        
        warn_report("acpi/mmio64: phase2 bus [%02x] remaining_bars=%u remaining_demand=0x%"PRIx64,
                    pci_bus_num(this_bus), remaining_bars->len, remaining_demand);
        
        /* Compute valid range for THIS bus by finding nearest claims */
        uint64_t leftmost_anchor = g_array_index(fixed_bars, AddressInterval, 0).start;
        uint64_t rightmost_anchor_end = g_array_index(fixed_bars, AddressInterval, 
                                                        fixed_bars->len - 1).end;
        
        uint64_t valid_start = mmio_start;
        uint64_t valid_end = mmio_end;
        
        /* Scan global claims to find boundaries */
        if (virt_fixed_claims) {
            for (guint i = 0; i < virt_fixed_claims->len; i++) {
                FixedClaim *claim = &g_array_index(virt_fixed_claims, FixedClaim, i);
                
                /* Find highest claim ending before our leftmost anchor */
                if (claim->end < leftmost_anchor && claim->end >= valid_start) {
                    valid_start = claim->end + 1;
                }
                
                /* Find lowest claim starting after our rightmost anchor */
                if (claim->start > rightmost_anchor_end && claim->start <= valid_end) {
                    valid_end = claim->start - 1;
                }
            }
        }
        
        warn_report("acpi/mmio64: phase2 bus [%02x] valid_range=[0x%"PRIx64"..0x%"PRIx64"] "
                    "anchors=[0x%"PRIx64"..0x%"PRIx64"]",
                    pci_bus_num(this_bus), valid_start, valid_end, 
                    leftmost_anchor, rightmost_anchor_end);
        
        /* Compute REAL holes within THIS bus's valid range */
        GArray *holes = compute_real_holes(fixed_bars, valid_start, valid_end);
        
        warn_report("acpi/mmio64: phase2 bus [%02x] found %u fixed BARs, %u real holes",
                    pci_bus_num(this_bus), fixed_bars->len, holes->len);
        
        /* Categorize holes: leftmost (before first anchor), middle, rightmost (after last anchor) */
        CategorizedHoles cat = categorize_holes(holes, fixed_bars);
        
        /* Strategy: Try largest middle hole first, then rightmost, then leftmost */
        int selected_hole = -1;
        uint64_t pack_start = 0, pack_end = 0;
        const char *hole_type = NULL;
        
        /* 1. Try largest middle hole first (optimal for bridge window) */
        if (cat.middle_holes->len > 0) {
            int largest_middle = -1;
            uint64_t largest_size = 0;
            
            for (guint i = 0; i < cat.middle_holes->len; i++) {
                int h = g_array_index(cat.middle_holes, int, i);
                AddressInterval *hole = &g_array_index(holes, AddressInterval, h);
                uint64_t hole_size = hole->end - hole->start + 1;
                
                if (hole_size >= remaining_demand && hole_size > largest_size) {
                    largest_size = hole_size;
                    largest_middle = h;
                }
            }
            
            if (largest_middle >= 0) {
                selected_hole = largest_middle;
                hole_type = "MIDDLE";
                warn_report("acpi/mmio64: phase2 bus [%02x] selected largest MIDDLE hole %d (size=0x%"PRIx64")",
                            pci_bus_num(this_bus), selected_hole, largest_size);
            }
        }
        
        /* 2. Try rightmost hole (after last anchor) */
        if (selected_hole < 0 && cat.rightmost_hole >= 0) {
            AddressInterval *hole = &g_array_index(holes, AddressInterval, cat.rightmost_hole);
            uint64_t hole_size = hole->end - hole->start + 1;
            
            if (hole_size >= remaining_demand) {
                selected_hole = cat.rightmost_hole;
                hole_type = "RIGHT";
                warn_report("acpi/mmio64: phase2 bus [%02x] selected RIGHTMOST hole %d (size=0x%"PRIx64")",
                            pci_bus_num(this_bus), selected_hole, hole_size);
            }
        }
        
        /* 3. Try leftmost hole (before first anchor) as last resort */
        if (selected_hole < 0 && cat.leftmost_hole >= 0) {
            AddressInterval *hole = &g_array_index(holes, AddressInterval, cat.leftmost_hole);
            uint64_t hole_size = hole->end - hole->start + 1;
            
            if (hole_size >= remaining_demand) {
                selected_hole = cat.leftmost_hole;
                hole_type = "LEFT";
                warn_report("acpi/mmio64: phase2 bus [%02x] selected LEFTMOST hole %d (size=0x%"PRIx64")",
                            pci_bus_num(this_bus), selected_hole, hole_size);
            }
        }
        
        g_array_free(cat.middle_holes, true);
        
        if (selected_hole < 0) {
            error_report("acpi/mmio64: phase2 bus [%02x] insufficient contiguous space for "
                         "remaining_demand=0x%"PRIx64,
                         pci_bus_num(this_bus), remaining_demand);
            g_array_free(holes, true);
            g_array_free(fixed_bars, true);
            g_array_free(remaining_bars, true);
            exit(1);
        }
        
        /* Pack in selected hole */
        AddressInterval *selected = &g_array_index(holes, AddressInterval, selected_hole);
        pack_start = selected->start;
        pack_end = selected->end;
        
        warn_report("acpi/mmio64: phase2 bus [%02x] packing %s in hole %d: [0x%"PRIx64"..0x%"PRIx64"]",
                    pci_bus_num(this_bus), hole_type, selected_hole, pack_start, pack_end);
        
        g_array_free(holes, true);
        
        /* Pack and program BARs (largest first) */
        uint64_t bus_min_addr, bus_max_addr;
        if (!pack_bars_into_region(remaining_bars, pack_start, pack_end, cfg,
                                    &bus_min_addr, &bus_max_addr)) {
            error_report("acpi/mmio64: phase2 bus [%02x] failed to pack BARs",
                         pci_bus_num(this_bus));
            g_array_free(fixed_bars, true);
            g_array_free(remaining_bars, true);
            exit(1);
        }
        
        /* Include fixed BARs in bridge window calculation */
        for (guint i = 0; i < fixed_bars->len; i++) {
            AddressInterval *fixed = &g_array_index(fixed_bars, AddressInterval, i);
            bus_min_addr = MIN(bus_min_addr, fixed->start);
            bus_max_addr = MAX(bus_max_addr, fixed->end);
        }
        
        /* Finalize bridge window */
        finalize_bridge_window(this_bus, bus_min_addr, bus_max_addr, "phase2");
        
        g_array_free(fixed_bars, true);
        g_array_free(remaining_bars, true);
        break;
    }
    case PCI_PHASE_PACK_BARS_FOR_NON_FIXED_DEVICES:
        /* Phase 3: TODO - implement later */
        break;
    default:
        break;
    }
}

static void pci_bus_program_bars_phase(PCIBus *bus, void *opaque)
{
    pci_for_each_device_under_bus(bus, pci_dev_program_bars_phase, opaque);
}

static void pci_collect_mmio64_window(PCIBus *bus, PCIDevice *dev, void *opaque)
{
    VirtPciAllocCfg *ncfg = (VirtPciAllocCfg *)opaque;
    uint64_t rbase, rlimit;
    uint32_t idx;

    for (idx = 0; idx < PCI_ROM_SLOT; idx++) {
        PCIIORegion *res = &dev->io_regions[idx];

        if ((!res->size) ||
            ((res->addr < ncfg->wbase64) || (res->addr > ncfg->wlimit64))) {
            continue;
        }
        rbase = res->addr;
        rlimit = res->addr + res->size - 1;
        ncfg->rbase = MIN(ncfg->rbase, rbase);
        ncfg->rlimit = MAX(ncfg->rlimit, rlimit);
    }

    if (IS_PCI_BRIDGE(dev)) {
        rbase = pci_bridge_get_base(dev, PCI_BASE_ADDRESS_MEM_PREFETCH);
        rlimit = pci_bridge_get_limit(dev, PCI_BASE_ADDRESS_MEM_PREFETCH);

        if ((rbase < ncfg->wbase64) ||
            (rbase > ncfg->wlimit64) ||
            (rlimit < ncfg->wbase64) ||
            (rlimit > ncfg->wlimit64)) {
            return;
        }

        ncfg->rbase = MIN(ncfg->rbase, rbase);
        ncfg->rlimit = MAX(ncfg->rlimit, rlimit);
    }
}

static void pci_bus_update_prefetch_window(PCIBus *bus, void *opaque)
{
    VirtPciAllocCfg *ncfg = (VirtPciAllocCfg *)opaque;
    ncfg->rbase = ~0;
    ncfg->rlimit = 0;

    assert(pci_bridge_get_device(bus));
    pci_for_each_device_under_bus(bus, pci_collect_mmio64_window, ncfg);

    if (ncfg->rlimit > ncfg->rbase) {
        pci_update_prefetch_window(bus, ncfg->rbase, ncfg->rlimit);
        warn_report("acpi/mmio64: bridge [%02x:%02x.%x] PREF window "
                    "=[0x%"PRIx64"..0x%"PRIx64"]",
                    pci_dev_bus_num(pci_bridge_get_device(bus)),
                    PCI_SLOT(pci_bridge_get_device(bus)->devfn),
                    PCI_FUNC(pci_bridge_get_device(bus)->devfn),
                    (uint64_t)ncfg->rbase, (uint64_t)ncfg->rlimit);
    }
}

static void pci_dev_check_unassigned_mmio64(PCIBus *bus, PCIDevice *dev, void *opaque)
{
    VirtPciAllocCfg *ncfg0 = (VirtPciAllocCfg *)opaque;
    uint64_t base, limit;

    if (!IS_PCI_BRIDGE(dev)) {
        return;
    }

    base = pci_bridge_get_base(dev, PCI_BASE_ADDRESS_MEM_PREFETCH);
    limit = pci_bridge_get_limit(dev, PCI_BASE_ADDRESS_MEM_PREFETCH);

    warn_report("acpi/mmio64: bridge [%02x:%02x.%x] current PREF "
                "=[0x%"PRIx64"..0x%"PRIx64"] window=[0x%"PRIx64"..0x%"PRIx64"]",
                pci_dev_bus_num(dev), PCI_SLOT(dev->devfn), PCI_FUNC(dev->devfn),
                (uint64_t)base, (uint64_t)limit,
                (uint64_t)ncfg0->wbase64, (uint64_t)ncfg0->wlimit64);

    /* Unprogrammed or empty window: This behavior is typical when no child devices
     * downstream of the bridge have requested any Base Address Register*/
    if (base >= limit) {
        warn_report("acpi/mmio64: bridge [%02x:%02x.%x] PREF window unprogrammed/empty; no action",
                    pci_dev_bus_num(dev), PCI_SLOT(dev->devfn), PCI_FUNC(dev->devfn));
        return;
    }

    /* Already in-window: nothing to do */
    if ((base >= ncfg0->wbase64) &&
        (limit <= ncfg0->wlimit64)) {
        warn_report("acpi/mmio64: bridge [%02x:%02x.%x] PREF already in-window; skip",
                    pci_dev_bus_num(dev), PCI_SLOT(dev->devfn), PCI_FUNC(dev->devfn));
        return;
    }

    /* Out-of-window: This should never happen */
    warn_report("acpi/mmio64: bridge [%02x:%02x.%x] PREF out of mmio64 window; no action",
                pci_dev_bus_num(dev), PCI_SLOT(dev->devfn), PCI_FUNC(dev->devfn));
}

static void pci_bus_check_unassigned_mmio64(PCIBus *bus, void *opaque)
{
    pci_for_each_device_under_bus(bus, pci_dev_check_unassigned_mmio64, opaque);
}

/* Update Device Tree PCI ranges: one MMIO64 entry for the entire high MMIO window (EDK2 expects a single range). */
static void virt_update_fdt_pcie_ranges(VirtMachineState *vms)
{
    MachineState *ms = MACHINE(vms);
    hwaddr base_mmio = vms->memmap[VIRT_PCIE_MMIO].base;
    hwaddr size_mmio = vms->memmap[VIRT_PCIE_MMIO].size;
    hwaddr base_pio = vms->memmap[VIRT_PCIE_PIO].base;
    hwaddr size_pio = vms->memmap[VIRT_PCIE_PIO].size;
    hwaddr base_mmio_high = vms->memmap[VIRT_HIGH_PCIE_MMIO].base;
    hwaddr size_mmio_high = vms->memmap[VIRT_HIGH_PCIE_MMIO].size;
    const char *nodename = vms->pciehb_nodename;
    GArray *ranges;

    if (!nodename) {
        return;
    }

    ranges = g_array_new(FALSE, FALSE, sizeof(uint32_t));

    /* 1 IO + 1 MMIO32 + 1 MMIO64 (full window), matching virt.c and EDK2 expectation */
    uint32_t io_range[] = {
        cpu_to_be32(FDT_PCI_RANGE_IOPORT), 0, 0,
        cpu_to_be32(base_pio >> 32), cpu_to_be32(base_pio),
        cpu_to_be32(size_pio >> 32), cpu_to_be32(size_pio)
    };
    g_array_append_vals(ranges, io_range, 7);

    uint32_t mmio32_range[] = {
        cpu_to_be32(FDT_PCI_RANGE_MMIO),
        cpu_to_be32(base_mmio >> 32), cpu_to_be32(base_mmio),
        cpu_to_be32(base_mmio >> 32), cpu_to_be32(base_mmio),
        cpu_to_be32(size_mmio >> 32), cpu_to_be32(size_mmio)
    };
    g_array_append_vals(ranges, mmio32_range, 7);

    uint32_t mmio64_range[] = {
        cpu_to_be32(FDT_PCI_RANGE_MMIO_64BIT),
        cpu_to_be32(base_mmio_high >> 32), cpu_to_be32(base_mmio_high),
        cpu_to_be32(base_mmio_high >> 32), cpu_to_be32(base_mmio_high),
        cpu_to_be32(size_mmio_high >> 32), cpu_to_be32(size_mmio_high)
    };
    g_array_append_vals(ranges, mmio64_range, 7);

    int ret = qemu_fdt_setprop(ms->fdt, nodename, "ranges",
                               ranges->data, ranges->len * sizeof(uint32_t));
    if (ret < 0) {
        warn_report("Failed to update FDT ranges: %s", fdt_strerror(ret));
    } else {
        warn_report("Successfully updated FDT ranges with %u entries", (unsigned)(ranges->len / 7));
    }

    g_array_free(ranges, TRUE);
}

void pci_fixed_bar_allocator(struct GPEXConfig *cfg, VirtMachineState *vms)
{
    VirtPciAllocCfg ncfg1, *ncfg = &ncfg1;
    PCIBus *bus = cfg->bus;

    /* Reset fixed-claims tracking (first-win across devices) */
    virt_fixed_claims_reset();

    warn_report("acpi/mmio64: allocator begin mmio64=[0x%"PRIx64"..0x%"PRIx64"]",
                (uint64_t)cfg->mmio64.base,
                (uint64_t)(cfg->mmio64.base + cfg->mmio64.size - 1));

    pci_debug_print_bus_pref64_demand(bus);
    /* Phase 1: program all fixed BARs and claim them */
    {
        VirtPciProgramCtx pctx = {
            .cfg = cfg,
            .phase = PCI_PHASE_CLAIM_AND_PROGRAM_FIXED_BARS,
            .had_fixed = g_hash_table_new(NULL, NULL),
        };
        pci_for_each_bus(bus, pci_bus_program_bars_phase, &pctx);

        /* Phase 2: pack remaining bars for devices that had fixed BARs */
        pctx.phase = PCI_PHASE_PACK_BARS_FOR_FIXED_BAR_DEVICES;
        pci_for_each_bus(bus, pci_bus_program_bars_phase, &pctx);

        /* Phase 3: pack bars for devices with no pci-boot-config */
        pctx.phase = PCI_PHASE_PACK_BARS_FOR_NON_FIXED_DEVICES;
        pci_for_each_bus(bus, pci_bus_program_bars_phase, &pctx);

        g_hash_table_destroy(pctx.had_fixed);
    }
    
    /* Validate: detect any global device-level 64-bit prefetchable span overlaps */
    // TODO
    //
    if (!cfg->preserve_config) {
        /* Cleanup */
        virt_fixed_claims_reset();
        warn_report("acpi/mmio64: allocator end ");
        return;
    }

    memset(ncfg, 0, sizeof(VirtPciAllocCfg));
    ncfg->cfg = cfg;

    /* TODO: 32-bit MMIO/ROM adjustment */

    /* TODO: PIO assignment */

    /* TODO: 64-bit non-pretetcable */


    pci_get_bridge_window(bus, ncfg);

    QLIST_FOREACH(bus, &bus->child, sibling) {
        ncfg->bus = bus;
        /* Use the full mmio64 window */
        ncfg->wbase64 = cfg->mmio64.base;
        ncfg->wlimit64 = cfg->mmio64.base + cfg->mmio64.size - 1;

        pci_for_each_bus(bus, pci_bus_update_prefetch_window, ncfg);
        pci_for_each_bus(bus, pci_bus_check_unassigned_mmio64, ncfg);
    }

    /* All root port bridge windows are programmed; add FDT ranges for each root port */
    virt_update_fdt_pcie_ranges(vms);

    /* Cleanup */
    virt_fixed_claims_reset();
    warn_report("acpi/mmio64: allocator end");
}
