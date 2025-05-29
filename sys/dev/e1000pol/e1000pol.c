/*
 * Conceptual FreeBSD driver for Intel PRO/1000 (e1000) family.
 * Based on a conversion from an LWK kernel driver.
 *
 * IMPORTANT: This is a structural sketch and not a complete, production-ready driver.
 * It attempts to follow the "no network stack libraries" constraint by omitting
 * ifnet integration and preparing for a raw packet interface (e.g., char device).
 */

 #include <sys/cdefs.h>
 __FBSDID("$FreeBSD$"); // Standard FreeBSD CVS/SVN Id tag
 
 #include <sys/param.h>
 #include <sys/systm.h>
 #include <sys/bus.h>
 #include <sys/kernel.h>
 #include <sys/module.h>
 #include <sys/malloc.h>
 #include <sys/rman.h>
 #include <sys/endian.h> // For byte order conversions
 #include <sys/ioccom.h> // For ioctls if we add a char dev (not fully implemented)
 #include <sys/sockio.h> // For SIOCGIFADDR, etc. if we were to get/set MAC (partially)
 #include <sys/param.h>
 #include <sys/lock.h>
 #include <sys/mutex.h>
 
 #include <machine/bus.h> // For bus_space_tag_t, etc.
 #include <machine/resource.h>
 
 #include <dev/pci/pcireg.h>
 #include <dev/pci/pcivar.h>
 
 // Forward declarations for device methods
 static int e1000_fbsd_probe(device_t dev);
 static int e1000_fbsd_attach(device_t dev);
 static int e1000_fbsd_detach(device_t dev);
 static void e1000_fbsd_intr(void *arg);
 // static int e1000_fbsd_shutdown(device_t dev); // Optional
 
 /*
  * Register definitions for e1000.
  * A full driver would have a comprehensive e1000_hw.h.
  * These are common registers.
  */
 #define E1000_REG_CTRL     0x00000 // Device Control
 #define E1000_REG_STATUS   0x00008 // Device Status
 #define E1000_REG_EECD     0x00010 // EEPROM Control
 #define E1000_REG_EERD     0x00014 // EEPROM Read Data
 
 #define E1000_REG_ICR      0x000C0 // Interrupt Cause Read
 #define E1000_REG_ICS      0x000C8 // Interrupt Cause Set
 #define E1000_REG_IMS      0x000D0 // Interrupt Mask Set
 #define E1000_REG_IMC      0x000D8 // Interrupt Mask Clear
 
 #define E1000_REG_RCTL     0x00100 // Receive Control
 #define E1000_REG_RDBAL    0x02800 // RX Descriptor Base Address Low
 #define E1000_REG_RDBAH    0x02804 // RX Descriptor Base Address High
 #define E1000_REG_RDLEN    0x02808 // RX Descriptor Length
 #define E1000_REG_RDH      0x02810 // RX Descriptor Head
 #define E1000_REG_RDT      0x02818 // RX Descriptor Tail
 
 #define E1000_REG_TCTL     0x00400 // Transmit Control
 #define E1000_REG_TDBAL    0x03800 // TX Descriptor Base Address Low
 #define E1000_REG_TDBAH    0x03804 // TX Descriptor Base Address High
 #define E1000_REG_TDLEN    0x03808 // TX Descriptor Length
 #define E1000_REG_TDH      0x03810 // TX Descriptor Head
 #define E1000_REG_TDT      0x03818 // TX Descriptor Tail
 
 #define E1000_REG_RAL_BASE 0x05400 // Receive Address Low (for MAC address array)
 #define E1000_REG_RAH_BASE 0x05404 // Receive Address High (for MAC address array)
 
 /* CTRL register bits */
 #define E1000_CTRL_RST     (1 << 26) /* Software Reset */
 #define E1000_CTRL_SLU     (1 << 6)  /* Set Link Up */
 
 /* RCTL register bits */
 #define E1000_RCTL_EN      (1 << 1)  /* Receiver Enable */
 #define E1000_RCTL_SBP     (1 << 2)  /* Store Bad Packets */
 #define E1000_RCTL_UPE     (1 << 3)  /* Unicast Promiscuous Enabled */
 #define E1000_RCTL_MPE     (1 << 4)  /* Multicast Promiscuous Enabled */
 #define E1000_RCTL_BAM     (1 << 15) /* Broadcast Accept Mode */
 #define E1000_RCTL_SZ_2048 (0 << 16) /* Rx Buffer Size 2048 */
 #define E1000_RCTL_SECRC   (1 << 26) /* Strip Ethernet CRC */
 
 /* TCTL register bits */
 #define E1000_TCTL_EN      (1 << 1)  /* Transmit Enable */
 #define E1000_TCTL_PSP     (1 << 3)  /* Pad Short Packets */
 #define E1000_TCTL_CT_SHIFT 4        /* Collision Threshold shift */
 #define E1000_TCTL_COLD_SHIFT 12     /* Collision Distance shift */
 
 /* Interrupt Mask/Cause bits */
 #define E1000_ICR_TXDW     (1 << 0)  /* Transmit Descriptor Written Back */
 #define E1000_ICR_LSC      (1 << 2)  /* Link Status Change */
 #define E1000_ICR_RXDMT0   (1 << 4)  /* RX Descriptor Minimum Threshold hit */
 #define E1000_ICR_RXT0     (1 << 7)  /* Receiver Timer Interrupt */
 
 
 // Helper macros for register access (from original e1000.c, adapted)
 #define E1000_WRITE_REG(sc, reg, value) \
     bus_space_write_4((sc)->mem_bst, (sc)->mem_bsh, (reg), (value))
 #define E1000_READ_REG(sc, reg) \
     bus_space_read_4((sc)->mem_bst, (sc)->mem_bsh, (reg))
 #define E1000_WRITE_FLUSH(sc) \
     E1000_READ_REG(sc, E1000_REG_STATUS) // Flush previous writes
 
 // Descriptor structures (simplified from typical e1000 descriptors)
 struct e1000_rx_desc {
     volatile uint64_t buffer_addr;
     volatile uint16_t length;
     volatile uint16_t checksum;
     volatile uint8_t  status;
     volatile uint8_t  errors;
     volatile uint16_t special;
 } __attribute__((packed));
 
 struct e1000_tx_desc {
     volatile uint64_t buffer_addr;
     volatile union {
         uint32_t data;
         struct {
             uint16_t length;
             uint8_t cso;
             uint8_t cmd;
         } flags;
     } lower;
     volatile union {
         uint32_t data;
         struct {
             uint8_t status;
             uint8_t css;
             uint16_t special;
         } fields;
     } upper;
 } __attribute__((packed));
 
 /* TX Command bits */
 #define E1000_TXD_CMD_EOP    (1 << 0) /* End of Packet */
 #define E1000_TXD_CMD_IFCS   (1 << 1) /* Insert FCS (CRC) */
 #define E1000_TXD_CMD_RS     (1 << 3) /* Report Status */
 /* RX Status bits */
 #define E1000_RXD_STAT_DD    (1 << 0) /* Descriptor Done */
 #define E1000_RXD_STAT_EOP   (1 << 1) /* End of Packet */
 
 
 #define E1000_NUM_TX_DESC  64
 #define E1000_NUM_RX_DESC  64
 #define E1000_RX_BUFFER_SIZE 2048 // Must be large enough for max Ethernet frame + CRC
 #define E1000_RING_ALIGN   16    // Descriptors often need 16-byte alignment
 #define E1000_MAX_TX_SEGS  1     // Simplified to 1 segment for raw packet model
 
 // Per-instance software context
 struct e1000_softc {
     device_t        dev;
     struct resource *mem_res;   // Memory-mapped BAR
     int             mem_rid;
     bus_space_tag_t mem_bst;
     bus_space_handle_t mem_bsh;
 
     struct resource *irq_res;   // IRQ resource
     int             irq_rid;
     void            *intr_cookie;
 
     uint8_t         mac_addr[6];
 
     // Transmit resources
     struct e1000_tx_desc *tx_desc_ring;
     bus_addr_t      tx_desc_ring_paddr;
     bus_dma_tag_t   tx_desc_dtag;
     bus_dmamap_t    tx_desc_dmamap;
     // For raw packet send, we'd need buffer management similar to RX
     // This is highly simplified for now.
     void            **tx_buffers; // Placeholder for actual buffer management
     bus_dma_tag_t   tx_buffer_dtag;
     bus_dmamap_t    *tx_buffer_dmamaps;
     int             tx_head;
     int             tx_tail;
     int             tx_descriptors_free;
 
 
     // Receive resources
     struct e1000_rx_desc *rx_desc_ring;
     bus_addr_t      rx_desc_ring_paddr;
     bus_dma_tag_t   rx_desc_dtag;
     bus_dmamap_t    rx_desc_dmamap;
 
     void            **rx_buffers; // Array of KVA pointers to RX buffers
     bus_dma_tag_t   rx_buffer_dtag;
     bus_dmamap_t    *rx_buffer_dmamaps; // DMA maps for each RX buffer
     int             rx_head; // Next descriptor to check for received packet
     // int             rx_tail; // Next descriptor to give to HW (implicit by updating RDT)
 
     // struct cdev     *cdev; // For character device interface (not implemented)
     struct mtx      sc_mtx; // General purpose lock
 };
 
 #define E1000_LOCK_INIT(sc) mtx_init(&(sc)->sc_mtx, device_get_nameunit((sc)->dev), "e1000 softc lock", MTX_DEF)
 #define E1000_LOCK(sc)      mtx_lock(&(sc)->sc_mtx)
 #define E1000_UNLOCK(sc)    mtx_unlock(&(sc)->sc_mtx)
 #define E1000_LOCK_DESTROY(sc) mtx_destroy(&(sc)->sc_mtx)
 
 
 // Table of PCI device IDs supported by this driver (from original e1000.c)
 static const struct {
     uint16_t vendor;
     uint16_t device;
     const char *desc;
 } e1000_fbsd_devs[] = {
     { 0x8086, 0x100f, "Intel PRO/1000 MT Desktop Adapter (82540EM)" },
     // Can add more supported Intel e1000 variants here
     { 0, 0, NULL }
 };
 
 // DMA map load callback (generic)
 static void
 e1000_dma_map_cb(void *arg, bus_dma_segment_t *segs, int nseg, int error)
 {
     if (error) {
         *(bus_addr_t *)arg = 0; // Indicate error
         return;
     }
     KASSERT(nseg == 1, ("e1000_dma_map_cb: too many segments %d", nseg));
     *(bus_addr_t *)arg = segs[0].ds_addr;
 }
 
 static int
 e1000_fbsd_probe(device_t dev)
 {
     uint16_t vendor = pci_get_vendor(dev);
     uint16_t device = pci_get_device(dev);
     int i;
 
     for (i = 0; e1000_fbsd_devs[i].desc != NULL; i++) {
         if (vendor == e1000_fbsd_devs[i].vendor &&
             device == e1000_fbsd_devs[i].device) {
             device_set_desc(dev, e1000_fbsd_devs[i].desc);
             return BUS_PROBE_DEFAULT;
         }
     }
     return ENXIO;
 }
 
 static void e1000_fbsd_reset_hw(struct e1000_softc *sc)
 {
     E1000_WRITE_REG(sc, E1000_REG_IMC, 0xFFFFFFFF); // Disable all interrupts
     E1000_WRITE_REG(sc, E1000_REG_CTRL, E1000_READ_REG(sc, E1000_REG_CTRL) | E1000_CTRL_RST);
     E1000_WRITE_FLUSH(sc);
     DELAY(10000); // Wait 10ms for reset to complete
 }
 
 // Simplified MAC address reading. Real e1000 drivers have complex EEPROM routines.
 static void e1000_fbsd_read_mac_addr(struct e1000_softc *sc)
 {
     uint32_t ral, rah;
     // This assumes the MAC is in the Receive Address Register 0 (RAL[0]/RAH[0])
     // after reset if EEPROM is valid. More robust reading is needed for production.
     ral = E1000_READ_REG(sc, E1000_REG_RAL_BASE); // RAL[0]
     rah = E1000_READ_REG(sc, E1000_REG_RAH_BASE); // RAH[0]
 
     sc->mac_addr[0] = (uint8_t)(ral & 0xFF);
     sc->mac_addr[1] = (uint8_t)((ral >> 8) & 0xFF);
     sc->mac_addr[2] = (uint8_t)((ral >> 16) & 0xFF);
     sc->mac_addr[3] = (uint8_t)((ral >> 24) & 0xFF);
     sc->mac_addr[4] = (uint8_t)(rah & 0xFF);
     sc->mac_addr[5] = (uint8_t)((rah >> 8) & 0xFF);
 
     // Check for invalid MAC address (all 0s or all Fs)
     if ((sc->mac_addr[0] == 0 && sc->mac_addr[1] == 0 && sc->mac_addr[2] == 0 &&
          sc->mac_addr[3] == 0 && sc->mac_addr[4] == 0 && sc->mac_addr[5] == 0) ||
         (sc->mac_addr[0] == 0xFF && sc->mac_addr[1] == 0xFF && sc->mac_addr[2] == 0xFF &&
          sc->mac_addr[3] == 0xFF && sc->mac_addr[4] == 0xFF && sc->mac_addr[5] == 0xFF)) {
         device_printf(sc->dev, "Invalid MAC address found, using default.\n");
         // Use a default or locally administered MAC if necessary
         sc->mac_addr[0] = 0x00; sc->mac_addr[1] = 0xAA; sc->mac_addr[2] = 0xBB;
         sc->mac_addr[3] = 0xCC; sc->mac_addr[4] = 0xDD; sc->mac_addr[5] = 0xEE;
     }
 }
 
 static int e1000_alloc_dma_resources(struct e1000_softc *sc)
 {
     int error, i;
 
     // Parent DMA tag
     error = bus_dma_tag_create(bus_get_dma_tag(sc->dev),
                                1, 0, BUS_SPACE_MAXADDR, BUS_SPACE_MAXADDR,
                                NULL, NULL, BUS_SPACE_MAXSIZE_32BIT, 0,
                                BUS_SPACE_MAXSIZE_32BIT, 0, NULL, NULL,
                                &sc->tx_buffer_dtag); // Use one tag for all buffers for simplicity
     if (error) {
         device_printf(sc->dev, "Failed to create parent DMA tag\n");
         return error;
     }
     sc->rx_buffer_dtag = sc->tx_buffer_dtag; // Share for simplicity
 
     // Descriptor ring DMA tags and memory
     error = bus_dma_tag_create(bus_get_dma_tag(sc->dev), E1000_RING_ALIGN, 0,
                                BUS_SPACE_MAXADDR, BUS_SPACE_MAXADDR, NULL, NULL,
                                E1000_NUM_TX_DESC * sizeof(struct e1000_tx_desc), 1,
                                E1000_NUM_TX_DESC * sizeof(struct e1000_tx_desc),
                                0, NULL, NULL, &sc->tx_desc_dtag);
     if (error) goto fail_tags;
     error = bus_dmamem_alloc(sc->tx_desc_dtag, (void **)&sc->tx_desc_ring,
                              BUS_DMA_NOWAIT | BUS_DMA_ZERO | BUS_DMA_COHERENT, &sc->tx_desc_dmamap);
     if (error) goto fail_tags;
     error = bus_dmamap_load(sc->tx_desc_dtag, sc->tx_desc_dmamap, sc->tx_desc_ring,
                             E1000_NUM_TX_DESC * sizeof(struct e1000_tx_desc),
                             e1000_dma_map_cb, &sc->tx_desc_ring_paddr, BUS_DMA_NOWAIT);
     if (error || sc->tx_desc_ring_paddr == 0) goto fail_tags;
 
 
     error = bus_dma_tag_create(bus_get_dma_tag(sc->dev), E1000_RING_ALIGN, 0,
                                BUS_SPACE_MAXADDR, BUS_SPACE_MAXADDR, NULL, NULL,
                                E1000_NUM_RX_DESC * sizeof(struct e1000_rx_desc), 1,
                                E1000_NUM_RX_DESC * sizeof(struct e1000_rx_desc),
                                0, NULL, NULL, &sc->rx_desc_dtag);
     if (error) goto fail_tags;
     error = bus_dmamem_alloc(sc->rx_desc_dtag, (void **)&sc->rx_desc_ring,
                              BUS_DMA_NOWAIT | BUS_DMA_ZERO | BUS_DMA_COHERENT, &sc->rx_desc_dmamap);
     if (error) goto fail_tags;
     error = bus_dmamap_load(sc->rx_desc_dtag, sc->rx_desc_dmamap, sc->rx_desc_ring,
                             E1000_NUM_RX_DESC * sizeof(struct e1000_rx_desc),
                             e1000_dma_map_cb, &sc->rx_desc_ring_paddr, BUS_DMA_NOWAIT);
     if (error || sc->rx_desc_ring_paddr == 0) goto fail_tags;
 
     // Allocate RX buffers and dmamaps
     sc->rx_buffers = malloc(sizeof(void *) * E1000_NUM_RX_DESC, M_DEVBUF, M_WAITOK | M_ZERO);
     sc->rx_buffer_dmamaps = malloc(sizeof(bus_dmamap_t) * E1000_NUM_RX_DESC, M_DEVBUF, M_WAITOK | M_ZERO);
     for (i = 0; i < E1000_NUM_RX_DESC; i++) {
         error = bus_dmamem_alloc(sc->rx_buffer_dtag, &sc->rx_buffers[i],
                                  BUS_DMA_NOWAIT | BUS_DMA_ZERO | BUS_DMA_COHERENT,
                                  &sc->rx_buffer_dmamaps[i]); // One map per buffer
         if (error) {
             device_printf(sc->dev, "Failed to alloc RX buffer %d\n", i);
             goto fail_rx_bufs;
         }
         // Maps will be loaded when filling descriptors
     }
     return 0;
 
 fail_rx_bufs:
     // Free already allocated RX buffers
     for (int j = 0; j < i; j++) {
         bus_dmamap_destroy(sc->rx_buffer_dtag, sc->rx_buffer_dmamaps[j]);
         bus_dmamem_free(sc->rx_buffer_dtag, sc->rx_buffers[j], sc->rx_buffer_dmamaps[j]);
     }
     free(sc->rx_buffers, M_DEVBUF);
     free(sc->rx_buffer_dmamaps, M_DEVBUF);
 fail_tags:
     // Free other DMA resources if allocated
     if (sc->rx_desc_ring_paddr) bus_dmamap_unload(sc->rx_desc_dtag, sc->rx_desc_dmamap);
     if (sc->rx_desc_ring) bus_dmamem_free(sc->rx_desc_dtag, sc->rx_desc_ring, sc->rx_desc_dmamap);
     if (sc->rx_desc_dtag) bus_dma_tag_destroy(sc->rx_desc_dtag);
     if (sc->tx_desc_ring_paddr) bus_dmamap_unload(sc->tx_desc_dtag, sc->tx_desc_dmamap);
     if (sc->tx_desc_ring) bus_dmamem_free(sc->tx_desc_dtag, sc->tx_desc_ring, sc->tx_desc_dmamap);
     if (sc->tx_desc_dtag) bus_dma_tag_destroy(sc->tx_desc_dtag);
     if (sc->tx_buffer_dtag) bus_dma_tag_destroy(sc->tx_buffer_dtag); // Also rx_buffer_dtag
     return error;
 }
 
 static void e1000_free_dma_resources(struct e1000_softc *sc)
 {
     int i;
     // Free RX buffers
     if (sc->rx_buffers) {
         for (i = 0; i < E1000_NUM_RX_DESC; i++) {
             if (sc->rx_buffers[i]) {
                 bus_dmamap_unload(sc->rx_buffer_dtag, sc->rx_buffer_dmamaps[i]);
                 bus_dmamem_free(sc->rx_buffer_dtag, sc->rx_buffers[i], sc->rx_buffer_dmamaps[i]);
             }
             if (sc->rx_buffer_dmamaps[i]) {
                  bus_dmamap_destroy(sc->rx_buffer_dtag, sc->rx_buffer_dmamaps[i]);
             }
         }
         free(sc->rx_buffers, M_DEVBUF);
         free(sc->rx_buffer_dmamaps, M_DEVBUF);
     }
 
     // Free descriptor rings
     if (sc->rx_desc_ring_paddr) bus_dmamap_unload(sc->rx_desc_dtag, sc->rx_desc_dmamap);
     if (sc->rx_desc_ring) bus_dmamem_free(sc->rx_desc_dtag, sc->rx_desc_ring, sc->rx_desc_dmamap);
     if (sc->rx_desc_dtag) bus_dma_tag_destroy(sc->rx_desc_dtag);
 
     if (sc->tx_desc_ring_paddr) bus_dmamap_unload(sc->tx_desc_dtag, sc->tx_desc_dmamap);
     if (sc->tx_desc_ring) bus_dmamem_free(sc->tx_desc_dtag, sc->tx_desc_ring, sc->tx_desc_dmamap);
     if (sc->tx_desc_dtag) bus_dma_tag_destroy(sc->tx_desc_dtag);
 
     if (sc->tx_buffer_dtag) bus_dma_tag_destroy(sc->tx_buffer_dtag); // Also rx_buffer_dtag
 }
 
 
 static int e1000_init_rx_ring(struct e1000_softc *sc)
 {
     int i, error;
     bus_addr_t paddr;
 
     for (i = 0; i < E1000_NUM_RX_DESC; i++) {
         error = bus_dmamap_load(sc->rx_buffer_dtag, sc->rx_buffer_dmamaps[i],
                                 sc->rx_buffers[i], E1000_RX_BUFFER_SIZE,
                                 e1000_dma_map_cb, &paddr, BUS_DMA_NOWAIT);
         if (error || paddr == 0) {
             device_printf(sc->dev, "Failed to DMA map RX buffer %d\n", i);
             return error ? error : EFAULT;
         }
         sc->rx_desc_ring[i].buffer_addr = htole64(paddr);
         sc->rx_desc_ring[i].status = 0;
         sc->rx_desc_ring[i].length = 0;
     }
 
     E1000_WRITE_REG(sc, E1000_REG_RDBAL, (uint32_t)sc->rx_desc_ring_paddr);
     E1000_WRITE_REG(sc, E1000_REG_RDBAH, (uint32_t)((uint64_t)sc->rx_desc_ring_paddr >> 32));
     E1000_WRITE_REG(sc, E1000_REG_RDLEN, E1000_NUM_RX_DESC * sizeof(struct e1000_rx_desc));
     E1000_WRITE_REG(sc, E1000_REG_RDH, 0);
     E1000_WRITE_REG(sc, E1000_REG_RDT, E1000_NUM_RX_DESC - 1);
     sc->rx_head = 0;
     return 0;
 }
 
 static void e1000_init_tx_ring(struct e1000_softc *sc)
 {
     // TX buffers are not pre-allocated here, but managed on send.
     // Initialize TX descriptor ring pointers.
     E1000_WRITE_REG(sc, E1000_REG_TDBAL, (uint32_t)sc->tx_desc_ring_paddr);
     E1000_WRITE_REG(sc, E1000_REG_TDBAH, (uint32_t)((uint64_t)sc->tx_desc_ring_paddr >> 32));
     E1000_WRITE_REG(sc, E1000_REG_TDLEN, E1000_NUM_TX_DESC * sizeof(struct e1000_tx_desc));
     E1000_WRITE_REG(sc, E1000_REG_TDH, 0);
     E1000_WRITE_REG(sc, E1000_REG_TDT, 0);
     sc->tx_head = 0;
     sc->tx_tail = 0;
     sc->tx_descriptors_free = E1000_NUM_TX_DESC;
 }
 
 
 static int
 e1000_fbsd_attach(device_t dev)
 {
     struct e1000_softc *sc = device_get_softc(dev);
     int error = 0;
     uint32_t reg_val;
 
     sc->dev = dev;
     E1000_LOCK_INIT(sc);
 
     // Enable PCI bus mastering
     pci_enable_busmaster(dev);
 
     // Allocate memory-mapped I/O BAR
     sc->mem_rid = PCIR_BAR(0); // Assuming MMIO is at BAR 0
     sc->mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &sc->mem_rid, RF_ACTIVE);
     if (sc->mem_res == NULL) {
         device_printf(dev, "could not allocate MMIO resource\n");
         error = ENXIO;
         goto fail_lock;
     }
     sc->mem_bst = rman_get_bustag(sc->mem_res);
     sc->mem_bsh = rman_get_bushandle(sc->mem_res);
 
     // Allocate IRQ
     sc->irq_rid = 0; // Try for legacy IRQ or first MSI vector if available
     // A real driver would check for MSI/MSI-X support here
     // E.g., if (pci_alloc_msi(dev, &one_vector) == 0) sc->irq_rid = 1;
     sc->irq_res = bus_alloc_resource_any(dev, SYS_RES_IRQ, &sc->irq_rid, RF_SHAREABLE | RF_ACTIVE);
     if (sc->irq_res == NULL) {
         device_printf(dev, "could not allocate IRQ resource\n");
         error = ENXIO;
         goto fail_mem_res;
     }
 
     // Reset hardware
     e1000_fbsd_reset_hw(sc);
 
     // Read MAC address
     e1000_fbsd_read_mac_addr(sc);
     device_printf(dev, "MAC Address: %02x:%02x:%02x:%02x:%02x:%02x\n",
                   sc->mac_addr[0], sc->mac_addr[1], sc->mac_addr[2],
                   sc->mac_addr[3], sc->mac_addr[4], sc->mac_addr[5]);
 
     // Allocate DMA resources (rings, buffers)
     error = e1000_alloc_dma_resources(sc);
     if (error) {
         device_printf(dev, "Failed to allocate DMA resources\n");
         goto fail_irq_res;
     }
 
     // Initialize transmit ring
     e1000_init_tx_ring(sc);
 
     // Initialize receive ring and populate with buffers
     error = e1000_init_rx_ring(sc);
     if (error) {
         device_printf(dev, "Failed to initialize RX ring\n");
         goto fail_dma_res;
     }
 
     // Configure Transmit Control Register (TCTL)
     reg_val = E1000_READ_REG(sc, E1000_REG_TCTL);
     reg_val |= E1000_TCTL_EN | E1000_TCTL_PSP;
     reg_val = (reg_val & ~(0xFF << E1000_TCTL_CT_SHIFT)) | (0x10 << E1000_TCTL_CT_SHIFT); // CT = 0x10
     reg_val = (reg_val & ~(0x3FF << E1000_TCTL_COLD_SHIFT)) | (0x40 << E1000_TCTL_COLD_SHIFT); // COLD = 0x40
     E1000_WRITE_REG(sc, E1000_REG_TCTL, reg_val);
 
     // Configure Receive Control Register (RCTL)
     E1000_WRITE_REG(sc, E1000_REG_RCTL, 0); // Disable receiver before configuring
     E1000_WRITE_FLUSH(sc);
     DELAY(1000);
     reg_val = E1000_RCTL_EN | E1000_RCTL_BAM | E1000_RCTL_SZ_2048 | E1000_RCTL_SECRC;
     // reg_val |= E1000_RCTL_UPE; // Optional: promiscuous mode
     E1000_WRITE_REG(sc, E1000_REG_RCTL, reg_val);
 
 
     // Setup interrupt handler
     error = bus_setup_intr(dev, sc->irq_res, INTR_TYPE_NET | INTR_MPSAFE,
                            NULL, e1000_fbsd_intr, sc, &sc->intr_cookie);
     if (error) {
         device_printf(dev, "could not setup IRQ handler\n");
         goto fail_dma_res;
     }
 
     // Enable interrupts at the hardware level
     E1000_WRITE_REG(sc, E1000_REG_IMS, E1000_ICR_RXT0 | E1000_ICR_RXDMT0 | E1000_ICR_TXDW | E1000_ICR_LSC);
     E1000_WRITE_FLUSH(sc);
 
     device_printf(dev, "e1000 driver attached successfully.\n");
     return 0;
 
 fail_dma_res:
     e1000_free_dma_resources(sc);
 fail_irq_res:
     bus_release_resource(dev, SYS_RES_IRQ, sc->irq_rid, sc->irq_res);
 fail_mem_res:
     bus_release_resource(dev, SYS_RES_MEMORY, sc->mem_rid, sc->mem_res);
 fail_lock:
     E1000_LOCK_DESTROY(sc);
     pci_disable_busmaster(dev);
     return error;
 }
 
 static int
 e1000_fbsd_detach(device_t dev)
 {
     struct e1000_softc *sc = device_get_softc(dev);
 
     device_printf(dev, "Detaching e1000\n");
     E1000_LOCK(sc);
 
     // Disable interrupts at hardware
     E1000_WRITE_REG(sc, E1000_REG_IMC, 0xFFFFFFFF);
     // Disable receiver and transmitter
     E1000_WRITE_REG(sc, E1000_REG_RCTL, E1000_READ_REG(sc, E1000_REG_RCTL) & ~E1000_RCTL_EN);
     E1000_WRITE_REG(sc, E1000_REG_TCTL, E1000_READ_REG(sc, E1000_REG_TCTL) & ~E1000_TCTL_EN);
     E1000_WRITE_FLUSH(sc);
     DELAY(10000);
 
     E1000_UNLOCK(sc);
 
     // Teardown interrupt handler
     if (sc->intr_cookie) {
         bus_teardown_intr(dev, sc->irq_res, sc->intr_cookie);
     }
 
     // Free DMA and other resources
     e1000_free_dma_resources(sc);
     bus_release_resource(dev, SYS_RES_IRQ, sc->irq_rid, sc->irq_res);
     bus_release_resource(dev, SYS_RES_MEMORY, sc->mem_rid, sc->mem_res);
     pci_disable_busmaster(dev);
     E1000_LOCK_DESTROY(sc);
 
     return 0;
 }
 
 static void e1000_handle_rx(struct e1000_softc *sc)
 {
     int i = sc->rx_head;
     int processed_count = 0;
 
     bus_dmamap_sync(sc->rx_desc_dtag, sc->rx_desc_dmamap, BUS_DMASYNC_POSTREAD | BUS_DMASYNC_POSTWRITE);
 
     while (sc->rx_desc_ring[i].status & E1000_RXD_STAT_DD) {
         struct e1000_rx_desc *desc = &sc->rx_desc_ring[i];
         //void *pkt_data = sc->rx_buffers[i];
         //uint16_t pkt_len = le16toh(desc->length);
 
         // Sync the specific buffer before CPU access
         bus_dmamap_sync(sc->rx_buffer_dtag, sc->rx_buffer_dmamaps[i], BUS_DMASYNC_POSTREAD);
 
         // device_printf(sc->dev, "RX: slot %d, len %u, status 0x%02x\n", i, pkt_len, desc->status);
         // TODO: Process the received packet (pkt_data, pkt_len)
         // This is where you'd copy it to a user-space queue if using a char device,
         // or hand it to a custom raw packet processing function.
 
         // Unmap (not strictly necessary if buffer is reused) and reload for reuse
         // bus_dmamap_unload(sc->rx_buffer_dtag, sc->rx_buffer_dmamaps[i]); // Optional
 
         bus_addr_t paddr;
         int error = bus_dmamap_load(sc->rx_buffer_dtag, sc->rx_buffer_dmamaps[i],
                                     sc->rx_buffers[i], E1000_RX_BUFFER_SIZE,
                                     e1000_dma_map_cb, &paddr, BUS_DMA_NOWAIT);
         if (error || paddr == 0) {
             device_printf(sc->dev, "Failed to remap RX buffer %d in ISR!\n", i);
             // Critical error, perhaps stop RX
             break;
         }
         desc->buffer_addr = htole64(paddr);
         desc->status = 0; // Clear DD bit
         desc->length = 0;
 
         // Sync descriptor writeback
         bus_dmamap_sync(sc->rx_desc_dtag, sc->rx_desc_dmamap, BUS_DMASYNC_PREREAD | BUS_DMASYNC_PREWRITE);
 
         // Update tail pointer for HW
         E1000_WRITE_REG(sc, E1000_REG_RDT, i);
         processed_count++;
         i = (i + 1) % E1000_NUM_RX_DESC;
     }
     sc->rx_head = i;
 
     if (processed_count > 0) {
          bus_dmamap_sync(sc->rx_desc_dtag, sc->rx_desc_dmamap, BUS_DMASYNC_PREREAD | BUS_DMASYNC_PREWRITE);
     }
 }
 
 static void e1000_handle_tx_completions(struct e1000_softc *sc) {
     int i = sc->tx_head;
     int cleaned_count = 0;
 
     bus_dmamap_sync(sc->tx_desc_dtag, sc->tx_desc_dmamap, BUS_DMASYNC_POSTREAD | BUS_DMASYNC_POSTWRITE);
 
     while (i != sc->tx_tail) {
         if (!(sc->tx_desc_ring[i].upper.fields.status & E1000_TXD_CMD_RS)) { // DD bit in e1000
             break; // Descriptor not done
         }
         // device_printf(sc->dev, "TX completion: slot %d\n", i);
 
         // TODO: Free the corresponding TX buffer and unmap it.
         // This requires tracking which buffer was associated with this descriptor.
         // For now, just advance the head and increment free count.
         // bus_dmamap_unload(sc->tx_buffer_dtag, sc->tx_buffer_dmamaps[i]);
 
         sc->tx_desc_ring[i].upper.fields.status = 0; // Clear status
         sc->tx_descriptors_free++;
         cleaned_count++;
         i = (i + 1) % E1000_NUM_TX_DESC;
     }
     sc->tx_head = i;
     if (cleaned_count > 0) {
         bus_dmamap_sync(sc->tx_desc_dtag, sc->tx_desc_dmamap, BUS_DMASYNC_PREREAD | BUS_DMASYNC_PREWRITE);
     }
 }
 
 
 static void
 e1000_fbsd_intr(void *arg)
 {
     struct e1000_softc *sc = (struct e1000_softc *)arg;
     uint32_t icr;
 
     E1000_LOCK(sc);
 
     icr = E1000_READ_REG(sc, E1000_REG_ICR);
     if (icr == 0) { // Not our interrupt or shared IRQ with no cause bits set
         E1000_UNLOCK(sc);
         return;
     }
     // Acknowledge (clear) interrupts by writing to ICR, or some NICs clear on read.
     // For e1000, ICR bits are cleared by reading or by writing 1 to them if ICS/IMC used for masking.
     // For simplicity, we just read it. If issues, may need explicit E1000_WRITE_REG(sc, E1000_REG_ICR, icr);
     // Or better, disable interrupts via IMC during handler and re-enable specific ones via IMS.
     E1000_WRITE_REG(sc, E1000_REG_IMC, 0xFFFFFFFF); // Mask all interrupts
 
     if (icr & (E1000_ICR_RXT0 | E1000_ICR_RXDMT0)) {
         e1000_handle_rx(sc);
     }
     if (icr & E1000_ICR_TXDW) {
         e1000_handle_tx_completions(sc);
     }
     if (icr & E1000_ICR_LSC) {
         device_printf(sc->dev, "Link status change detected.\n");
         // TODO: Implement link status check and handling (e.g., e1000_config_link)
     }
 
     // Re-enable interrupts
     E1000_WRITE_REG(sc, E1000_REG_IMS, E1000_ICR_RXT0 | E1000_ICR_RXDMT0 | E1000_ICR_TXDW | E1000_ICR_LSC);
     E1000_WRITE_FLUSH(sc);
     E1000_UNLOCK(sc);
 }
 
 // Placeholder for a raw packet transmit function
 // This would be called, for example, by a char device write operation.
 static int e1000_fbsd_raw_transmit(struct e1000_softc *sc, void *data, int len)
 {
     // This function needs proper buffer management:
     // 1. Get a free TX descriptor and its associated DMA-able buffer.
     // 2. Copy `data` into the DMA-able buffer.
     // 3. Load/sync the DMA map for the buffer.
     // 4. Fill the TX descriptor.
     // 5. Update TX tail register.
     // This is a highly complex part if not using mbufs.
     // For now, it's just a conceptual placeholder.
     device_printf(sc->dev, "e1000_fbsd_raw_transmit: Not fully implemented.\n");
     return ENOSYS;
 }
 
 
 // Define device methods table
 static device_method_t e1000_fbsd_methods[] = {
     /* Device interface */
     DEVMETHOD(device_probe,     e1000_fbsd_probe),
     DEVMETHOD(device_attach,    e1000_fbsd_attach),
     DEVMETHOD(device_detach,    e1000_fbsd_detach),
     // DEVMETHOD(device_shutdown,  e1000_fbsd_shutdown), // Optional
     DEVMETHOD_END
 };
 
 // Define driver structure
 static driver_t e1000_fbsd_driver = {
     "e1000_fbsd", // Driver name
     e1000_fbsd_methods,
     sizeof(struct e1000_softc)
 };
 
 // Module registration
 DRIVER_MODULE(e1000_fbsd, pci, e1000_fbsd_driver, 0, 0);
 MODULE_DEPEND(e1000_fbsd, pci, 1, 1, 1);
 MODULE_VERSION(e1000_fbsd, 1);