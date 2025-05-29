/*
 * Modified if_em.c for CHERI capability-based userspace access
 */

 #include <sys/cdefs.h>
 #include <sys/param.h>
 #include <sys/systm.h>
 #include <sys/kernel.h>
 #include <sys/module.h>
 #include <sys/bus.h>
 #include <sys/rman.h>
 #include <sys/socket.h>
 #include <sys/sysctl.h>
 #include <sys/sx.h>
 #include <sys/taskqueue.h>
 
 /* CHERI-specific includes */
 #include <sys/capability.h>
 #include <sys/cheric.h>
 #include <cheri/cheric.h>
 
 #include <machine/bus.h>
 #include <dev/pci/pcireg.h>
 #include <dev/pci/pcivar.h>
 
 /* Include existing driver header */
 #include "if_em.h"
 
 /* 
  * Add CHERI capability fields to the driver softc structure 
  * Only showing the additional fields, not the complete structure
  */
 struct e1000_cheri_extension {
     /* Character device for userspace access */
     struct cdev *cdev;
     
     /* Capability references for controlled delegation */
     void * __capability regs_cap;         /* Capability to device registers */
     void * __capability *dma_tx_caps;     /* Array of TX DMA buffer capabilities */
     void * __capability *dma_rx_caps;     /* Array of RX DMA buffer capabilities */
     
     /* Process allowed to access this device */
     pid_t authorized_pid;
     
     /* Original bus handles for capability creation */
     bus_space_tag_t    mem_bus_space_tag;
     bus_space_handle_t mem_bus_space_handle;
     
     /* Flags for userspace driver state */
     bool userspace_active;
     bool kernel_driver_disabled;
 };
 
 /* IOCTL commands for capability delegation */
 #define EM_CHERI_IOC_GET_REGS_CAP    _IOR('e', 1, void * __capability)
 #define EM_CHERI_IOC_GET_TX_DMA_CAP  _IOR('e', 2, void * __capability)
 #define EM_CHERI_IOC_GET_RX_DMA_CAP  _IOR('e', 3, void * __capability)
 #define EM_CHERI_IOC_SET_AUTHORIZED  _IOW('e', 4, pid_t)
 #define EM_CHERI_IOC_ENABLE_KERNEL   _IO('e', 5)
 #define EM_CHERI_IOC_DISABLE_KERNEL  _IO('e', 6)
 
 /* Function prototypes */
 static int em_cheri_open(struct cdev *dev, int flags, int fmt, struct thread *td);
 static int em_cheri_close(struct cdev *dev, int flags, int fmt, struct thread *td);
 static int em_cheri_ioctl(struct cdev *dev, u_long cmd, caddr_t data, int fflag, struct thread *td);
 
 /* Character device operations */
 static struct cdevsw em_cheri_cdevsw = {
     .d_version = D_VERSION,
     .d_open = em_cheri_open,
     .d_close = em_cheri_close,
     .d_ioctl = em_cheri_ioctl,
     .d_name = "em_cheri",
 };
 
 /*
  * Modified em_if_attach_pre function to create CHERI capabilities
  * Only showing the added code, not the complete function
  */
 static int
 em_if_attach_pre(if_ctx_t ctx)
 {
     struct e1000_softc *sc = iflib_get_softc(ctx);
     device_t dev = iflib_get_dev(ctx);
     struct e1000_hw *hw = &sc->hw;
     int error = 0;
     
     /* Original attach_pre code here ... */
     
     /* Allocate and initialize the CHERI extension structure */
     sc->cheri_ext = malloc(sizeof(struct e1000_cheri_extension), 
                            M_DEVBUF, M_WAITOK | M_ZERO);
     if (sc->cheri_ext == NULL) {
         device_printf(dev, "Unable to allocate CHERI extension\n");
         return (ENOMEM);
     }
     
     /* Store the bus space handles for capability creation */
     sc->cheri_ext->mem_bus_space_tag = sc->osdep.mem_bus_space_tag;
     sc->cheri_ext->mem_bus_space_handle = sc->osdep.mem_bus_space_handle;
     
     /* Create capability to device registers */
     sc->cheri_ext->regs_cap = cheri_capability_build_user_data(
         (void *)sc->osdep.mem_bus_space_handle,  /* Base address */
         rman_get_size(sc->memory),               /* Size of region */
         cheri_perm_load | cheri_perm_store       /* Permissions */
     );
     
     /* Create character device for userspace access */
     error = make_dev_p(MAKEDEV_CHECKNAME | MAKEDEV_WAITOK,
                       &sc->cheri_ext->cdev,
                       &em_cheri_cdevsw,
                       0,
                       UID_ROOT,
                       GID_WHEEL,
                       0600,
                       "em_cheri%d", device_get_unit(dev));
     if (error != 0) {
         device_printf(dev, "Failed to create character device: %d\n", error);
         free(sc->cheri_ext, M_DEVBUF);
         sc->cheri_ext = NULL;
         return (error);
     }
     
     /* Store the softc in the character device for reference */
     sc->cheri_ext->cdev->si_drv1 = sc;
     
     device_printf(dev, "CHERI capability support enabled\n");
     
     return (0);
 }
 
 /*
  * Modified em_if_attach_post function to create DMA capabilities
  * Only showing the added code, not the complete function
  */
 static int
 em_if_attach_post(if_ctx_t ctx)
 {
     struct e1000_softc *sc = iflib_get_softc(ctx);
     if_softc_ctx_t scctx = sc->shared;
     struct e1000_tx_queue *tx_que;
     struct e1000_rx_queue *rx_que;
     int i;
     
     /* Original attach_post code here ... */
     
     /* Create capabilities for TX DMA buffers */
     if (sc->cheri_ext != NULL) {
         /* Allocate arrays to hold capability pointers */
         sc->cheri_ext->dma_tx_caps = malloc(sizeof(void * __capability) * sc->tx_num_queues,
                                           M_DEVBUF, M_WAITOK | M_ZERO);
         sc->cheri_ext->dma_rx_caps = malloc(sizeof(void * __capability) * sc->rx_num_queues,
                                           M_DEVBUF, M_WAITOK | M_ZERO);
         
         /* Create capabilities for TX queues */
         for (i = 0, tx_que = sc->tx_queues; i < sc->tx_num_queues; i++, tx_que++) {
             struct tx_ring *txr = &tx_que->txr;
             
             sc->cheri_ext->dma_tx_caps[i] = cheri_capability_build_user_data(
                 txr->tx_base,                             /* Base address */
                 scctx->isc_ntxd[0] * sizeof(struct e1000_tx_desc), /* Size */
                 cheri_perm_load | cheri_perm_store        /* Permissions */
             );
         }
         
         /* Create capabilities for RX queues */
         for (i = 0, rx_que = sc->rx_queues; i < sc->rx_num_queues; i++, rx_que++) {
             struct rx_ring *rxr = &rx_que->rxr;
             
             sc->cheri_ext->dma_rx_caps[i] = cheri_capability_build_user_data(
                 rxr->rx_base,                             /* Base address */
                 scctx->isc_nrxd[0] * sizeof(union e1000_rx_desc_extended), /* Size */
                 cheri_perm_load | cheri_perm_store        /* Permissions */
             );
         }
     }
     
     return (0);
 }
 
 /*
  * Modified em_if_detach function to clean up CHERI resources
  * Only showing the added code, not the complete function
  */
 static int
 em_if_detach(if_ctx_t ctx)
 {
     struct e1000_softc *sc = iflib_get_softc(ctx);
     
     /* Clean up CHERI capability resources */
     if (sc->cheri_ext != NULL) {
         if (sc->cheri_ext->cdev != NULL) {
             destroy_dev(sc->cheri_ext->cdev);
             sc->cheri_ext->cdev = NULL;
         }
         
         /* Free capability arrays */
         if (sc->cheri_ext->dma_tx_caps != NULL) {
             free(sc->cheri_ext->dma_tx_caps, M_DEVBUF);
             sc->cheri_ext->dma_tx_caps = NULL;
         }
         
         if (sc->cheri_ext->dma_rx_caps != NULL) {
             free(sc->cheri_ext->dma_rx_caps, M_DEVBUF);
             sc->cheri_ext->dma_rx_caps = NULL;
         }
         
         free(sc->cheri_ext, M_DEVBUF);
         sc->cheri_ext = NULL;
     }
     
     /* Original detach code here ... */
     
     return (0);
 }
 
 /*
  * Character device open operation
  */
 static int
 em_cheri_open(struct cdev *dev, int flags, int fmt, struct thread *td)
 {
     struct e1000_softc *sc = dev->si_drv1;
     
     /* 
      * Only allow the authorized process to open the device, 
      * or allow the first opener to become the authorized process
      */
     if (sc->cheri_ext->authorized_pid != 0 && 
         sc->cheri_ext->authorized_pid != td->td_proc->p_pid) {
         return EPERM;
     }
     
     return 0;
 }
 
 /*
  * Character device close operation
  */
 static int
 em_cheri_close(struct cdev *dev, int flags, int fmt, struct thread *td)
 {
     struct e1000_softc *sc = dev->si_drv1;
     
     /* If the authorized process is closing, clear the authorization */
     if (sc->cheri_ext->authorized_pid == td->td_proc->p_pid) {
         /* If userspace driver was active, re-enable kernel driver */
         if (sc->cheri_ext->kernel_driver_disabled) {
             sc->cheri_ext->userspace_active = false;
             sc->cheri_ext->kernel_driver_disabled = false;
             /* Re-enable kernel driver here */
         }
     }
     
     return 0;
 }
 
 /*
  * Character device ioctl operations for capability delegation
  */
 static int
 em_cheri_ioctl(struct cdev *dev, u_long cmd, caddr_t data, int fflag, struct thread *td)
 {
     struct e1000_softc *sc = dev->si_drv1;
     struct e1000_cheri_extension *cheri_ext = sc->cheri_ext;
     int error = 0;
     
     /* Check if this is the authorized process or if no process is authorized yet */
     if (cheri_ext->authorized_pid != 0 && 
         cheri_ext->authorized_pid != td->td_proc->p_pid) {
         return EPERM;
     }
     
     switch (cmd) {
     case EM_CHERI_IOC_GET_REGS_CAP:
         /* Transfer register capability to userspace */
         *((void * __capability *)data) = cheri_ext->regs_cap;
         
         /* Record the authorized PID if not already set */
         if (cheri_ext->authorized_pid == 0)
             cheri_ext->authorized_pid = td->td_proc->p_pid;
         break;
         
     case EM_CHERI_IOC_GET_TX_DMA_CAP:
         /* Check if index is valid */
         int tx_index = *(int *)data;
         if (tx_index < 0 || tx_index >= sc->tx_num_queues) {
             return EINVAL;
         }
         
         /* Transfer TX DMA capability to userspace */
         *((void * __capability *)data) = cheri_ext->dma_tx_caps[tx_index];
         
         /* Record the authorized PID if not already set */
         if (cheri_ext->authorized_pid == 0)
             cheri_ext->authorized_pid = td->td_proc->p_pid;
         break;
         
     case EM_CHERI_IOC_GET_RX_DMA_CAP:
         /* Check if index is valid */
         int rx_index = *(int *)data;
         if (rx_index < 0 || rx_index >= sc->rx_num_queues) {
             return EINVAL;
         }
         
         /* Transfer RX DMA capability to userspace */
         *((void * __capability *)data) = cheri_ext->dma_rx_caps[rx_index];
         
         /* Record the authorized PID if not already set */
         if (cheri_ext->authorized_pid == 0)
             cheri_ext->authorized_pid = td->td_proc->p_pid;
         break;
         
     case EM_CHERI_IOC_SET_AUTHORIZED:
         /* Only allow root to change the authorized PID */
         if (priv_check(td, PRIV_DRIVER) != 0) {
             return EPERM;
         }
         
         /* Set the authorized PID */
         cheri_ext->authorized_pid = *(pid_t *)data;
         break;
         
     case EM_CHERI_IOC_DISABLE_KERNEL:
         /* Disable the kernel driver for userspace control */
         cheri_ext->userspace_active = true;
         cheri_ext->kernel_driver_disabled = true;
         
         /* 
          * Stop the kernel driver functionality here
          * This would typically involve:
          * 1. Disabling interrupts
          * 2. Stopping any running timers
          * 3. Marking the interface as down from the network stack perspective
          */
         em_if_stop(sc->ctx);
         break;
         
     case EM_CHERI_IOC_ENABLE_KERNEL:
         /* Re-enable the kernel driver */
         cheri_ext->userspace_active = false;
         cheri_ext->kernel_driver_disabled = false;
         
         /* 
          * Re-initialize the kernel driver here
          * This would typically involve calling the interface init function
          */
         em_if_init(sc->ctx);
         break;
         
     default:
         error = ENOTTY;
         break;
     }
     
     return error;
 }
 