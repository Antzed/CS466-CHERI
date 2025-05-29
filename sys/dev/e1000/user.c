/* userspace_em_driver.c */
#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>

/* CHERI-specific includes */
#include <cheri/cheric.h>

/* Define ioctl commands to match the kernel driver */
#define EM_CHERI_IOC_GET_REGS_CAP    _IOR('e', 1, void * __capability)
#define EM_CHERI_IOC_GET_TX_DMA_CAP  _IOR('e', 2, void * __capability)
#define EM_CHERI_IOC_GET_RX_DMA_CAP  _IOR('e', 3, void * __capability)
#define EM_CHERI_IOC_DISABLE_KERNEL  _IO('e', 6)

/* E1000 register definitions */
#define E1000_CTRL     0x00000  /* Device Control Register */
#define E1000_STATUS   0x00008  /* Device Status Register */
#define E1000_CTRL_RST 0x04000000  /* Device Reset */
/* Add more register definitions as needed */

struct userspace_em_driver {
    int fd;  /* File descriptor for the device */
    
    /* Capabilities to device resources */
    volatile uint32_t * __capability regs;           /* Register space */
    void * __capability tx_dma;                      /* TX DMA buffers */
    void * __capability rx_dma;                      /* RX DMA buffers */
    
    /* Driver state */
    bool initialized;
};

/* Helper for register access */
static inline uint32_t
em_read_reg(volatile uint32_t * __capability regs, int reg)
{
    return *(volatile uint32_t * __capability)((char * __capability)regs + reg);
}

static inline void
em_write_reg(volatile uint32_t * __capability regs, int reg, uint32_t val)
{
    *(volatile uint32_t * __capability)((char * __capability)regs + reg) = val;
}

/* Initialize the userspace driver */
int em_user_init(struct userspace_em_driver *drv, const char *device_path)
{
    /* Open the device */
    drv->fd = open(device_path, O_RDWR);
    if (drv->fd < 0) {
        perror("Failed to open E1000 device");
        return -1;
    }
    
    /* Get the register capability */
    if (ioctl(drv->fd, EM_CHERI_IOC_GET_REGS_CAP, &drv->regs) < 0) {
        perror("Failed to get register capability");
        close(drv->fd);
        return -1;
    }
    
    /* Get the TX DMA buffer capability for queue 0 */
    int tx_index = 0;
    if (ioctl(drv->fd, EM_CHERI_IOC_GET_TX_DMA_CAP, &tx_index) < 0) {
        perror("Failed to get TX DMA buffer capability");
        close(drv->fd);
        return -1;
    }
    
    /* Get the RX DMA buffer capability for queue 0 */
    int rx_index = 0;
    if (ioctl(drv->fd, EM_CHERI_IOC_GET_RX_DMA_CAP, &rx_index) < 0) {
        perror("Failed to get RX DMA buffer capability");
        close(drv->fd);
        return -1;
    }
    
    /* Disable the kernel driver to take full control */
    if (ioctl(drv->fd, EM_CHERI_IOC_DISABLE_KERNEL) < 0) {
        perror("Failed to disable kernel driver");
        close(drv->fd);
        return -1;
    }
    
    /* Initialize the hardware */
    /* Reset the device */
    uint32_t ctrl = em_read_reg(drv->regs, E1000_CTRL);
    em_write_reg(drv->regs, E1000_CTRL, ctrl | E1000_CTRL_RST);
    
    /* Wait for reset to complete */
    usleep(10000);
    
    /* Configure the device for operation */
    /* ... */
    
    drv->initialized = true;
    return 0;
}

/* Close and clean up the userspace driver */
void em_user_cleanup(struct userspace_em_driver *drv)
{
    if (drv->fd >= 0) {
        /* Re-enable the kernel driver before closing */
        ioctl(drv->fd, EM_CHERI_IOC_ENABLE_KERNEL);
        close(drv->fd);
        drv->fd = -1;
    }
    
    /* Clear all capabilities */
    drv->regs = NULL;
    drv->tx_dma = NULL;
    drv->rx_dma = NULL;
    drv->initialized = false;
}

/* Example main function */
int main(int argc, char *argv[])
{
    struct userspace_em_driver drv = {0};
    
    if (em_user_init(&drv, "/dev/em_cheri0") < 0) {
        fprintf(stderr, "Failed to initialize E1000 userspace driver\n");
        return 1;
    }
    
    printf("E1000 device initialized successfully\n");
    printf("Device status: 0x%08x\n", em_read_reg(drv.regs, E1000_STATUS));
    
    /* Example: main driver processing loop */
    while (1) {
        /* Process any received packets */
        /* Check for packets to transmit */
        /* ... */
        
        /* Break condition - would be application-specific */
        break;
    }
    
    em_user_cleanup(&drv);
    return 0;
}
