# Path to your FreeBSD source tree (where you see sys/, include/, etc.)
.PATH: ${SRCTOP}/sys/dev/uart_cheri

# Name of the module (this will produce if_e1000pol.ko)
KMOD   = uart_cheri
# Your driver source file(s)
SRCS   = uart.c

# If you need additional include paths:
CFLAGS += -I$(KERN_SRC)/sys

.include <bsd.kmod.mk>