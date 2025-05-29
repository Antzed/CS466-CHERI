# Path to your FreeBSD source tree (where you see sys/, include/, etc.)
.PATH: ${SRCTOP}/sys/dev/e1000pol

# Name of the module (this will produce if_e1000pol.ko)
KMOD   = if_empol

# Your driver source file(s)
SRCS   = e1000pol.c

# If you need additional include paths:
CFLAGS += -I$(KERN_SRC)/sys
.include <bsd.kmod.mk>
