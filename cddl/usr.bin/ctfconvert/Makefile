.include <src.opts.mk>

.PATH: ${SRCTOP}/cddl/contrib/opensolaris/tools/ctf/common
.PATH: ${SRCTOP}/cddl/contrib/opensolaris/tools/ctf/cvt

PACKAGE=	ctf-tools
PROG=		ctfconvert
SRCS=		alist.c \
		ctf.c \
		ctfconvert.c \
		dwarf.c \
		fixup_tdescs.c \
		hash.c \
		iidesc.c \
		input.c \
		list.c \
		memory.c \
		merge.c \
		output.c \
		stack.c \
		strtab.c \
		symbol.c \
		tdata.c \
		traverse.c \
		util.c

CFLAGS+= -DIN_BASE
CFLAGS+= -I${ZFSTOP}/include
CFLAGS+= -I${ZFSTOP}/lib/libspl/include/
CFLAGS+= -I${ZFSTOP}/lib/libspl/include/os/freebsd
CFLAGS+= -I${SRCTOP}/cddl/compat/opensolaris/include
CFLAGS+=	-I${SRCTOP}/sys/cddl/compat/opensolaris \
		-I${SRCTOP}/cddl/compat/opensolaris/include \
		-I${OPENSOLARIS_USR_DISTDIR} \
		-I${OPENSOLARIS_SYS_DISTDIR} \
		-I${OPENSOLARIS_USR_DISTDIR}/head \
		-I${OPENSOLARIS_USR_DISTDIR}/tools/ctf/common \
		-I${OPENSOLARIS_USR_DISTDIR}/tools/ctf/cvt \
		-I${OPENSOLARIS_SYS_DISTDIR}/uts/common
CFLAGS+= -DHAVE_ISSETUGID 

LIBADD=		spl dwarf elf z pthread

HAS_TESTS=
SUBDIR.${MK_TESTS}+= tests

.include <bsd.prog.mk>
