.include <src.opts.mk>
.include "${SRCTOP}/secure/ssh.mk"

CONFS=	moduli sshd_config
CONFSDIR=	/etc/ssh
PROG=	sshd
SRCS=	sshd.c \
	platform-listen.c \
	servconf.c sshpty.c srclimit.c groupaccess.c auth2-methods.c
PACKAGE=	ssh

MAN=	sshd.8 sshd_config.5

# Don't rebuild based on moduli.c
moduli: .MADE

LIBADD=	ssh util

.if ${MK_TCP_WRAPPERS} != "no"
CFLAGS+=	-DLIBWRAP=1
LIBADD+=	wrap
.endif

LIBADD+=	crypto

.include <bsd.prog.mk>

.PATH:	${SSHDIR}
