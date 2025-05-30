/*-
 * Copyright (c) 1999 Luoqi Chen <luoqi@freebsd.org>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 *	from: FreeBSD: src/sys/i386/include/globaldata.h,v 1.27 2001/04/27
 */

#ifdef __arm__
#include <arm/pcpu.h>
#else /* !__arm__ */

#ifndef	_MACHINE_PCPU_H_
#define	_MACHINE_PCPU_H_

#include <machine/cpu.h>
#include <machine/cpufunc.h>

typedef int (*pcpu_bp_harden)(void);
typedef int (*pcpu_ssbd)(int);
struct debug_monitor_state;

#ifdef __CHERI_PURE_CAPABILITY__
#define	PCPU_MD_FIELDS_PAD 0
#else
#define	PCPU_MD_FIELDS_PAD 197
#endif

#define	PCPU_MD_FIELDS							\
	u_int	pc_acpi_id;	/* ACPI CPU id */			\
	u_int	pc_midr;	/* stored MIDR value */			\
	uint64_t pc_clock;						\
	pcpu_bp_harden pc_bp_harden;					\
	pcpu_ssbd pc_ssbd;						\
	struct pmap *pc_curpmap;					\
	struct pmap *pc_curvmpmap;					\
	uint64_t pc_mpidr;						\
	u_int	pc_bcast_tlbi_workaround;				\
	char __pad[PCPU_MD_FIELDS_PAD]	/* Pad to factor of PAGE_SIZE */

#ifdef _KERNEL

struct pcb;
struct pcpu;

#ifdef __CHERI_PURE_CAPABILITY__
register struct pcpu *pcpup __asm ("c18");
#else
register struct pcpu *pcpup __asm ("x18");
#endif

static inline struct pcpu *
get_pcpu(void)
{
	struct pcpu *pcpu;

#ifdef __CHERI_PURE_CAPABILITY__
	__asm __volatile("mov   %0, c18" : "=&C"(pcpu));
#else
	__asm __volatile("mov   %0, x18" : "=&r"(pcpu));
#endif
	return (pcpu);
}

static inline struct thread *
get_curthread(void)
{
	struct thread *td;

#ifdef __CHERI_PURE_CAPABILITY__
	__asm __volatile("ldr	%0, [c18]" : "=&C"(td));
#else
	__asm __volatile("ldr	%0, [x18]" : "=&r"(td));
#endif
	return (td);
}

/*
 * Set the pcpu pointer with a backup in tpidr_el1 to be
 * loaded when entering the kernel from userland.
 */
static inline void
init_cpu_pcpup(void *pcpup)
{
#ifdef __CHERI_PURE_CAPABILITY__
	__asm __volatile(
	    "mov c18, %0 \n"
	    "msr ctpidr_el1, %0" :: "C"(pcpup));
#else
	__asm __volatile(
	    "mov x18, %0 \n"
	    "msr tpidr_el1, %0" :: "r"(pcpup));
#endif
}

#define	curthread get_curthread()

#define	PCPU_GET(member)	(pcpup->pc_ ## member)
#define	PCPU_ADD(member, value)	(pcpup->pc_ ## member += (value))
#define	PCPU_PTR(member)	(&pcpup->pc_ ## member)
#define	PCPU_SET(member,value)	(pcpup->pc_ ## member = (value))

#define	PCPU_GET_MPIDR(pc)	((pc)->pc_mpidr)

#endif	/* _KERNEL */

#endif	/* !_MACHINE_PCPU_H_ */

#endif /* !__arm__ */
// CHERI CHANGES START
// {
//   "updated": 20230509,
//   "target_type": "header",
//   "changes_purecap": [
//     "support"
//   ]
// }
// CHERI CHANGES END
