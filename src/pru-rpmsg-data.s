/* (c) 2026 Leonardo Romor <leonardo.romor@gmail.com>
 *
 * This file is part of BeagleG. http://github.com/hzeller/beagleg
 *
 * BeagleG is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * BeagleG is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with BeagleG.  If not, see <http://www.gnu.org/licenses/>.
 */

/* Static data sections of the remoteproc firmware ELF, assembled with
 * pru-elf-as (no compiler involved). The kernel's pru_rproc loader is
 * the only consumer:
 *  - .resource_table declares the virtio-rpmsg device with its two
 *    vrings; the kernel writes the allocated vring addresses and the
 *    vdev status back into it in PRU DRAM.
 *  - .pru_irq_map (parsed from the ELF file, never loaded) tells
 *    kernels >= 5.10 how to program the PRUSS INTC.
 *  - the name-service announcement template is read by rpmsg_init in
 *    pru-rpmsg.hp.
 */

#include "pru-rpmsg-layout.h"

	.section .resource_table, "a"
	.balign 8
	/* struct resource_table */
	.4byte 1			/* ver: only 1 is supported */
	.4byte 1			/* num entries */
	.4byte 0, 0			/* reserved */
	.4byte 0x14			/* offset[0]: the vdev entry below */
	/* struct fw_rsc_vdev */
	.4byte 3			/* type RSC_VDEV */
	.4byte 7			/* id VIRTIO_ID_RPMSG */
	.4byte 0			/* notifyid, filled by the kernel */
	.4byte 1			/* dfeatures: VIRTIO_RPMSG_F_NS */
	.4byte 0			/* gfeatures */
	.4byte 0			/* config_len */
	.byte 0				/* status, written by the kernel */
	.byte 2				/* num_of_vrings */
	.byte 0, 0			/* reserved */
	/* struct fw_rsc_vdev_vring x 2; da is filled by the kernel */
	.4byte 0xFFFFFFFF, VRING_ALIGN, VRING_NUM, 0, 0
	.4byte 0xFFFFFFFF, VRING_ALIGN, VRING_NUM, 0, 0

	.section .pru_irq_map, ""
	/* struct pru_irq_rsc: {type, num, {sysevt, intc ch, host irq}...}.
	 * Only the event the PRU receives needs mapping; the to-ARM vring
	 * kick is routed on the kernel side via the device tree. */
	.byte 0, 1
	.byte RPMSG_EVT_FROM_ARM, 0, 0

	.section .rpmsg_ns_tmpl, "a"
	.balign 4
	/* struct rpmsg_ns_msg: {char name[32], u32 addr, u32 flags} */
	.asciz "rpmsg-pru"
	.fill 22			/* pad name[] to 32 bytes */
	.4byte RPMSG_OUR_ADDR
	.4byte 0			/* RPMSG_NS_CREATE */
