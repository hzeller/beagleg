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

/* Shared layout constants of the rpmsg firmware variant. Included by
 * the PASM sources (pru-rpmsg.hp), the static-data assembly
 * (pru-rpmsg-data.s) and the packaging linker script
 * (pru-firmware.ld.in), so every address and struct offset lives
 * exactly once.
 *
 * PRU0 DRAM map:
 *   0x0000  motion ring buffer (host-owned, struct PRUCommunication)
 *   0x0F80  rpmsg state block (ST_* offsets below)
 *   0x1000  resource table (kernel writes vring addresses + vdev
 *           status back into it at boot)
 *   0x1100  rpmsg name-service announcement template
 */
#ifndef BEAGLEG_PRU_RPMSG_LAYOUT_H
#define BEAGLEG_PRU_RPMSG_LAYOUT_H

#define RPMSG_STATE_ADDR   0x0F80
#define RPMSG_RT_ADDR      0x1000
#define RPMSG_NS_TMPL_ADDR 0x1100

/* Offsets into the resource table (mirrors the layout emitted by
 * pru-rpmsg-data.s: 16-byte header, one offset entry, fw_rsc_vdev,
 * two fw_rsc_vdev_vring of 20 bytes each). */
#define RT_VDEV_STATUS_OFFSET 0x2C /* u8 fw_rsc_vdev.status */
#define RT_VRING0_OFFSET      0x30 /* fw_rsc_vdev_vring.da of vring 0 */
#define RT_VRING1_OFFSET      0x44 /* fw_rsc_vdev_vring.da of vring 1 */

/* rpmsg state block, offsets from RPMSG_STATE_ADDR. The vring base
 * pointers are computed once at init from the kernel-written device
 * addresses in the resource table. */
#define ST_VQ0_DESC  0x00 /* u32: vring0 descriptor table (PRU -> ARM) */
#define ST_VQ0_AVAIL 0x04 /* u32 */
#define ST_VQ0_USED  0x08 /* u32 */
#define ST_VQ1_DESC  0x0C /* u32: vring1 (ARM -> PRU) */
#define ST_VQ1_AVAIL 0x10 /* u32 */
#define ST_VQ1_USED  0x14 /* u32 */
#define ST_VQ0_LAST  0x18 /* u16: last seen avail->idx on vring0 */
#define ST_VQ1_LAST  0x1A /* u16 */
#define ST_HOST_ADDR 0x1C /* u16: host endpoint; 0 = not yet learned */

/* virtio ring geometry; must match the static resource table.
 *   desc:  VRING_NUM entries of {u64 addr; u32 len; u16 flags; u16 next}
 *   avail: {u16 flags; u16 idx; u16 ring[VRING_NUM]}
 *   used:  {u16 flags; u16 idx; {u32 id; u32 len} ring[VRING_NUM]},
 *          aligned to VRING_ALIGN after avail. */
#define VRING_NUM       16
#define VRING_ALIGN     16
#define VRING_AVAIL_OFF 0x100 /* VRING_NUM * 16 */
#define VRING_USED_OFF  0x130 /* align16(VRING_AVAIL_OFF + 4 + 2*VRING_NUM) */

/* rpmsg wire format: every message starts with
 * {u32 src; u32 dst; u32 reserved; u16 len; u16 flags}. */
#define RPMSG_HDR_SIZE 16
#define RPMSG_OUR_ADDR 30   /* channel port -> /dev/rpmsg_pru30 */
#define RPMSG_NS_ADDR  0x35 /* name-service endpoint */
#define RPMSG_NS_LEN   40   /* {char name[32]; u32 addr; u32 flags} */

/* PRU system events for the virtio kicks (TI convention, PRU0).
 * TO_ARM is raised via R31 (strobe value 32 + (event - 16)); FROM_ARM
 * is routed to the PRU by the kernel per the .pru_irq_map section. */
#define RPMSG_EVT_TO_ARM   16
#define RPMSG_EVT_FROM_ARM 17

/* fw_rsc_vdev.status bit the kernel sets when the virtio link is up. */
#define VIRTIO_CONFIG_S_DRIVER_OK 4

#endif /* BEAGLEG_PRU_RPMSG_LAYOUT_H */
