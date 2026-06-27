/* (c) 2026 Leonardo Romor <leonardo.romor@gmail.com>
 *
 * Minimal remoteproc resource table for the BeagleG PRU firmware.
 *
 * The kernel's pru_rproc driver parses this section out of the firmware
 * ELF to discover declared resources (rpmsg vrings, carveouts, traces).
 * BeagleG uses bare shared-memory access plus an INTC event to ARM, so
 * no resources need to be declared. An empty table is required nonetheless
 * because the loader rejects firmware without one.
 */

#include <stdint.h>

struct beagleg_resource_table {
  uint32_t ver;
  uint32_t num;
  uint32_t reserved[2];
};

__attribute__((section(".resource_table"), used))
const struct beagleg_resource_table resource_table = {
  .ver = 1,
  .num = 0,
  .reserved = {0, 0},
};
