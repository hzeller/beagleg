/* Linker script for BeagleG PRU firmware loaded by remoteproc.
 *
 * Places PASM-assembled instructions in PRU IMEM starting at address 0
 * (where the PRU begins execution) and the resource_table in PRU DRAM.
 * Sizes match AM335x PRU0: 8 KiB IMEM + 8 KiB DRAM.
 */

OUTPUT_ARCH(pru)

MEMORY
{
  PRUIMEM (rx) : ORIGIN = 0x00000000, LENGTH = 0x2000
  PRUDMEM (rw) : ORIGIN = 0x00000000, LENGTH = 0x2000
}

SECTIONS
{
  .text : { *(.text) *(.text.*) } > PRUIMEM
  .resource_table : { KEEP(*(.resource_table)) } > PRUDMEM
  /DISCARD/ : { *(.note*) *(.comment) *(.eh_frame) }
}
