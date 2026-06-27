/* BeagleG PRU firmware resource_table for remoteproc.
 *
 * The kernel loader rejects firmware without a .resource_table section.
 * BeagleG declares no rpmsg vrings / carveouts / traces, so the table
 * is just the four-word header with num=0.
 */
        .section .resource_table, "a"
        .long 1         /* ver       */
        .long 0         /* num       */
        .long 0         /* reserved0 */
        .long 0         /* reserved1 */
