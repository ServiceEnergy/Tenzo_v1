/*
 *  ======== CC1310_LAUNCHXL.cmd ========
 */

--stack_size=1024   /* C stack is also used for ISR stack */

HEAPSIZE = 0x1000;  /* Size of heap buffer used by HeapMem */

/* Override default entry point.                                             */
--entry_point ResetISR
/* Allow main() to take args                                                 */
--args 0x8
/* Suppress warnings and errors:                                             */
/* - 10063: Warning about entry point not being _c_int00                     */
/* - 16011, 16012: 8-byte alignment errors. Observed when linking in object  */
/*   files compiled using Keil (ARM compiler)                                */
--diag_suppress=10063,16011,16012

/* The starting address of the application.  Normally the interrupt vectors  */
/* must be located at the beginning of the application.                      */
#define FLASH_BASE              0x0
#define FLASH_SIZE              0x20000
#define RAM_BASE                0x20000000
#define RAM_SIZE                0x5000

/* System memory map */

MEMORY
{
    /* Application stored in and executes from internal flash */
    FLASH (RX) : origin = FLASH_BASE, length = FLASH_SIZE
    /* Application uses internal RAM for data */
    SRAM (RWX) : origin = RAM_BASE, length = RAM_SIZE
}

/* Section allocation in memory */

SECTIONS
{
    .text           :   >> FLASH
    .TI.ramfunc     : {} load=FLASH, run=SRAM, table(BINIT)
    .const          :   >> FLASH
    .constdata      :   >> FLASH
    .rodata         :   >> FLASH
    .cinit          :   > FLASH
    .pinit          :   > FLASH
    .init_array     :   > FLASH
    .emb_text       :   >> FLASH
    .ccfg           :   > FLASH (HIGH)

    .data           :   > SRAM
    .bss            :   > SRAM
    .sysmem         :   > SRAM
    .nonretenvar    :   > SRAM

    /* Heap buffer used by HeapMem */
    .priheap   : {
        __primary_heap_start__ = .;
        . += HEAPSIZE;
        __primary_heap_end__ = .;
    } > SRAM align 8

    .stack          :   > SRAM (HIGH)
}
