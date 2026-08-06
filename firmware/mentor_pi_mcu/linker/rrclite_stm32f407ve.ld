/* RRCLite V1.0, STM32F407VET6, 512 KiB flash + 128 KiB SRAM + 64 KiB CCM. */

ENTRY(Reset_Handler)

MEMORY
{
  FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 512K
  RAM   (rw)  : ORIGIN = 0x20000000, LENGTH = 128K
  CCM   (rw)  : ORIGIN = 0x10000000, LENGTH = 64K
}

/* Keep executable flash and writable memory in distinct ELF load segments.
   GNU ld 13 treats an implicit RWX segment as a warning; the release link
   promotes every linker warning to an error. */
PHDRS
{
  flash PT_LOAD FLAGS(5); /* PF_R | PF_X */
  data  PT_LOAD FLAGS(6); /* PF_R | PF_W, initialized from flash */
  ram   PT_LOAD FLAGS(6); /* PF_R | PF_W, NOLOAD */
  ccm   PT_LOAD FLAGS(6); /* PF_R | PF_W */
}

_estack = ORIGIN(RAM) + LENGTH(RAM);
_Min_Heap_Size = 0x0;
_Min_Stack_Size = 0x800;

SECTIONS
{
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector))
    . = ALIGN(4);
  } > FLASH :flash

  .text :
  {
    . = ALIGN(4);
    *(.text)
    *(.text*)
    *(.glue_7)
    *(.glue_7t)
    *(.eh_frame)
    KEEP(*(.init))
    KEEP(*(.fini))
    . = ALIGN(4);
    _etext = .;
  } > FLASH :flash

  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)
    *(.rodata*)
    . = ALIGN(4);
  } > FLASH :flash

  .ARM.extab :
  {
    *(.ARM.extab* .gnu.linkonce.armextab.*)
  } > FLASH :flash

  .ARM.exidx :
  {
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
  } > FLASH :flash

  .preinit_array :
  {
    PROVIDE_HIDDEN(__preinit_array_start = .);
    KEEP(*(.preinit_array*))
    PROVIDE_HIDDEN(__preinit_array_end = .);
  } > FLASH :flash

  .init_array :
  {
    PROVIDE_HIDDEN(__init_array_start = .);
    KEEP(*(SORT(.init_array.*)))
    KEEP(*(.init_array*))
    PROVIDE_HIDDEN(__init_array_end = .);
  } > FLASH :flash

  .fini_array :
  {
    PROVIDE_HIDDEN(__fini_array_start = .);
    KEEP(*(SORT(.fini_array.*)))
    KEEP(*(.fini_array*))
    PROVIDE_HIDDEN(__fini_array_end = .);
  } > FLASH :flash

  _sidata = LOADADDR(.data);
  .data :
  {
    . = ALIGN(4);
    _sdata = .;
    *(.data)
    *(.data*)
    . = ALIGN(4);
    _edata = .;
  } > RAM AT > FLASH :data

  /* DMA2 cannot access CCM. All first-party DMA objects carry the explicit
     .dma_buffer attribute and are retained in the DMA-accessible SRAM bank. */
  .dma_buffer (NOLOAD) :
  {
    . = ALIGN(32);
    __dma_buffer_start__ = .;
    KEEP(*(.dma_buffer))
    KEEP(*(.dma_buffer.*))
    . = ALIGN(32);
    __dma_buffer_end__ = .;
  } > RAM AT > RAM :ram

  .bss (NOLOAD) :
  {
    . = ALIGN(8);
    _sbss = .;
    __bss_start__ = _sbss;
    *(.bss)
    *(.bss*)
    *(COMMON)
    . = ALIGN(8);
    _ebss = .;
    __bss_end__ = _ebss;
  } > RAM AT > RAM :ram

  .noinit (NOLOAD) :
  {
    . = ALIGN(8);
    __noinit_start__ = .;
    KEEP(*(.noinit.watchdog_retention))
    *(.noinit)
    *(.noinit*)
    . = ALIGN(8);
    __noinit_end__ = .;
  } > RAM AT > RAM :ram

  ._user_heap_stack (NOLOAD) :
  {
    . = ALIGN(8);
    PROVIDE(end = .);
    PROVIDE(_end = .);
    . += _Min_Heap_Size;
    . += _Min_Stack_Size;
    . = ALIGN(8);
    __ram_used_end__ = .;
  } > RAM AT > RAM :ram

  /* CPU-only objects, including the 48 KiB resettable micro-ROS arena. */
  .ccmram (NOLOAD) :
  {
    . = ALIGN(8);
    __ccm_start__ = .;
    KEEP(*(.ccmram))
    KEEP(*(.ccmram.*))
    . = ALIGN(8);
    __ccm_end__ = .;
  } > CCM AT > CCM :ccm

  /DISCARD/ : { *(.note.GNU-stack) }

  .ARM.attributes 0 : { *(.ARM.attributes) } :NONE

  __flash_image_end__ = LOADADDR(.data) + SIZEOF(.data);

  /* Release gates: no more than 80% of each memory class. */
  ASSERT(__flash_image_end__ - ORIGIN(FLASH) <= 419430,
         "RRCLite flash exceeds 80% release budget")
  ASSERT(__ram_used_end__ - ORIGIN(RAM) <= 104857,
         "RRCLite SRAM exceeds 80% release budget")
  ASSERT(__ccm_end__ - ORIGIN(CCM) <= 52428,
         "RRCLite CCM exceeds 80% release budget")
  ASSERT(__dma_buffer_start__ >= ORIGIN(RAM) &&
         __dma_buffer_end__ <= ORIGIN(RAM) + LENGTH(RAM),
         "DMA object escaped DMA-accessible SRAM")
}
