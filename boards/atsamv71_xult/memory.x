
MEMORY
{
  /* Virtual FLASH region using the first 256KB of internal SRAM */
  /* FLASH : ORIGIN = 0x400000, LENGTH = 2M */
  FLASH : ORIGIN = 0x20400000, LENGTH = 256K

  /* Virtual RAM region using the remaining 128KB of internal SRAM */
  RAM   : ORIGIN = 0x20440000, LENGTH = 128K
}
/* REGION_ALIAS("FLASH", ITCM); */
/* REGION_ALIAS("RAM", DTCM); */

/* Force the vector table to be at the exact beginning of your new FLASH region */
EXTERN(RESET_VECTOR);
ENTRY(RESET_VECTOR);

SECTIONS {
  .can (NOLOAD) :
  {
    *(.can .can.*);
  } > CAN
}

/* This is where the call stack will be allocated. */
/* The stack is of the full descending type. */
/* You may want to use this variable to locate the call stack and static
   variables in different memory regions. Below is shown the default value */
/* _stack_start = ORIGIN(RAM) + LENGTH(RAM); */

/* You can use this symbol to customize the location of the .text section */
/* If omitted the .text section will be placed right after the .vector_table
   section */
/* This is required only on microcontrollers that store some configuration right
   after the vector table */
/* _stext = ORIGIN(FLASH) + 0x400; */

/* Example of putting non-initialized variables into custom RAM locations. */
/* This assumes you have defined a region RAM2 above, and in the Rust
   sources added the attribute `#[link_section = ".ram2bss"]` to the data
   you want to place there. */
/* Note that the section will not be zero-initialized by the runtime! */
/* SECTIONS {
     .ram2bss (NOLOAD) : ALIGN(4) {
       *(.ram2bss);
       . = ALIGN(4);
     } > RAM2
   } INSERT AFTER .bss;
*/
