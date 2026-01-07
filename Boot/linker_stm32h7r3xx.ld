/*
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
 */


ENTRY(Reset_Handler)


_estack = ORIGIN(DTCM) + LENGTH(DTCM);
_min_heap_size = 0;
_min_stack_size = 256;


MEMORY 
{
    FLASH       (rx)  : ORIGIN = 0x08000000,    LENGTH = 64K
    ITCM        (xrw) : ORIGIN = 0x00000000,    LENGTH = 64K
    DTCM        (xrw) : ORIGIN = 0x20000000,    LENGTH = 64K
    AXISRAM     (xrw) : ORIGIN = 0x24000000,    LENGTH = 456K
    SRAM        (xrw) : ORIGIN = 0x30000000,    LENGTH = 32K
    BKPSRAM     (xrw) : ORIGIN = 0x38800000,    LENGTH = 4K
}


SECTIONS
{
	.isr_vectors : {
    	. = ALIGN(4);
    	KEEP(*(.isr_vectors))
    	. = ALIGN(4);
  	} >FLASH

  
  	.text : {
    	. = ALIGN(4);
    	*(.text)
    	*(.text*)
    	*(.glue_7)
    	*(.glue_7t)
    	*(.eh_frame)

    	KEEP (*(.init))
    	KEEP (*(.fini))
		
    	. = ALIGN(4);
    	_etext = .;
  	} >FLASH


  	.rodata : {
    	. = ALIGN(4);
    	*(.rodata)
    	*(.rodata*)
    	. = ALIGN(4);
  	} >FLASH

  
  	.ARM.extab (READONLY) : {
    	. = ALIGN(4);
    	*(.ARM.extab* .gnu.linkonce.armextab.*)
    	. = ALIGN(4);
  	} >FLASH
  
  	.ARM (READONLY) : {
    	. = ALIGN(4);
    	__exidx_start = .;
    	*(.ARM.exidx*)
    	__exidx_end = .;
    	. = ALIGN(4);
  	} >FLASH
	
  	.preinit_array (READONLY) : {
    	. = ALIGN(4);
    	PROVIDE_HIDDEN (__preinit_array_start = .);
    	KEEP (*(.preinit_array*))
    	PROVIDE_HIDDEN (__preinit_array_end = .);
    	. = ALIGN(4);
  	} >FLASH

  	.init_array (READONLY) : {
    	. = ALIGN(4);
    	PROVIDE_HIDDEN (__init_array_start = .);
    	KEEP (*(SORT(.init_array.*)))
    	KEEP (*(.init_array*))
    	PROVIDE_HIDDEN (__init_array_end = .);
    	. = ALIGN(4);
  	} >FLASH

  	.fini_array (READONLY) : {
    	. = ALIGN(4);
    	PROVIDE_HIDDEN (__fini_array_start = .);
    	KEEP (*(SORT(.fini_array.*)))
    	KEEP (*(.fini_array*))
    	PROVIDE_HIDDEN (__fini_array_end = .);
    	. = ALIGN(4);
  	} >FLASH

	
	_sidata = LOADADDR(.data);

  	
  	.data : {
    	. = ALIGN(4);
    	_sdata = .;
    	*(.data)
    	*(.data*)
    	*(.RamFunc)
    	*(.RamFunc*)

    	. = ALIGN(4);
    	_edata = .;
  	} >AXISRAM AT> FLASH
	
	
  	. = ALIGN(4);
  
  
  	.bss : {
    	_sbss = .;
    	*(.bss)
    	*(.bss*)
    	*(COMMON)

    	. = ALIGN(4);
    	_ebss = .;
  	} >AXISRAM


  	.user_heap_stack : {
    	. = ALIGN(8);
    	PROVIDE ( end = . );
    	PROVIDE ( _end = . );
    	. = . + _min_heap_size;
    	. = . + _min_stack_size;
    	. = ALIGN(8);
  	} >AXISRAM


  	/DISCARD/ : {
    	libc.a ( * )
    	libm.a ( * )
    	libgcc.a ( * )
  	}

  	.ARM.attributes 0 : { *(.ARM.attributes) }
}
