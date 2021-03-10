target remote :3333
set print asm-demangle on

monitor arm semihosting enable
load
break main
continue
continue
