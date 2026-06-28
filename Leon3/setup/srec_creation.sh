sparc-gaisler-elf-gcc -O2 -mcpu=leon3 systest.c -o systest.exe
sparc-gaisler-elf-objcopy -O srec systest.exe systest.srec
