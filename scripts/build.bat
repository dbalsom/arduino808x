nasm ./asm/%1_regs.asm -o ./bin/regs.bin
nasm ./asm/%1.asm -o ./bin/program.bin -l ./asm/%1.lst
