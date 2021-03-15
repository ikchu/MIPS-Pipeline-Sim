# A simple test case that includes a load-use stall and a branch.
.set noreorder
addi $t4, $zero, next+4;
lw $t0, 0($t4);
addi $t2, $t0, 0x44;
bne $t2, $t0, next;
addi $t1, $zero, 300;
sll  $t1, $t1, 16
next:
.word 0xfeedfeed
.word 0x24
