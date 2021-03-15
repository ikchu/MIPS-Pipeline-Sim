#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <inttypes.h>

#include <cstdio>
#include <cstdlib>
#include <iostream>

#include "MemoryStore.h"
#include "RegisterInfo.h"
#include "EndianHelpers.h"

#define OP_SHIFT      26
#define RS_SHIFT      21
#define RT_SHIFT      16
#define RD_SHIFT      11
#define IMM_SHIFT     0
#define ADDR_SHIFT    0
#define SHAMT_SHIFT   6
#define FUNCT_SHIFT   0

#define OP_MASK       0x3f
#define REG_MASK      0x1f
#define IMM_MASK      0xffff
#define ADDR_MASK     0x3ffffff
#define SHAMT_MASK    0x1f
#define FUNCT_MASK    0x3f

#define ADD_FUNCT     0x20
#define ADDU_FUNCT    0x21
#define AND_FUNCT     0x24
#define JR_FUNCT      0x8
#define NOR_FUNCT     0x27
#define OR_FUNCT      0x25
#define SLT_FUNCT     0x2a
#define SLTU_FUNCT    0x2b
#define SLL_FUNCT     0x0
#define SRL_FUNCT     0x2
#define SUB_FUNCT     0x22
#define SUBU_FUNCT    0x23

enum InstType
{
  R_TYPE,
  I_TYPE,
  J_TYPE,
  HALT,
  UNKNOWN_TYPE
};

struct InstDecoded
{
  InstType type;
  uint8_t opcode;
  uint8_t rs;
  uint8_t rt;
  uint8_t rd;
  uint16_t imm;
  uint32_t addr;
  uint8_t shamt;
  uint8_t funct;
};

int decode_inst( uint32_t inst, InstDecoded &decoded );
uint32_t *get_arch_register( RegisterInfo &registers, uint32_t reg );
int handle_r_type( InstDecoded decoded );
int handle_i_type( InstDecoded decoded );
int handle_j_type( InstDecoded decoded );

uint32_t sim_registers[32];
uint32_t pc;
uint32_t jmp_dst; // destination for delayed jumps
uint8_t in_delay = 0; // if currently in a delay slot
MemoryStore *memory_store;

int main( int argc, char **argv )
{
  if ( argc != 2 )
  {
    std::cerr << "Usage: sim <binary file>\n";
    return 1;
  }

  int bytes_read;
  uint32_t mem_index = 0;
  char *in_filename = argv[1];
  int read_word = 0;

  // create memory and registers
  memory_store = createMemoryStore();
  RegisterInfo registers = { 0 };

  int fd = open( in_filename, O_RDONLY );

  bytes_read = read( fd, &read_word, sizeof(int) );
  if ( bytes_read == -1 )
  {
    perror("Couldn't read binary file: ");
    return 1;
  }

  while ( bytes_read )
  {
    // write bytes read to memory
    read_word = ConvertWordToBigEndian( read_word );
    memory_store->setMemValue( mem_index, read_word, WORD_SIZE );

    // read more bytes
    bytes_read = read( fd, &read_word, sizeof(int) );
    if ( bytes_read == -1 )
    {
      perror("Couldn't read binary file: ");
      return 1;
    }
    mem_index += 4;
  }

  // initialize $pc to 0
  pc = 0x0;

  // processor loop
  while ( 1 )
  {
    uint32_t current_inst;
    InstDecoded decoded;

    memory_store->getMemValue( pc, current_inst, WORD_SIZE );
    decode_inst( current_inst, decoded );

    pc += 4; // increment now since branch uses pc+4

    if ( in_delay == 1 )
      in_delay = 2;

    switch ( decoded.type )
    {
      case HALT:
        // don't need to copy zero register
        for ( int i = 1; i < 32; i++ )
          *(get_arch_register( registers, i )) = sim_registers[i];
        dumpRegisterState( registers );
        dumpMemoryState( memory_store );
        return 0;
      case R_TYPE:
        handle_r_type( decoded );
        break;
      case I_TYPE:
        handle_i_type( decoded );
        break;
      case J_TYPE:
        handle_j_type( decoded );
        break;
      default:
        std::cerr << "Executing illegal instruction type; quitting\n";
        // don't need to copy zero register
        for ( int i = 1; i < 32; i++ )
          *(get_arch_register( registers, i )) = sim_registers[i];
        dumpRegisterState( registers );
        dumpMemoryState( memory_store );
        return 1;
    }

    // if currently in a delay slot, change pc to jump destination
    if ( in_delay == 2 )
    {
#ifdef DEBUG
      std::cerr << "In delay slot -- setting pc to " << std::hex << jmp_dst << "\n";
#endif
      pc = jmp_dst;
      in_delay = 0;
    }
  }
}

int decode_inst( uint32_t inst, InstDecoded &decoded )
{
  if ( inst == 0xfeedfeed )
  {
    decoded.type = HALT;
#ifdef DEBUG
    std::cerr << "Detected HALT instruction!\n";
#endif
    return 0;
  }

  decoded.opcode = (inst >> OP_SHIFT) & OP_MASK;

  switch ( decoded.opcode )
  {
    case 0x0:
      decoded.type  = R_TYPE;
      decoded.rs    = (inst >> RS_SHIFT) & REG_MASK;
      decoded.rt    = (inst >> RT_SHIFT) & REG_MASK;
      decoded.rd    = (inst >> RD_SHIFT) & REG_MASK;
      decoded.shamt = (inst >> SHAMT_SHIFT) & SHAMT_MASK;
      decoded.funct = (inst >> FUNCT_SHIFT) & FUNCT_MASK;
      break;
    case 0x8:  // addi
    case 0x9:  // addiu
    case 0xc:  // andi
    case 0x4:  // beq
    case 0x5:  // bne
    case 0x24: // lbu
    case 0x25: // lhu
    case 0xf:  // lui
    case 0x23: // lw
    case 0xd:  // ori
    case 0xa:  // slti
    case 0xb:  // sltiu
    case 0x28: // sb
    case 0x29: // sh
    case 0x2b: // sw
    case 0x6:  // blez
    case 0x7:  // bgtz
      decoded.type = I_TYPE;
      decoded.rs   = (inst >> RS_SHIFT) & REG_MASK;
      decoded.rt   = (inst >> RT_SHIFT) & REG_MASK;
      decoded.imm  = (inst >> IMM_SHIFT) & IMM_MASK;
      break;
    case 0x2:  // j
    case 0x3:  // jal
      decoded.type = J_TYPE;
      decoded.addr = (inst >> ADDR_SHIFT) & ADDR_MASK;
      break;
    default:
      decoded.type = UNKNOWN_TYPE;
      break;
  }

#ifdef DEBUG
  std::cerr << "Instruction: " << std::hex << inst << "\n";
  switch ( decoded.type )
  {
    case R_TYPE:
      std::cerr << "R_TYPE inst\n";
      break;
    case I_TYPE:
      std::cerr << "I_TYPE inst\n";
      break;
    case J_TYPE:
      std::cerr << "J_TYPE inst\n";
      break;
    default:
      std::cerr << "We done screwed up -- UNKNOWN_TYPE\n";
      break;
  }

  std::cerr << "\tOpcode = 0x" << std::hex << (uint32_t) decoded.opcode << "\n";
  if ( decoded.type == R_TYPE || decoded.type == I_TYPE )
  {
    std::cerr << "\trs = 0x" << std::hex << (uint32_t) decoded.rs << "\n";
    std::cerr << "\trt = 0x" << std::hex << (uint32_t) decoded.rt << "\n";
  }
  if ( decoded.type == R_TYPE )
  {
    std::cerr << "\trd = 0x"    << std::hex << (uint32_t) decoded.rd << "\n";
    std::cerr << "\tshamt = 0x" << std::hex << (uint32_t) decoded.shamt << "\n";
    std::cerr << "\tfunct = 0x" << std::hex << (uint32_t) decoded.funct << "\n";
  }
  // XXX Should this be case to int32_t instead? Or just print decimal
  // answer: depends on if it's signed or unsigned instruction
  else if ( decoded.type == I_TYPE )
    std::cerr << "\timm = 0x" << std::hex << (uint32_t) decoded.imm << "\n";
  else if ( decoded.type == J_TYPE )
    std::cerr << "\taddr = 0x" << std::hex << (uint32_t) decoded.addr << "\n";
#endif
}

uint32_t *get_arch_register( RegisterInfo &registers, uint32_t reg )
{
  if ( reg == 1 )
    return &registers.at;
  if ( reg == 2 || reg == 3 )
    return &registers.v[reg - 2];
  if ( reg >= 4 && reg <= 7 )
    return &registers.a[reg - 4];
  if ( reg >= 8 && reg <= 15 )
    return &registers.t[reg - 8];
  if ( reg >= 16 && reg <= 23 )
    return &registers.s[reg - 16];
  if ( reg == 24 || reg == 25 )
    return &registers.t[reg - 16];
  if ( reg == 26 || reg == 27 )
    return &registers.k[reg - 26];
  if ( reg == 28 )
    return &registers.gp;
  if ( reg == 29 )
    return &registers.sp;
  if ( reg == 30 )
    return &registers.fp;
  if ( reg == 31 )
    return &registers.ra;
  std::cerr << "Failed to find register in get_arch_register()\n";
  return NULL;
}

int handle_r_type( InstDecoded decoded )
{
  // pointer to unsigned register
  uint32_t *rs_u = &sim_registers[decoded.rs];
  uint32_t *rt_u = &sim_registers[decoded.rt];
  uint32_t *rd_u = &sim_registers[decoded.rd];

  // signed cast of each register
  int32_t *rs_s = (int32_t *)(&sim_registers[decoded.rs]);
  int32_t *rt_s = (int32_t *)(&sim_registers[decoded.rt]);
  int32_t *rd_s = (int32_t *)(&sim_registers[decoded.rd]);

  switch ( decoded.funct )
  {
    case ADD_FUNCT:
      *rd_s = *rs_s + *rt_s;
      break;
    case ADDU_FUNCT:
      *rd_u = *rs_u + *rt_u;
      break;
    case AND_FUNCT:
      *rd_u = *rs_u & *rt_u;
      break;
    case JR_FUNCT:
      jmp_dst = *rs_u;
      in_delay = 1;
      // pc = *rs_u;
      break;
    case NOR_FUNCT:
      *rd_u = ~(*rs_u | *rt_u);
      break;
    case OR_FUNCT:
      *rd_u = *rs_u | *rt_u;
      break;
    case SLT_FUNCT:
      *rd_u = *rs_s < *rt_s;
      break;
    case SLTU_FUNCT:
      *rd_u = *rs_u < *rt_u;
      break;
    case SLL_FUNCT:
      *rd_u = *rt_u << decoded.shamt;
      break;
    case SRL_FUNCT:
      *rd_u = *rt_u >> decoded.shamt;
      break;
    case SUB_FUNCT:
      *rd_s = *rs_s - *rt_s;
      break;
    case SUBU_FUNCT:
      *rd_u = *rs_u - *rt_u;
      break;
    default:
      std::cerr << "Tried to execute illegal R-type instruction!\n";
      return -1;
  }

  // in case $zero register was overwritten
  sim_registers[0] = 0;

  return 0;
}

int handle_i_type( InstDecoded decoded )
{
  // pointer to unsigned register
  uint32_t *rs_u = &sim_registers[decoded.rs];
  uint32_t *rt_u = &sim_registers[decoded.rt];
  uint32_t imm_u = (uint32_t) (uint16_t) decoded.imm;

  // signed cast of each register
  int32_t *rs_s = (int32_t *)(&sim_registers[decoded.rs]);
  int32_t *rt_s = (int32_t *)(&sim_registers[decoded.rt]);
  int32_t imm_s = (int32_t) (int16_t) decoded.imm;
  uint32_t tmp;

  switch ( decoded.opcode )
  {
    case 0x8: // addi
    case 0x9: // addiu
      *rt_s = *rs_s + imm_s;
      break;
    case 0xc: // andi
      *rt_u = *rs_u & imm_u;
      break;
    case 0x4: // beq
      jmp_dst = *rs_u == *rt_u ? pc + (imm_s << 2) : pc;
      in_delay = *rs_u == *rt_u;
      break;
    case 0x5: // bne
      jmp_dst = *rs_u != *rt_u ? pc + (imm_s << 2) : pc;
      in_delay = *rs_u != *rt_u;
      break;
    case 0x24: // lbu
      memory_store->getMemValue( (*rs_u + imm_s), tmp, BYTE_SIZE );
      *rt_u = 0xff & tmp;
      break;
    case 0x25: // lhu
      memory_store->getMemValue( (*rs_u + imm_s), tmp, HALF_SIZE );
      *rt_u = 0xffff & tmp;
      break;
    case 0xf: // lui
      *rt_u = imm_u << 16;
      break;
    case 0x23: // lw
      memory_store->getMemValue( (*rs_u + imm_s), *rt_u, WORD_SIZE );
      break;
    case 0xd: // ori
      *rt_u = *rs_u | imm_u;
      break;
    case 0xa: // slti
      *rt_u = *rs_s < imm_s;
      break;
    case 0xb: // sltiu
      *rt_u = *rs_u < (uint32_t) imm_s;
      break;
    case 0x28: // sb
      memory_store->setMemValue( (*rs_u + imm_s), *rt_u & 0xff, BYTE_SIZE );
      break;
    case 0x29: // sh
      memory_store->setMemValue( (*rs_u + imm_s), *rt_u & 0xffff, HALF_SIZE );
      break;
    case 0x2b: // sw
      memory_store->setMemValue( (*rs_u + imm_s), *rt_u, WORD_SIZE );
      break;
    case 0x6: // blez
      jmp_dst = *rs_s <= 0 ? pc + (imm_s << 2) : pc;
      in_delay = *rs_s <= 0;
      break;
    case 0x7: // bgtz
      jmp_dst = *rs_s > 0 ? pc + (imm_s << 2) : pc;
      in_delay = *rs_s > 0;
      break;
    default:
      std::cerr << "Tried to execute illegal I-type instruction!\n";
      break;
  }

  // in case $zero register was overwritten
  sim_registers[0] = 0;
}

int handle_j_type( InstDecoded decoded )
{
  switch( decoded.opcode )
  {
    case 0x2: // j
      jmp_dst = ((pc & 0xf0000000) | (decoded.addr << 2)) & 0xfffffffc;
      in_delay = 1;
      // pc = ((pc & 0xf0000000) | (decoded.addr << 2)) & 0xfffffffc;
      break;
    case 0x3: // jal
      sim_registers[31] = pc + 4; // pc is already pc+4
      jmp_dst = ((pc & 0xf0000000) | (decoded.addr << 2)) & 0xfffffffc;
      in_delay = 1;
      // pc = ((pc & 0xf0000000) | (decoded.addr << 2)) & 0xfffffffc;
      break;
    default:
      std::cerr << "Tried to execute illegal J-type instruction!\n";
      break;
  }
}
