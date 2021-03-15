#include <cstring>
#include "MemoryStore.h"
#include "RegisterInfo.h"
#include "EndianHelpers.h"
#include "DriverFunctions.h"

#define NUM_REGS 32
#define ADDRESS_WIDTH 32
#define EXIT_FAILURE 1
#define EXIT_INSTRUCTION 0xfeedfeed

#define SIGNEX(v, sb) ((v) | (((v) & (1 << (sb))) ? ~((1 << (sb))-1) : 0))

using namespace std;

// From sim.cpp
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

enum InstType {
    R_TYPE,
    I_TYPE,
    J_TYPE,
    LOAD,
    STORE,
    BRANCH,
    HALT,
    UNKNOWN_TYPE
};

enum HazardType {
    NONE,           // no action required
    STALL,          // Load/Use, Load/Branch, Load/JR, Load/_/Branch, Load/_/JR, R-Type/Branch, R-Type/JR, I-Type/Branch, I-Type/JR
    EX_EX_A,        // R-Type/Use, I-Type/Use
    EX_EX_B,        // R-Type/Use, I-Type/Use
    EX_MM_A,        // Load/_/Use
    EX_MM_B,        // Load/_/Use
    MM_MM,          // Load/Store
    ID_MM_A,        // Load/_/_/Branch, Load/_/_/JR*
    ID_MM_B,        // Load/_/_/Branch, 
    ID_EX_A,        // R-Type/_/Branch, I-Type/_/Branch, R-Type/_/JR*
    ID_EX_B,        // R-Type/_/Branch, I-Type/_/Branch, 
};

struct InstDecoded {
    InstType type;
    HazardType hazard1;
    HazardType hazard2;
    uint32_t bits;
    uint8_t  opcode;
    uint8_t  rs;
    uint8_t  rt;
    uint8_t  rd;
    uint16_t imm;
    uint32_t addr;
    uint8_t  shamt;
    uint8_t  funct;
};

enum RegOffsets {
    AT_OFFSET  = 1,
    V_OFFSET   = 2,
    A_OFFSET   = 4,
    T07_OFFSET = 8,
    S_OFFSET   = 16,
    T89_OFFSET = 24,
    T8_OFFSET  = 24,
    T9_OFFSET  = 25,
    K_OFFSET   = 26,
    GP_OFFSET  = 28,
    SP_OFFSET  = 29,
    FP_OFFSET  = 30,
    RA_OFFSET  = 31
};

enum stages {
    IF = 0,
    ID = 1,
    EX = 2,
    MM = 3,
    WB = 4
};


class Cache {
    private:
        uint32_t    size, blockSize, numBlocks, assoc, missLatency;
        uint32_t    hits, misses;
        uint8_t     *data;
        bool        *valid;
        bool        *dirty;
        uint8_t     *lru;
        uint32_t    *tags;
        int         offsetStart, offsetEnd, indexStart, indexEnd, tagStart, tagEnd;
        MemoryStore *mainMem;
        void read_memory_to_block(int blockNum, uint32_t memAddr);
        void write_block_to_memory(int blockNum);
        void update_lru_bits(int startBlock, int recentlyUsed);
        int block_to_replace(int startBlock);
        int replace_block(int startBlock, uint32_t address, uint32_t tag);
        int getCacheByte(uint32_t address, uint32_t & value);
        int setCacheByte(uint32_t address, uint32_t value);
    public:
        Cache(CacheConfig & config, MemoryStore *mem);
        int getCacheValue(uint32_t address, uint32_t & value, MemEntrySize size);
        int setCacheValue(uint32_t address, uint32_t value, MemEntrySize size);
        void drain();
        uint32_t getHits();
        uint32_t getMisses();
        ~Cache();
};

MemoryStore *myMem;
Cache *ic;
Cache *dc;

InstDecoded instrs[5];
uint32_t regs[NUM_REGS] = {0};
uint32_t pc = 0;
uint32_t fetched = 0;
uint32_t exe_out = 0;
uint32_t mem_out = 0;
uint32_t a = 0;
uint32_t b = 0;

InstDecoded instrs_next[5];
uint32_t regs_next[NUM_REGS];
uint32_t pc_next;
uint32_t fetched_next;
uint32_t exe_out_next;
uint32_t mem_out_next;
uint32_t a_next;
uint32_t b_next;

bool decodedHalt = false;
bool stall = false;
uint32_t ic_stall;

uint32_t numCycles;
int totalCycles = 0;

void dump(uint32_t (&regs)[NUM_REGS], MemoryStore *myMem);
Cache *createCache(CacheConfig config, MemoryStore *mem);
void deleteCache(Cache *cache);
int fetch();
void decode();
void execute();
int memory();
void writeback();
void update_regs();
int decode_inst(uint32_t inst, InstDecoded &decoded);
void hazardDetection();

void initPS(PipeState &ps);


int initSimulator(CacheConfig & icConfig, CacheConfig & dcConfig, MemoryStore *mainMem) {
    myMem = mainMem;
    ic = createCache(icConfig, mainMem);
    dc = createCache(dcConfig, mainMem);
    return 0;
}

int runCycles(uint32_t cycles) {
    // initialize instructions to nop 
    for (int i = 0; i < 5; i++) {
        decode_inst(0, instrs[i]);
    }
    numCycles = cycles;
    int out = 0;
    while (totalCycles < numCycles) {
        if (fetch()) { totalCycles = numCycles; break; } // ran out of cycles during icache miss
        decode();
        execute();
        if (memory()) { totalCycles = numCycles; break; } // ran out of cycles during dcache miss
        writeback();
        update_regs();
        totalCycles++;
        if (instrs[WB].type == HALT) { break; }
    }
    PipeState ps;
    initPS(ps);
    dumpPipeState(ps);
    return (totalCycles == numCycles) ? 1 : 0;
}

int runTillHalt() {
    // initialize instructions to nop 
    for (int i = 0; i < 5; i++) {
        decode_inst(0, instrs[0]);
    }
    while(true) {
        if (fetch()) break;
        decode();
        execute();
        if (memory()) break;
        writeback();
        update_regs();
        totalCycles++;
        if (instrs[WB].type == HALT) { break; }
    }
    PipeState ps;
    initPS(ps);
    dumpPipeState(ps);
    return 0;
}

int finaliseSimulator() {
    SimulationStats s;
    s.totalCycles = totalCycles;
    s.icHits = ic->getHits();
    s.icMisses = ic->getMisses();
    s.dcHits = dc->getHits();
    s.dcMisses = dc->getMisses();
    printSimStats(s);

    ic->drain();
    dc->drain();

    deleteCache(ic);
    deleteCache(dc);

    dump(regs, myMem);

    return 0;
}

//-------------------------------------------------------------------------
// Function Implementations Below
//-------------------------------------------------------------------------
void exception(int stage){
    // Insert nops
    decode_inst(0, instrs[IF]);
    decode_inst(0, instrs[ID]);
    if (stage == EX) {
        decode_inst(0, instrs[EX]);
    }
    // Update pc
    pc_next = 0x8000;

    // So that if HALT is in ID but EX has exception and wipes HALT, we should be able to fetch nonzero instrs
    decodedHalt = false;
}


int log2(uint32_t val) {
    int index = (int) val;
    int targetlevel = 0;
    while (index >>= 1) ++targetlevel;
    return targetlevel;
}

void dump(uint32_t (&regs)[NUM_REGS], MemoryStore *myMem) {
    RegisterInfo reg;
    for (int i = 1; i < NUM_REGS; i++) {
        switch (i) {
            case AT_OFFSET:                         // regs[1] - The $at register.
                reg.at = regs[i];
                break;
            case V_OFFSET ... A_OFFSET-1:           // regs[2:3] - The $v registers.
                reg.v[i - V_OFFSET] = regs[i];
                break;
            case A_OFFSET ... T07_OFFSET-1:         // regs[4:7] - The $a registers.
                reg.a[i - A_OFFSET] = regs[i];
                break;
            case T07_OFFSET ... S_OFFSET-1:         // regs[8:15] - The $t0-$t7 registers.
                reg.t[i - T07_OFFSET] = regs[i];
                break;
            case S_OFFSET ... T8_OFFSET-1:          // regs[16:23] - The $s registers.
                reg.s[i - S_OFFSET] = regs[i];
                break;
            case T8_OFFSET:                         // regs[24] - The $t8 register.
                reg.t[8] = regs[i];
                break;
            case T9_OFFSET:                         // regs[25] - The $t9 register.
                reg.t[9] = regs[i];
                break;
            case K_OFFSET ... GP_OFFSET-1:          // regs[26:27] - The $k registers.
                reg.k[i - K_OFFSET] = regs[i];
                break;
            case GP_OFFSET:                         // regs[28] - The $gp register.
                reg.gp = regs[i];
                break;
            case SP_OFFSET:                         // regs[29] - The $sp register.
                reg.sp = regs[i];
                break;
            case FP_OFFSET:                         // regs[30] - The $fp register.
                reg.fp = regs[i];
                break;
            case RA_OFFSET:                         // regs[31] - The $ra register.
                reg.ra = regs[i];
                break;
        }
    }
    dumpRegisterState(reg);
    dumpMemoryState(myMem);
}

void Cache::read_memory_to_block(int blockNum, uint32_t memAddr) {
    for (int byte_offset = 0; byte_offset < blockSize; byte_offset++) {
        uint32_t temp;
        mainMem->getMemValue(memAddr + byte_offset, temp, BYTE_SIZE);
        data[blockNum * blockSize + byte_offset] = (uint8_t) temp;
    }
}

void Cache::write_block_to_memory(int blockNum) {
    uint32_t memAddr = (tags[blockNum] << tagStart) | ((blockNum / assoc) << indexStart);
    for (int byte_offset = 0; byte_offset < blockSize; byte_offset++) {
        mainMem->setMemValue(memAddr + byte_offset, (uint32_t) data[blockNum * blockSize + byte_offset], BYTE_SIZE);
    }
}

// loop through blocks in a set and update the lru bits
// highest val is most recently used. 0 val is least recently used
void Cache::update_lru_bits(int startBlock, int recentlyUsed) {
    for (int i = 0; i < assoc; i++) {
        int blockNum = startBlock + i;
        if (lru[blockNum] > lru[recentlyUsed]) {
            lru[blockNum]--;
        }
    }
    lru[recentlyUsed] = assoc - 1;
}

// return the number of the block to replace according to LRU policy
// replaces invalid (empty) blocks first
int Cache::block_to_replace(int startBlock) {
    for (int i = 0; i < assoc; i++) {
        int blockNum = startBlock + i;
        if (!valid[blockNum] || lru[blockNum] == 0) {
            return blockNum; // if !valid, it's free. if lru==0, it's either free or the lru, so return.
        }
    }
    return startBlock; // shouldn't ever get here
}

// replace block, return number of block that was replaced
int Cache::replace_block(int startBlock, uint32_t address, uint32_t tag) {
    int replaceBlock = block_to_replace(startBlock);

    if (dirty[replaceBlock]) {
        write_block_to_memory(replaceBlock);
    }

    uint32_t blockStartMemAddr = (address >> offsetEnd) << offsetEnd; // removing byte offset from address
    read_memory_to_block(replaceBlock, blockStartMemAddr);
    valid[replaceBlock] = true;
    dirty[replaceBlock] = false;
    update_lru_bits(startBlock, replaceBlock);
    tags[replaceBlock] = tag;
    return replaceBlock;
}

int Cache::getCacheByte(uint32_t address, uint32_t & value) {
    uint32_t addressCopy = address;
    uint32_t tag = addressCopy << (ADDRESS_WIDTH - tagEnd) >> (ADDRESS_WIDTH - tagEnd) >> tagStart;
    addressCopy = address;
    uint32_t index = addressCopy << (ADDRESS_WIDTH - indexEnd) >> (ADDRESS_WIDTH - indexEnd) >> indexStart;
    addressCopy = address;
    uint32_t offset = addressCopy << (ADDRESS_WIDTH - offsetEnd) >> (ADDRESS_WIDTH - offsetEnd) >> offsetStart;

    // startBlock represents the blockNum of the first block in the indexed set
    int startBlock = index * assoc;

    // loop through blocks in the set, starting at startBlock
    for (int i = 0; i < assoc; i++) {
        int blockNum = startBlock + i;
        if (valid[blockNum] && tag == tags[blockNum]) { // READ HIT
            value = data[blockNum * blockSize + offset];
            update_lru_bits(startBlock, blockNum);
            return 0;
        }
    }

    // READ MISS
    int newBlock = replace_block(startBlock, address, tag);
    value = data[newBlock * blockSize + offset];
    return 1;
}

int Cache::setCacheByte(uint32_t address, uint32_t value) {
    uint32_t addressCopy = address;
    uint32_t tag = addressCopy << (ADDRESS_WIDTH - tagEnd) >> (ADDRESS_WIDTH - tagEnd) >> tagStart;
    addressCopy = address;
    uint32_t index = addressCopy << (ADDRESS_WIDTH - indexEnd) >> (ADDRESS_WIDTH - indexEnd) >> indexStart;
    addressCopy = address;
    uint32_t offset = addressCopy << (ADDRESS_WIDTH - offsetEnd) >> (ADDRESS_WIDTH - offsetEnd) >> offsetStart;

    // startBlock represents the blockNum of the first block in the indexed set
    int startBlock = index * assoc;

    // loop through blocks in the set, starting at startBlock
    for (int i = 0; i < assoc; i++) {
        int blockNum = startBlock + i;
        if (valid[blockNum] && tag == tags[blockNum]) { // WRITE HIT
            data[blockNum * blockSize + offset] = (uint8_t) value;
            dirty[blockNum] = true;
            update_lru_bits(startBlock, blockNum);
            return 0;
        }
    }

    // WRITE MISS
    int newBlock = replace_block(startBlock, address, tag);
    data[newBlock * blockSize + offset] = (uint8_t) value;
    dirty[newBlock] = true;
    return 1;
}

Cache::Cache(CacheConfig & config, MemoryStore *mem) {
    size = config.cacheSize;
    blockSize = config.blockSize;
    numBlocks = size / blockSize;
    assoc = (config.type == TWO_WAY_SET_ASSOC ? 2 : 1);
    missLatency = config.missLatency;

    hits = 0;
    misses = 0;

    data  = new uint8_t[size]();
    valid = new bool[numBlocks]();
    dirty = new bool[numBlocks]();
    lru   = new uint8_t[numBlocks]();
    tags  = new uint32_t[numBlocks]();

    mainMem = mem;

    offsetStart = 0;
    offsetEnd   = log2(blockSize);
    indexStart  = log2(blockSize);
    indexEnd    = indexStart + log2(numBlocks/assoc);
    tagStart    = indexStart + log2(numBlocks/assoc);
    tagEnd      = ADDRESS_WIDTH;
}

int Cache::getCacheValue(uint32_t address, uint32_t & value, MemEntrySize size) {
    value = 0; // clear any prior value so that bitwise or work properly
    uint32_t latency = 0;
    bool miss;
    for (int i = 0; i < size; i++) {
        uint32_t byte;
        miss = getCacheByte(address + i, byte);
        if (i == 0) { // don't tally hits/misses for each byte - only once per query
            if (miss)   { misses++; }
            else        { hits++; }
        }
        if (miss) { latency = missLatency;}
        value = value | (byte << ((size-1-i)*8));
    }
    return latency;
}

int Cache::setCacheValue(uint32_t address, uint32_t value, MemEntrySize size) {
    uint32_t mask = 0xFF;
    uint32_t latency = 0;
    bool miss;
    for (int i = 0; i < size; i++) {
        uint32_t byte = (value & (mask << ((size-1-i)*8))) >> ((size-1-i)*8);
        miss = setCacheByte(address + i, byte);
        if (i == 0) { // don't tally hits/misses for each byte - only once per query
            if (miss)   { misses++; }
            else        { hits++; }
        }
        if (miss) { latency = missLatency; }
    }
    return latency;
}

void Cache::drain() {
    for (int blockNum = 0; blockNum < numBlocks; blockNum++) {
        if (valid[blockNum] && dirty[blockNum]) {
            write_block_to_memory(blockNum);
        }
    }
}

uint32_t Cache::getHits() {
    return hits;
}

uint32_t Cache::getMisses() {
    return misses;
}

Cache::~Cache() {
    delete []data;
    delete []valid;
    delete []dirty;
    delete []lru;
    delete []tags;
}

Cache *createCache(CacheConfig config, MemoryStore *mem) {
    return new Cache(config, mem);
}

void deleteCache(Cache *cache) {
    delete cache;
}

// Testing a 64B, 4B-block, direct-mapped cache
// Address Breakdown: Tag[26] | Index[4] | Offset[2]
int testCache_64_4_direct() {
    uint32_t value = 0;

    myMem->setMemValue(0x00, 0x12345678, WORD_SIZE);
    myMem->setMemValue(0x40, 0x87654321, WORD_SIZE);
    myMem->setMemValue(0x3c, 0x02468642, WORD_SIZE);

    dc->getCacheValue(0x042, value, BYTE_SIZE);        // tag:1, idx:0,  off:2 --> MISS(cmp) --> value = 0x43 (d'67)
    dc->getCacheValue(0x000, value, BYTE_SIZE);        // tag:0, idx:0,  off:0 --> MISS(cmp) --> value = 0x12 (d'18)
    dc->getCacheValue(0x003, value, BYTE_SIZE);        // tag:0, idx:0,  off:3 --> HIT       --> value = 0x78 (d'120)
    dc->getCacheValue(0x043, value, BYTE_SIZE);        // tag:1, idx:0,  off:3 --> MISS(cnf) --> value = 0x21 (d'33)
    dc->getCacheValue(0x040, value, BYTE_SIZE);        // tag:1, idx:0,  off:0 --> HIT       --> value = 0x87 (d'135)

    dc->setCacheValue(0x1f1, value + 5, BYTE_SIZE);    // tag:7, idx:12, off:1 --> MISS(cmp) --> M[0x1f1] = 0x8c (d'140)
    dc->setCacheValue(0x03d, value + 1, BYTE_SIZE);    // tag:0, idx:15, off:1 --> MISS(cmp) --> M[0x03d] = 0x88 (d'136)
    dc->setCacheValue(0x1b1, value + 2, BYTE_SIZE);    // tag:6, idx:12, off:1 --> MISS(cmp) --> M[0xEX_EX_B1] = 0x89 (d'137)
    dc->setCacheValue(0x1f2, value + 3, BYTE_SIZE);    // tag:7, idx:12, off:2 --> MISS(cnf) --> M[0x1f2] = 0x8a (d'138)
    dc->setCacheValue(0x1f3, value + 4, BYTE_SIZE);    // tag:7, idx:12, off:3 --> HIT       --> M[0x1f3] = 0x8b (d'139)

    return 0;
}

// Testing a 51EX_MM_B, 64B-block, two-way-set-associative cache
// Address Breakdown: Tag[24] | Index[2] | Offset[6]
int testCache_512_64_2wsa() {
    uint32_t value = 0;

    myMem->setMemValue(0x28, 0x13579753, WORD_SIZE);
    myMem->setMemValue(0x2c, 0x10203040, WORD_SIZE);
    myMem->setMemValue(0x100, 0xfeeddeef, WORD_SIZE);
    myMem->setMemValue(0x13c, 0xabcddcba, WORD_SIZE);

    dc->getCacheValue(0x100, value, BYTE_SIZE);        // tag:1, idx:0, off:0  --> MISS(cmp) --> value = 0xfe (d'254)
    dc->getCacheValue(0x029, value, BYTE_SIZE);        // tag:0, idx:0, off:41 --> MISS(cmp) --> value = 0x57 (d'87)
    dc->getCacheValue(0x02f, value, BYTE_SIZE);        // tag:0, idx:0, off:47 --> HIT       --> value = 0x40 (d'64)
    dc->getCacheValue(0x102, value, BYTE_SIZE);        // tag:1, idx:0, off:2  --> HIT       --> value = 0xde (d'222)
    dc->getCacheValue(0x13f, value, BYTE_SIZE);        // tag:1, idx:0, off:63 --> HIT       --> value = 0xba (d'186)

    dc->setCacheValue(0x193, value + 5, BYTE_SIZE);    // tag:1, idx:2, off:19 --> MISS(cmp) --> M[0x193] = 0xbf (d'191)
    dc->setCacheValue(0x0a4, value + 2, BYTE_SIZE);    // tag:0, idx:2, off:36 --> MISS(cmp) --> M[0x0a4] = 0xbc (d'188)
    dc->setCacheValue(0x280, 0xcc,      BYTE_SIZE);    // tag:2, idx:2, off:0  --> MISS(cmp) --> M[0x280] = 0x00 (d'000)
    dc->setCacheValue(0x0a8, value + 1, BYTE_SIZE);    // tag:0, idx:2, off:40 --> HIT       --> M[0x0a8] = 0xbb (d'187)
    dc->setCacheValue(0x180, value + 3, BYTE_SIZE);    // tag:1, idx:2, off:0  --> MISS(cnf) --> M[0x180] = 0xbd (d'189)
    dc->setCacheValue(0x1b7, value + 4, BYTE_SIZE);    // tag:1, idx:2, off:55 --> HIT       --> M[0xEX_EX_B7] = 0xbe (d'190)

    dc->getCacheValue(0x280, value, BYTE_SIZE);        // tag:2, idx:2, off:0  --> MISS(cnf) --> value = 0xcc (d'204)
    dc->getCacheValue(0x0a8, value, BYTE_SIZE);        // tag:0, idx:2, off:40 --> MISS(cnf) --> value = 0xbb (d'187)

    return 0;
}

// From sim.cpp
int decode_inst(uint32_t inst, InstDecoded &decoded){
    decoded.bits = inst;
    if (inst == 0xfeedfeed){
        decoded.type = HALT;
#ifdef DEBUG
        std::cerr << "Detected HALT instruction!\n";
#endif
        return 0;
    }

    decoded.opcode = (inst >> OP_SHIFT) & OP_MASK;

    switch ( decoded.opcode ) {
        case 0x0:
            decoded.type  = R_TYPE;
            decoded.rs    = (inst >> RS_SHIFT) & REG_MASK;
            decoded.rt    = (inst >> RT_SHIFT) & REG_MASK;
            decoded.rd    = (inst >> RD_SHIFT) & REG_MASK;
            decoded.shamt = (inst >> SHAMT_SHIFT) & SHAMT_MASK;
            decoded.funct = (inst >> FUNCT_SHIFT) & FUNCT_MASK;
            if (decoded.funct == 0x08) decoded.type = J_TYPE;
            break;
        case 0x8:  // addi
        case 0x9:  // addiu
        case 0xc:  // andi
        case 0xd:  // ori
        case 0xa:  // slti
        case 0xb:  // sltiu
            decoded.type = I_TYPE;
        decoded.rs   = (inst >> RS_SHIFT) & REG_MASK;
        decoded.rt   = (inst >> RT_SHIFT) & REG_MASK;
        decoded.imm  = (inst >> IMM_SHIFT) & IMM_MASK;
        break;
    case 0x24: // lbu
    case 0x25: // lhu
    case 0xf:  // lui
    case 0x23: // lw
        decoded.type = LOAD;
        decoded.rs   = (inst >> RS_SHIFT) & REG_MASK;
        decoded.rt   = (inst >> RT_SHIFT) & REG_MASK;
        decoded.imm  = (inst >> IMM_SHIFT) & IMM_MASK;
        break;
    case 0x28: // sb
    case 0x29: // sh
    case 0x2b: // sw
        decoded.type = STORE;
        decoded.rs   = (inst >> RS_SHIFT) & REG_MASK;
        decoded.rt   = (inst >> RT_SHIFT) & REG_MASK;
        decoded.imm  = (inst >> IMM_SHIFT) & IMM_MASK;
        break;
    case 0x4:  // beq
    case 0x5:  // bne
    case 0x6:  // blez
    case 0x7:  // bgtz
        decoded.type = BRANCH;
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

  return 0;
}

int fetch() {
    ic_stall = 0;
    if (decodedHalt) { fetched_next = 0; }
    else {
        // set default (in case we have exception, we should set default to 0xdeefdeef)
        decode_inst(0xdeefdeef, instrs_next[IF]);
        // try to get instruction from cache
        ic_stall = ic->getCacheValue(pc, fetched_next, WORD_SIZE);
        // if cache miss and penalty would put us over cycles limit, return
        if (totalCycles + ic_stall >= numCycles) return 1; 
        decode_inst(fetched_next, instrs_next[IF]);
        pc_next = pc + 4;
    }
    return 0;
}

void decode() {
    // Update instrs with DEEP COPY
    instrs_next[WB] = instrs[MM];
    instrs_next[MM] = instrs[EX];
    instrs_next[EX] = instrs[ID];
    if (stall)  { instrs_next[ID] = instrs[IF]; }
    else        { decode_inst(fetched, instrs_next[ID]); }

    if (instrs_next[ID].type == HALT) {
        decodedHalt = true;
        decode_inst(0, instrs_next[IF]);
    }

    // Exception Handling
    if (instrs_next[ID].type == UNKNOWN_TYPE) {
        exception(ID);
        return;
    }

	// A = Reg[IR[25-21]] (rs)
	a_next = regs[instrs_next[ID].rs];
	// B = Reg[IR[20-16]] (rt)
	b_next = regs[instrs_next[ID].rt];

    // Hazard Detection
    hazardDetection();

    stall = false;
    switch (instrs_next[ID].hazard1) {
        case NONE:
            break;
        case STALL:
            stall = true;
            instrs_next[IF] = instrs_next[ID]; // DEEP
            decode_inst(0, instrs_next[ID]);
            pc_next = pc;
            break;
        case ID_MM_A:
            a_next = mem_out;
            break;
        case ID_MM_B:
            b_next = mem_out;
            break;
        case ID_EX_A:
            a_next = exe_out;
            break;
        case ID_EX_B:
            b_next = exe_out;
            break;
        default:
            break;
    }
    switch (instrs_next[ID].hazard2) {
        case NONE:
            break;
        case STALL:
            stall = true;
            instrs_next[IF] = instrs_next[ID]; // DEEP
            decode_inst(0, instrs_next[ID]);
            pc_next = pc;
            break;
        case ID_MM_A:
            a_next = mem_out;
            break;
        case ID_MM_B:
            b_next = mem_out;
            break;
        case ID_EX_A:
            a_next = exe_out;
            break;
        case ID_EX_B:
            b_next = exe_out;
            break;
        default:
            break;
    }

	// ALUOUT = PC + SignExt(IR[15-0] << 2)
	int32_t aluout = pc + 4 + ((int32_t)(int16_t)instrs_next[ID].imm << 2);

    // Resolve branch in the decode stage
    // Branch: if true, pc = aluout
    if (instrs_next[ID].type == BRANCH){
        // beq
        if (instrs_next[ID].opcode == 0x4 && a_next == b_next){
            pc_next = aluout;
        }
        // bne
        if (instrs_next[ID].opcode == 0x5 && a_next != b_next){
            pc_next = aluout;
        }
        // blez
        if (instrs_next[ID].opcode == 0x6 && a_next <= 0){
            pc_next = aluout;
        }
        // bgtz
        if (instrs_next[ID].opcode == 0x7 && a_next > 0){
            pc_next = aluout;
        }
    }

    // J-Type: Set PC
    if (instrs_next[ID].type == J_TYPE) {
        switch(instrs_next[ID].opcode) {
            // jr
            case 0x0: {
                uint32_t addr = regs[instrs_next[ID].rs];
                pc_next = ((pc & 0xf0000000) | (addr << 2)) & 0xfffffffc;
                break;
            }
            // j
            case 0x2: {
                pc_next = ((pc & 0xf0000000) | (instrs_next[ID].addr << 2)) & 0xfffffffc;
                break;
            }
            // jal
            case 0x3: {
                pc_next = ((pc & 0xf0000000) | (instrs_next[ID].addr << 2)) & 0xfffffffc;
                break;
            }
        }
    }
}

void execute() {

    switch(instrs_next[EX].hazard1) {
        case NONE:        break;
        case EX_EX_A:     a = exe_out; break;
        case EX_EX_B:     b = exe_out; break;
        case EX_MM_A:     a = mem_out; break;
        case EX_MM_B:     b = mem_out; break;
        default:     break;
    }
    switch(instrs_next[EX].hazard2) {
        case NONE:        break;
        case EX_EX_A:     a = exe_out; break;
        case EX_EX_B:     b = exe_out; break;
        case EX_MM_A:     a = mem_out; break;
        case EX_MM_B:     b = mem_out; break;
        default:     break;
    } 

	switch (instrs_next[EX].type){
    	// Memory: ALUOUT = A + SignExt(IR[15-0])
        case LOAD:
    	case STORE:
    		exe_out_next = a + ((int32_t)(int16_t)instrs_next[EX].imm << 2);
    		break;

		// R-Type: ALUOUT = A op B
		case R_TYPE:
            // add
            if (instrs_next[EX].funct == 0x20){
                int temp;
                temp = (int32_t) a + (int32_t) b;
                if ((a < 0 == b < 0) && (a < 0 != temp < 0)){
                    exception(EX);
                    return;
                }
                else { exe_out_next = temp; }
            }
            // addu
            if (instrs_next[EX].funct == 0x21) exe_out_next = a + b;
            // and
            if (instrs_next[EX].funct == 0x24) exe_out_next = a & b;
            // nor
            if (instrs_next[EX].funct == 0x27) exe_out_next = ~(a | b);
            // or
            if (instrs_next[EX].funct == 0x25) exe_out_next = a | b;
			// slt
            if (instrs_next[EX].funct == 0x2a) exe_out_next = (int32_t) a < (int32_t) b;
            // sltu
            if (instrs_next[EX].funct == 0x2b) exe_out_next = a < b;
            // sll
            if (instrs_next[EX].funct == 0x0) exe_out_next = b << instrs_next[EX].shamt;
            // srl
            if (instrs_next[EX].funct == 0x2) exe_out_next = b >> instrs_next[EX].shamt;
            // sub*
            if (instrs_next[EX].funct == 0x22){
                int temp;
                temp = (int32_t) a - (int32_t) b;
                if ((a < 0 == -b < 0) && (a < 0 != temp < 0)){
                    exception(EX);
                    return;
                }
                else { exe_out_next = temp; }
            }
            // subu
            if (instrs_next[EX].funct == 0x23) exe_out_next = a - b;
            break;

		// I-Type: ALUOUT = A op Immediate
		case I_TYPE:
            // addi
            if (instrs_next[EX].opcode == 0x8){
                int temp;
                int32_t immediate = (int32_t) (uint32_t) instrs_next[EX].imm;
                temp = (int32_t) a + immediate;
                if ((a < 0 == immediate < 0) && (a < 0 != temp < 0)){
                    exception(EX);
                    return;
                }
                else { exe_out_next = temp; }
            }
            // addiu
            if (instrs_next[EX].opcode == 0x9) exe_out_next = a + (uint32_t) instrs_next[EX].imm;
            // ori
            if (instrs_next[EX].opcode == 0xd) exe_out_next = a | (uint32_t) instrs_next[EX].imm;
            // slti
            if (instrs_next[EX].opcode == 0xa) exe_out_next = (int32_t) a < (int16_t) instrs_next[EX].imm;
            // sltiu
            if (instrs_next[EX].opcode == 0xb) exe_out_next = a < instrs_next[EX].imm;
			break;
        
        default:
            break;
	}
}

int memory() {
    uint32_t dc_stall = 0;

    switch(instrs_next[MM].hazard1) {
        case NONE:      break;
        case MM_MM:     b = mem_out; break;
        default:   break;
    }
    switch(instrs_next[MM].hazard2) {
        case NONE:      break;
        case MM_MM:     b = mem_out; break;
        default:   break;
    }

	// Load: mem_out = Memory[ALUOUT]
    if (instrs_next[MM].type == LOAD){
        // lbu
        if(instrs_next[MM].opcode == 0x24) dc_stall = dc->getCacheValue(exe_out, mem_out_next, BYTE_SIZE);
        // lhu
        if(instrs_next[MM].opcode == 0x25) dc_stall = dc->getCacheValue(exe_out, mem_out_next, HALF_SIZE);
        // lui -> How many cycles should load immediate take?
        if(instrs_next[MM].opcode == 0xf) mem_out_next = (uint32_t) instrs_next[MM].imm << 16;
        // lw
        if(instrs_next[MM].opcode == 0x23) dc_stall = dc->getCacheValue(exe_out, mem_out_next, WORD_SIZE);
    }
	// Store: Memory[ALUOUT] = B
    if (instrs_next[MM].type == STORE){
        // sb
        if(instrs_next[MM].opcode == 0x28) dc_stall = dc->setCacheValue(exe_out, b, BYTE_SIZE);
        // sh
        if(instrs_next[MM].opcode == 0x29) dc_stall = dc->setCacheValue(exe_out, b, HALF_SIZE);
        // sw
        if(instrs_next[MM].opcode == 0x2b) dc_stall = dc->setCacheValue(exe_out, b, WORD_SIZE);
    }
    // R-Type: Just pass exe_out through to WB
    if (instrs_next[MM].type == R_TYPE || instrs_next[MM].type == I_TYPE) {
        mem_out_next = exe_out;
    }

    if (totalCycles + dc_stall >= numCycles) return 1;

    if (dc_stall > ic_stall)    { totalCycles += dc_stall; }
    else                        { totalCycles += ic_stall; }

    return 0;
}

void writeback() {
	// Load: Reg[IR[20-16]] = MDR
	if (instrs_next[WB].opcode == LOAD) regs_next[instrs_next[WB].rt] = mem_out;
    // R-Type: Reg[IR[15-11]] = ALUOUT
    if (instrs_next[WB].opcode == R_TYPE) regs_next[instrs_next[WB].rd] = mem_out;
    // I-Type: Reg[IR[20-16]] = ALUOUT
    if (instrs_next[WB].opcode == I_TYPE) regs_next[instrs_next[WB].rt] = mem_out;
    // JAL: Reg[31] = pc + 8
    if (instrs_next[WB].opcode == 0x3) regs[31] = pc + 8;
}

void update_regs() {
    // ---------- Update Regs -----------
    memcpy(instrs, instrs_next, sizeof(instrs));
    memcpy(regs, regs_next, sizeof(regs));
    pc = pc_next;
    fetched = fetched_next;
    exe_out = exe_out_next;
    mem_out = mem_out_next;
    a = a_next;
    b = b_next;
}

void initPS(PipeState &ps){
    ps.cycle = totalCycles;
    ps.ifInstr = instrs[IF].bits;
    ps.idInstr = instrs[ID].bits;
    ps.exInstr = instrs[EX].bits;
    ps.memInstr = instrs[MM].bits;
    ps.wbInstr = instrs[WB].bits;
}

void hazardDetection() {
    instrs_next[ID].hazard1 = NONE;
    instrs_next[ID].hazard2 = NONE;

    if (instrs_next[EX].type == R_TYPE && instrs_next[EX].rd == instrs_next[ID].rs && instrs_next[ID].rs != 0) {
        if (instrs_next[ID].hazard1 == NONE) { instrs_next[ID].hazard1 = EX_EX_A; }
        else { instrs_next[ID].hazard2 = EX_EX_A; }
    }
    if (instrs_next[EX].type == R_TYPE && instrs_next[EX].rd == instrs_next[ID].rt && instrs_next[ID].rt != 0) {
        if (instrs_next[ID].hazard1 == NONE) { instrs_next[ID].hazard1 = EX_EX_B; }
        else { instrs_next[ID].hazard2 = EX_EX_B; }
    }
    
    if (instrs_next[MM].type == LOAD && instrs_next[MM].rt == instrs_next[ID].rt && instrs_next[ID].rt != 0) {
        if (instrs_next[ID].hazard1 == NONE) { instrs_next[ID].hazard1 = EX_MM_A; }
        else { instrs_next[ID].hazard2 = EX_MM_A; }
    }
    if (instrs_next[MM].type == LOAD && instrs_next[MM].rt == instrs_next[ID].rt && instrs_next[ID].rt != 0) {
        if (instrs_next[ID].hazard1 == NONE) { instrs_next[ID].hazard1 = EX_MM_B; }
        else { instrs_next[ID].hazard2 = EX_MM_B; }
    }

    if (instrs_next[EX].type == LOAD && instrs_next[EX].rt == instrs_next[ID].rs && instrs_next[ID].rs != 0) {
        if (instrs_next[ID].hazard1 == NONE) { instrs_next[ID].hazard1 = STALL; }
        else { instrs_next[ID].hazard2 = STALL; }
    }
    if (instrs_next[EX].type == LOAD && instrs_next[EX].rt == instrs_next[ID].rt && instrs_next[ID].rt != 0) {
        if (instrs_next[ID].hazard1 == NONE) { instrs_next[ID].hazard1 = STALL; }
        else { instrs_next[ID].hazard2 = STALL; }
    }
    
    if (instrs_next[EX].type == LOAD && instrs_next[ID].type == STORE && instrs_next[EX].rt == instrs_next[ID].rs && instrs_next[EX].rt != 0) {
        if (instrs_next[ID].hazard1 == NONE) { instrs_next[ID].hazard1 = MM_MM; }
        else { instrs_next[ID].hazard2 = MM_MM; }
    }
    
    if (instrs_next[EX].type == LOAD && instrs_next[ID].type == BRANCH && ((instrs_next[EX].rt == instrs_next[ID].rs && instrs_next[ID].rs != 0) || (instrs_next[EX].rt == instrs_next[ID].rt && instrs_next[ID].rt != 0))) {
        if (instrs_next[ID].hazard1 == NONE) { instrs_next[ID].hazard1 = STALL; }
        else { instrs_next[ID].hazard2 = STALL; }
    }
    
    if (instrs_next[MM].type == LOAD && instrs_next[ID].type == BRANCH && ((instrs_next[MM].rt == instrs_next[ID].rs && instrs_next[ID].rs != 0) || (instrs_next[MM].rt == instrs_next[ID].rt && instrs_next[ID].rt != 0))) {
        if (instrs_next[ID].hazard1 == NONE) { instrs_next[ID].hazard1 = STALL; }
        else { instrs_next[ID].hazard2 = STALL; }
    }
    
    if (instrs_next[WB].type == LOAD && instrs_next[ID].type == BRANCH && (instrs_next[WB].rt == instrs_next[ID].rs) && instrs_next[ID].rs != 0) {
        if (instrs_next[ID].hazard1 == NONE) { instrs_next[ID].hazard1 = ID_MM_A; }
        else { instrs_next[ID].hazard2 = ID_MM_A; }
    }
    if (instrs_next[WB].type == LOAD && instrs_next[ID].type == BRANCH && (instrs_next[WB].rt == instrs_next[ID].rt) && instrs_next[ID].rt != 0) {
        if (instrs_next[ID].hazard1 == NONE) { instrs_next[ID].hazard1 = ID_MM_B; }
        else { instrs_next[ID].hazard2 = ID_MM_B; }
    }
    
    if (instrs_next[EX].type == R_TYPE && instrs_next[ID].type == BRANCH && ((instrs_next[EX].rt == instrs_next[ID].rs && instrs_next[ID].rs != 0) || (instrs_next[EX].rt == instrs_next[ID].rt && instrs_next[ID].rt != 0))) {
        if (instrs_next[ID].hazard1 == NONE) { instrs_next[ID].hazard1 = STALL; }
        else { instrs_next[ID].hazard2 = STALL; }
    }
    
    if (instrs_next[MM].type == R_TYPE && instrs_next[ID].type == BRANCH && (instrs_next[MM].rd == instrs_next[ID].rs) && instrs_next[ID].rs != 0) {
        if (instrs_next[ID].hazard1 == NONE) { instrs_next[ID].hazard1 = ID_EX_A; }
        else { instrs_next[ID].hazard2 = ID_EX_A; }
    }
    if (instrs_next[MM].type == R_TYPE && instrs_next[ID].type == BRANCH && (instrs_next[MM].rd == instrs_next[ID].rt) && instrs_next[ID].rt != 0) {
        if (instrs_next[ID].hazard1 == NONE) { instrs_next[ID].hazard1 = ID_EX_B; }
        else { instrs_next[ID].hazard2 = ID_EX_B; }
    }

    // I-Type/Branch
    if (instrs_next[EX].type == I_TYPE && instrs_next[ID].type == BRANCH && ((instrs_next[EX].rt == instrs_next[ID].rs && instrs_next[ID].rs != 0) || (instrs_next[EX].rt == instrs_next[ID].rt && instrs_next[ID].rt != 0))) {
        if (instrs_next[ID].hazard1 == NONE) { instrs_next[ID].hazard1 = STALL; }
        else { instrs_next[ID].hazard2 = STALL; }
    }
    
    // I-Type/_/Branch
    if (instrs_next[MM].type == I_TYPE && instrs_next[ID].type == BRANCH && (instrs_next[MM].rt == instrs_next[ID].rs) && instrs_next[ID].rs != 0) {
        if (instrs_next[ID].hazard1 == NONE) { instrs_next[ID].hazard1 = ID_EX_A; }
        else { instrs_next[ID].hazard2 = ID_EX_A; }
    }
    if (instrs_next[MM].type == I_TYPE && instrs_next[ID].type == BRANCH && (instrs_next[MM].rt == instrs_next[ID].rt) && instrs_next[ID].rt != 0) {
        if (instrs_next[ID].hazard1 == NONE) { instrs_next[ID].hazard1 = ID_EX_B; }
        else { instrs_next[ID].hazard2 = ID_EX_B; }
    }

    // I-Type/JR
    if (instrs_next[MM].type == I_TYPE && instrs_next[ID].type == J_TYPE && instrs_next[ID].funct == 0x08 && (instrs_next[MM].rt == instrs_next[ID].rs) && instrs_next[ID].rs != 0) {
        if (instrs_next[ID].hazard1 == NONE) { instrs_next[ID].hazard1 = STALL; }
        else { instrs_next[ID].hazard2 = STALL; }
    }

    // R-Type/_/JR
    if (instrs_next[MM].type == R_TYPE && instrs_next[ID].type == J_TYPE && instrs_next[ID].funct == 0x08 && (instrs_next[MM].rd == instrs_next[ID].rs) && instrs_next[ID].rs != 0) {
        if (instrs_next[ID].hazard1 == NONE) { instrs_next[ID].hazard1 = ID_EX_A; }
        else { instrs_next[ID].hazard2 = ID_EX_A; }
    }

    // R-Type/JR
    if (instrs_next[EX].type == R_TYPE && instrs_next[ID].type == J_TYPE && instrs_next[ID].funct == 0x08 && (instrs_next[EX].rd == instrs_next[ID].rs) && instrs_next[ID].rs != 0) {
        if (instrs_next[ID].hazard1 == NONE) { instrs_next[ID].hazard1 = STALL; }
        else { instrs_next[ID].hazard2 = STALL; }
    }

    // Load/_/_/JR
    if (instrs_next[WB].type == LOAD && instrs_next[ID].type == J_TYPE && instrs_next[ID].funct == 0x08 && (instrs_next[WB].rt == instrs_next[ID].rs) && instrs_next[ID].rs != 0) {
        if (instrs_next[ID].hazard1 == NONE) { instrs_next[ID].hazard1 = ID_MM_A; }
        else { instrs_next[ID].hazard2 = ID_MM_A; }
    }

    // I-Type/Use
    if (instrs_next[EX].type == I_TYPE && instrs_next[EX].rt == instrs_next[ID].rs && instrs_next[ID].rs != 0) {
        if (instrs_next[ID].hazard1 == NONE) { instrs_next[ID].hazard1 = EX_EX_A; }
        else { instrs_next[ID].hazard2 = EX_EX_A; }
    }
    if (instrs_next[EX].type == I_TYPE && instrs_next[EX].rt == instrs_next[ID].rt && instrs_next[ID].rt != 0) {
        if (instrs_next[ID].hazard1 == NONE) { instrs_next[ID].hazard1 = EX_EX_B; }
        else { instrs_next[ID].hazard2 = EX_EX_B; }
    }
    
    // Load/JR
    if (instrs_next[EX].type == LOAD && instrs_next[ID].type == J_TYPE && instrs_next[ID].funct == 0x08 && (instrs_next[EX].rt == instrs_next[ID].rs && instrs_next[ID].rs != 0)) {
        if (instrs_next[ID].hazard1 == NONE) { instrs_next[ID].hazard1 = STALL; }
        else { instrs_next[ID].hazard2 = STALL; }
    }
    
    // Load/_/JR
    if (instrs_next[MM].type == LOAD && instrs_next[ID].type == J_TYPE && instrs_next[ID].funct == 0x08 && (instrs_next[MM].rt == instrs_next[ID].rs && instrs_next[ID].rs != 0)) {
        if (instrs_next[ID].hazard1 == NONE) { instrs_next[ID].hazard1 = STALL; }
        else { instrs_next[ID].hazard2 = STALL; }
    }
    
}
