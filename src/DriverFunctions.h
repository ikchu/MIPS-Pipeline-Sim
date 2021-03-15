#include "CacheConfig.h"

struct PipeState
{
    uint32_t cycle;
    uint32_t ifInstr;
    uint32_t idInstr;
    uint32_t exInstr;
    uint32_t memInstr;
    uint32_t wbInstr;
};

struct SimulationStats
{
    uint32_t totalCycles;
    uint32_t icHits;
    uint32_t icMisses;
    uint32_t dcHits;
    uint32_t dcMisses;
};

//Implemented in UtilityFunctions.o
int dumpPipeState(PipeState & state);
int printSimStats(SimulationStats & stats);

//You must implement the following functions.
int initSimulator(CacheConfig & icConfig, CacheConfig & dcConfig, MemoryStore *mainMem);
int runCycles(uint32_t cycles);
int runTillHalt();
int finaliseSimulator();

int testCache_64_4_direct();
int testCache_512_64_2wsa();
