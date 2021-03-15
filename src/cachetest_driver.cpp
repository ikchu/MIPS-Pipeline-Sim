#include <iostream>
#include <iomanip>
#include <fstream>
#include <errno.h>
#include "../src/MemoryStore.h"
#include "../src/RegisterInfo.h"
#include "../src/EndianHelpers.h"
#include "../src/DriverFunctions.h"

using namespace std;

static MemoryStore *mem;

int main(int argc, char **argv)
{
    mem = createMemoryStore();

    CacheConfig icConfig;
    icConfig.cacheSize = 64;
    icConfig.blockSize = 4;
    icConfig.type = DIRECT_MAPPED;
    icConfig.missLatency = 5;
    CacheConfig dcConfig = icConfig;

    initSimulator(icConfig, dcConfig, mem);
    testCache_64_4_direct(); // if want to test, need to add this to DriverFunctions.h
    finaliseSimulator();

    icConfig.cacheSize = 512;
    icConfig.blockSize = 64;
    icConfig.type = TWO_WAY_SET_ASSOC;
    icConfig.missLatency = 5;
    dcConfig = icConfig;

    initSimulator(icConfig, dcConfig, mem);
    testCache_512_64_2wsa(); // if want to test, need to add this to DriverFunctions.h
    finaliseSimulator();

    delete mem;
    return 0;
}
