#include <inttypes.h>

enum CacheType
{
    DIRECT_MAPPED,
    TWO_WAY_SET_ASSOC
};

struct CacheConfig
{
    //Cache size in bytes.
    uint32_t cacheSize;
    //Cache block size in bytes.
    uint32_t blockSize;
    //Type of cache - direct-mapped or two-way set-assoc?
    CacheType type;
    //Miss latency in cycles.
    uint32_t missLatency;
};
