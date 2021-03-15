#define V_REG_SIZE 2
#define A_REG_SIZE 4
#define T_REG_SIZE 10
#define S_REG_SIZE 8
#define K_REG_SIZE 2

//A struct that can hold the values of all architectural registers.
struct RegisterInfo
{
    //The $at register.
    uint32_t at;
    //The $v registers.
    uint32_t v[V_REG_SIZE];
    //The $a registers.
    uint32_t a[A_REG_SIZE];
    //The $t registers.
    uint32_t t[T_REG_SIZE];
    //The $s registers.
    uint32_t s[S_REG_SIZE];
    //The $k registers.
    uint32_t k[K_REG_SIZE];
    //The $gp register.
    uint32_t gp;
    //The $sp register.
    uint32_t sp;
    //The $fp register.
    uint32_t fp;
    //The $ra register.
    uint32_t ra;
};

extern void dumpRegisterState(RegisterInfo & reg);
