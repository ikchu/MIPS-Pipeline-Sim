#include <iostream>
#include <iomanip>
#include "MemoryStore.h"
#include "RegisterInfo.h"

using namespace std;

int main()
{
    uint32_t value = 0;
    MemoryStore *myMem = createMemoryStore();

    myMem->getMemValue(0x10, value, WORD_SIZE);

    cout << "The 32-bit (word) value of address 0x10 is 0x" << hex << setfill('0') << setw(8) << value << endl;
    
    cout << "Setting byte 0x10 to 0xff" << endl;
    myMem->setMemValue(0x10, 0xff, BYTE_SIZE);

    myMem->getMemValue(0x10, value, WORD_SIZE);
    cout << "The 32-bit (word) value of address 0x10 is now 0x" << hex << setfill('0') << setw(8) << value << endl;

    myMem->getMemValue(0x10, value, BYTE_SIZE);
    cout << "The 8-bit (byte) value of address 0x10 is now 0x" << hex << setfill('0') << setw(2) << value << endl;

    cout << "Printing memory from address 0x0 to address 0xff" << endl;
    myMem->printMemory(0, 0xff);

    RegisterInfo reg;

    dumpRegisterState(reg);
    dumpMemoryState(myMem);

    delete myMem;
    return 0;
}
