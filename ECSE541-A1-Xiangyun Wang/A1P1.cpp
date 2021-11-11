#include "systemc.h"
#include <fstream>
#include <string>

#include "A1P1.h"

using namespace std;

int memory[MEMORY_SIZE];
unsigned int input1Addr, input2Addr, outputAddr;

void memoryInit(string memfile) {
    // open the memory initialization file
    ifstream memfileStream(memfile);

    string entry = "";
    int i = 0;

    cout << "Initializing memory of size " << MEMORY_SIZE << endl;

    // iterate until we run out of memory or the file ends
    for (; i<MEMORY_SIZE; i++) {
        // get a token, delimited by a space
        if (getline(memfileStream, entry, ' ')) {
            // turn the token into an int and save it
            memory[i] = stoi(entry);
        }
        else {
            // leave the loop when the file ends
            break;
        }
    } // for

    // if the init file isn't big enough to initialize all memory locations,
    // fill the rest with zeroes
    for (; i<MEMORY_SIZE; i++)
        memory[i] = 0;

} // memoryInit

void memoryPrint(unsigned int start, unsigned int end) {
    // ensure end is valid
    if (end >= MEMORY_SIZE) {
        end = MEMORY_SIZE-1;
        cerr << "*** ERROR in memoryPrint: end > MEMORY_SIZE-1" << endl;
    }

    // print the requested values
    for (unsigned int i=start; i<end; i++) {
        cout << memory[i] << " ";
    }

    cout << endl;
} // memoryPrint

// ---------------- simple_mem_if interface ---------------------
class simple_mem_if : virtual public sc_interface{
  public:
    virtual bool Write(unsigned int addr, unsigned int data) = 0;
    virtual bool Read(unsigned int addr, unsigned int& data) = 0;
};

// --------------- memory --------------
class Memory : public sc_module, public simple_mem_if{
  public:
    Memory (sc_module_name name, char* mem_init_file) : sc_module(name){
      memoryInit(mem_init_file);
    }

    //write
    bool Write(unsigned int addr, unsigned int data){
      if (addr > MEMORY_SIZE) {   // if invalid address
          cerr << "*** ERROR in Write: invalid write address " << addr << "! Address out of bound! "<< endl;
          return false;
      }
      memory[addr] = data;      // otherwise, save data
      return true;
    }

    // read
    bool Read(unsigned int addr, unsigned int& data){
      if (addr > MEMORY_SIZE) {     // if invalid address
          cerr << "*** ERROR in Read: invalid read address " << addr << "! Address out of bound! " << endl;
          return false;
      }
      data = memory[addr];      // otherwise, read data
      return true;
    }
};

// ---------------------------- SAD -----------------------
class SAD : public sc_module{
  public:
    unsigned int address_offset;

    sc_port<Memory> transmit;   // communicate with memory
    SC_HAS_PROCESS(SAD);

    unsigned int sad, read1, read2;
    int v;
    bool read_flag_1, read_flag_2, write_flag;    // flags for read and write
    SAD (sc_module_name name) : sc_module(name){
      SC_THREAD(operate);
    }

    void operate(){
      for (int block=0; block<NUM_BLOCKS; block++){
              sad = 0;
              for (int i=0; i<BLOCK_SIZE; i++)
              {
                  address_offset = (block*BLOCK_SIZE)+i;
                  read_flag_1 = transmit->Read(input1Addr+address_offset,read1);    // read first data
                  read_flag_2 = transmit->Read(input2Addr+address_offset,read2);    // read second data
                  if(read_flag_1 && read_flag_2){   // if both data valid
                    v = read1 - read2;
                    if (v < 0) {
                      v = -v;
                    }
                    sad += v;
                  }
                }
                write_flag = transmit->Write(outputAddr+block,sad); // write back (dont really care if write is successful...)
                cout << "The result is: " << sad << endl;
            }
    }
  };

//----------- system (top) ---------------
  class System : public sc_module{
    public:
      Memory *memory_inst;    // memory
      SAD *SAD_inst;          // SAD

      System(sc_module_name name, char* mem_init_file) : sc_module(name){
        memory_inst = new Memory("Memory1",mem_init_file);
        SAD_inst = new SAD("SAD1");
        SAD_inst->transmit(*memory_inst); // connect
      }
  };

  int sc_main (int argc, char* argv[]){

    input1Addr = INPUT1_ADDR;
    input2Addr = INPUT2_ADDR;
    outputAddr = SAD_OUTPUT_ADDR;

    // check input values
    if (argc != 2 && argc != 4 && argc !=5) {
        cerr << "Usage: " << argv[0] << " <filename> [intput1Addr input2Addr [outputAddr]]" << endl;
        return 0;
    } // if

    // set custom addresses
    if (argc >= 4) {
        input1Addr = stoi(argv[2]);
        input2Addr = stoi(argv[3]);

        if (argc > 4)
            outputAddr = stoi(argv[4]);
    } // if

    System test("test",argv[1]);  // create test instance
    sc_start();                   // start!!!!!!!!!!!!!!!!!
    return 0;
  }
