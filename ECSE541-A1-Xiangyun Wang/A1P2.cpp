#include "systemc.h"
#include <fstream>
#include <string>

#include "A1P2.h"

using namespace std;

unsigned int input1Addr, input2Addr, outputAddr;  //addresses

bool finish_flag = false;   // finish flag for stopping oscillator thread

void memoryInit(string memfile, unsigned int memory[MEMORY_SIZE]) {
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

void memoryPrint(unsigned int start, unsigned int end, unsigned int memory[MEMORY_SIZE]) {
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

// -----------------simple_mem_if class------------------
class simple_mem_if : virtual public sc_interface{
  public:
    virtual bool Write(unsigned int addr, unsigned int data) = 0;
    virtual bool Read(unsigned int addr, unsigned int& data) = 0;
};

// -----------------------memory rtl-------------------------
class MEMORY_RTL : public sc_module{
  public:
    sc_in<sc_logic> Clk;        // clock signal
    sc_in<sc_logic> Ren, Wen;   // read and write enable
    sc_in<int> Addr;            // address
    sc_in<int> DataIn;          // data into memory rtl
    sc_out<int> DataOut;        // data out from memory rtl
    sc_out<sc_logic> Ack;       // flag for operation validity

    unsigned int memData[MEMORY_SIZE];    // memory array

    SC_HAS_PROCESS(MEMORY_RTL);

    MEMORY_RTL(sc_module_name name, char* memInitFilename) : sc_module(name){
      memoryInit(memInitFilename,memData);  // initialize memory
      SC_METHOD(rtl);
      sensitive << Clk.pos();               // sensitive to positive edge
    }

    void rtl(){
      if (Addr.read() > MEMORY_SIZE) {          // if address invalid
          Ack.write(SC_LOGIC_0);                // set Ack to low
      }else{
        if(Ren.read() == SC_LOGIC_1){           // when read
          DataOut.write(memData[Addr.read()]);  // put read data on bus
          Ack.write(SC_LOGIC_1);                // set Ack to high
        }else if(Wen.read() == SC_LOGIC_1){     // when write
          memData[Addr.read()] = DataIn.read(); // write data to mem
          Ack.write(SC_LOGIC_1);                //set Ack to high
        }else{
          Ack.write(SC_LOGIC_0);                // set Ack to low otherwise
        }
      }
    }
};

// ------------ Memory ----------------------
class Memory : public sc_module, public simple_mem_if{
  public:
    MEMORY_RTL *memory_rtl_inst;        // memory rtl instance
    sc_signal<sc_logic> clk;            // signals to connect rtl
    sc_signal<sc_logic> Ren, Wen;
    sc_signal<int> Addr;
    sc_signal<int> DataIn;
    sc_signal<int> DataOut;
    sc_signal<sc_logic> Ack;

    SC_HAS_PROCESS(Memory);

    Memory (sc_module_name name, char* mem_init_file) : sc_module(name){
      SC_THREAD(oscillator);
      memory_rtl_inst = new MEMORY_RTL("memory_rtl_1",mem_init_file);   // create new memory rtl
      memory_rtl_inst->Clk(clk);        // port map
      memory_rtl_inst->Ren(Ren);
      memory_rtl_inst->Wen(Wen);
      memory_rtl_inst->Addr(Addr);
      memory_rtl_inst->DataIn(DataIn);
      memory_rtl_inst->DataOut(DataOut);
      memory_rtl_inst->Ack(Ack);
    }

    //write
    bool Write(unsigned int addr, unsigned int data){
      Wen.write(SC_LOGIC_1);      // write enable to high
      Addr.write(addr);           // give address
      DataIn.write(data);         // give data to write
      wait(CLK_PERIOD, SC_NS);    // wait for 1 cc to get the stable Ack
      if(Ack.read()==SC_LOGIC_1){ // read success
        Wen.write(SC_LOGIC_0);    // reset write enable
        //cout << "Write Success!" <<endl;
        return true;
      }else{
        Wen.write(SC_LOGIC_0);      //read fail
        cout << "Memory Error! Write Address Invalid!" <<endl;  // warning
        return false;
      }
    }

    // read
    // same logic as Write() implementation
    bool Read(unsigned int addr, unsigned int& data){

      Ren.write(SC_LOGIC_1);    // read enable flag
      Addr.write(addr);         // address
      wait(CLK_PERIOD, SC_NS);    // wait for 1 cc to get the stable Ack
      if(Ack.read()==SC_LOGIC_1){   // read success
        data = DataOut.read();
        Ren.write(SC_LOGIC_0);    // reset flag
        //cout << "Read Success!" <<endl;
        return true;
      }else{                      // read fail
        Ren.write(SC_LOGIC_0);
        cout << "Memory Error! Read Address Invalid!" <<endl; // warning
        return false;
      }
    }

    void oscillator(){
        while(!finish_flag){              // if not finished
          clk.write(SC_LOGIC_0);
          wait(CLK_HALF_PERIOD,SC_NS);
          clk.write(SC_LOGIC_1);
          wait(CLK_HALF_PERIOD,SC_NS);
        }
    }
};


// ------------- SAD -------------------
class SAD : public sc_module{
  public:
    unsigned int address_offset;

    sc_port<Memory> transmit;
    SC_HAS_PROCESS(SAD);

    unsigned int sad, read1, read2;

    int v;

    bool read_flag_1, read_flag_2, write_flag;

    SAD (sc_module_name name) : sc_module(name){
      SC_THREAD(operate);
    }

    void operate(){
      for (int block=0; block<NUM_BLOCKS; block++){
              wait(10, SC_NS); // delay for block incrementation
              sad = 0;
              for (int i=0; i<BLOCK_SIZE; i++)
              {
                  wait(10, SC_NS);  // delay for block size incrementation
                  wait(10, SC_NS); // for loop comparison
                  wait(10, SC_NS);  // delay for multiplication
                  wait(10, SC_NS);  // delay for addition
                  address_offset = (block*BLOCK_SIZE)+i;

                  wait(10, SC_NS);  // delay for addition
                  read_flag_1 = transmit->Read(input1Addr+address_offset,read1);

                  wait(10, SC_NS);  // delay for addition
                  read_flag_2 = transmit->Read(input2Addr+address_offset,read2);

                  wait(10, SC_NS);  // delay for logical AND
                  wait(10, SC_NS);  // delay for comparison
                  if(read_flag_1 && read_flag_2){
                    wait(10, SC_NS);  // delay for subtraction
                    v = read1 - read2;
                    wait(10, SC_NS);  // delay for comparison
                    if (v < 0) {
                      wait(10, SC_NS);  // delay for negate, if true
                      v = -v;
                    }
                    wait(10, SC_NS);  // delay for addition of sad
                    sad += v;
                  }
                }

                wait(10, SC_NS);  // delay for addition of write address
                write_flag = transmit->Write(outputAddr+block,sad);
                cout << "The result is: " << sad << endl;
            }
            finish_flag = true;
       }
  };

// ------------- System (Top) ----------------------
  class System : public sc_module{
    public:
      Memory *memory_inst;    // memory instance
      SAD *SAD_inst;          // SAD instance

      System(sc_module_name name, char* mem_init_file) : sc_module(name){
        memory_inst = new Memory("Memory1",mem_init_file);
        SAD_inst = new SAD("SAD1");
        SAD_inst->transmit(*memory_inst);     // port map
      }
  };

//----------- main -----------------
  int sc_main (int argc, char* argv[]){

    input1Addr = INPUT1_ADDR;
    input2Addr = INPUT2_ADDR;
    outputAddr = SAD_OUTPUT_ADDR;

    // check input
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

    System test("test",argv[1]);  // system instance

// ---------------- signal for debug ------------------------
    sc_trace_file *wf = sc_create_vcd_trace_file("WaveForm");
    sc_trace(wf, test.memory_inst->Ren, "ren");
    sc_trace(wf, test.memory_inst->Wen, "wen");
    sc_trace(wf, test.memory_inst->clk, "clk");
    sc_trace(wf, test.memory_inst->Ack, "Ack");

    sc_start();     // start !!!!!!

    sc_close_vcd_trace_file(wf);
    return 0;
  }
