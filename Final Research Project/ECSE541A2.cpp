#include "systemc.h"
#include <fstream>
#include <string>
#include <queue>
#include <string>
#include <iomanip>

#define MEMORY_SIZE (1 << (10+6))
#define IO_ONLY 5
#define ADDR_A 5
#define ADDR_B 30
#define ADDR_C 55
#define LOOPS 10
#define CLK_HALF_PERIOD 3.333
#define CLK_PERIOD 6.666
#define MATRIX_DEFAULT_SIZE 5
#define CACHE_SIZE 100

// opcode
#define R 0 // read
#define W 1 // write
#define C 2 // calculate
#define I 3 // idle
#define NONE 4

//#include "header file"

using namespace std;

unsigned int memory[MEMORY_SIZE];
int cache_size = MATRIX_DEFAULT_SIZE*MATRIX_DEFAULT_SIZE;
int matrix_size = MATRIX_DEFAULT_SIZE;
int loops = LOOPS;
unsigned int process_ID = 1;

unsigned int MatrixA_Addr = ADDR_A;
unsigned int MatrixB_Addr = ADDR_B;
unsigned int MatrixC_Addr = ADDR_C;

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


class bus_master_if : virtual public sc_interface
{
  public:
    virtual void Request(unsigned int mst_id, unsigned int addr, unsigned int op, unsigned int len) = 0;
    virtual bool WaitForAcknowledge(unsigned int mst_id) = 0;
    virtual void ReadData(unsigned int &data) = 0;
    virtual void WriteData(unsigned int data) = 0;

    virtual void AckFinish(bool status) = 0;
    virtual bool WaitForFinish() = 0;
};


// Bus Servant Interface
class bus_minion_if : virtual public sc_interface
{
  public:
    virtual void Listen(unsigned int &req_addr, unsigned int &req_op, unsigned int &req_len) = 0;
    virtual void Acknowledge(bool status) = 0;
    virtual void SendReadData(unsigned int data) = 0;
    virtual void ReceiveWriteData(unsigned int &data) = 0;
};

// -------------- oscillator ---------------------------------------

class Oscillator: public sc_module{
  // oscillator
  public :
    sc_out<sc_logic> clk;

    SC_HAS_PROCESS(Oscillator);

    void oscillator_implementation(){
      while(true){
        clk.write(SC_LOGIC_0);
        wait(CLK_HALF_PERIOD,SC_NS);
        clk.write(SC_LOGIC_1);
        wait(CLK_HALF_PERIOD,SC_NS);
      }
    }

    Oscillator(sc_module_name name):sc_module(name){
      SC_THREAD(oscillator_implementation);
    }

};

struct Task_Info{
  unsigned int mst_id;
  unsigned int addr;
  unsigned int op;
  unsigned int len;

  Task_Info(unsigned int local_mstid, unsigned int local_addr, unsigned int local_op, unsigned int local_len){
    mst_id = local_mstid;
    addr = local_addr;
    op = local_op;
    len = local_len;
  }
  Task_Info(){
    mst_id = 0;
    addr = 0;
    op = NONE;
    len = 0;
  }
};

//-------------
class Bus : public sc_module, public bus_master_if, public bus_minion_if{

public:

  std::queue<unsigned int> data_buffer;
  std::queue<struct Task_Info> task, ack_task; //, idle_task
  std::queue<bool> ack_status, finish_status;

  int target_transfer_counter; //= 0;
  int actual_transfer_counter; //= 0;
  bool bus_is_busy;

  sc_in<sc_logic> clk;

  struct Task_Info current_task;// = Task_Info(0,0,NONE,0);  // initialized to invalid task

  int request_count;

  SC_HAS_PROCESS(Bus);

  Bus (sc_module_name name) : sc_module(name){
    current_task = Task_Info(0,0,NONE,0);
    bus_is_busy = false;
    request_count = 0;
    SC_THREAD(arbiter); // implement below
      sensitive << clk.pos();
  }

  void arbiter(){
    while(true){
      wait(CLK_PERIOD,SC_NS);
      wait(CLK_PERIOD,SC_NS);
      if(!task.empty()){
        if(!bus_is_busy){
          if(current_task.mst_id!=0){
            task.push(current_task);
          }
          current_task = task.front();
          task.pop();
        }
      }
    }
  }

  void Request(unsigned int mst_id, unsigned int addr, unsigned int op, unsigned int len){
    // wait 2  cycles
    wait(CLK_PERIOD,SC_NS);
    wait(CLK_PERIOD,SC_NS);
    if(request_count == 0){
      //TODO: // give this to current_task
      current_task = Task_Info(mst_id, addr, op, len);
    }else{
      task.push(Task_Info(mst_id, addr, op, len));
    }
    cout << "New Request ID = " << mst_id << endl;
    request_count++;
  }

  bool WaitForAcknowledge(unsigned int mst_id){
    while(true){
      wait(CLK_PERIOD,SC_NS);
      if(!ack_task.empty() && !bus_is_busy){
        Task_Info check = ack_task.front();
        if (check.mst_id == mst_id){
          target_transfer_counter = check.len;
          actual_transfer_counter = 0;
          if(check.op == C){
            bus_is_busy = false;
            //idle_task.push(idle_task.pop());
          }else{
            bus_is_busy = true;
          }
          ack_task.pop(); // !!!
          bool output = ack_status.front();
          ack_status.pop();
          return output;
        }
      }
    }
  }

  void ReadData(unsigned int &data){
    wait(CLK_PERIOD,SC_NS);
    wait(CLK_PERIOD,SC_NS);
    while(data_buffer.empty()){
      wait(CLK_PERIOD,SC_NS);
    }
    data = data_buffer.front();
    data_buffer.pop();
    actual_transfer_counter++;
    if(actual_transfer_counter==target_transfer_counter){
      bus_is_busy = false;
    }
  }

  void WriteData(unsigned int data){
    wait(CLK_PERIOD,SC_NS);
    data_buffer.push(data);
  }

// need to be in a while loop, keep listening
  void Listen(unsigned int &req_addr, unsigned int &req_op, unsigned int &req_len){
    req_addr = current_task.addr;
    req_len = current_task.len;
    req_op = current_task.op;
  }

  void Acknowledge(bool status){
    wait(CLK_PERIOD,SC_NS);
    ack_task.push(current_task);
    ack_status.push(status);
    request_count--;
    // reset current_task (make current task invalid)
    current_task.mst_id = 0;
    current_task.op = NONE;
  }

  void SendReadData(unsigned int data){
    data_buffer.push(data);
  }

  void ReceiveWriteData(unsigned int &data){
    while(data_buffer.empty()){
      wait(CLK_PERIOD,SC_NS);
    }
    data = data_buffer.front();
    data_buffer.pop();
    actual_transfer_counter++;
    if(actual_transfer_counter==target_transfer_counter){
      bus_is_busy = false;
    }
  }
//-------------------------- extra ----------------------
  bool WaitForFinish(){
    while(true){
      wait(CLK_PERIOD,SC_NS);
      if(!finish_status.empty()){
        bool output = finish_status.front();
        finish_status.pop();
        return output;
      }
    }
  }

  void AckFinish(bool status){
    finish_status.push(status);
  }

};

class Memory: public sc_module{
public:

  sc_port<bus_minion_if> bus_mem_interface;
  sc_in<sc_logic> clk;

  unsigned int addr, op, len;

  bool valid_access;// = false;

  SC_HAS_PROCESS(Memory);

  Memory (sc_module_name name, char* mem_init_file) : sc_module(name){
    valid_access = false;
    memoryInit(mem_init_file);
    SC_THREAD(Memory_Access);
      sensitive << clk.pos();
  }

  void Memory_Access(){
    while(true){
      //wait(CLK_PERIOD,SC_NS);
      valid_access = false;
      while(true){
        wait(CLK_PERIOD,SC_NS);
        bus_mem_interface->Listen(addr,op,len);
        if(op==R || op==W){ // check if anyone making request to memory
          break;
        }
      }
      if(addr>=0 && addr+len-1<=MEMORY_SIZE){ // if memory access in range
        cout << "Memory: Access Acked"<< endl;
        bus_mem_interface->Acknowledge(true);
        valid_access = true;
      }else{
        bus_mem_interface->Acknowledge(false);
      }
      if(valid_access){
        for (int i = 0; i<len; i++){
          if(op==R){
            bus_mem_interface->SendReadData(memory[addr+i]);
          }else{
            bus_mem_interface->ReceiveWriteData(memory[addr+i]);
          }
        }
      }
    }
  }
};

class Hardware: public sc_module{
public:
  sc_port<bus_minion_if> slave;
  sc_port<bus_master_if> master;
  sc_in<sc_logic> clk;

  unsigned int addr, op, len;
  unsigned int command_addr;

  bool io_access;// = false;
  bool memory_access_A;// = false;
  bool memory_access_B;// = false;
  bool memory_access_C;
  bool calculate;// = false;

  unsigned int cache_A[CACHE_SIZE];
  unsigned int cache_B[CACHE_SIZE];
  unsigned int cache_C[CACHE_SIZE];

  unsigned int location[3];
  unsigned int size;

  unsigned int ID;

  SC_HAS_PROCESS(Hardware);

  Hardware (sc_module_name name) : sc_module(name){
    io_access = false;
    memory_access_A = false;
    memory_access_B = false;
    memory_access_C = false;
    calculate = false;
    SC_THREAD(function)
      sensitive << clk.pos();
    // SC_THREAD()
    //   sensitive << Clk.pos();
  }

  void function(){
    //wait for 1 cc
    //wait(CLK_PERIOD,SC_NS);
    while(true){
      // slave mode
      while(true){
        wait(CLK_PERIOD,SC_NS);
        io_access = false;
        slave->Listen(addr,op,len);
        if(op==C){
          cout << "Hardware: Received Requested to Calculate"<< endl;
          command_addr = addr;
          break;
        }
      }
      if(addr >= 0 && addr <= IO_ONLY && len == 4){
        io_access = true;
        cout << "Hardware: Acked to Calculate"<< endl;
        slave->Acknowledge(true);
      }else{
        slave->Acknowledge(false);
      }

      // master mode, access command from reserved IO
      if(io_access == true){
        // ready to access data from memory
        ID = process_ID;
        process_ID++;
        cout << "Hardware: Request to Access Command"<< endl;
        master->Request(ID,command_addr,R,4);
        if(master->WaitForAcknowledge(ID)){
          cout << "Hardware: Acked to Access Command"<< endl;
          for (int i = 0; i<3;i++){
            master->ReadData(location[i]);
          }
          master->ReadData(size);
          memory_access_C = true;
        }else{
          memory_access_C = false;
        }
      }

      if(memory_access_C){
        ID = process_ID;
        process_ID++;
        cout << "Hardware: Request to Reset Memory"<< endl;
        master->Request(ID,location[2],W,size*size);
        if(master->WaitForAcknowledge(ID)){
          cout << "Hardware: Acked to Reset Memory"<< endl;
          for (int i = 0; i<size*size; i++){
            master->WriteData(0);
          }
        }
        memory_access_A = true;
      }else{
        memory_access_A = false;
      }

      // first matrix access
      if(memory_access_A){
        ID = process_ID;
        process_ID++;
        //cout << "size = " << size << endl;
        cout << "Hardware: Request to Access Matrix A"<< endl;
        master->Request(ID,location[0],R,size*size);
        if(master->WaitForAcknowledge(ID)){
          cout << "Hardware: Acked to Access Matrix A"<< endl;
          for (int i = 0; i<size*size; i++){
             master->ReadData(cache_A[i]);
          }
          memory_access_B = true;
        }else{
          memory_access_B = false;
        }
      }
      // second matrix access
      if(memory_access_B){
        ID = process_ID;
        process_ID++;
        cout << "Hardware: Request to Access Matrix B"<< endl;
        master->Request(ID,location[1],R,size*size);
        if(master->WaitForAcknowledge(ID)){
          cout << "Hardware: Acked to Access Matrix B"<< endl;
          for (int i = 0; i<size*size; i++){
            master->ReadData(cache_B[i]);
          }
          calculate = true;
        }else{
          calculate = false;
        }
      }

      // calculation (acceleration)
      if(calculate){
        for(int i = 0; i<size; i++){
          for(int j = 0; j<size; j++){
            cache_C[i*size+j] = 0;
            for (int k = 0; k<size; k++){
              cache_C[i*size+j] += cache_A[i*size+k] * cache_B[k*size+j];
            }
          }
        }
      }

      if(calculate){
        ID = process_ID;
        process_ID++;
        cout << "Hardware: Request to Access Matrix C"<< endl;
        master->Request(ID,location[2],W,size*size);
        if(master->WaitForAcknowledge(ID)){
          cout << "Hardware: Ack to Access Matrix C"<< endl;
          for (int i = 0; i<size*size; i++){
            master->WriteData(cache_C[i]);
          }
        }
        cout << "Hardware: Task Finished"<< endl;
        master->AckFinish(true);
      }else{
        cout << "Hardware: Task Failed"<< endl;
        master->AckFinish(false);
      }
    }
  }
};

class Software: public sc_module{
public:

  sc_port<bus_master_if> master;
  sc_in<sc_logic> clk;
  unsigned int ID;

  SC_HAS_PROCESS(Software);

  Software(sc_module_name name) : sc_module(name){
    SC_THREAD(With_Hardware);
    //SC_THREAD(Without_Hardware);
      sensitive << clk.pos();
  }

  void Without_Hardware(){
    for(int a = 0; a<loops; a++){
      ID = process_ID;
      process_ID++;
      master->Request(ID,MatrixC_Addr,W,matrix_size*matrix_size);
      if(master->WaitForAcknowledge(ID)){
        for (int i = 0; i<matrix_size*matrix_size; i++){
          master->WriteData(0);
        }
      }
      unsigned int cache1, cache2, cache3;

      for(int i = 0; i<matrix_size; i++){
        for(int j = 0; j<matrix_size; j++){
          cache3 = 0;
          for (int k = 0; k<matrix_size; k++){
            ID = process_ID;
            process_ID++;
            master->Request(ID,MatrixA_Addr+i*matrix_size+k,R,1);
            if(master->WaitForAcknowledge(ID)){
              master->ReadData(cache1);
            }
            ID = process_ID;
            process_ID++;
            master->Request(ID,MatrixB_Addr+k*matrix_size+j,R,1);
            if(master->WaitForAcknowledge(ID)){
              master->ReadData(cache2);
            }
            cache3 += cache1*cache2;
          }
          ID = process_ID;
          process_ID++;
          master->Request(ID,MatrixC_Addr+i*matrix_size+j,W,1);
          if(master->WaitForAcknowledge(ID)){
            master->WriteData(cache3);
          }
        }
      }
    }
    sc_stop();
  }

  void With_Hardware(){
    for (int i=0; i<loops; i++){
      cout << "==================LOOP "<< i <<"==================" << endl;
      ID = process_ID;
      process_ID++;
      unsigned int command[4] = {MatrixA_Addr,MatrixB_Addr,MatrixC_Addr,matrix_size};
      master->Request(ID,0,W,4);
      cout << "Software: Request Write to Memory"<< endl;
      if(master->WaitForAcknowledge(ID)){
        cout << "Software: Request Acknowledged"<< endl;
        for (int i = 0; i<4; i++){
          master->WriteData(command[i]);
        }
      }
      ID = process_ID;
      process_ID++;
      master->Request(ID,0,C,4);
      if(master->WaitForAcknowledge(ID)){
        if(master->WaitForFinish()){
          cout << "Software: Task Finished" << endl;
        }else{
          cout << "Software: Task FAILLED" << endl;
        }
      }else{
        cout << "Software: Task FAILLED" << endl;
      }
    }
    sc_stop();
  }
};

class System : public sc_module{
public:
  sc_signal<sc_logic> clk;

  Memory *memory;
  Hardware *hardware;
  Software *software;
  Bus *bus;
  Oscillator *oscillator;

  System(sc_module_name name, char* mem_init_file) : sc_module(name){
    memory = new Memory("MEM1", mem_init_file);
    hardware = new Hardware("HW1");
    software = new Software("SW1");
    bus = new Bus("BUS1");
    oscillator = new Oscillator("OSC1");

    memory->bus_mem_interface(*bus);
    hardware->master(*bus);
    hardware->slave(*bus);
    software->master(*bus);
    bus->clk(clk);
    memory->clk(clk);
    hardware->clk(clk);
    software->clk(clk);
    oscillator->clk(clk);
    cout << "System Start!" << endl;
  }
};

int sc_main(int argc, char* argv[]){

  if(argc != 2 && argc != 5 && argc != 6 && argc != 7){
    cerr << "Usage: " << argv[0] << " <filename> [[[addrC addrA addrB] size] loops]" << endl;
    return 0;
  }

  if(argc>2){
    MatrixC_Addr = stoi(argv[2]);
    MatrixA_Addr = stoi(argv[3]);
    MatrixB_Addr = stoi(argv[4]);
    if(argc>5){
      matrix_size = stoi(argv[5]);
      if(argc>6){
        loops = stoi(argv[6]);
      }
    }
  }

  cache_size = matrix_size*matrix_size;
  unsigned int golden_answer[cache_size];

  System test("test",argv[1]);

  sc_start();

  for(int i = 0; i<matrix_size; i++){
    for(int j = 0; j<matrix_size; j++){
      golden_answer[i*matrix_size+j] = 0;
      for (int k = 0; k<matrix_size; k++){
        golden_answer[i*matrix_size+j] += memory[i*matrix_size+k+MatrixA_Addr] * memory[k*matrix_size+j+MatrixB_Addr];
      }
    }
  }
  cout <<  "Golden Answer: " <<endl;

  int counter = 0;

  for(int i = 0; i<cache_size; i++){
    cout << golden_answer[i] << " " ;
    counter++;
    if(counter == matrix_size){
      cout << endl;
      counter = 0;
    }
  }
  cout << endl;

  counter = 0;
  cout <<  "Actual Answer: " <<endl;
  for(int i = 0; i<matrix_size; i++){
    for(int j = 0; j<matrix_size; j++){
      cout << memory[i*matrix_size+j+MatrixC_Addr] << " " ;
      counter++;
      if(counter == matrix_size){
        cout << endl;
        counter = 0;
      }
    }
  }
  cout << endl;

  cout << "Time Consumed: " << setprecision(8) << sc_time_stamp().to_seconds()*1e9 << "(ns)" << endl;
  cout << "Time Consumed: " << setprecision(8) << sc_time_stamp().to_seconds()*1e9/CLK_PERIOD << "(cc)" << endl;


  return 0;

}
