#include "systemc.h"
#include <fstream>
#include <string>
#include <queue>
#include <string>
#include <iomanip>

#define MEMORY_SIZE (1 << (10 + 6))
#define DATA_LEN 32 // datalenth is 32 bit, 4 byte
#define BUS_CAP 10  // bus is able to take 10 nodes
#define CLK_HALF_PERIOD 500
#define CLK_PERIOD 1000      // clock at 1 MHz
#define FLIGHT_COMP_CODE {0,0,0,0,0,0,0,0,0,0,0} //message code for flight computer
#define LAND_GEAR_CODE 001   //message code for landing gear
#define SENSOR_CODE 010      //message code for sensor
//#include "header file"
// hey it's alfred testing git
unsigned int memory[MEMORY_SIZE];
unsigned int process_ID = 1;

class bus_master_if : virtual public sc_interface
{
public:
  virtual void WriteMessage(Message msg) = 0;
};


// -------------- oscillator ---------------------------------------

class Oscillator : public sc_module
{
  // oscillator
public:
  sc_out<sc_logic> clk;

  SC_HAS_PROCESS(Oscillator);

  void oscillator_implementation()
  {
    while (true)
    {
      clk.write(SC_LOGIC_0);
      wait(CLK_HALF_PERIOD, SC_NS);
      clk.write(SC_LOGIC_1);
      wait(CLK_HALF_PERIOD, SC_NS);
    }
  }

  Oscillator(sc_module_name name) : sc_module(name)
  {
    SC_THREAD(oscillator_implementation);
  }
};

struct Message
{
  unsigned int IFS[3] = {};
  unsigned int SOF[1] = {0};
  unsigned int base_ID[11];
  unsigned int SRR[1] = {1};
  unsigned int IDE[1] = {1};
  unsigned int extension_ID[18] = {}; //initialize to zeros
  unsigned int RTR[1] = {0};
  unsigned int control_field[6] = {0, 0, 0, 1, 0, 0};
  unsigned int data;
  unsigned int CRC[15];
  unsigned int CRC_deli[1] = {1};
  unsigned int ACK[1] = {1};
  unsigned int ACK_deli[1] = {1};
  unsigned int end[7] = {1, 1, 1, 1, 1, 1, 1};

  Message(unsigned int *base_ID, unsigned int *data, unsigned int *CRC, unsigned int *ACK)
  {
    base_ID = base_ID;
    data = data;
    CRC = CRC;
    ACK = ACK;
  }
  // Message(){
  //   mst_id = 0;
  //   addr = 0;
  //   op = NONE;
  //   len = 0;
  // }
};

//-------------
class Bus : public sc_module, public bus_master_if, public bus_minion_if
{

public:
  sc_in<sc_logic> clk;
  sc_in<Message> data_in;
  sc_in<sc_logic> busy;
  sc_out<Message> data_out;

  SC_HAS_PROCESS(Bus);

  Bus(sc_module_name name) : sc_module(name)
  {
    SC_THREAD(arbiter);
      sensitive << clk.pos();
  }

  void arbiter()
  {
    while (true)
    {
      data_out.write(data_in.read());
    }
  }
};

struct Log{
  unsigned int time_stamp;
  unsigned int data;
};


class Memory : public sc_module
{
public:
  sc_in<sc_logic> clk;
  sc_in<int> addr;
  sc_in<sc_logic> read_flag;
  sc_in<sc_logic> write_flag;
  sc_in<Log> data_in;
  sc_out<Log> data_out;
  struct Log mem[2048];

  SC_HAS_PROCESS(Memory);

  Memory(sc_module_name name) : sc_module(name)
  {
    SC_THREAD(Memory_Access);
      sensitive << clk.pos();
  }

  void Memory_Access()
  {
    while (true)
    {
      if (read_flag.read()==SC_LOGIC_1&&write_flag.read()==SC_LOGIC_0){//read
        data_out.write(mem[addr.read()]);
      }
      else if(read_flag.read()==SC_LOGIC_0&&write_flag.read()==SC_LOGIC_1){//write
        mem[addr.read()] = data_in.read();
      }
    }
  }
};

// controller serves as transciver
class CAN_ctrl : public sc_module, bus_master_if
{
public:
  sc_in<sc_logic> clk;
  sc_in<Message> msg_from_bus;
  sc_in<unsigned int> data_from_LRU;
  sc_in<sc_logic> mode;     // 0 is write, 1 is read

  sc_out<sc_logic> listening; // enter listening mode once backoff
  sc_out<Message> msg_to_bus_ack;
  sc_out<Message> msg_to_bus_og;
  sc_out<unsigned int> id_to_LRU;
  sc_out<unsigned int> data_to_LRU;
  sc_out<sc_logic> data_ready;
  //sc_signal<sc_lv<9999>> can_msg; // construct CAN data frame message
  bool back_off;
  unsigned int base_ID[11];
  //bool data_ready;
  //bool bus_free; // need to know when bus is free
  int counter;

  SC_HAS_PROCESS(CAN_ctrl);

  CAN_ctrl(sc_module_name name, unsigned int base_ID[11]) : sc_module(name)
  {
    counter = 0;
    base_ID = base_ID;
    back_off = false;
    // SC_THREAD(ctrl_transmit);
    //   sensitive << clk.pos();
    SC_THREAD(ctrl_receive);
      sensitive << clk.pos();
  }

  void WriteMessage(Message msg){
    msg_to_bus_og.write(msg);
  }

  void ctrl_receive()
  {
    while (true)
    {
      wait(CLK_PERIOD,SC_NS);
      // listenting
      data_to_LRU.write(msg_from_bus.read().data);
      struct Message message = msg_from_bus.read();
      unsigned int new_ACK[1] = {0};
      memcpy(message.ACK,new_ACK,32);
      msg_to_bus_ack.write(message);
    }

  }

};

class Flight_computer : public sc_module
{
public:
  sc_in<sc_logic> clk;
  sc_signal<unsigned int> received_id;
  sc_signal<unsigned int> received_data;
  sc_signal<unsigned int> transmit_data;
  CAN_ctrl *ctrl_inst;
  SC_HAS_PROCESS(Flight_computer);
  unsigned int flight_comp_code[11] = {0,0,0,0,0,0,0,0,0,0,0};
  unsigned int sensor_code[11] = {0,1,0,0,0,0,0,0,0,0,0};
  Flight_computer(sc_module_name name) : sc_module(name)
  {
    ctrl_inst = new CAN_ctrl("Flight_computer_CAN_ctrl",flight_comp_code);
    ctrl_inst -> data_from_LRU(transmit_data);
    ctrl_inst -> data_to_LRU(received_data);
    ctrl_inst -> id_to_LRU(received_id);
    SC_THREAD(control);
      sensitive << clk;
  }

  void control(){
    if (received_id == sensor_code){//data from sensor
      if (received_data <= 500){//when height less than 500m
        //tell landing gear to prepare
        transmit_data
      }

    }
  }
};

class Landing_gear : public sc_module
{
public:
  sc_in<sc_logic> clk;
  sc_out<sc_lv<DATA_LEN> > data_out;
  sc_signal<sc_lv<DATA_LEN> > received_data;
  CAN_ctrl *ctrl_inst;
  SC_HAS_PROCESS(Landing_gear);

  Landing_gear(sc_module_name name) : sc_module(name)
  {
    ctrl_inst = new CAN_ctrl("Landing_gear_CAN_ctrl");
    ctrl_inst->data(received_data);
    SC_THREAD(deployment);
    sensitive << clk.pos();
  }

  void deployment()
  {
    while (true)
    {
      // landing code = 10
      if (received_data.to_int() == 10)
      {
        wait(CLK_PERIOD * 1000, SC_NS);
        // landing complete code = 100
        data_out.write(static_cast<sc_lv<DATA_LEN> > bitset<DATA_LEN>(100));
      }
    }
  }
};

class Sensor : public sc_module
{
public:
  sc_in<sc_logic> clk;
  sc_signal<sc_lv<DATA_LEN> > sensor_data;
  CAN_ctrl *ctrl_inst;
  SC_HAS_PROCESS(Sensor);

  Sensor(sc_module_name name) : sc_module(name)
  {
    // map to its controller
    ctrl_inst = new CAN_ctrl("Sensor_CAN_ctrl");
    ctrl_inst->data(sensor_data);
    // need to map all signal from controller

    SC_THREAD(update_data);

    sensitive << clk.pos();
  }

  // simulate that the plane descend 1 m every 100 clock cycles
  void update_data()
  {
    for (int i = 1000; i > 0; i--)
    {
      wait(CLK_PERIOD * 100, SC_NS);
      sensor_data.write(static_cast<sc_lv<DATA_LEN> > bitset<DATA_LEN>(i)); // really?
    }
  }
};

void convert_decimal_to_array(){
  bitset<32> A=N;//A will hold the binary representation of N
  for(int i=0,j=31;i<32;i++,j--)
  {
     //Assigning the bits one by one.
     O[i]=A[j];
  }
}

class System : public sc_module
{
public:
  sc_signal<sc_logic> clk;
  sc_lv<BUS_CAP> bit_buffer;

  Memory *memory;
  Hardware *hardware;
  Software *software;
  Bus *bus;
  Oscillator *oscillator;

  System(sc_module_name name, char *mem_init_file) : sc_module(name)
  {
    for (int i = 0; i < BUS_CAP; i++)
    {
      bit_buffer.set_bit(i, SC_LOGIC);
    }

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

int sc_main(int argc, char *argv[])
{

  if (argc != 2 && argc != 5 && argc != 6 && argc != 7)
  {
    cerr << "Usage: " << argv[0] << " <filename> [[[addrC addrA addrB] size] loops]" << endl;
    return 0;
  }

  if (argc > 2)
  {
    MatrixC_Addr = stoi(argv[2]);
    MatrixA_Addr = stoi(argv[3]);
    MatrixB_Addr = stoi(argv[4]);
    if (argc > 5)
    {
      matrix_size = stoi(argv[5]);
      if (argc > 6)
      {
        loops = stoi(argv[6]);
      }
    }
  }

  cache_size = matrix_size * matrix_size;
  unsigned int golden_answer[cache_size];

  System test("test", argv[1]);

  sc_start();

  for (int i = 0; i < matrix_size; i++)
  {
    for (int j = 0; j < matrix_size; j++)
    {
      golden_answer[i * matrix_size + j] = 0;
      for (int k = 0; k < matrix_size; k++)
      {
        golden_answer[i * matrix_size + j] += memory[i * matrix_size + k + MatrixA_Addr] * memory[k * matrix_size + j + MatrixB_Addr];
      }
    }
  }
  cout << "Golden Answer: " << endl;

  int counter = 0;

  for (int i = 0; i < cache_size; i++)
  {
    cout << golden_answer[i] << " ";
    counter++;
    if (counter == matrix_size)
    {
      cout << endl;
      counter = 0;
    }
  }
  cout << endl;

  counter = 0;
  cout << "Actual Answer: " << endl;
  for (int i = 0; i < matrix_size; i++)
  {
    for (int j = 0; j < matrix_size; j++)
    {
      cout << memory[i * matrix_size + j + MatrixC_Addr] << " ";
      counter++;
      if (counter == matrix_size)
      {
        cout << endl;
        counter = 0;
      }
    }
  }
  cout << endl;

  cout << "Time Consumed: " << setprecision(8) << sc_time_stamp().to_seconds() * 1e9 << "(ns)" << endl;
  cout << "Time Consumed: " << setprecision(8) << sc_time_stamp().to_seconds() * 1e9 / CLK_PERIOD << "(cc)" << endl;

  return 0;
}
