#include "systemc.h"
#include <fstream>
#include <string>
#include <queue>
#include <string>
#include <iomanip>

//#define MEMORY_SIZE (1 << (10 + 6))
//#define DATA_LEN 32 // datalenth is 32 bit, 4 byte
//#define BUS_CAP 10  // bus is able to take 10 nodes
#define CLK_HALF_PERIOD 500
#define CLK_PERIOD 1000      // clock at 1 MHz
//#define FLIGHT_COMP_CODE {0,0,0,0,0,0,0,0,0,0,0} //message code for flight computer
//#define LAND_GEAR_CODE 001   //message code for landing gear
//#define SENSOR_CODE 010      //message code for sensor
//#include "header file"
// hey it's alfred testing git
//unsigned int memory[MEMORY_SIZE];
//unsigned int process_ID = 1;

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
  unsigned int IFS[3] = {}; //initialize to zeros
  unsigned int SOF[1] = {0};
  unsigned int base_ID[11];
  unsigned int SRR[1] = {1};
  unsigned int IDE[1] = {1};
  unsigned int extension_ID[18] = {}; //initialize to zeros
  unsigned int RTR[1] = {0};
  unsigned int control_field[6] = {0, 0, 0, 1, 0, 0};// datalength fixed at 4 byte
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
  Message(){

  }
};

//-------------
class Bus : public sc_module, public bus_master_if
{

public:
  sc_in<sc_logic> clk;
  sc_in<Message> msg_in;
  sc_out<Message> msg_out;
  sc_in<sc_logic> busy_in;
  sc_out<sc_logic> busy_out;

  SC_HAS_PROCESS(Bus);

  Bus(sc_module_name name) : sc_module(name)
  {
    SC_THREAD(bus_op);
      sensitive << clk.pos();
  }

  void bus_op()
  {
    while (true)
    {
      msg_out.write(msg_in.read());
    }
  }
};


struct Log{
  unsigned int time_stamp;
  unsigned int id;
  unsigned int data;

  Log(unsigned int time_stamp, unsigned int id, unsigned int data)
  {
    time_stamp = time_stamp;
    id = id;
    data = data;
  }
  Log(){
  	time_stamp = 0;
  	id = 0;
  	data = 0;
  }

};


class Memory : public sc_module
{
public:
  sc_in<sc_logic> clk;
  sc_in<int> addr;
  sc_in<sc_logic> read_flag;
  sc_in<sc_logic> write_flag;
  sc_in<Log> log_in;
  sc_out<Log> log_out;
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
      if (read_flag.read() == SC_LOGIC_1 && write_flag.read()==SC_LOGIC_0){//read
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
  sc_in<struct Message> msg_from_bus;
  sc_in<unsigned int> data_from_LRU;
  sc_in<unsigned int> id_from_LRU;
  sc_in<sc_logic> mode;     // ??????????????????????

  sc_out<sc_logic> listening; // ?????????????????????????
  sc_out<struct Message> msg_to_bus_ack;
  sc_out<struct Message> msg_to_bus_og;
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

  CAN_ctrl(sc_module_name name) : sc_module(name)
  {
    // counter = 0;
    base_ID = base_ID;
    // back_off = false;
    // SC_THREAD(ctrl_transmit);
    //   sensitive << clk.pos();
    SC_THREAD(ctrl_receive);
      sensitive << clk.pos();
  }

  struct Message Encode(unsigned int id, unsigned int data){
  	// TODO: construct CRC
  	struct Message msg = new Message(unsigned int *id, unsigned int *data, unsigned int *CRC, unsigned int *ACK);
  	return msg;
  }

  // CAN controller transmit message
  void WriteMessage(unsigned int id, unsigned int data){
    while(true){
    	msg_to_bus_og.write(Encode(id,data));
    	wait(CLK_PERIOD,SC_NS);
    	if(msg_from_bus.read()==Encode(id,data)){ // check if the bus content is same as transmitted
    		while(true){
    			wait(CLK_PERIOD,SC_NS);
    			if(msg_from_bus.read().ACK[0] == 0){ // acked!
    				return;
    			}
    			// if not acked, wait!
    		}
    	}
    	else{
    		wait(CLK_PERIOD*990,SC_NS,NEED_TO_CHECK!!!!!!!);

 		   	// time to transmit the entire CAN message
    	}
    }
  }

  // THREAD
  void ctrl_receive()
  {
    while (true)
    {
      wait(CLK_PERIOD,SC_NS);
      // listenting
      data_to_LRU.write(msg_from_bus.read().data); // decode the data and write to LRU
      id_to_LRU.write(msg_from_bus.read().base_ID);
      struct Message *message = msg_from_bus.read();// need to check
      unsigned int new_ACK[1] = {0};
      memcpy(message.ACK,new_ACK,32);
      msg_to_bus_ack.write(message);
    }
  }
  // void ack_msg(struct Message input_msg){
  // 	struct Message msg = new Message();
  // 	memcpy(&msg, &input_msg, sizeof(input_msg));
  // 	msg_to_bus_ack.write()
  // }


};

class Flight_computer : public sc_module
{
public:
  sc_port<bus_master_if> bus_port_fc;
  sc_in<sc_logic> clk;
  sc_signal<unsigned int> received_id;
  //sc_signal<unsigned int> transmit_id;
  sc_signal<unsigned int> received_data;
  //sc_signal<unsigned int> transmit_data;
  CAN_ctrl *ctrl_inst;
  SC_HAS_PROCESS(Flight_computer);
  unsigned int flight_comp_code[11] = {0,0,0,0,0,0,0,0,0,0,0};
  unsigned int landing_gear_code[11]= {0,0,1,0,0,0,0,0,0,0,0};
  unsigned int sensor_code[11] 		= {0,1,0,0,0,0,0,0,0,0,0};
  Flight_computer(sc_module_name name) : sc_module(name)
  {
    ctrl_inst = new CAN_ctrl("Flight_computer_CAN_ctrl");
    //ctrl_inst -> data_from_LRU(transmit_data);
    ctrl_inst -> data_to_LRU(received_data);
    ctrl_inst -> id_to_LRU(received_id);
    //ctrl_inst -> id_from_LRU(transmit_id);
    SC_THREAD(control);
      sensitive << clk;
  }

  void control(){
  	while(true){
  		wait(CLK_PERIOD,SC_NS);
  		if (compare_array(received_id, sensor_code, 11)){//data from sensor(need to check)
      		if (received_data == 500){//when height less than 500m
        		//tell landing gear to prepare: data = 9999
        		// transmit_data.write(9999);
        		// transmit_id.write(flight_comp_code);
        		bus_port_fc -> WriteMessage(flight_comp_code, 9999);
      		}
    	}
    	else if(compare_array(received_id, landing_gear_code, 11)){
    		if(received_data == 9999){
    			cout << "landing gear deployed successfully" << endl;
    		}
    	}
  	}

  }
};

class Landing_gear : public sc_module
{
public:
  sc_in<sc_logic> clk;
  sc_port<bus_master_if> bus_port_lg;
  sc_out<sc_lv<DATA_LEN> > data_out;
  sc_signal<unsigned int> received_id;
  //sc_signal<unsigned int> transmit_id;
  sc_signal<unsigned int> received_data;
  //sc_signal<unsigned int> transmit_data;
  CAN_ctrl *ctrl_inst;
  unsigned int flight_comp_code[11] = {0,0,0,0,0,0,0,0,0,0,0};
  unsigned int landing_gear_code[11]= {0,0,1,0,0,0,0,0,0,0,0};
  unsigned int sensor_code[11] 		= {0,1,0,0,0,0,0,0,0,0,0};
  SC_HAS_PROCESS(Landing_gear);

  Landing_gear(sc_module_name name) : sc_module(name)
  {
    ctrl_inst = new CAN_ctrl("Landing_gear_CAN_ctrl");
    ctrl_inst->data(received_data);
    ctrl_inst ->
    SC_THREAD(deployment);
    sensitive << clk.pos();
  }

  void deployment()
  {
    while (true)
    {
      // landing code = 10
    wait(CLK_PERIOD, SC_NS);
      if (compare_array(received_id, flight_comp_code, 11)){
      	if (received_data == 9999){
      		cout << "landing gear deploying!" << endl;
      		wait(CLK_PERIOD * 1000, SC_NS);
      		bus_port_lg -> WriteMessage(landing_gear_code, 9999);
      	}
      }
    }
  }
};

class Sensor : public sc_module
{
public:
  sc_port<bus_master_if> bus_port_sr;
  sc_in<sc_logic> clk;
  //sc_signal<unsigned int> received_id;
  //sc_signal<unsigned int> transmit_id;
  //sc_signal<unsigned int> received_data;
  //sc_signal<unsigned int> transmit_data;
  CAN_ctrl *ctrl_inst;
  unsigned int flight_comp_code[11] = {0,0,0,0,0,0,0,0,0,0,0};
  unsigned int landing_gear_code[11]= {0,0,1,0,0,0,0,0,0,0,0};
  unsigned int sensor_code[11] 		= {0,1,0,0,0,0,0,0,0,0,0};
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
      wait(CLK_PERIOD * 10, SC_NS);
      bus_port_sr -> WriteMessage(sensor_code,i);
    }
  }
};

bool compare_array(unsigned int *array1, unsigned int *array2, int length){
	for(int i=0;i<length;i++){
		if(array1[i] != array2[i]){
			return false;
		}
	}
	return true;
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
