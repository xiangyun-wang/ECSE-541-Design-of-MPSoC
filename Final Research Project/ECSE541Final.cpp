#include "systemc.h"
#include <fstream>
#include <string>
#include <queue>
#include <string>
#include <iomanip>


#define CLK_HALF_PERIOD 5
#define CLK_PERIOD 10      // clock at 100 MHz


class ctrl_interface : virtual public sc_interface
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

// -------------- struct types and custom functions ---------------------------------------

/* CAN message frame: 99 bits in total
    assume transmit one bit needs 1 clock cycle
    takes 100 clock cycles to transmit the message

*/
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

  Message(unsigned int base_ID, unsigned int data, unsigned int CRC, unsigned int ACK)
  {
    base_ID = base_ID;
    data = data;
    CRC = CRC;
    ACK = ACK;
  }
  Message(){

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

bool compare_array(unsigned int *array1, unsigned int *array2, int length){
  for(int i=0;i<length;i++){
    if(array1[i] != array2[i]){
      return false;
    }
  }
  return true;
}

// -------------- modules ---------------------------------------
class Bus : public sc_module
{

public:
  sc_in<sc_logic> clk;
  sc_in<Message> msg_in;

  sc_out<Message> msg_out;

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

/*

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

*/

// controller serves as transciver, encoder, and decoder, is submodule of LRUs 
class CAN_ctrl : public sc_module, ctrl_interface
{
public:
  sc_in<sc_logic> clk;
  sc_in<struct Message> msg_from_bus;
  //sc_in<unsigned int> data_from_LRU;
  //sc_in<unsigned int> id_from_LRU;
  //sc_in<sc_logic> mode;     // ??????????????????????

  sc_out<struct Message> msg_to_bus_ack;
  sc_out<struct Message> msg_to_bus_og;
  sc_out<unsigned int> id_to_LRU;
  sc_out<unsigned int> data_to_LRU;
  //sc_out<sc_logic> data_ready;

  bool back_off;
  int counter;

  SC_HAS_PROCESS(CAN_ctrl);

  CAN_ctrl(sc_module_name name) : sc_module(name)
  {
    SC_THREAD(ctrl_receive);
      sensitive << clk.pos();
  }

  // encode to write to bus
  struct Message Encode(unsigned int id, unsigned int data){
  	// TODO: construct CRC
  	CRC[15] = {};
    ACK[1] = {1};
    struct Message msg = new Message(unsigned int id, unsigned int data, unsigned int CRC, unsigned int ACK);
  	return msg;
  }

  // CAN controller encode and transmit message to bus
  void WriteMessage(unsigned int id, unsigned int data){
    while(true){

    	msg_to_bus_og.write(Encode(id,data));
      // bus does arbitration upon receiving the message
      wait(CLK_PERIOD, SC_NS);

      // check arbition result: if the bus content is same as transmitted
    	if(compare_array(msg_from_bus.read(),Encode(id,data),99)){ 
    		while(true){
    			wait(CLK_PERIOD,SC_NS);
          // check if the message written to bus is acked
    			if(msg_from_bus.read().ACK[0] == 0){  
    				return;
    			}
    			// if not acked, wait!
    		}
    	}

      // if loses arbition, wait for the other message on bus to transmit
    	else{
        // time to transmit the entire CAN message
    		wait(CLK_PERIOD*100,SC_NS);
    	}
    }
  }

  // THREAD
  void ctrl_receive()
  {
    while (true)
    {
      wait(CLK_PERIOD,SC_NS);
      // decode the data and write to LRU
      data_to_LRU.write(msg_from_bus.read().data); 
      id_to_LRU.write(msg_from_bus.read().base_ID);

      struct Message *message_read = msg_from_bus.read();// need to check
      unsigned int new_ACK[1] = {0};
      memcpy(message_read.ACK,new_ACK,32);
      msg_to_bus_ack.write(message);
    }
  }
};

// class Flight_computer_LRU: public sc_module{
// public:
//   sc_in<sc_logic> clk;

//   sc_in<struct Message> msg_from_bus;
//   sc_out<struct Message> msg_to_bus_ack;
//   sc_out<struct Message> msg_to_bus_og;
  
//   sc_signal<sc_logic> clk_relay;
//   sc_signal<struct Message> msg_to_bus_ack_relay;
//   sc_signal<struct Message> msg_to_bus_og_relay;
//   sc_signal<unsigned int> id_ctrl_to_proc;
//   sc_signal<unsigned int> data_ctrl_to_proc;
//   sc_signal<unsigned int> data_proc_to_ctrl;  

//   CAN_ctrl *ctrl_inst;
//   Flight_computer_processor *fc_proc_inst;

//   Flight_computer_LRU(sc_module_name name) : sc_module(name){
//     ctrl_inst = new CAN_ctrl("Flight_computer_CAN_ctrl");
//     fc_proc_inst = new Flight_computer_processor("Flight_computer_processor");

//     // interface port map
//     fc_proc_inst -> ctrl_port_fc(ctrl_inst);

//     // signal map: LRU contains both CAN ctrl and processor
//     ctrl_inst -> clk(clk);
//   }
// };

class Flight_computer_processor : public sc_module
{
public:
  // need to map the port in top
  sc_port<ctrl_interface> ctrl_port_fc;
  

  sc_in<sc_logic> clk;
  sc_in<unsigned int> id_from_ctrl;
  sc_in<unsigned int> data_from_ctrl;
  sc_out<unsigned int> id_to_ctrl;
  sc_out<unsigned int> data_to_ctrl;

  unsigned int flight_comp_id[11] = {0,0,0,0,0,0,0,0,0,0,0};
  unsigned int landing_gear_id[11]= {0,0,0,0,0,0,0,0,0,0,1};
  unsigned int sensor_id[11]    = {0,0,0,0,0,0,0,0,0,1,0};
  SC_HAS_PROCESS(Flight_computer);

  Flight_computer(sc_module_name name) : sc_module(name)
  {
    SC_THREAD(control);
      sensitive << clk;
  }

  void control(){
  	while(true){
  		wait(CLK_PERIOD,SC_NS);
      // check if the data comes from sensor
  		if (compare_array(id_from_ctrl.read(), sensor_code, 11)){
          // check if the data reached
      		if (data_from_ctrl.read() == 500){


        		ctrl_port_fc -> WriteMessage(flight_comp_code, 9999);
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
  sc_port<ctrl_interface> ctrl_port_lg;
  sc_in<sc_logic> clk;
  sc_in<struct Message> msg_from_bus;
  sc_out<struct Message> msg_to_bus_ack;
  sc_out<struct Message> msg_to_bus_og;

  sc_signal<unsigned int> received_id;
  sc_signal<unsigned int> received_data;

  CAN_ctrl *ctrl_inst;

  unsigned int flight_comp_code[11] = {0,0,0,0,0,0,0,0,0,0,0};
  unsigned int landing_gear_code[11]= {0,0,1,0,0,0,0,0,0,0,0};
  unsigned int sensor_code[11] 		= {0,1,0,0,0,0,0,0,0,0,0};
  SC_HAS_PROCESS(Landing_gear);

  Landing_gear(sc_module_name name) : sc_module(name)
  {
    ctrl_inst = new CAN_ctrl("Landing_gear_CAN_ctrl");

    // direct data pass through
    ctrl_inst -> clk(clk.read());
    ctrl_inst -> msg_from_bus(msg_from_bus.read());
    ctrl_inst -> msg_to_bus_ack(msg_to_bus_ack.read());
    ctrl_inst -> msg_to_bus_og(msg_to_bus_og.read());
    // received decoded message from controller
    ctrl_inst -> id_to_LRU(received_id);
    ctrl_inst -> data_to_LRU(received_data);
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
      		wait(CLK_PERIOD * 100, SC_NS);
      		ctrl_port_lg -> WriteMessage(landing_gear_code, 9999);
      	}
      }
    }
  }
};

class Sensor : public sc_module
{
public:
  sc_port<ctrl_interface> ctrl_port_sr;
  sc_in<sc_logic> clk;
  sc_in<struct Message> msg_from_bus;
  sc_out<struct Message> msg_to_bus_ack;
  sc_out<struct Message> msg_to_bus_og;

  CAN_ctrl *ctrl_inst;

  unsigned int flight_comp_code[11] = {0,0,0,0,0,0,0,0,0,0,0};
  unsigned int landing_gear_code[11]= {0,0,1,0,0,0,0,0,0,0,0};
  unsigned int sensor_code[11] 		= {0,1,0,0,0,0,0,0,0,0,0};
  SC_HAS_PROCESS(Sensor);

  Sensor(sc_module_name name) : sc_module(name)
  {
    // map to its controller
    ctrl_inst = new CAN_ctrl("Sensor_CAN_ctrl");
    // direct data pass through
    ctrl_inst -> clk(clk.read());
    ctrl_inst -> msg_from_bus(msg_from_bus.read());
    ctrl_inst -> msg_to_bus_ack(msg_to_bus_ack.read());
    ctrl_inst -> msg_to_bus_og(msg_to_bus_og.read());
    
    SC_THREAD(update_data);

    sensitive << clk.pos();
  }

  // simulate that the plane descend 1 m every 1000 clock cycles
  // remember that transmit data takes 100 clock cycles!
  void update_data()
  {
    for (int i = 1000; i > 0; i--)
    {
      wait(CLK_PERIOD * 1000, SC_NS);
      ctrl_port_sr -> WriteMessage(sensor_code,i);
      cout << "sensor data read: " << i << endl;
    }
  }
};



class System : public sc_module
{
public:
  Oscillator *oscillator;  
  Bus *bus;
  Flight_computer *fc_inst;
  Landing_gear *lg_inst;
  Sensor *sensor_inst;

  sc_signal<sc_logic> clk;
  sc_signal<struct Message> fc_message;
  sc_signal<struct Message> lg_message;
  sc_signal<struct Message> sensor_message;

  sc_signal<struct Message> win_message;
  sc_signal<struct Message> data_on_bus;


  System(sc_module_name name, char *mem_init_file) : sc_module(name)
  {
    oscillator = new Oscillator("OSC1");
    bus = new Bus("bus");
    fc_inst = new Flight_computer("fc1");
    lg_inst = new Landing_gear("lg1");
    sensor_inst = new Sensor("alt sensor");

    fc_inst -> ctrl_port_fc(*ctrl_inst)
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
