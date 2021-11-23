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
// ------------ message struct -------------------------------------
/* CAN message frame: 99 bits in total
    assume transmit one bit needs 1 clock cycle
    takes 100 clock cycles to transmit the message
*/
struct Message
{
  unsigned int IFS[3] = {}; //initialize to zeros
  unsigned int SOF[1] = {0};
  unsigned int base_ID;
  unsigned int SRR[1] = {1};
  unsigned int IDE[1] = {1};
  unsigned int extension_ID[18] = {}; //initialize to zeros
  unsigned int RTR[1] = {0};
  unsigned int control_field[6] = {0, 0, 0, 1, 0, 0};// datalength fixed at 4 byte
  unsigned int data;
  unsigned int CRC;
  unsigned int CRC_deli[1] = {1};
  bool ACK;
  unsigned int ACK_deli[1] = {1};
  unsigned int end[7] = {1, 1, 1, 1, 1, 1, 1};

  Message(unsigned int base_ID, unsigned int data, unsigned int CRC, bool ACK)
  {
    base_ID = base_ID;
    data = data;
    CRC = CRC;
    ACK = ACK;
  }
  Message(){
    ACK = false;
  }
};

bool cmpmsg(struct Message msg1, struct Message msg2){    // ------------------------------ need to implement----------------
  return true;
}
// -------------- end of message struct -------------------------

//---------------------- log struct --------------------------
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

// end fo log struct

// -------------- modules ---------------------------------------
// ------------BUS -----------------------
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
      wait(CLK_PERIOD,SC_NS);
      msg_out.write(msg_in.read());
    }
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



// controller serves as transciver, encoder, and decoder, is submodule of LRUs
class CAN_ctrl : public sc_module, ctrl_interface
{
public:
  sc_in<sc_logic> clk;
  sc_in<struct Message> msg_from_bus;
  sc_in<unsigned int> data_from_proc;
  sc_in<unsigned int> id_from_proc;
  // whichever the following two messages going onto the bus will be determined in the top module
  sc_out<struct Message> msg_to_bus_ack;
  sc_out<struct Message> msg_to_bus_og;
  sc_out<unsigned int> id_to_proc;
  sc_out<unsigned int> data_to_proc;

  SC_HAS_PROCESS(CAN_ctrl);

  CAN_ctrl(sc_module_name name) : sc_module(name)
  {
    SC_THREAD(ctrl_receive);
      sensitive << clk.pos();
  }

  // CAN controller encode and transmit message to bus
  //void WriteMessage(unsigned int id, unsigned int data){
  void WriteMessage(){
    while(true){
      wait(CLK_PERIOD, SC_NS);
      // encode (recalculate each time, to avoid outdate data)--------- need to implement
      CRC[15] = {};
      ACK[1] = {1};
      // read id and data from input signals
      struct Message msg = new Message(unsigned int id, unsigned int data, unsigned int CRC, unsigned int ACK);
      // bus does arbitration upon receiving the message
      msg_to_bus_og.write(msg);
      // check if msg on bus it myself
    	if(cmpmsg(msg_from_bus.read(),msg)){
    		while(true){
    			wait(CLK_PERIOD,SC_NS);
          // check if the message written to bus is acked
    			if(msg_from_bus.read().ACK) return;
    		}
    	}
    	else{
        // if loses arbition, wait for the other message on bus to transmit
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
      data_to_proc.write(msg_from_bus.read().data);
      id_to_proc.write(msg_from_bus.read().base_ID);
      // ack the received message
      struct Message message_read = new Message();
      memcpy(&message_read,&(msg_from_bus.read()),sizeof(struct Message));// need to check
      message_read.ack = true;
      msg_to_bus_ack.write(message);
    }
  }
};

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

  unsigned int flight_comp_id = 0;
  unsigned int landing_gear_id= 1;
  unsigned int sensor_id = 2;

  SC_HAS_PROCESS(Flight_computer_processor);

  Flight_computer_processor(sc_module_name name) : sc_module(name)
  {
    SC_THREAD(control);
      sensitive << clk;
  }

  void control(){
  	while(true){
  		wait(CLK_PERIOD,SC_NS);
      // check if the data comes from sensor
  		if (id_from_ctrl.read()==sensor_id){
          // check if the data reached
      		if (data_from_ctrl.read() == 500){
            id_to_ctrl.wirte(flight_comp_id)
            data_to_ctrl.write(9999);
        		ctrl_port_fc -> WriteMessage();
      		}
    	}
    	else if(received_id == landing_gear_id){
    		if(data_from_ctrl.read() == 9999){
    			cout << "landing gear deployed successfully" << endl;
    		}
    	}
  	}

  }
};

class Landing_gear_processor : public sc_module
{
public:
  // need to map the port in top
  sc_port<ctrl_interface> ctrl_port_fc;=

  sc_in<sc_logic> clk;
  sc_in<unsigned int> id_from_ctrl;
  sc_in<unsigned int> data_from_ctrl;
  sc_out<unsigned int> id_to_ctrl;
  sc_out<unsigned int> data_to_ctrl;

  unsigned int flight_comp_id = 0;
  unsigned int landing_gear_id= 1;
  unsigned int sensor_id = 2;

  SC_HAS_PROCESS(Landing_gear_processor);

  Landing_gear_processor(sc_module_name name) : sc_module(name)
  {
    SC_THREAD(deployment);
      sensitive << clk.pos();
  }

  void deployment()
  {
    while (true)
    {
    // landing code = 10
    wait(CLK_PERIOD, SC_NS);
      if (id_from_ctrl.read()==flight_comp_id){
      	if (data_from_ctrl.read() == 9999){
      		cout << "landing gear deploying!" << endl;
      		wait(CLK_PERIOD * 100, SC_NS);
          id_to_ctrl.wirte(landing_gear_id);
          data_to_ctrl.write(9999);
      		ctrl_port_lg -> WriteMessage();
      	}
      }
    }
  }
};

class Sensor_processor : public sc_module
{
public:
  sc_port<ctrl_interface> ctrl_port_fc;

  sc_in<sc_logic> clk;
  sc_in<unsigned int> id_from_ctrl;
  sc_in<unsigned int> data_from_ctrl;
  sc_out<unsigned int> id_to_ctrl;
  sc_out<unsigned int> data_to_ctrl;

  unsigned int flight_comp_id = 0;
  unsigned int landing_gear_id= 1;
  unsigned int sensor_id = 2;

  SC_HAS_PROCESS(Sensor_processor);

  Sensor_processor(sc_module_name name) : sc_module(name)
  {
    SC_THREAD(update_data);
      sensitive << clk.pos();
    SC_THREAD(send_data);
      sensitive << clk.pos();
  }

  // simulate that the plane descend 1 m every 1000 clock cycles
  // remember that transmit data takes 100 clock cycles!
  void update_data()
  {
    for (int i = 1000; i > 0; i--)
    {
      wait(CLK_PERIOD * 1000, SC_NS);
      id_to_ctrl.write(sensor_id);
      data_to_ctrl.write(i);
      //ctrl_port_sr -> WriteMessage(sensor_id,i);
      //cout << "sensor data read: " << i << endl;
    }
  }
  void send_data()
  {
    while (true){
      wait(CLK_PERIOD * 1000, SC_NS);
      ctrl_port_sr -> WriteMessage();
      cout << "sensor data sent successfully (with ack detected)" << endl;
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
