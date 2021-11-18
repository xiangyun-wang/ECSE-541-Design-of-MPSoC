#include "systemc.h"
#include <fstream>
#include <string>
#include <queue>
#include <string>
#include <iomanip>

#define MEMORY_SIZE (1 << (10+6))
#define DATA_LEN 32 // datalenth is 32 bit, 4 byte
#define BUS_CAP 10 // bus is able to take 10 nodes
#define CLK_HALF_PERIOD 500
#define CLK_PERIOD 1000 // clock at 1 MHz

//#include "header file"

unsigned int memory[MEMORY_SIZE];
int cache_size = MATRIX_DEFAULT_SIZE*MATRIX_DEFAULT_SIZE;
int matrix_size = MATRIX_DEFAULT_SIZE;
int loops = LOOPS;
unsigned int process_ID = 1;


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
  sc_in<sc_logic> clk;
  sc_in<sc_lv<BUS_CAP>> sig; // 
  sc_in<sc_logic> busy;
  sc_out<sc_logic> out;

  SC_HAS_PROCESS(Bus);

  Bus (sc_module_name name) : sc_module(name){

   	SC_THREAD(arbiter); // implement below
      sensitive << clk.pos();
  }

  void arbiter(){
    while(true){
    	out.write(sig.and_reduce());// 0 dominats
    }
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

// controller serves as transciver
class CAN_ctrl: public sc_module{
public:
	sc_in<sc_logic> clk;
	sc_in<sc_lv<DATA_LEN>> data;
	sc_in<sc_logic> bus_busy;
	sc_in<sc_logic> mode; // 0 is write, 1 is read
	sc_in<sc_logic> from_bus; //current value on bus

	sc_out<sc_logic> listening; // enter listening mode once backoff
	sc_out<sc_logic> to_bus;
	sc_out<sc_lv<DATA_LEN>> received_data;
	sc_out<sc_logic> data_ready;
	//sc_signal<sc_lv<9999>> can_msg; // construct CAN data frame message
	bool back_off;
	//bool data_ready;
	//bool bus_free; // need to know when bus is free
	int counter;


	SC_HAS_PROCESS(CAN_ctrl);

	CAN_ctrl(sc_module_name name): sc_module(name){
		counter = 0;
		back_off = false;
		//data_ready = false;
		SC_THREAD(ctrl_op);
			sensitive << clk.pos();
	}

	void ctrl_op{
		while(true){
			if(mode.read()==SC_LOGIC_1 || back_off){
				// listenting
				TODO:
				//construct received data
				//check if want to take the data
				if(bus_busy == SC_LOGIC_0 && back_off){
					back_off = false;
				}

			}
			else {
				// writing
				wait(CLK_PERIOD, SC_NS);
				to_bus.write(can_msg.get_bit(counter));
			
				if(from_bus.read() != can_msg.get_bit(counter)){
					back_off = true;
					counter = 0;
				}
				else{
					counter ++;
				}
				
			}
		}
	}

}

class Flight_computer: public sc_module{
public:
	sc_in<sc_logic> clk;
	sc_in<sc_lv<DATA_LEN>> received_data;
	CAN_ctrl *ctrl_inst;
	SC_HAS_PROCESS(Flight_computer);
	Flight_computer(sc_module_name name): sc_module(name){
		ctrl_inst = new CAN_ctrl("Flight_computer_CAN_ctrl");
		SC_THREAD(control);
	}

	void control(){

	}



};


class Landing_gear: public sc_module{
public:
	sc_in<sc_logic> clk;
	sc_out<sc_lv<DATA_LEN>> data_out;
	sc_signal<sc_lv<DATA_LEN>> received_data;
	CAN_ctrl *ctrl_inst;
	SC_HAS_PROCESS(Landing_gear);

	Landing_gear(sc_module_name name): sc_module(name){
		ctrl_inst = new CAN_ctrl("Landing_gear_CAN_ctrl");
		ctrl_inst -> data(received_data);
		SC_THREAD(deployment);
			sensitive << clk.pos();
	}

	void deployment(){
		while(true){
			// landing code = 10
			if(received_data.to_int() == 10){
				wait(CLK_PERIOD*1000, SC_NS);
				// landing complete code = 100
				data_out.write(static_cast<sc_lv<DATA_LEN>>bitset<DATA_LEN>(100));
			}
		}
	}
};


class Sensor: public sc_module{
public:
  sc_in<sc_logic> clk;
  sc_signal<sc_lv<DATA_LEN>> sensor_data;
  CAN_ctrl *ctrl_inst;
  SC_HAS_PROCESS(Sensor);

  Sensor(sc_module_name name): sc_module(name){
    // map to its controller
    ctrl_inst = new CAN_ctrl("Sensor_CAN_ctrl");
    ctrl_inst -> data(sensor_data);
    // need to map all signal from controller

    SC_THREAD(update_data);

      sensitive << clk.pos();
  }

  // simulate that the plane descend 1 m every 100 clock cycles
  void update_data(){
  	for (int i=1000;i>0;i--){
  		wait(CLK_PERIOD*100, SC_NS);
  		sensor_data.write(static_cast<sc_lv<DATA_LEN>>bitset<DATA_LEN>(i)); // really?
  	}
  }
};




class System : public sc_module{
public:
  sc_signal<sc_logic> clk;
  sc_lv<BUS_CAP> bit_buffer;

  Memory *memory;
  Hardware *hardware;
  Software *software;
  Bus *bus;
  Oscillator *oscillator;

  System(sc_module_name name, char* mem_init_file) : sc_module(name){
    for (int i=0;i<BUS_CAP;i++){
  		bit_buffer.set_bit(i,SC_LOGIC);
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
