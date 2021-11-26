#include "systemc.h"
#include <fstream>
#include <string>
#include <queue>
#include <string>
#include <iomanip>


#define CLK_HALF_PERIOD 5
#define CLK_PERIOD 10      // clock at 100 MHz

// -------------- struct types and custom functions ---------------------------------------
// ------------ message struct -------------------------------------
/* CAN message frame: 99 bits in total
    assume transmit one bit needs 1 clock cycle
    takes 100 clock cycles to transmit the message
*/
struct Message
{
  //unsigned int IFS[3] = {}; //initialize to zeros
  //unsigned int SOF[1] = {0};
  unsigned int base_ID;
  //unsigned int SRR[1] = {1};
  //unsigned int IDE[1] = {1};
  //unsigned int extension_ID[18] = {}; //initialize to zeros
  //unsigned int RTR[1] = {0};
  //unsigned int control_field[6] = {0, 0, 0, 1, 0, 0};// datalength fixed at 4 byte
  unsigned int data;
  unsigned int CRC;
  //unsigned int CRC_deli[1] = {1};
  bool ACK;
  //unsigned int ACK_deli[1] = {1};
  //unsigned int end[7] = {1, 1, 1, 1, 1, 1, 1};

  Message(unsigned int base_ID, unsigned int data, unsigned int CRC, bool ACK)
  {
    base_ID = base_ID;
    data = data;
    CRC = CRC;
    ACK = ACK;
  }
  Message(){
    base_ID = 999;
    data = 1;
    CRC=0;
    ACK = false;
  }
};

bool cmpmsg(struct Message *msg1, struct Message *msg2){    // ------------------------------ need to implement (finished)----------------
  if(msg1->base_ID == msg2->base_ID && msg1->data == msg2->data && msg1->CRC == msg2->CRC && msg1->ACK == msg2->ACK){
    return true;
  }
  return false;
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

struct Log* mem[2048];

class ctrl_interface : virtual public sc_interface
{
public:
  virtual void WriteMessage() = 0;
};


// -------------- oscillator (finished) ---------------------------------------

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



// end fo log struct

// -------------- modules ---------------------------------------
// ------------BUS -----------------------
class Bus : public sc_module
{

public:
  sc_in<sc_logic> clk;
  sc_in<struct Message*> msg_in;
  sc_out<struct Message*> msg_out;

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
  sc_in<unsigned int> addr;
  sc_in<sc_logic> read_en;
  sc_in<sc_logic> write_en;
  sc_in<struct Log*> log_in;
  sc_out<struct Log*> log_out;

  //struct Log* mem[2048];
  unsigned int write_address;

  SC_HAS_PROCESS(Memory);

  Memory(sc_module_name name) : sc_module(name)
  {
    write_address = 0;
    SC_THREAD(Memory_Access);
      sensitive << clk;
  }

  void Memory_Access()
  {
    wait(CLK_PERIOD,SC_NS);
      if (read_en.read() == SC_LOGIC_1){//read
        log_out.write(mem[addr.read()]);
      }
      else if(write_en.read()==SC_LOGIC_1){//write
        mem[addr.read()] = log_in.read();
        write_address++;
        cout<<"Memory: next address is "<<write_address<<endl;
      }
  }
};



// controller serves as transciver, encoder, and decoder, is submodule of LRUs
class CAN_ctrl : public sc_module, public ctrl_interface
{
public:
  sc_out<struct Message*> msg_to_bus_og;

  sc_out<struct Message*> msg_to_bus_ack;
  sc_in<sc_logic> clk;
  sc_in<struct Message*> msg_from_bus;
  sc_in<unsigned int> data_from_proc;
  sc_in<unsigned int> id_from_proc;
  sc_out<unsigned int> id_to_proc;
  sc_out<unsigned int> data_to_proc;
// whichever the following two messages going onto the bus will be determined in the top module




  //struct Message* dummy; //= new Message(999,0,0,false);

  SC_HAS_PROCESS(CAN_ctrl);

  CAN_ctrl(sc_module_name name) : sc_module(name)
  {
    //dummy = new Message(999,0,0,false);


    SC_THREAD(ctrl_receive);
      sensitive << clk.pos();
  }

  // CAN controller encode and transmit message to bus
  //void WriteMessage(unsigned int id, unsigned int data){
  void WriteMessage(){
    struct Message* msg = new Message();
    while(true){
      //cout<<"Write Message is called"<<endl;
      wait(CLK_PERIOD, SC_NS);
      // encode (recalculate each time, to avoid outdate data)--------- need to implement (dont want to implement, wont cause error)
      // read id and data from input signals
      int data = data_from_proc.read();
      int id = id_from_proc.read();
      //cout<< "data is: " << data <<endl;
      msg->ACK=false;
      msg->data = data;
      msg->base_ID=id;
      msg_to_bus_og.write(msg);
      // check if msg on bus it myself
      wait(CLK_PERIOD*100,SC_NS);
      //cout<<"waited 100 CC"<<endl;
    	if(cmpmsg(msg_from_bus.read(),msg)){
        //cout<<"message is on the bus, waiting for ack"<<endl;
    		while(true){
    			wait(CLK_PERIOD,SC_NS);
          // check if the message written to bus is acked

    			if(msg_from_bus.read()->ACK) {
            msg_to_bus_og.write(NULL);
            return;
          }
    		}
    	}
    	else{
        // if loses arbition, wait for the other message on bus to transmit
        // time to transmit the entire CAN message
    		//wait(CLK_PERIOD*100,SC_NS);
    	}
    }
  }

  // THREAD
  void ctrl_receive()
  {
    //msg_to_bus_ack.write(dummy);
    //msg_to_bus_og.write(dummy);
    //struct Message* message_read = new Message(msg_from_bus.read()->base_ID,msg_from_bus.read()->data,0,true);
    unsigned int data, id;
    unsigned int my_id = id_from_proc.read();
    struct Message* message_read = new Message();
    while (true)
    {
      wait(CLK_PERIOD,SC_NS);
      //msg_to_bus_og.write(dummy);
      // decode the data and write to LRU
      //cout<<"receiving message"<<endl;
      if(msg_from_bus.read() != NULL){
        my_id = id_from_proc.read();
        data = msg_from_bus.read()->data;
        id = msg_from_bus.read()->base_ID;
        data_to_proc.write(data);
        id_to_proc.write(id);
        //if(data==9999&&id==0){
          //cout<<"WriteMessage: message from flight computer is detected"<<endl;
          //cout<<"My ID: "<<my_id<<endl;
        //}

        //cout<<"receiving message end"<<endl;
        if(id != my_id){//&&!msg_from_bus.read()->ACK
          // ack the received message
          //cout << "Receiver: before memcpy"<< endl;
          memcpy(message_read,msg_from_bus.read(),sizeof(struct Message));// need to check
          //cout << "Receiver: after memcpy"<< endl;
          msg_from_bus.read()->ACK=true;
          //message_read->ACK = true;
          msg_to_bus_ack.write(message_read);
          //if(data==9999&&id==0){
            //cout<<"WriteMessage: message from flight computer is acked"<<endl;
          //}

        }else{
          msg_to_bus_ack.write(NULL);
        }
      }
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

  unsigned int flight_comp_id;
  unsigned int landing_gear_id;
  unsigned int sensor_id;

  SC_HAS_PROCESS(Flight_computer_processor);

  Flight_computer_processor(sc_module_name name) : sc_module(name)
  {
    flight_comp_id = 0;
    landing_gear_id= 1;
    sensor_id = 2;
    SC_THREAD(control);
      sensitive << clk;
  }



  void control(){
    bool lg_deploy_req = false;

  	while(true){
      id_to_ctrl.write(0);
      wait(CLK_PERIOD,SC_NS);
      // check if the data comes from sensor
      //cout << "Flight controller -  id received from others: " << id_from_ctrl.read() << endl;
  		if (id_from_ctrl.read()==sensor_id){
          // check if the data reached
          //cout<<"data received from sensor"<<endl;
          //cout << "data is: " << data_from_ctrl.read()<<endl;
      		if (data_from_ctrl.read() <= 500 && !lg_deploy_req){
            cout<<"Flight Computer: altitude target reached"<<endl;
            //sc_stop();
            lg_deploy_req=true;

            //int a = 9999;
            data_to_ctrl.write(9999);
        		ctrl_port_fc -> WriteMessage();
      		}
          wait(CLK_PERIOD*99,SC_NS);
    	}
    	else if(id_from_ctrl.read() == landing_gear_id){
    		if(data_from_ctrl.read() == 9999){
    			cout << "Flight Computer: landing gear deployed successfully" << endl;
          //sc_stop();
    		}
    	}
  	}
  }
};

class Landing_gear_processor : public sc_module
{
public:
  // need to map the port in top
  sc_port<ctrl_interface> ctrl_port_lg;

  sc_in<sc_logic> clk;
  sc_in<unsigned int> id_from_ctrl;
  sc_in<unsigned int> data_from_ctrl;
  sc_out<unsigned int> id_to_ctrl;
  sc_out<unsigned int> data_to_ctrl;

  unsigned int flight_comp_id;
  unsigned int landing_gear_id;
  unsigned int sensor_id;

  SC_HAS_PROCESS(Landing_gear_processor);

  Landing_gear_processor(sc_module_name name) : sc_module(name)
  {
    flight_comp_id = 0;
    landing_gear_id= 1;
    sensor_id = 2;
    SC_THREAD(deployment);
      sensitive << clk.pos();
  }

  void deployment()
  {

    while (true)
    {
    // landing code = 10
    id_to_ctrl.write(1);
    wait(CLK_PERIOD, SC_NS);
      if (id_from_ctrl.read()==flight_comp_id){
        //cout << "landing gear message received from flight computer!" << endl;
        //cout << "received message is: "<< data_from_ctrl.read() << endl;
      	if (data_from_ctrl.read() == 9999){
      		cout << "Landing Gear: landing gear deploying!" << endl;
      		wait(CLK_PERIOD * 10000, SC_NS);
          //cout<<"test test"<<endl;
          data_to_ctrl.write(9999);
          cout<<"test test"<<endl;
      		ctrl_port_lg -> WriteMessage();

          //sc_stop();
      	}
      }
    }
  }
};

class Sensor_processor : public sc_module
{
public:
  sc_port<ctrl_interface> ctrl_port_sensor;

  sc_in<sc_logic> clk;
  sc_in<unsigned int> id_from_ctrl;
  sc_in<unsigned int> data_from_ctrl;
  sc_out<unsigned int> id_to_ctrl;
  sc_out<unsigned int> data_to_ctrl;

  unsigned int flight_comp_id;
  unsigned int landing_gear_id;
  unsigned int sensor_id;

  SC_HAS_PROCESS(Sensor_processor);

  Sensor_processor(sc_module_name name) : sc_module(name)
  {
    flight_comp_id = 0;
    landing_gear_id= 1;
    sensor_id = 2;
    SC_THREAD(update_data);
      sensitive << clk.pos();
    SC_THREAD(send_data);
      sensitive << clk.pos();
  }

  // simulate that the plane descend 1 m every 1000 clock cycles
  // remember that transmit data takes 100 clock cycles!
  void update_data()
  {
    //id_to_ctrl.write(sensor_id);
    for (int i = 1000; i > 0; i--)
    {
      id_to_ctrl.write(2);
      data_to_ctrl.write(i);
      wait(CLK_PERIOD * 1000, SC_NS);
      //ctrl_port_sr -> WriteMessage(sensor_id,i);
      //cout << "sensor data read: " << i << endl;
    }
    sc_stop();
  }
  void send_data()
  {
    while (true){
      ctrl_port_sensor -> WriteMessage();
      //cout << "sensor data sent successfully (with ack detected)" << endl;
      wait(CLK_PERIOD * 1000, SC_NS);
    }
  }
};
// --------------------------------------- need to implement ------------------
class Bus_Ruler : public sc_module{
public:
  sc_in<sc_logic> clk;
  sc_in<struct Message*> msg_to_bus_ack_fc;
  sc_in<struct Message*> msg_to_bus_og_fc;
  sc_in<struct Message*> msg_to_bus_ack_lg;
  sc_in<struct Message*> msg_to_bus_og_lg;
  sc_in<struct Message*> msg_to_bus_ack_sensor;
  sc_in<struct Message*> msg_to_bus_og_sensor;
  sc_out<struct Message*> msg_to_bus;

  sc_out<struct Log*> log_to_mem;
  sc_out<sc_logic> write_en;
  //sc_out<unsigned int> address;

  bool message_on_bus;
  bool log_sent;
  unsigned int data, base_ID, time_stamp;

  SC_HAS_PROCESS(Bus_Ruler);

  Bus_Ruler(sc_module_name name) : sc_module(name)
  {
    message_on_bus = false;
    log_sent = false;
    SC_THREAD(Update_Bus);
      sensitive << clk.pos();
  }


  void Update_Bus(){

    while(true){
      wait(CLK_PERIOD,SC_NS);
      message_on_bus = false;
      log_sent = false;
      read_en.write(SC_LOGIC_0);
      if(msg_to_bus_og_fc.read()!=NULL){
        msg_to_bus.write(msg_to_bus_og_fc.read());
        cout<<"Ruler: message from fc is on the bus, message is: "<<msg_to_bus_og_fc.read()->data<<endl;
        wait(CLK_PERIOD*100,SC_NS);
        data = msg_to_bus_og_fc.read()->data;
        base_ID = msg_to_bus_og_fc.read()->base_ID;
        time_stamp = sc_time_stamp().to_seconds() * 1e9;
        message_on_bus = true;
      }else if(msg_to_bus_og_lg.read()!=NULL){
        msg_to_bus.write(msg_to_bus_og_lg.read());
        cout<<"Ruler: message from lg is on the bus, message is: "<<msg_to_bus_og_lg.read()->data<<endl;
        wait(CLK_PERIOD*100,SC_NS);
        data = msg_to_bus_og_lg.read()->data;
        base_ID = msg_to_bus_og_lg.read()->base_ID;
        time_stamp = sc_time_stamp().to_seconds() * 1e9;
        message_on_bus = true;
      }else if(msg_to_bus_og_sensor.read()!=NULL){
        msg_to_bus.write(msg_to_bus_og_sensor.read());
        cout<<"Ruler: message from sensor is on the bus, message is: "<<msg_to_bus_og_sensor.read()->data<<endl;
        wait(CLK_PERIOD*100,SC_NS);
        data = msg_to_bus_og_sensor.read()->data;
        base_ID = msg_to_bus_og_sensor.read()->base_ID;
        time_stamp = sc_time_stamp().to_seconds() * 1e9;
        message_on_bus = true;
      }

      if(message_on_bus){
        while(true){
          //wait for ack
          wait(CLK_PERIOD,SC_NS);
          if(!log_sent){
            write_en.write(SC_LOGIC_1);
            struct Log *bus_log = new Log(data,base_ID,time_stamp);
            log_to_mem.write(bus_log);
            log_sent = true;
            cout<<"Ruler: logging"<<endl;
          }
          cout<<"Ruler: checking for ack"<<endl;
          if(msg_to_bus_ack_fc.read()!=NULL&&msg_to_bus_ack_fc.read()->ACK){
            msg_to_bus.write(msg_to_bus_ack_fc.read());
            cout<<"Ruler: ack from flight computer"<<endl;
            cout<<"Ruler: data is from: " << msg_to_bus_ack_fc.read()->base_ID << " acked data is: " << msg_to_bus_ack_fc.read()->data<<endl;
            break;
          }

          if(msg_to_bus_ack_lg.read()!=NULL&&msg_to_bus_ack_lg.read()->ACK){
            msg_to_bus.write(msg_to_bus_ack_lg.read());
            cout<<"Ruler: ack from landing gear" <<endl;
            cout<<"Ruler: data is from: " << msg_to_bus_ack_lg.read()->base_ID << " acked data is: " << msg_to_bus_ack_lg.read()->data <<endl;
            break;
          }

          if(msg_to_bus_ack_sensor.read()!=NULL&&msg_to_bus_ack_sensor.read()->ACK){
            msg_to_bus.write(msg_to_bus_ack_sensor.read());
            cout<<"Ruler: ack from sensor"<<endl;
            cout<<"Ruler: data is from: " << msg_to_bus_ack_sensor.read()->base_ID << " acked data is: " << msg_to_bus_ack_sensor.read()->data<<endl;
            break;
          }
        }
      }
    }
  }

};
// ------------------------------------ need to implement --------------------
class System : public sc_module
{
public:
  Oscillator *oscillator;
  Bus_Ruler *bus_ruler;
  Bus *bus;
  Flight_computer_processor *fc_proc;
  CAN_ctrl *fc_ctrl;
  Landing_gear_processor *lg_proc;
  CAN_ctrl *lg_ctrl;
  Sensor_processor *sensor_proc;
  CAN_ctrl *sensor_ctrl;
  Memory *log_mem;

// clock signal
  sc_signal<sc_logic> clk;

// signal for bus module
  sc_signal<struct Message*> msg_in_bus;  // this signal needs to be determined in this module
  sc_signal<struct Message*> msg_out_bus;

// signal for flight computer
  //sc_signal<sc_logic> clk;
  sc_signal<unsigned int> id_from_ctrl_fc;
  sc_signal<unsigned int> data_from_ctrl_fc;
  sc_signal<unsigned int> id_to_ctrl_fc;
  sc_signal<unsigned int> data_to_ctrl_fc;

// signal for landing gear
  //sc_signal<sc_logic> clk;
  sc_signal<unsigned int> id_from_ctrl_lg;
  sc_signal<unsigned int> data_from_ctrl_lg;
  sc_signal<unsigned int> id_to_ctrl_lg;
  sc_signal<unsigned int> data_to_ctrl_lg;

// signal for sensor
  //sc_signal<sc_logic> clk;
  sc_signal<unsigned int> id_from_ctrl_sensor;
  sc_signal<unsigned int> data_from_ctrl_sensor;
  sc_signal<unsigned int> id_to_ctrl_sensor;
  sc_signal<unsigned int> data_to_ctrl_sensor;

// signal for flight computer can controller
  //sc_in<sc_logic> clk;
  //sc_in<struct Message> msg_from_bus; ===== the same as msg_out_bus
  //sc_in<unsigned int> data_from_proc;
  //sc_in<unsigned int> id_from_proc;
  sc_signal<struct Message*> msg_to_bus_ack_fc;
  sc_signal<struct Message*> msg_to_bus_og_fc;
  //sc_out<unsigned int> id_to_proc;
  //sc_out<unsigned int> data_to_proc;

// signal for landing gear can controller
  //sc_in<sc_logic> clk;
  //sc_in<struct Message> msg_from_bus; ===== the same as msg_out_bus
  //sc_in<unsigned int> data_from_proc;
  //sc_in<unsigned int> id_from_proc;
  sc_signal<struct Message*> msg_to_bus_ack_lg;
  sc_signal<struct Message*> msg_to_bus_og_lg;
  //sc_out<unsigned int> id_to_proc;
  //sc_out<unsigned int> data_to_proc;

// signal for sensor can controller
  //sc_in<sc_logic> clk;
  //sc_in<struct Message> msg_from_bus; ===== the same as msg_out_bus
  //sc_in<unsigned int> data_from_proc;
  //sc_in<unsigned int> id_from_proc;
  sc_signal<struct Message*> msg_to_bus_ack_sensor;
  sc_signal<struct Message*> msg_to_bus_og_sensor;
  //sc_out<unsigned int> id_to_proc;
  //sc_out<unsigned int> data_to_proc;

// signal for log memory
  //sc_signal<sc_logic> clk;
  sc_signal<unsigned int> addr;
  sc_signal<sc_logic> read_en;
  sc_signal<sc_logic> write_en;
  sc_signal<struct Log*> log_in;
  sc_signal<struct Log*> log_out;


  SC_HAS_PROCESS(System);

  System(sc_module_name name) : sc_module(name)
  {

    oscillator = new Oscillator("OSC1");
    bus_ruler = new Bus_Ruler("bus_ruler1");
    bus = new Bus("bus");
    fc_proc = new Flight_computer_processor("fc1");
    lg_proc = new Landing_gear_processor("lg1");
    sensor_proc = new Sensor_processor("alt_sensor");


    cout << "creating instances" <<endl;
    lg_ctrl = new CAN_ctrl("lg_CAN");
    fc_ctrl = new CAN_ctrl("fc_CAN");
    sensor_ctrl = new CAN_ctrl("sensor_CAN");

    log_mem = new Memory("log_memory");


    // port map (3)
    fc_proc -> ctrl_port_fc(*fc_ctrl);
    lg_proc -> ctrl_port_lg(*lg_ctrl);
    sensor_proc -> ctrl_port_sensor(*sensor_ctrl);

    // clk mapping (8)
    oscillator->clk(clk);
    bus->clk(clk);
    bus_ruler->clk(clk);
    fc_proc->clk(clk);
    fc_ctrl->clk(clk);
    lg_proc->clk(clk);
    lg_ctrl->clk(clk);
    sensor_proc->clk(clk);
    sensor_ctrl->clk(clk);
    log_mem->clk(clk);

    // bus signal mapping (2)
    bus->msg_in(msg_in_bus);
    bus->msg_out(msg_out_bus);

    // bus ruler mapping(7)
    bus_ruler->msg_to_bus_ack_fc(msg_to_bus_ack_fc);
    bus_ruler->msg_to_bus_og_fc(msg_to_bus_og_fc);
    bus_ruler->msg_to_bus_ack_lg(msg_to_bus_ack_lg);
    bus_ruler->msg_to_bus_og_lg(msg_to_bus_og_lg);
    bus_ruler->msg_to_bus_ack_sensor(msg_to_bus_ack_sensor);
    bus_ruler->msg_to_bus_og_sensor(msg_to_bus_og_sensor);
    bus_ruler->msg_to_bus(msg_in_bus);
    bus_ruler->write_en(write_en);
    bus_ruler->log_to_mem(log_in);

    // flight computer processor mapping (4)
    fc_proc->id_from_ctrl(id_from_ctrl_fc);
    fc_proc->data_from_ctrl(data_from_ctrl_fc);
    fc_proc->id_to_ctrl(id_to_ctrl_fc);
    fc_proc->data_to_ctrl(data_to_ctrl_fc);

    // landing gear processor mapping (4)
    lg_proc->id_from_ctrl(id_from_ctrl_lg);
    lg_proc->data_from_ctrl(data_from_ctrl_lg);
    lg_proc->id_to_ctrl(id_to_ctrl_lg);
    lg_proc->data_to_ctrl(data_to_ctrl_lg);

    // sensor processor mapping (4)
    sensor_proc->id_from_ctrl(id_from_ctrl_sensor);
    sensor_proc->data_from_ctrl(data_from_ctrl_sensor);
    sensor_proc->id_to_ctrl(id_to_ctrl_sensor);
    sensor_proc->data_to_ctrl(data_to_ctrl_sensor);

    // flight computer CAN controller
    fc_ctrl->msg_from_bus(msg_out_bus);
    fc_ctrl->data_from_proc(data_to_ctrl_fc);
    fc_ctrl->id_from_proc(id_to_ctrl_fc);
    fc_ctrl->id_to_proc(id_from_ctrl_fc);
    fc_ctrl->data_to_proc(data_from_ctrl_fc);
    fc_ctrl->msg_to_bus_ack(msg_to_bus_ack_fc);   // important
    fc_ctrl->msg_to_bus_og(msg_to_bus_og_fc);     // important

    // landing gear CAN controller
    lg_ctrl->msg_from_bus(msg_out_bus);
    lg_ctrl->data_from_proc(data_to_ctrl_lg);
    lg_ctrl->id_from_proc(id_to_ctrl_lg);
    lg_ctrl->id_to_proc(id_from_ctrl_lg);
    lg_ctrl->data_to_proc(data_from_ctrl_lg);
    lg_ctrl->msg_to_bus_ack(msg_to_bus_ack_lg);   // important
    lg_ctrl->msg_to_bus_og(msg_to_bus_og_lg);     // important

    // flight computer CAN controller
    sensor_ctrl->msg_from_bus(msg_out_bus);
    sensor_ctrl->data_from_proc(data_to_ctrl_sensor);
    sensor_ctrl->id_from_proc(id_to_ctrl_sensor);
    sensor_ctrl->id_to_proc(id_from_ctrl_sensor);
    sensor_ctrl->data_to_proc(data_from_ctrl_sensor);
    sensor_ctrl->msg_to_bus_ack(msg_to_bus_ack_sensor);   // important
    sensor_ctrl->msg_to_bus_og(msg_to_bus_og_sensor);     // important

    // log mem
    log_mem->log_in(log_in);
    log_mem->log_out(log_out);
    log_mem->read_en(read_en);
    log_mem->write_en(write_en);
    log_mem->addr(addr);

    cout << "System Start!" << endl;

    //SC_THREAD(Update_Bus);
      //sensitive << clk.pos(); ?????
  }

  // need a thread to track data on bus

};

int sc_main(int argc, char *argv[])
{
  System test("test");
  sc_start();

  //cout << "Time Consumed: " << std::setprecision(8) << sc_time_stamp().to_seconds() * 1e9 << "(ns)" << endl;
  //cout << "Time Consumed: " << std::setprecision(8) << sc_time_stamp().to_seconds() * 1e9 / CLK_PERIOD << "(cc)" << endl;

  return 0;
}
