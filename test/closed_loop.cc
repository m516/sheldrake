#include <gtest/gtest.h>
#include <queue>
#include "sheldrake.hpp"


// Setup a minimal reproducible example of buffers that can be used to test the Serializer
//     Broadcaster -> Broadcaster::buffer -> Broadcaster::receive() -> Receiver::receive()

class Broadcaster{
  std::queue<uint8_t> buffer;
  public:
  void send(uint8_t data){
    buffer.push(data);
  }
  int16_t receive(){
    if(buffer.empty()) return -1;
    uint8_t data = buffer.front();
    buffer.pop();
    return data;
  }
};

class Receiver{
  private:
  Broadcaster &broadcaster;
  public:
  Receiver(Broadcaster &broadcaster): broadcaster(broadcaster){}
  int16_t receive(){
    return broadcaster.receive();
  }
};

/*
We're emulating two systems in this test at the same time: 
1. the Host (on a device with Drake or some other way to compute Controllers)
2. the Peripheral (on a device that uses the model to control something)

The Host and Peripheral are connected over some interface that sends bytes back and forth.
*/

// Host side
Broadcaster controller_broadcaster;
// Peripheral side
Broadcaster state_broadcaster;

// Peripheral side
Receiver controller_receiver(controller_broadcaster);
// Host side
Receiver state_receiver(state_broadcaster);



TEST(SheldrakeTests, Loop_LTI) {

  // Create a controller
  tcp::Controller_LTI<4,5,8,float> controller_model;
  controller_model.A = tmm::Identity<8>();
  controller_model.B = tmm::Zeros<8,4>()+2;
  controller_model.C = tmm::Zeros<5,8>()+3;
  controller_model.D = tmm::Zeros<5,4>()+4;

  // Create a host
  sheldrake::ByteStream host_stream = {
    .is_open = [](){return true;},
    .write = [](uint8_t data){controller_broadcaster.send(data);}
  };
  sheldrake::Host<tcp::Controller_LTI<4,5,8,float>, 8> host(host_stream, 42);

  // Create a peripheral
  sheldrake::ByteStream peripheral_stream = {
    .is_open = [](){return true;},
    .write = [](uint8_t data){state_broadcaster.send(data);}
  };
  sheldrake::Peripheral<tcp::Controller_LTI<4,5,8,float>, 8> peripheral(peripheral_stream, 42);

  // Send the controller model from the Host to the Peripheral
  host.model = controller_model;
  host.send_model();

  // The Peripheral will keep receiving bytes. 
  // When it has received enough bytes to fill its model, it will update its internalmodel.
  for(int i = 0; i < sizeof(controller_model); i++){
    peripheral.read_model(controller_receiver.receive());
  }

  // Ensure that the Peripheral's internal model matches the Host's model
  ASSERT_EQ(peripheral.model.A, controller_model.A);
  ASSERT_EQ(peripheral.model.B, controller_model.B);
  ASSERT_EQ(peripheral.model.C, controller_model.C);
  ASSERT_EQ(peripheral.model.D, controller_model.D);


} // end Loop_LTI

