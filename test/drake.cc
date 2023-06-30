#include <gtest/gtest.h>
#include "sheldrake.hpp"

// Demonstrate some basic assertions.
TEST(DrakeTests, Make_PID_Host) {

    // Make a PID controller with Drake
    Eigen::VectorXd Kp(3);
    Eigen::VectorXd Ki(3);
    Eigen::VectorXd Kd(3);
    drake::systems::controllers::PidController<double> pid(Kp, Ki, Kd);

    // Make a dummy bytestream that doesn't do anything
    sheldrake::ByteStream bytestream = {
        .is_open = [](){return true;},
        .write = [](uint8_t data){}
    };

    // Should make a sheldrake::Host<tcp::Controller_PID<3,float>, 3>
    auto host = sheldrake::create<3,3,double,float>(bytestream, 1, pid);
}