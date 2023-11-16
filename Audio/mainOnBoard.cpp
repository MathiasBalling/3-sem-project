#include "PAsound.h"
#include "consts.h"
#include "protocol.h"
#include <iostream>
//#include "RB3_cpp_publisher.h"

int main(int argc, char** argv) {
  /*rclcpp::init(argc, argv);
  auto publisher = RB3_cpp_publisher();*/
  PAsound sound;
  sound.init(0);
  sound.setListening(true);
  
  float currentLinear=0.0;
  float currentAngular=0.0;
  while (1) {
    if(!sound.isListening()){
        std::cout << "dd" << std::endl;
    std::pair<Operation, std::vector<float>> input = DTMFtoData(sound.getInputBuffer());
    
    switch(input.first){
        case Operation::FORWARD:{
            currentLinear+=0.1;
        }
        case Operation::BACKWARD:{
            currentLinear-=0.1;
        }
        case Operation::LEFT:{
            currentAngular-=0.1;
        }
        case Operation::RIGHT:{
            currentAngular+=0.1;
        }
        case Operation::STOP:{
            currentAngular=0.0;
            currentLinear=0.0;
        }
        case Operation::MOVEMENT:{
            currentLinear=input.second[0];
            currentAngular=input.second[1];
        }
        default: break;
    }
    //publisher->publish_vel(currentLinear,currentAngular)
    sound.setListening(true);
    }
    
    
  }
  //rclcpp::shutdown();
  return 0;
}