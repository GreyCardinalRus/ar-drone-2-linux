// g++ -o main main.cpp -I/usr/local/include/ -L/usr/local/lib/ -lboost_thread -lboost_date_time -lSDL2 -lavcodec -lavformat -lswscale `pkg-config --cflags --libs opencv`

#include <iostream>
#include <boost/date_time.hpp>

#define ARDRONE_DEFAULT_ADDR "192.168.1.1"
#define ARDRONE_VERSION      2

#include "ArDrone2.hpp"
#include "video.hpp"

int main (int argc, char** argv) {

  if (argc < 2) {
    std::cout << "usage:" << argv[0] << " <drone's address>" << std::endl;
    std::cout << "usage default :" <<  ARDRONE_DEFAULT_ADDR<< std::endl;
    //exit(1);
  }

  GreyCardinal::Media     media;
  GreyCardinal::Navdata   navdata(&media, (argc < 2?ARDRONE_DEFAULT_ADDR:argv[1]));
  GreyCardinal::Command   command(&media, (argc < 2?ARDRONE_DEFAULT_ADDR:argv[1]));
  GreyCardinal::Decode    decode(&media, (argc < 2?ARDRONE_DEFAULT_ADDR:argv[1]));
  GreyCardinal::OpenCV    opencv(&media);
  GreyCardinal::Display   display(&media);

  // start receiving navdata
  navdata.start();

  // start communication
  command.start();

  // start decoding after video mode changed
  while(!command.video_enabled){}
  decode.start();

  // start image processing after video stream is decoded
  while(!decode.started){}
  opencv.start();

  // start displaying images after the image processing thread is initialized
  while(!opencv.started){}
  display.start();

  // wait while terminate conditions are satisfied

  // terminate thread
  display.join();
  opencv.join();
  decode.join();
  command.join();
  navdata.join();
}
