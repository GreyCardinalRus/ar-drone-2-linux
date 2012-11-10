#ifndef GCArDorne
#define GCArDorne

#include <iostream>
#include <sstream>

#include <netdb.h>
#include <string.h>
#include <stdio.h>

#include <boost/thread.hpp>

namespace GreyCardinal {
  
  class Navdata;
  class Command;
  class Decode;
  class OpenCV;
  class Display;
  
  class Media {
    
    public:
	  GreyCardinal::Navdata*   navdata;
	  GreyCardinal::Command*   command;
	  GreyCardinal::Decode*    decode;
	  GreyCardinal::OpenCV*    opencv;
	  GreyCardinal::Display*   display;
    
      Media(){decode= NULL;}
    
  };
  
  class ArDrone2 {
    
    protected:
      
      int                   socket_descriptor;
      struct sockaddr_in    ardrone_sockaddr;
      struct sockaddr_in    client_sockaddr;
      struct hostent        *ardrone_addr;
      struct timespec       command_interval;
      
      // constructor
      ArDrone2(std::string _ardrone_addr, int port){
      
        // get host address
        ardrone_addr = gethostbyname(_ardrone_addr.c_str());
        
        // create socket
        if (createSocket(&socket_descriptor, 
                         &ardrone_sockaddr, 
                         &client_sockaddr, 
                         ardrone_addr, 
                         port)<0) {
          std::cout << "Command: error socket creation\n";
        }
        
        // set timing variable
        command_interval.tv_sec  = 0; 
        command_interval.tv_nsec = 35000000; // 30 helz is recommended
      }
      
      // create socket for designated address and port
      int createSocket(int                  *sd, 
                       struct sockaddr_in   *droneAddr, 
                       struct sockaddr_in   *clientAddr, 
                       struct hostent       *host, 
                       int                  port) {
        
        // create structure for ardrone address & port
        droneAddr->sin_family = host->h_addrtype;
        droneAddr->sin_port   = htons(port);
        memcpy((char *) &(droneAddr->sin_addr.s_addr), host->h_addr_list[0], host->h_length);
        
        // create structure for this client
        clientAddr->sin_family = AF_INET;
        clientAddr->sin_addr.s_addr = htonl(INADDR_ANY);
        clientAddr->sin_port = htons(0);
        
        // socket creation
        *sd = socket(AF_INET,SOCK_DGRAM,0);
        if(sd < 0)
          return -1;
        
        // bind client's the port and address
        if(bind(*sd, (struct sockaddr *) clientAddr, sizeof(*clientAddr)) < 0)
          return -2;
        
        // set timeout
        struct timeval tv;
        tv.tv_sec  = 0;
        tv.tv_usec = 100000; // in micro (10^-6) sec 
        setsockopt(*sd, SOL_SOCKET, SO_RCVTIMEO, (char *)&tv,sizeof(struct timeval));
        
        return 0;
      };
      
      // send command to designated address and port
      int sendCommand(std::string command) {
        
        // wait a while to adjust command interval
        nanosleep(&command_interval, NULL);
        
        if( sendto(socket_descriptor, 
                   command.c_str(), strlen(command.c_str())+1, 0, 
                   (struct sockaddr *) &ardrone_sockaddr, 
                   sizeof(ardrone_sockaddr)) < 0 ) {
          return -1;
        } else {
          return 0;
        }
      };
  };
  

  
  typedef float   float32_t;
  
  // navdata header
  typedef struct _navdata_header_t {
    uint32_t    header;                   // header:55667788 
    uint32_t    state;                    // the state of the drone 
    uint32_t    seq;                      // sequence number 
    uint32_t    vision;                   // vision flag 
  } navdata_header_t;

  // navdata option, demo mode
  typedef struct _navdata_demo_t {
    uint16_t    id;                        // Navdata block ('option') identifier 
    uint16_t    size;                      // set this to the size of this structure 
    
    uint32_t    ctrl_state;               // Flying state (landed, flying, hovering, etc.) defined in CTRL_STATES enum. 
    uint32_t    vbat_flying_percentage;   // battery voltage filtered (mV) 
    
    float32_t   theta;                    // pitch angle in milli-degrees 
    float32_t   phi;                      // roll  angle
    float32_t   psi;                      // yaw   angle
    
    int32_t     altitude;                 // altitude in centimeters[??] 
    
    float32_t   vx;                       // estimated linear velocity
    float32_t   vy;                       // estimated linear velocity
    float32_t   vz;                       // estimated linear velocity

    uint32_t    num_frames;               //!< streamed frame index  // Not used -> To integrate in video stage.
  } navdata_demo_t;

  // navdata structure
  typedef struct _navdata_t {
    navdata_header_t     navdata_header;  // navdata header 
    navdata_demo_t       navdata_option;  // navdata option 
  } navdata_t;
  
  
  #define DRONE_NAVDATA_PORT 5554
  #define MAX_NAVDATA 1024
  class Navdata : public ArDrone2 {
    private:
      
      boost::thread   m_Thread;
      
      char            navdata[MAX_NAVDATA];
      socklen_t       socketsize;
      navdata_t       *navdata_struct;
      bool            terminated;
      GreyCardinal::Media     *media;
      
    public:
      
      uint32_t    *battery;
      float       *pit_row;
      float       *rol_row;
      float       *yaw_row;
      int32_t     *alt_row;
      bool        drone_land;
      
      // constructor
      Navdata(GreyCardinal::Media*   _media,  std::string _ardrone_addr)
        : ArDrone2(_ardrone_addr, DRONE_NAVDATA_PORT) {
        
    	drone_land  =true;
    	  // define two way access from/to media object to this objct
        media             = _media;
        media->navdata    = this;
        
        // init UDP variables
        socketsize        = sizeof(ardrone_sockaddr);
        navdata_struct = (navdata_t*)navdata;
        memset(navdata, '\0', sizeof(navdata));
        
        terminated        = false;
        
        battery = &(navdata_struct->navdata_option.vbat_flying_percentage);
        pit_row = &(navdata_struct->navdata_option.theta);
        rol_row = &(navdata_struct->navdata_option.phi);
        yaw_row = &(navdata_struct->navdata_option.psi);
        alt_row = &(navdata_struct->navdata_option.altitude);
        
        std::cout << "[NAV] initialized" << std::endl;
      }
      
      ~Navdata(){
        close(socket_descriptor);
      }
      
      // poke navdata port
      int poke() {
        return sendCommand("\x01\x00");
      }
      
      // receive data
      int receive() {
        // poke drone
        if (poke() >= 0) {
          // receive
          // memset(navdata, '\0', sizeof(navdata));
          return recvfrom(socket_descriptor, navdata, sizeof(navdata), 0, 
                                  (struct sockaddr *)&ardrone_sockaddr, &socketsize);
        } else {
          return -1;
        }
      }
      
      int optionSize() {
        return navdata_struct->navdata_option.size;
      }
      
      int getControlCommandAck() {
        int controlCommandAck = ((navdata_struct->navdata_header.state & (1 <<  6))!=0);
        return controlCommandAck;
      }
      
      bool getEmergencyState() {
        return ((navdata_struct->navdata_header.state & (1 << 31))!=0);
      }
      
      bool sensorTrimed() {
        return ((navdata_struct->navdata_header.state & (1 <<  7))!=0);
      }
      
      
      // display navdata
      void coutNavdata() {
        printf("navdata header:\n");
        printf("\t%13x:%s\n", navdata_struct->navdata_header.header, "Header"          );
        printf("\t%13x:%s\n", navdata_struct->navdata_header.state,  "drone's state"   );
        printf("\t%13d:%s\n", navdata_struct->navdata_header.seq,    "sequence number" );
        printf("\t%13d:%s\n", navdata_struct->navdata_header.vision, "vision flag"     );
        printf("\t%13d:%s\n", navdata_struct->navdata_option.id,     "Option1 ID"      );
        printf("\t%13d:%s\n", navdata_struct->navdata_option.size,   "Option1 size"    );
        printf("drone's state:\n");
        printf("\t%13d:%s\n",(navdata_struct->navdata_header.state & (1 <<  0))!=0, "FLY MASK");
        printf("\t%13d:%s\n",(navdata_struct->navdata_header.state & (1 <<  1))!=0, "VIDEO MASK");
        printf("\t%13d:%s\n",(navdata_struct->navdata_header.state & (1 <<  2))!=0, "VISION MASK");
        printf("\t%13d:%s\n",(navdata_struct->navdata_header.state & (1 <<  3))!=0, "CONTROL ALGO");
        printf("\t%13d:%s\n",(navdata_struct->navdata_header.state & (1 <<  4))!=0, "ALTITUDE CONTROL ALGO");
        printf("\t%13d:%s\n",(navdata_struct->navdata_header.state & (1 <<  5))!=0, "USER feedback");
        printf("\t%13d:%s\n",(navdata_struct->navdata_header.state & (1 <<  6))!=0, "Control command ACK");
        printf("\t%13d:%s\n",(navdata_struct->navdata_header.state & (1 <<  7))!=0, "Trim command ACK");
        printf("\t%13d:%s\n",(navdata_struct->navdata_header.state & (1 <<  8))!=0, "Trim running");
        printf("\t%13d:%s\n",(navdata_struct->navdata_header.state & (1 <<  9))!=0, "Trim result");
        printf("\t%13d:%s\n",(navdata_struct->navdata_header.state & (1 << 10))!=0, "Navdata demo");
        printf("\t%13d:%s\n",(navdata_struct->navdata_header.state & (1 << 11))!=0, "Navdata bootstrap");
        printf("\t%13d:%s\n",(navdata_struct->navdata_header.state & (1 << 12))!=0, "Motors status");
        printf("\t%13d:%s\n",(navdata_struct->navdata_header.state & (1 << 13))!=0, "Communication Lost");
        printf("\t%13d:%s\n",(navdata_struct->navdata_header.state & (1 << 14))!=0, "problem with gyrometers");
        printf("\t%13d:%s\n",(navdata_struct->navdata_header.state & (1 << 15))!=0, "VBat low");
        printf("\t%13d:%s\n",(navdata_struct->navdata_header.state & (1 << 16))!=0, "VBat high");
        printf("\t%13d:%s\n",(navdata_struct->navdata_header.state & (1 << 17))!=0, "Timer elapsed");
        printf("\t%13d:%s\n",(navdata_struct->navdata_header.state & (1 << 18))!=0, "Power");
        printf("\t%13d:%s\n",(navdata_struct->navdata_header.state & (1 << 19))!=0, "Angles");
        printf("\t%13d:%s\n",(navdata_struct->navdata_header.state & (1 << 20))!=0, "Wind");
        printf("\t%13d:%s\n",(navdata_struct->navdata_header.state & (1 << 21))!=0, "Ultrasonic sensor");
        printf("\t%13d:%s\n",(navdata_struct->navdata_header.state & (1 << 22))!=0, "Cutout system detection");
        printf("\t%13d:%s\n",(navdata_struct->navdata_header.state & (1 << 23))!=0, "PIC Version number OK");
        printf("\t%13d:%s\n",(navdata_struct->navdata_header.state & (1 << 24))!=0, "ATCodec thread");
        printf("\t%13d:%s\n",(navdata_struct->navdata_header.state & (1 << 25))!=0, "Navdata thread");
        printf("\t%13d:%s\n",(navdata_struct->navdata_header.state & (1 << 26))!=0, "Video thread");
        printf("\t%13d:%s\n",(navdata_struct->navdata_header.state & (1 << 27))!=0, "Acquisition thread");
        printf("\t%13d:%s\n",(navdata_struct->navdata_header.state & (1 << 28))!=0, "CTRL watchdog");
        printf("\t%13d:%s\n",(navdata_struct->navdata_header.state & (1 << 29))!=0, "ADC Watchdog");
        printf("\t%13d:%s\n",(navdata_struct->navdata_header.state & (1 << 30))!=0, "Communication Watchdog");
        printf("\t%13d:%s\n",(navdata_struct->navdata_header.state & (1 << 31))!=0, "Emergency landing");
        printf("navdata_demo:\n");
        printf("\t%13d:%s\n",   navdata_struct->navdata_option.ctrl_state,             "Control State");
        printf("\t%13d:%s\n",   navdata_struct->navdata_option.vbat_flying_percentage, "battery");
        printf("\t%13.3f:%s\n", navdata_struct->navdata_option.theta,                  "pitch angle");
        printf("\t%13.3f:%s\n", navdata_struct->navdata_option.phi,                    "roll  angle");
        printf("\t%13.3f:%s\n", navdata_struct->navdata_option.psi,                    "yaw   angle");
        printf("\t%13d:%s\n",   navdata_struct->navdata_option.altitude,               "altitude");
        printf("\t%13.3f:%s\n", navdata_struct->navdata_option.vx,                     "estimated vx");
        printf("\t%13.3f:%s\n", navdata_struct->navdata_option.vy,                     "estimated vy");
        printf("\t%13.3f:%s\n", navdata_struct->navdata_option.vz,                     "estimated vz");
      }
      
      void start()
      {
        std::cout << "[NAV] Starting Navdata Thread\n";
        m_Thread = boost::thread(&Navdata::run, this);
      }
      
      void join()
      {
        terminated = true;
        m_Thread.join();
        std::cout << "[NAV] Thread Terminated\n";
      }
      
      void run()
      {
        int zerocounter = 40;
        while(!terminated){
          
          receive();
          
          if (*(alt_row) == 0 ) {
            if (zerocounter<100)
              zerocounter++;
            if (zerocounter > 30)
              drone_land = true;
          } else {
            zerocounter = 0;
            drone_land = false;
          }
          
          if ((navdata_struct->navdata_option.id == 0) && 
              (optionSize() >= 148)) {
            // coutNavdata();
            std::cout << "[NAV] battery: " << *(this->battery) 
                      << ", angles: "      << *(this->pit_row) << ", " 
                                           << *(this->rol_row) << ", " 
                                           << *(this->yaw_row) 
                      << ", altitude: "    << *(this->alt_row) << std::endl;
          }
          boost::this_thread::sleep (boost::posix_time::microseconds (30000));
        }
      }
  };
  #undef DRONE_NAVDATA_PORT
  #undef MAX_NAVDATA
  
  #define DRONE_COMMAND_PORT 5556
  #define SESSION_ID         "ad1efdac"
  #define USER_ID            "ad1efdac"
  #define APPLICATION_ID     "ad1efdac"
  class Command : public ArDrone2 {
    private:
      
      boost::thread   m_Thread;
      int             command_sequence;
      bool            terminated;
      
      GreyCardinal::Media     *media;
      
    public:
      
      float   arg1, arg2, arg3, arg4, arg5, arg6;
      bool    video_enabled;
      bool    flag_takeoff;
      bool    flag_land;
      bool    flag_PCMD;
      bool    flag_trim;  // trime sensors
      bool    flag_cYaw;  // combined yaw mode
      bool    flag_reset; // reset emergency mode
      bool    flag_mgnt;  // calibrate magnetometer
      bool    flag_absl;  // absolute control mode
      
      // constructor
      Command(GreyCardinal::Media*  _media, std::string  _ardrone_addr)
        : ArDrone2(_ardrone_addr, DRONE_COMMAND_PORT) {
        
        // define two way access from/to media object to this objct
        media             = _media;
        media->command    = this;
        
        // initialize sequence number, controll parameter, and flags
        command_sequence  = 0;
        arg1 = arg2 = arg3 = arg4 = arg5 = arg6 = 0;
        terminated    = false; 
        video_enabled = false; 
        flag_takeoff  = false; 
        flag_land     = false; 
        flag_PCMD     = false; 
        flag_trim     = false; 
        flag_cYaw     = false; 
        flag_reset    = false; 
        flag_mgnt     = false; 
        flag_absl     = false; 
        
        std::cout << "[COM] initialized" << std::endl;
      }
      
      ~Command(){
        close(socket_descriptor);
      }
      
      int getSequence() {return command_sequence;}
      
      // send command to trim sensors
      int trimSensors(bool verbose = false) {
        std::stringstream  command_buffer;
        command_buffer << "AT*FTRIM=" << command_sequence++ << ",\r";
        
        if (verbose) 
          std::cout << command_buffer.str() << std::endl;
        
        return sendCommand(command_buffer.str());
      }
      
      // send command to take off
      int takeOff(bool verbose = false) {
        std::stringstream  command_buffer;
        command_buffer << "AT*REF=" << command_sequence++ << ",290718208\r";
        
        if (verbose) 
          std::cout << command_buffer.str() << std::endl;
        
        return sendCommand(command_buffer.str());
      }
      
      // send command to land
      int land(bool verbose = false) {
        std::stringstream  command_buffer;
        command_buffer << "AT*REF=" << command_sequence++ << ",290717696\r";
        
        if (verbose) 
          std::cout << command_buffer.str() << std::endl;
        
        return sendCommand(command_buffer.str());
      }
      
      // send command to toggle emergency flag
      int toggleEmergency(bool verbose = false) {
        std::stringstream  command_buffer;
        command_buffer << "AT*REF=" << command_sequence++ << ",290717952\r";
        
        if (verbose) 
          std::cout << command_buffer.str() << std::endl;
        
        return sendCommand(command_buffer.str());
      }
      
      // send command watchdog
      int keepConnection(bool verbose = false) {
        std::stringstream  command_buffer;
        command_buffer << "AT*COMWDG=" << command_sequence++ << "\r";
        
        if (verbose) 
          std::cout << command_buffer.str() << std::endl;
        
        return sendCommand(command_buffer.str());
      }
      
      // move drone 
      int move(bool ctrl, float _arg1, float _arg2, float _arg3, 
               float _arg4, float _arg5, float _arg6, bool verbose = false) {
        
        // arg0 : contrl mode 
        //        (bit2) absolute control mode 
        //        (bit1) combined yaw mode 
        //        (bit0) enable control : 0 : hovering mode, can't move but can rotate and as/descend
        //                                1 : control mode, full control
        // arg1 : left-right tilt [-1:1]
        // arg2 : front-back tilt [-1:1]
        // arg3 : vertical speed  [-1:1]
        // arg4 : angular         [-1:1]
        // arg5 
        
        int arg0;
        if (ctrl) { 
          arg0 = 1 + 2*(int)flag_cYaw+4*(int)flag_absl;
        } else {
          arg0 = 0 + 2*(int)flag_cYaw+4*(int)flag_absl;
        }
        arg1 = _arg1; arg2 = _arg2; arg3 = _arg3; 
        arg4 = _arg4; arg5 = _arg5; arg6 = _arg6; 
        
        std::stringstream  command_buffer;
        
#if ARDRONE_VERSION > 1
        command_buffer << "AT*PCMD_MAG=" << command_sequence++ << ',' 
                       << arg0 << ',' 
                       << *(int32_t*)(&arg1) << ',' << *(int32_t*)(&arg2) << ',' 
                       << *(int32_t*)(&arg3) << ',' << *(int32_t*)(&arg4) << ',' 
                       << *(int32_t*)(&arg5) << ',' << *(int32_t*)(&arg6) << "\r";
#else
        command_buffer << "AT*PCMD=" << command_sequence++ << ',' 
                       << arg0 << ',' 
                       << *(int32_t*)(&arg1) << ',' << *(int32_t*)(&arg2) << ',' 
                       << *(int32_t*)(&arg3) << ',' << *(int32_t*)(&arg4) << "\r";
        
#endif
        
        if (verbose) 
          std::cout << command_buffer.str() << std::endl;
        
        return sendCommand(command_buffer.str());
      }
      
      // stop bootstrap mode
      int enable_navdata_demo(bool verbose = false) {
        std::stringstream  command_buffer;
        command_buffer << "AT*CONFIG="     << command_sequence++      
                                           << ",\"general:navdata_demo\",\"TRUE\"\r";
        if (verbose) 
          std::cout << command_buffer.str() << std::endl;
        
        return sendCommand(command_buffer.str());
      }
      
      // send ack to start navdata
      int start_navdata(bool verbose = false) {
        std::stringstream  command_buffer;
        command_buffer << "AT*CTRL=" << command_sequence++ << ",0\r";
        
        if (verbose) 
          std::cout << command_buffer.str() << std::endl;
        
        return sendCommand(command_buffer.str());
      }
      
      // change config
      int change_config(std::string opt_name, std::string opt_value, bool verbose = false) {
        // TODO combine two commands to one stream
        std::stringstream  command_buffer;
        command_buffer  << "AT*CONFIG_IDS=" << command_sequence++      << ',' 
                                            << "\"" << SESSION_ID      << "\"," 
                                            << "\"" << USER_ID         << "\"," 
                                            << "\"" << APPLICATION_ID  << "\"\r";
        if (verbose) 
          std::cout << command_buffer.str() << std::endl;
        sendCommand(command_buffer.str());
        
        std::stringstream  command_buffer2;
        command_buffer2 << "AT*CONFIG="     << command_sequence++      << ',' 
                                            << "\"" << opt_name        << "\"," 
                                            << "\"" << opt_value       << "\"\r";
        if (verbose) 
          std::cout << command_buffer2.str() << std::endl;
        return sendCommand(command_buffer2.str());
      }
      
      // calibrate magnetometer
      int calibrate_magnetometer(bool verbose = false) {
        std::stringstream  command_buffer;
        command_buffer << "AT*CALIB=" << command_sequence++ << ",0\r";
        
        if (verbose) 
          std::cout << command_buffer.str() << std::endl;
        
        return sendCommand(command_buffer.str());
      }
      
      void start()
      {
        std::cout << "[COM] Starting command Thread\n";
        m_Thread = boost::thread(&Command::run, this);
      }
      
      void join()
      {
        terminated = true;
        m_Thread.join();
        std::cout << "[COM] Thread Terminated\n";
      }
      
      void run()
      {
        // enable navdata_demo mode
        std::cout << "[COM] enabling navdata_demo mode\n";
        while (media->navdata->optionSize() < 148) {
          enable_navdata_demo();
          start_navdata();
        }
        
        // change profile
        change_config("custom:session_id",     SESSION_ID);
        while(media->navdata->getControlCommandAck()!=1)
          keepConnection();
        
        change_config("custom:profile_id",     USER_ID);
        while(media->navdata->getControlCommandAck()!=1)
          keepConnection();
        
        change_config("custom:application_id", APPLICATION_ID);
        while(media->navdata->getControlCommandAck()!=1)
          keepConnection();
          
        // change video mode
        change_config("video:video_codec",     "131");
        while(media->navdata->getControlCommandAck()!=1)
          keepConnection();
        
        video_enabled = true;
        trimSensors();
        
        struct timespec start, stop;
        double accum;
        int PCMDcounter = 0;
        float tmp_arg1  = 0;
        float tmp_arg2  = 0;
        float tmp_arg3  = 0;
        float tmp_arg4  = 0;
        float tmp_arg5  = 0;
        float tmp_arg6  = 0;
        while (1) {
          clock_gettime( CLOCK_REALTIME, &stop);
          accum = ( stop.tv_sec - start.tv_sec )
                   + (double)( stop.tv_nsec - start.tv_nsec )
                   / (double)1000000000L;
          std::cout << "[COM] Command Frequence: " << 1/accum << "[Hz]\n";
          start = stop;
          
          // terminate only if drone is landed 
          if (terminated) {
            if (media->navdata->drone_land) {
              break;
            } else {
              land();
              continue;
            }
          }
          
          // if drone is in emergency mode
          if (media->navdata->getEmergencyState()) {
            // reset flags
            flag_takeoff = false;
            flag_land    = false;
            flag_PCMD    = false;
            flag_mgnt    = false;
            
            // reset emergency
            if (flag_reset){
              std::cout << "[COM] reset emergency\n";
              toggleEmergency();
              flag_reset = false;
              trimSensors();
            }
            
            keepConnection();
            continue;
          } 
          
          // if drone is on the ground
          if (media->navdata->drone_land) {
            std::cout << "[COM] drone is on the ground\n";
            // trim sensor
            if (flag_trim) {
              std::cout << "[COM] trim\n";
              trimSensors();
              flag_trim = false;
            } 
            // take off
            else if (flag_takeoff) {
              std::cout << "[COM] take off\n";
              takeOff();
              flag_takeoff = false;
              flag_land    = false;
              flag_mgnt    = false;
            }
            // wait
            else {
              std::cout << "[COM] :Watchdog\n";
              keepConnection();
            }
          }
          
          // if drone is on the air
          else {
            std::cout << "[COM] drone is on the air\n";
            // land 
            if (flag_land) {
              std::cout << "[COM] Land\n";
              flag_takeoff = false;
              flag_land    = false;
              land();
            }
            //calibrate magnetometer
            else if (flag_mgnt) {
              std::cout << "[COM] Calibrating Magnetometer\n";
              calibrate_magnetometer();
              flag_mgnt = false;
            }
            // move 
            else if (flag_PCMD) {
              std::cout << "[COM] PCMD\n";
              move(1, arg1, arg2, arg3, arg4, arg5, arg6);
              tmp_arg1 = arg1;
              tmp_arg2 = arg2;
              tmp_arg3 = arg3;
              tmp_arg4 = arg4;
              tmp_arg5 = arg5;
              tmp_arg6 = arg6;
              PCMDcounter = 10;
              flag_PCMD   = false;
            }
            // hover 
            else {
              if ( --PCMDcounter > 5){
                std::cout << "[COM] :PCMD remaining\n";
                move(1, tmp_arg1, tmp_arg2, tmp_arg3, tmp_arg4, tmp_arg5, tmp_arg6);
                tmp_arg1 *= .1;
                tmp_arg2 *= .1;
                tmp_arg3 *= .8;
                tmp_arg4 *= .8;
              } else if (--PCMDcounter > 0) {
                std::cout << "[COM] hover\n";
                arg1 = arg2 = arg3 = arg4 = 0;
                move(0, 0, 0, 0, 0, tmp_arg5, tmp_arg6);
              } else {
                std::cout << "[COM] :Watchdog\n";
                arg1 = arg2 = arg3 = arg4 = 0;
                keepConnection();
              }
            }
          }
        }
      }
  };
  #undef DRONE_COMMAND_PORT
};

#endif

