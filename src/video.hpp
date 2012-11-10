#ifndef VIDEO
#define VIDEO

#include <sstream>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

extern "C" {
  #include "libavcodec/avcodec.h"
  #include "libavformat/avformat.h"
  #include "libswscale/swscale.h"
}

#include <cv.h>
#include <highgui.h>
//#include "opencv2/core/core.hpp"
//#include "opencv2/highgui/highgui.hpp"
//#include "opencv2/imgproc/imgproc.hpp"

#include "SDL2/SDL.h"
#include "ArDrone2.hpp"

# define DRONE_VIDEO_PORT 5555
namespace GreyCardinal {
  
  class Decode {
    
    private: 
      
      std::stringstream      dest_addr;
      
      AVFormatContext   *pFormatCtx;
      AVCodecContext    *pCodecCtx;
      AVCodec           *pCodec;
      AVFrame           *pFrame;
      AVPacket          packet;
      
      boost::thread     m_Thread;
      
      bool              terminated;
      
      GreyCardinal::Media       *media;
      
    public:
      
      // decode flag
      int32_t           frameDecoded;
      bool              started;
      
      Decode(GreyCardinal::Media* _media, std::string _dest_addr){
        // define two way access from/to media object to this objct
        media         = _media;
        media->decode = this;
        dest_addr << "http://" << _dest_addr << ':' << DRONE_VIDEO_PORT;
        pFormatCtx  = NULL;
        pCodec = NULL;
        pCodecCtx= NULL;
        pFrame= NULL;
        frameDecoded=0;
        started     = false;
        terminated  = false;
        std::cout << "[DEC] Initialized\n";
      }
      
      AVCodecContext* getpCodecCtx(){
        return  pCodecCtx;
      }
      
      AVFrame* getpFrame(){
        return  pFrame;
      }
      
      void start()
      {
        std::cout << "[DEC] Starting Video Thread\n";
        m_Thread = boost::thread(&Decode::run, this);
      }
      
      void join()
      {
        terminated = true;
        m_Thread.join();
        std::cout << "[DEC] Thread Terminated\n";
      }
      
      void run()
      {
        std::cout << "[DEC] Initializing Decode System.\n";
        // 1.1 Register all formats and codecs
        av_register_all();
        avcodec_register_all();
        avformat_network_init();
        
        // 1.2. Open video file
        std::string dest_addr_ = dest_addr.str();
        while(avformat_open_input(&pFormatCtx, dest_addr_.c_str(), NULL, NULL) != 0)
          std::cout << "[DEC] Could not open the video stream, retrying...\n";
        
        // 1.3. Retrieve stream information
        avformat_find_stream_info(pFormatCtx, NULL);
        // Dump information about file to standard output
        av_dump_format(pFormatCtx, 0, dest_addr_.c_str(), 0);
        
        // 1.4. Get a pointer to the codec context for the video stream
        // and find the decoder for the video stream
        pCodecCtx = pFormatCtx->streams[0]->codec;
        pCodec    = avcodec_find_decoder(pCodecCtx->codec_id);
        
        // 1.5. Open Codec
        avcodec_open2(pCodecCtx, pCodec, NULL); 
        
        // 1.6. get video frames
        started = true;
        pFrame = avcodec_alloc_frame();
        while(!terminated) {
          
          // read frame
          if(av_read_frame(pFormatCtx, &packet)<0) {
            std::cout << "[DEC] Could not read frame!\n";
            continue;
          }
          
          // decode the frame
          if(avcodec_decode_video2(pCodecCtx, pFrame, &frameDecoded, &packet) < 0) {
            std::cout << "[DEC] Could not decode frame!\n";
            continue;
          }
        }
        
        // release
        av_free(pFrame);
        avcodec_close(pCodecCtx); // <- before freeing this, all other objects, allocated after this, must be freed
        avformat_close_input(&pFormatCtx);
      }
  };
  #undef DRONE_VIDEO_PORT
  
  class OpenCV {
    private: 
      
      // for decode
      AVCodecContext    *pCodecCtx;
      AVFrame           *pFrame;
      
      // for conversion
      AVFrame           *pFrame_BGR24_1;
      AVFrame           *pFrame_BGR24_2;
      uint8_t           *buffer_BGR24_1;
      uint8_t           *buffer_BGR24_2;
      struct SwsContext *pConvertCtx_BGR24;
      
      // multi threading
      boost::thread     m_Thread;
      bool              terminated;
      GreyCardinal::Media       *media;
      
      // recording video
      cv::VideoWriter   videoWriter;
      
    public:
      
      // flag
      bool              save_image;
      bool              started;
      bool              record_video;
      
      OpenCV(GreyCardinal::Media*   _media){
        terminated   = false;
        started      = false;
        save_image   = false;
        record_video = false;
        media         = _media;
        media->opencv = this;
        pFrame_BGR24_1 = NULL;
        pFrame_BGR24_2 = NULL;
        buffer_BGR24_1 = NULL;
        buffer_BGR24_2 = NULL;
        pConvertCtx_BGR24 = NULL;
        pCodecCtx = NULL;
        pFrame = NULL;
        
        std::cout << "[OCV] Initialized\n";
      }
      
      AVFrame* getpFrame_BGR24(){
        return  pFrame_BGR24_2;
      }
      
      void start()
      {
        std::cout << "[OCV] Starting Display Thread\n";
        m_Thread = boost::thread(&OpenCV::run, this);
      }
      
      void join()
      {
        terminated = true;
        m_Thread.join();
        std::cout << "[OCV] Thread Terminated\n";
      }
      
      void run()
      {
        std::cout << "[OCV] Initializing conversion\n";
        pCodecCtx = media->decode->getpCodecCtx();
        pFrame    = media->decode->getpFrame();
        
        // 4.2 do somethig with OpenCV 
        cv::Mat   img0(pCodecCtx->height, pCodecCtx->width, CV_8UC3, cv::Scalar(255));
        cv::Mat   img1(pCodecCtx->height, pCodecCtx->width, CV_8UC3, cv::Scalar(255));
        
        // 2.2.1. Prepare format conversion for OpenCV
        // Allocate an AVFrame structure
        pFrame_BGR24_1 = avcodec_alloc_frame();
        pFrame_BGR24_2 = avcodec_alloc_frame();
        if(pFrame_BGR24_1 == NULL || pFrame_BGR24_2 == NULL) {
          std::cout << "[OCV] Could not allocate pFrame_BGR24P\n";
          exit(1);
        }
        // Determine required buffer size and allocate buffer
        buffer_BGR24_1 = (uint8_t *)av_malloc(avpicture_get_size(PIX_FMT_BGR24, 
                                                               pCodecCtx->width, pCodecCtx->height));
        buffer_BGR24_2 = (uint8_t *)av_malloc(avpicture_get_size(PIX_FMT_BGR24, 
                                                               pCodecCtx->width, pCodecCtx->height));
        // Assign buffer to image planes
        avpicture_fill((AVPicture *)pFrame_BGR24_1, buffer_BGR24_1, 
                            PIX_FMT_BGR24, pCodecCtx->width, pCodecCtx->height);
        avpicture_fill((AVPicture *)pFrame_BGR24_2, buffer_BGR24_2  , 
                            PIX_FMT_BGR24, pCodecCtx->width, pCodecCtx->height);
        // format conversion context
        pConvertCtx_BGR24 = sws_getContext(pCodecCtx->width, pCodecCtx->height, pCodecCtx->pix_fmt, 
                                           pCodecCtx->width, pCodecCtx->height, PIX_FMT_BGR24, 
                                           SWS_SPLINE, NULL, NULL, NULL);
        
        // initlizing video writer
        videoWriter.open("./output.mpg", CV_FOURCC('P','I','M','1'), 30, 
                                          cv::Size(pCodecCtx->width, pCodecCtx->height));
        
        // 1.6. get video frames
        started = true;
        while(!terminated) {
          if (media->decode->frameDecoded) {
            // 2.2.2. convert frame to GRAYSCALE [or BGR] for OpenCV
            sws_scale(pConvertCtx_BGR24,   (const uint8_t * const*)pFrame->data, pFrame->linesize, 0,
                      pCodecCtx->height,   pFrame_BGR24_1->data,   pFrame_BGR24_1->linesize);
            
            // copy the BGR image to this object
            memcpy(img0.data, pFrame_BGR24_1->data[0], (pCodecCtx->width)*(pCodecCtx->height)*sizeof(uint8_t)*3);
            
            // save image if Enter is pressed
            if (save_image){
              std::time_t sec = std::time(NULL);
              char *time = ctime(&sec);
              time[strlen(time)-1] = '\0';
              std::stringstream filenamestring;
              filenamestring << "img_" << time << ".png";
              cv::imwrite(filenamestring.str(), img0);
              save_image = false;
            }
            
            // sharpen image with Gaussian Blur
            cv::GaussianBlur(img0, img1, cv::Size(3, 3), cv::BORDER_DEFAULT);
            cv::addWeighted(img0, 1.5, img1, -0.5, 0, img0);
            cv::Mat tmp;
            cv::cvtColor(img0, tmp, CV_BGR2GRAY);
            cv::Canny(tmp, tmp, 100, 300, 3);
            cv::cvtColor(tmp, img0, CV_GRAY2BGR);
            
            // record processed video
            if (record_video) { 
              if (videoWriter.isOpened()) { 
                std::cout << "[OCV] recording video frame" << std::endl;
                videoWriter.write(img0);
              } else {
                std::cout << "[OCV] Error: video writer is not opened" << std::endl;
              }
            }
            
            // copy back the modified image
            memcpy(pFrame_BGR24_2->data[0], img0.data, (pCodecCtx->width)*(pCodecCtx->height)*sizeof(uint8_t)*3);
          }
        }
        
      }
  };
  
  class Display {
    
    private: 
      
      // for decode
      AVCodecContext    *pCodecCtx;
      AVFrame           *pFrame;
      
      // for conversion
      AVFrame           *pFrame_YUV420P;
      uint8_t           *buffer_YUV420P;
      struct SwsContext *pConvertCtx_YUV420P;
      
      AVFrame           *pFrame_BGR24;
      // displaying
      SDL_Window        *pWindow1;
      SDL_Renderer      *pRenderer1;
      SDL_Texture       *bmpTex1;
      uint8_t           *pixels1;
      int               pitch1, size1;
      
      SDL_Window        *pWindow2;
      SDL_Renderer      *pRenderer2;
      SDL_Texture       *bmpTex2;
      uint8_t           *pixels2;
      int               pitch2, size2;
      
      // multi threading
      boost::thread     m_Thread;
      bool              terminated;
      
      // event hundling
      SDL_Event         event;
      
      // thread accessing 
      GreyCardinal::Media       *media;
      
    public:
      
      Display(GreyCardinal::Media*      _media){
        // 0. Initializes the video subsystem *must be done before anything other!!
        if (SDL_Init(SDL_INIT_VIDEO) < 0) {
          std::cout << "[DIS] Unable to init SDL: " << SDL_GetError() << std::endl;
          exit(1);
        }
        
        // define two way access from/to media object to this objct
        media          = _media;
        media->display = this;
        terminated     = false;
        buffer_YUV420P  =NULL;
        pFrame  =NULL;
        bmpTex2  =NULL;pFrame_YUV420P  =NULL;
        pCodecCtx = NULL;pFrame_BGR24= NULL;
        bmpTex1 = NULL;pWindow1 = NULL;pRenderer1  =NULL; pixels1 = NULL;pitch1=-1; size1=-1;
        bmpTex2=NULL;pWindow2 = NULL;pRenderer2  =NULL; pixels2 = NULL;pitch2=-1; size2=-1;
        pConvertCtx_YUV420P = NULL;
        std::cout << "[DIS] Initialized\n";
      }
      
      ~Display(){
        SDL_Quit();
      }
      
      void start() {
        std::cout << "[DIS] Starting Display Thread\n";
        m_Thread = boost::thread(&Display::run, this);
      }
      
      void join() {
        //terminated = true;
        m_Thread.join();
        std::cout << "[DIS] Thread Terminated\n";
      }
      
      void run() {
        std::cout << "[DIS] Initializing Video Display System.\n";
        pCodecCtx = media->decode->getpCodecCtx();
        pFrame    = media->decode->getpFrame();
        
        pFrame_BGR24 = media->opencv->getpFrame_BGR24();
        
        // 2.1.1. Prepare format conversion for diplaying with SDL
        // Allocate an AVFrame structure
        pFrame_YUV420P = avcodec_alloc_frame();
        if(pFrame_YUV420P == NULL) {
          std::cout << "[DIS] Could not allocate pFrame_YUV420P\n";
          exit(1);
        }
        
        // Determine required buffer size and allocate buffer
        buffer_YUV420P = (uint8_t *)av_malloc(avpicture_get_size(PIX_FMT_YUV420P, 
                                                  pCodecCtx->width, pCodecCtx->height));  
        // Assign buffer to image planes
        avpicture_fill((AVPicture *)pFrame_YUV420P, buffer_YUV420P, 
                            PIX_FMT_YUV420P, pCodecCtx->width, pCodecCtx->height);
        // format conversion context
        pConvertCtx_YUV420P = sws_getContext(pCodecCtx->width, pCodecCtx->height, pCodecCtx->pix_fmt, 
                                             pCodecCtx->width, pCodecCtx->height, PIX_FMT_YUV420P, 
                                             SWS_SPLINE, NULL, NULL, NULL);
        
        // 3.1.1 prepare SDL for YUV
        // allocate window, renderer, texture
        pWindow1    = SDL_CreateWindow( "YUV", 0, 0, pCodecCtx->width, pCodecCtx->height, SDL_WINDOW_SHOWN);  
        pRenderer1  = SDL_CreateRenderer(pWindow1, -1, SDL_RENDERER_ACCELERATED);
        bmpTex1     = SDL_CreateTexture(pRenderer1, SDL_PIXELFORMAT_YV12, 
                                        SDL_TEXTUREACCESS_STREAMING, pCodecCtx->width, pCodecCtx->height);
        size1       = pCodecCtx->width * pCodecCtx->height;
        if(pWindow1==NULL || pRenderer1==NULL || bmpTex1==NULL) {
          std::cout << "[DIS] Could not open window1: " << SDL_GetError() << std::endl;
          exit(1);
        }
        
        // 3.2.1 prepare SDL for BGR
        // allocate window, renderer, texture
        pWindow2    = SDL_CreateWindow( "BGR", pCodecCtx->width+5, 0, pCodecCtx->width, pCodecCtx->height, SDL_WINDOW_SHOWN);  
        pRenderer2  = SDL_CreateRenderer(pWindow2, -1, SDL_RENDERER_ACCELERATED);
        bmpTex2     = SDL_CreateTexture(pRenderer2, SDL_PIXELFORMAT_BGR24, 
                                        SDL_TEXTUREACCESS_STREAMING, pCodecCtx->width, pCodecCtx->height);
        size2       = pCodecCtx->width * pCodecCtx->height * 3;
        if(pWindow2==NULL || pRenderer2==NULL || bmpTex2==NULL) {
          std::cout << "[DIS] Could not open window2: " << SDL_GetError() << std::endl;
          exit(1);
        }
        
        // 1.6. get video frames
        while(!terminated) {
          if (media->decode->frameDecoded) {
            // 2.1.2. convert frame to YUV for Displaying
            sws_scale(pConvertCtx_YUV420P, (const uint8_t * const*)pFrame->data, pFrame->linesize, 0,
                      pCodecCtx->height,   pFrame_YUV420P->data, pFrame_YUV420P->linesize);
            
            // 3.1.2. copy converted YUV to SDL 2.0 texture
            SDL_LockTexture(bmpTex1, NULL, (void **)&pixels1, &pitch1);
            memcpy(pixels1,             pFrame_YUV420P->data[0], size1  );
            memcpy(pixels1 + size1,     pFrame_YUV420P->data[2], size1/4);
            memcpy(pixels1 + size1*5/4, pFrame_YUV420P->data[1], size1/4);
            SDL_UnlockTexture(bmpTex1);
            SDL_UpdateTexture(bmpTex1, NULL, pixels1, pitch1);
            
            // refresh screen
            SDL_RenderClear(pRenderer1);
            SDL_RenderCopy(pRenderer1, bmpTex1, NULL, NULL);
            SDL_RenderPresent(pRenderer1);
            
            // 3.2.2. copy converted BGR to SDL 2.0 texture
            SDL_LockTexture(bmpTex2, NULL, (void **)&pixels2, &pitch2);
            memcpy(pixels2,             pFrame_BGR24->data[0], size2);
            SDL_UnlockTexture(bmpTex2);
            SDL_UpdateTexture(bmpTex2, NULL, pixels2, pitch2);
            
            // refresh screen
            SDL_RenderClear(pRenderer2);
            SDL_RenderCopy(pRenderer2, bmpTex2, NULL, NULL);
            SDL_RenderPresent(pRenderer2);
          }
          
          // process user input
          SDL_PollEvent(&event);
          switch (event.type) {
            case SDL_KEYDOWN:
              // check for the keyboard state
              switch (event.key.keysym.sym) {
                // quit
                case SDLK_ESCAPE:
                  std::cout << "[DIS] ESC:" << event.key.keysym.sym << " was pressed\n";
                  media->display->terminated = true;
                  break;
                // reset emergency
                case SDLK_r:
                  std::cout << "[DIS] r:" << event.key.keysym.sym << " was pressed\n";
                  media->command->flag_reset = true;
                  break;
                // save image
                case SDLK_s:
                  std::cout << "[DIS] s:" << event.key.keysym.sym << " was pressed\n";
                  media->opencv->save_image = true;
                  break;
                // toggle video recording
                case SDLK_v:
                  std::cout << "[DIS] v:" << event.key.keysym.sym << " was pressed\n";
                  media->opencv->record_video = !media->opencv->record_video;
                  break;// Add Valentin!!!
                // trim sensors
                case SDLK_BACKSPACE:
                  std::cout << "[DIS] Backspace:" << event.key.keysym.sym << " was pressed\n";
                  media->command->flag_trim = true;
                  break;
                // land
                case SDLK_SPACE:
                  std::cout << "[DIS] Space:" << event.key.keysym.sym << " was pressed\n";
                  media->command->flag_land = true;
                  break;
                // toggle absolute control mode
                case SDLK_t:
                  std::cout << "[DIS] t:" << event.key.keysym.sym << " was pressed\n";
                  media->command->flag_absl = !(media->command->flag_absl);
                  break;
                // toggle combined yaw mode
                case SDLK_y:
                  std::cout << "[DIS] y:" << event.key.keysym.sym << " was pressed\n";
                  media->command->flag_cYaw = !(media->command->flag_cYaw);
                  break;
                // calibrate magnetometer
                case SDLK_c:
                  std::cout << "[DIS] c:" << event.key.keysym.sym << " was pressed\n";
                  media->command->flag_mgnt = true;
                  break;
                // take off
                case SDLK_RETURN:
                  std::cout << "[DIS] Enter:" << event.key.keysym.sym << " was pressed\n";
                  media->command->flag_takeoff = true;
                  media->opencv->record_video  = true;
                  break;
                default:
                  std::cout << "[DIS] " << event.key.keysym.sym << " was pressed\n";
                  break;
              }
            break;
          }
          
          // check for the keyboard state for moving keys
          Uint8 *state = SDL_GetKeyboardState(NULL);
          if (state[SDL_SCANCODE_KP_8]   || state[SDL_SCANCODE_KP_2] || 
              state[SDL_SCANCODE_KP_6]   || state[SDL_SCANCODE_KP_4] || 
              state[SDL_SCANCODE_KP_9]   || state[SDL_SCANCODE_KP_7] || 
              state[SDL_SCANCODE_KP_5]   || state[SDL_SCANCODE_KP_3] || 
              state[SDL_SCANCODE_LSHIFT] || state[SDL_SCANCODE_RSHIFT]
             ) {
            
            float   mult;
            float   base = 0.015;
            if(state[SDL_SCANCODE_LSHIFT] || state[SDL_SCANCODE_RSHIFT]){
              mult = 6;
            } else {
              mult = 1;
            }
            
            // 8 for ascend
            if ( state[SDL_SCANCODE_KP_8] ) {
              if (media->command->arg3 < 0 )
                media->command->arg3 = 0;
              media->command->arg3 += 2*mult*base;
              if (media->command->arg3 >  1 )
                media->command->arg3 =  1;
            }
            // 2 for descend
            if ( state[SDL_SCANCODE_KP_2] ) { 
              if (media->command->arg3 > 0 )
                media->command->arg3 = 0;
              media->command->arg3 -= 2*mult*base;
              if (media->command->arg3 < -1 )
                media->command->arg3 = -1;
            }
            // 6 for move right
            if ( state[SDL_SCANCODE_KP_6] ) { 
              if (media->command->arg1 < 0 )
                media->command->arg1 = 0;
              media->command->arg1 += mult*base;
              if (media->command->arg1 >  1 )
               media->command->arg1 =  1;
            }
            // 4 for move left
            if ( state[SDL_SCANCODE_KP_4] ) { 
              if (media->command->arg1 > 0 )
                media->command->arg1 = 0;
              media->command->arg1 -= mult*base;
              if (media->command->arg1 < -1 )
                media->command->arg1 = -1;
            }
            // 3 for move backword
            if ( state[SDL_SCANCODE_KP_3] ) { 
              if (media->command->arg2 < 0 )
                media->command->arg2 = 0;
              media->command->arg2 += mult*base;
              if (media->command->arg2 >  1 )
               media->command->arg2 =  1;
            }
            // 5 for move forward
            if ( state[SDL_SCANCODE_KP_5] ) { 
              if (media->command->arg2 > 0 )
                media->command->arg2 = 0;
              media->command->arg2 -= mult*base;
              if (media->command->arg2 < -1 )
                media->command->arg2 = -1;
            }
            // 9 for turn right
            if ( state[SDL_SCANCODE_KP_9] ) { 
              if (media->command->arg4 < 0 )
                media->command->arg4 = 0;
              media->command->arg4 += mult*base;
              if (media->command->arg4 >  1 )
               media->command->arg4 =  1;
            }
            // 7 for turn left
            if ( state[SDL_SCANCODE_KP_7] ) { 
              if (media->command->arg4 > 0 )
                media->command->arg4 = 0;
              media->command->arg4 -= mult*base;
              if (media->command->arg4 < -1 )
                media->command->arg4 = -1;
            }
            
            media->command->flag_PCMD  = true;
          }
          boost::this_thread::sleep (boost::posix_time::microseconds (30000));
        }
        
        // release
        // *note SDL objects have to be freed before closing avcodec.
        // otherwise it causes segmentation fault for some reason.
        SDL_DestroyTexture(bmpTex1);
        SDL_DestroyTexture(bmpTex2);
        
        SDL_DestroyRenderer(pRenderer1);
        SDL_DestroyRenderer(pRenderer2);
        
        SDL_DestroyWindow(pWindow1);
        SDL_DestroyWindow(pWindow2);
        
        av_free(pFrame_YUV420P);
        av_free(buffer_YUV420P);
        sws_freeContext(pConvertCtx_YUV420P);
        
      }
  };
  

}
#endif
