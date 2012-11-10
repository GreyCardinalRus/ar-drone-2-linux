// g++ -o testDrone2_video_withOpenCV testDrone2_video_withOpenCV.cpp -lSDL2 -lavcodec -lavformat -lswscale `pkg-config opencv --cflags --libs`
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
extern "C" {
  #include "libavcodec/avcodec.h"
  #include "libavformat/avformat.h"
  #include "libswscale/swscale.h"
}

#include "SDL2/SDL.h"
#include <cv.h>
#include <highgui.h>

int mai_n(int argc, char* argv[]) {

  // 3.0. Initializes the video subsystem *must be done before anything other!!
  if (SDL_Init(SDL_INIT_VIDEO) < 0) {
    fprintf(stderr, "Unable to init SDL: %s\n", SDL_GetError());
    return -1;
  }

  // prepare variables
  // decoding
  char              *drone_addr = "http://192.168.1.1:5555";
  AVFormatContext   *pFormatCtx = NULL;
  AVCodecContext    *pCodecCtx;
  AVCodec           *pCodec;
  AVPacket          packet;
  AVFrame           *pFrame;
  int               terminate, frameDecoded;

  // converting
  AVFrame           *pFrame_YUV420P;
  uint8_t           *buffer_YUV420P;
  struct SwsContext *pConvertCtx_YUV420P;

  AVFrame           *pFrame_BGR24;
  uint8_t           *buffer_BGR24;
  struct SwsContext *pConvertCtx_BGR24;

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

  SDL_Event         event;

  // 1.1 Register all formats and codecs
  av_register_all();
  avcodec_register_all();
  avformat_network_init();

  // 1.2. Open video file
  while(avformat_open_input(&pFormatCtx, drone_addr, NULL, NULL) != 0)
    printf("Could not open the video file\nRetrying...\n");

  // 1.3. Retrieve stream information
  avformat_find_stream_info(pFormatCtx, NULL);
  // Dump information about file to standard output
  av_dump_format(pFormatCtx, 0, drone_addr, 0);

  // 1.4. Get a pointer to the codec context for the video stream
  // and find the decoder for the video stream
  pCodecCtx = pFormatCtx->streams[0]->codec;
  pCodec    = avcodec_find_decoder(pCodecCtx->codec_id);

  // 1.5. Open Codec
  avcodec_open2(pCodecCtx, pCodec, NULL);


  // 2.1.1. Prepare format conversion for diplaying with SDL
  // Allocate an AVFrame structure
  pFrame_YUV420P = avcodec_alloc_frame();
  if(pFrame_YUV420P == NULL) {
    fprintf(stderr, "Could not allocate pFrame_YUV420P\n");
    return -1;
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

  // 2.2.1. Prepare format conversion for OpenCV
  // Allocate an AVFrame structure
  pFrame_BGR24 = avcodec_alloc_frame();
  if(pFrame_BGR24 == NULL) {
    fprintf(stderr, "Could not allocate pFrame_YUV420P\n");
    return -1;
  }
  // Determine required buffer size and allocate buffer
  buffer_BGR24 = (uint8_t *)av_malloc(avpicture_get_size(PIX_FMT_BGR24,
                                            pCodecCtx->width, pCodecCtx->height));
  // Assign buffer to image planes
  avpicture_fill((AVPicture *)pFrame_BGR24, buffer_BGR24,
                      PIX_FMT_BGR24, pCodecCtx->width, pCodecCtx->height);
  // format conversion context
  pConvertCtx_BGR24 = sws_getContext(pCodecCtx->width, pCodecCtx->height, pCodecCtx->pix_fmt,
                                     pCodecCtx->width, pCodecCtx->height, PIX_FMT_BGR24,
                                     SWS_SPLINE, NULL, NULL, NULL);

  // 3.1.1 prepare SDL for YUV
  // allocate window, renderer, texture
  pWindow1    = SDL_CreateWindow( "YUV", 0, 0, pCodecCtx->width, pCodecCtx->height, SDL_WINDOW_SHOWN);
  pRenderer1  = SDL_CreateRenderer(pWindow1, -1, SDL_RENDERER_ACCELERATED);
  bmpTex1     = SDL_CreateTexture(pRenderer1, SDL_PIXELFORMAT_YV12, SDL_TEXTUREACCESS_STREAMING, pCodecCtx->width, pCodecCtx->height);
  size1       = pCodecCtx->width * pCodecCtx->height;
  if(pWindow1==NULL || pRenderer1==NULL || bmpTex1==NULL) {
    fprintf(stderr, "Could not open window1\n");
    return -1;
  }

  // 3.2.1 prepare SDL for BGR
  // allocate window, renderer, texture
  pWindow2    = SDL_CreateWindow( "BGR", pCodecCtx->width+5, 0, pCodecCtx->width, pCodecCtx->height, SDL_WINDOW_SHOWN);
  pRenderer2  = SDL_CreateRenderer(pWindow2, -1, SDL_RENDERER_ACCELERATED);
  bmpTex2     = SDL_CreateTexture(pRenderer2, SDL_PIXELFORMAT_BGR24, SDL_TEXTUREACCESS_STREAMING, pCodecCtx->width, pCodecCtx->height);
  size2       = pCodecCtx->width * pCodecCtx->height * 3;
  if(pWindow2==NULL || pRenderer2==NULL || bmpTex2==NULL) {
    fprintf(stderr, "Could not open window2\n");
    return -1;
  }

  // 4.1 Prepare OpenCV matrices
  cv::Mat   img0(pCodecCtx->height, pCodecCtx->width, CV_8UC3, cv::Scalar(255));
  cv::Mat   img1(pCodecCtx->height, pCodecCtx->width, CV_8UC3, cv::Scalar(255));

  // 1.6. get video frames
  pFrame = avcodec_alloc_frame();
  terminate = 0;
  while(!terminate) {
    // read frame
    if(av_read_frame(pFormatCtx, &packet)<0) {
      fprintf(stderr, "Could not read frame!\n");
      continue;
    }

    // decode the frame
    if(avcodec_decode_video2(pCodecCtx, pFrame, &frameDecoded, &packet) < 0) {
      fprintf(stderr, "Could not decode frame!\n");
      continue;
    }

    if (frameDecoded) {
      // 2.1.2. convert frame to YUV for Displaying
        sws_scale(pConvertCtx_YUV420P, (const uint8_t * const*)pFrame->data, pFrame->linesize, 0,
                  pCodecCtx->height,   pFrame_YUV420P->data, pFrame_YUV420P->linesize);
      // 2.2.2. convert frame to GRAYSCALE [or BGR] for OpenCV
        sws_scale(pConvertCtx_BGR24,   (const uint8_t * const*)pFrame->data, pFrame->linesize, 0,
                  pCodecCtx->height,   pFrame_BGR24->data,   pFrame_BGR24->linesize);

      // 4.2 do somethig with OpenCV
      // copy the BGR image
      memcpy(img0.data, pFrame_BGR24->data[0], (pCodecCtx->width)*(pCodecCtx->height)*sizeof(uint8_t)*3);
      // sharpen image with Gaussian Blur
      cv::GaussianBlur(img0, img1, cv::Size(3, 3), cv::BORDER_DEFAULT);
      cv::addWeighted(img0, 1.5, img1, -0.5, 0, img0);
      // copy back the modified image
      memcpy(pFrame_BGR24->data[0], img0.data, (pCodecCtx->width)*(pCodecCtx->height)*sizeof(uint8_t)*3);

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

    SDL_PollEvent(&event);
    switch (event.type) {
      case SDL_KEYDOWN:
        terminate = 1;
        break;
    }
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

  img0.release();
  img1.release();

  av_free(pFrame_YUV420P);
  av_free(buffer_YUV420P);
  sws_freeContext(pConvertCtx_YUV420P);

  av_free(pFrame_BGR24);
  av_free(buffer_BGR24);
  sws_freeContext(pConvertCtx_BGR24);

  av_free(pFrame);
  avcodec_close(pCodecCtx); // <- before freeing this, all other objects, allocated after this, must be freed
  avformat_close_input(&pFormatCtx);

  SDL_Quit();

  return 0;

}
