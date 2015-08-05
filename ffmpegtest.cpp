/*
* FFMpeg simple test program
*
* This program can be used and distributed without restrictions.
*
* Authors: Ivan Zaitsev
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <unistd.h>
#include <getopt.h>
#include <g2d.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <binder/IServiceManager.h>
#include <binder/ProcessState.h>
#include <media/ICrypto.h>
#include <media/stagefright/foundation/ABuffer.h>
#include <media/stagefright/foundation/ADebug.h>
#include <media/stagefright/foundation/ALooper.h>
#include <media/stagefright/foundation/AMessage.h>
#include <media/stagefright/foundation/AString.h>
#include <media/stagefright/DataSource.h>
#include <media/stagefright/MediaCodec.h>
#include <media/stagefright/MediaCodecList.h>
#include <media/stagefright/MediaDefs.h>
#include <media/stagefright/MetaData.h>
#include <media/stagefright/NativeWindowWrapper.h>
#include <gui/ISurfaceComposer.h>
#include <gui/SurfaceComposerClient.h>
#include <gui/Surface.h>
#include <ui/DisplayInfo.h>
#include <android/native_window.h>
#include "Utils.h"

#define UINT64_C(x) (x ## ULL)
extern "C" {
#include <libavcodec/avcodec.h>
}

using namespace android;

//--------------------------------------------------------------------------
#define INBUF_SIZE              0x20000
#define NUM_OF_G2D_BUFFERS      3
#define STREAM_MIME_TYPE        "video/avc"

//--------------------------------------------------------------------------
typedef struct {
  size_t   mIndex;
  size_t   mOffset;
  size_t   mSize;
  int64_t  mPresentationTimeUs;
  uint32_t mFlags;
}BufferInfo;

uint8_t   inBuff[INBUF_SIZE + FF_INPUT_BUFFER_PADDING_SIZE];
uint32_t  inBuffSize;

sp<SurfaceComposerClient> gComposerClient;
sp<SurfaceControl>        gControl;
sp<Surface>               gSurface;
sp<NativeWindowWrapper>   gNativeWindowWrapper;
struct ANativeWindow     *gNativeWindow;
sp<MediaCodec>            gCodec;
sp<ALooper>               gLooper;
Vector<sp<ABuffer> >      gInBuffers;
Vector<sp<ABuffer> >      gOutBuffers;
struct g2d_buf*           gBuffers[ NUM_OF_G2D_BUFFERS ];
bool                      bSDecoder,
                          bHDecoder,
                          bCSC,
                          bSurface;

// --------------------------------------------------------------------------------
static void initOutputSurface( void )
{
  gComposerClient = new SurfaceComposerClient;
  if( OK == gComposerClient->initCheck() )
  {
    DisplayInfo info;
    sp<IBinder> display( SurfaceComposerClient::getBuiltInDisplay(
                         ISurfaceComposer::eDisplayIdMain ) );

    SurfaceComposerClient::getDisplayInfo( display, &info );
    size_t displayWidth  = info.w;
    size_t displayHeight = info.h;

    fprintf( stderr, "display is %d x %d\n", displayWidth, displayHeight );

    gControl = gComposerClient->createSurface( String8("A Surface"),
                                               displayWidth,
                                               displayHeight,
                                               PIXEL_FORMAT_RGB_565,
                                               0 );

    if( ( gControl != NULL ) && ( gControl->isValid() ) )
    {
      SurfaceComposerClient::openGlobalTransaction();
      if( ( OK == gControl->setLayer( INT_MAX ) ) &&
          ( OK == gControl->show() ) )
      {
        SurfaceComposerClient::closeGlobalTransaction();
        gSurface = gControl->getSurface();
      }
    }
  }

  if( gSurface == NULL )
  {
    fprintf( stderr, "Screen surface create error\n" );
  }
  gNativeWindowWrapper = new NativeWindowWrapper( gSurface );
  gNativeWindow = gSurface.get();

  //native_window_set_buffers_transform( gNativeWindow, NATIVE_WINDOW_TRANSFORM_FLIP_V );
  native_window_set_buffers_format( gNativeWindow, HAL_PIXEL_FORMAT_YV12 );
  native_window_set_buffers_dimensions( gNativeWindow, 800, 480 );
  native_window_set_scaling_mode( gNativeWindow, NATIVE_WINDOW_SCALING_MODE_SCALE_TO_WINDOW );

  fprintf( stderr, "Screen surface created\n" );
  fprintf( stderr, "Screen surface %s\n", (gSurface->isValid(gSurface)?"valid":"invalid") );
}

//-----------------------------------------------------------------------------
static void initHwCodec( void )
{
  sp<AMessage> format;
  sp<MetaData> meta = new MetaData;
  FILE        *file;
  uint8_t      configData[ 1024 ];
  uint32_t     configDataSize;
  status_t     res;

  gLooper = new ALooper;
  gLooper->start();

  gCodec = MediaCodec::CreateByType( gLooper, STREAM_MIME_TYPE, false );
  if( gCodec != NULL )
  {
    fprintf( stderr, "Codec created\n" );
  }

  file = fopen( "/boot/carplay/AnnexB/carplay_video_000.avc", "rb" );
  if( !file )
  {
    fprintf( stderr, "Could not open stream config file.\n" );
    exit(1);
  }
  configDataSize = fread( configData, 1, sizeof( configData ), file );
  fclose( file );

  if( configDataSize <= 0 )
  {
    fprintf( stderr, "Could not read stream config file.\n" );
    exit(1);
  }
  meta->clear();
  meta->setCString( kKeyMIMEType,         STREAM_MIME_TYPE );
  meta->setInt32(   kKeyTrackID,          1 );
  meta->setInt32(   kKeyWidth,            800 );
  meta->setInt32(   kKeyHeight,           480 );
  meta->setInt32(   kKeyDisplayWidth,     800 );
  meta->setInt32(   kKeyDisplayHeight,    480 );
  meta->setData(    kKeyAVCC, kTypeAVCC,  configData, configDataSize );
  meta->dumpToLog();
  convertMetaDataToMessage( meta, &format );

  res = gCodec->configure( format, gNativeWindowWrapper->getSurfaceTextureClient(), NULL, 0 );
  if( res != OK )
  {
    fprintf( stderr, "Codec configure error: %d\n", res );
    exit(1);
  }

  res = gCodec->start();
  if( res != OK )
  {
    fprintf( stderr, "Codec start error: %d\n", res );
    exit(1);
  }

  res = gCodec->getInputBuffers( &gInBuffers );
  if( res != OK )
  {
    fprintf( stderr, "Codec get input buffers error: %d\n", res );
    exit(1);
  }

  res = gCodec->getOutputBuffers( &gOutBuffers );
  if( res != OK )
  {
    fprintf( stderr, "Codec get output buffers error: %d\n", res );
    exit(1);
  }

  sp<ABuffer> srcBuffer;
  size_t j = 0;
  while( format->findBuffer( StringPrintf("csd-%d", j).c_str(), &srcBuffer ) )
  {
    size_t index;
    res = gCodec->dequeueInputBuffer( &index, -1ll );
    if( res != OK )
    {
      fprintf( stderr, "Dequeue error\n" );
    }

    const sp<ABuffer> &dstBuffer = gInBuffers.itemAt( index );

    dstBuffer->setRange( 0, srcBuffer->size() );
    memcpy( dstBuffer->data(), srcBuffer->data(), srcBuffer->size() );

    fprintf( stderr, "CSD data size: %d\n", srcBuffer->size() );
    for( size_t i = 0; i < srcBuffer->size(); i++ )
    {
      fprintf( stderr, "0x%02X ", srcBuffer->data()[ i ] );
      if( !( ( i + 1 ) % 8) )
        fprintf( stderr, "\n" );
    }
    fprintf(stderr, "\n" );

    res = gCodec->queueInputBuffer( index,
                                    0,
                                    dstBuffer->size(),
                                    0ll,
                                    MediaCodec::BUFFER_FLAG_CODECCONFIG );
    if( res != OK )
    {
      fprintf( stderr, "Queue error\n" );
    }

    j++;
  }

  fprintf( stderr, "Codec init OK.\n" );
}

// --------------------------------------------------------------------------------
uint32_t g2d_vaddr2paddr(void* vaddr)
{
  for( int i = 0; i < NUM_OF_G2D_BUFFERS; i++ )
    if( gBuffers[ i ]->buf_vaddr == vaddr )
      return gBuffers[ i ]->buf_paddr;

  return (uint32_t)vaddr;
}

// --------------------------------------------------------------------------------
#define PAGEMAP_LENGTH 8
#define PAGE_SHIFT 12
uint32_t get_phy_address(void *addr)
{
   // Open the pagemap file for the current process
   FILE *pagemap = fopen("/proc/self/pagemap", "rb");

   // Seek to the page that the buffer is on
   uint32_t offset = (uint32_t)addr / getpagesize() * PAGEMAP_LENGTH;
   if(fseek(pagemap, offset, SEEK_SET) != 0) {
      fprintf(stderr, "Failed to seek pagemap to proper location\n");
      exit(1);
   }

   // The page frame number is in bits 0-54 so read the first 7 bytes and clear the 55th bit
   uint64_t page_frame_number = 0;
   fread(&page_frame_number, 1, PAGEMAP_LENGTH-1, pagemap);

   page_frame_number &= 0x7FFFFFFFFFFFFF;

   fclose(pagemap);



   return ( page_frame_number << PAGE_SHIFT ) + ((uint32_t)addr % getpagesize());
}

// --------------------------------------------------------------------------------
static void postFrame( AVFrame* frame )
{
  status_t res;
  ANativeWindow_Buffer buffer;
  void*                g2dHandle;
  struct g2d_surface   src,
                       dst;
  int buffIdx;

  if( bCSC )
  {
    for( buffIdx = 0; buffIdx < NUM_OF_G2D_BUFFERS; buffIdx++ )
    {
      if( !gBuffers[buffIdx] )
      {
        int buffSize = frame->linesize[buffIdx] * frame->height / (( buffIdx > 0 ) ? 2 : 1 );
        gBuffers[buffIdx] = g2d_alloc( buffSize, 0 );
        if( gBuffers[ buffIdx ] )
        {
          fprintf( stderr, "Alloc buff %d, paddr 0x%08X, vaddr 0x%08X, size %d\n",
                   buffIdx,
                   (uint32_t)gBuffers[ buffIdx ]->buf_paddr,
                   (uint32_t)gBuffers[ buffIdx ]->buf_vaddr,
                   (uint32_t)gBuffers[ buffIdx ]->buf_size );
        }
      }
    }

    if( g2d_open( &g2dHandle ) == -1 || g2dHandle == NULL )
    {
      fprintf( stderr, "Fail to open g2d device!\n" );
      return;
    }
  }

  if( bSurface )
  {
    res = gSurface->lock( &buffer, NULL );
    if( res != OK )
    {
      fprintf( stderr, "Lock surface buffer error: %d\n", res );
    }
  }

  if( bCSC )
  {
    memcpy(gBuffers[0]->buf_vaddr, frame->data[0], gBuffers[0]->buf_size);
    memcpy(gBuffers[1]->buf_vaddr, frame->data[1], gBuffers[1]->buf_size);
    memcpy(gBuffers[2]->buf_vaddr, frame->data[2], gBuffers[2]->buf_size);

    src.planes[0] = gBuffers[0]->buf_paddr;
    src.planes[1] = gBuffers[1]->buf_paddr;
    src.planes[2] = gBuffers[2]->buf_paddr;
    src.left      = 0;
    src.top       = 0;
    src.right     = frame->width;
    src.bottom    = frame->height;
    src.stride    = frame->linesize[0];
    src.width     = frame->width;
    src.height    = frame->height;
    src.rot       = G2D_ROTATION_0;
    src.format    = G2D_I420;

    dst.planes[0] = get_phy_address(buffer.bits);
    dst.left      = 0;
    dst.top       = 0;
    dst.right     = buffer.width;
    dst.bottom    = buffer.height;
    dst.stride    = buffer.stride;
    dst.width     = buffer.width;
    dst.height    = buffer.height;
    dst.rot       = G2D_ROTATION_0;
    dst.format    = G2D_RGB565;

    g2d_blit( g2dHandle, &src, &dst );
    g2d_finish( g2dHandle );
  }
  else
  {
    int Ysize = buffer.width * buffer.height;
    int Csize = Ysize / 4;

    memcpy( buffer.bits, frame->data[0], Ysize );
    memcpy( (uint8_t*)buffer.bits + Ysize, frame->data[2], Csize );
    memcpy( (uint8_t*)buffer.bits + Ysize + Csize, frame->data[1], Csize );
  }

  if( bSurface )
  {
    res = gSurface->unlockAndPost();
    if( res != OK )
    {
      fprintf( stderr, "Unlock surface buffer error: %d\n", res );
    }
  }
}

//-----------------------------------------------------------------------------
static int decode_write_frame( AVCodecContext *avctx,
                               AVFrame        *frame,
                               int            *frame_count,
                               AVPacket       *pkt,
                               int             last )
{
  int   len = 0,
        got_frame = 0;

  if( bSDecoder || ( *frame_count < 1 ) )
  {
    len = avcodec_decode_video2( avctx, frame, &got_frame, pkt );
    if( len < 0)
    {
      fprintf(stderr, "Error while decoding frame %d\n", *frame_count);
      return len;
    }
  }
  else
  {
    got_frame = 1;
  }

  if( got_frame )
  {
    if( bCSC || bSurface )
      postFrame( frame );

    (*frame_count)++;
  }

  if( pkt->data )
  {
    pkt->size -= len;
    pkt->data += len;
  }

  return 0;
}

//-----------------------------------------------------------------------------
static void decode( void )
{
  AVFrame        *frame;
  AVPacket        avpkt;
  AVCodec        *codec;
  AVCodecContext *c= NULL;
  char            inFileName[50];
  FILE           *fIn;
  int             i, frame_count;
  struct timeval  tv1, tv2;
  double          elapsedTime;

  avcodec_register_all();
  av_init_packet(&avpkt);

  codec = avcodec_find_decoder( AV_CODEC_ID_H264 );
  if( !codec )
  {
    fprintf(stderr, "Codec not found\n");
    exit(1);
  }

  c = avcodec_alloc_context3( codec );
  if( !c )
  {
    fprintf(stderr, "Could not allocate video codec context\n");
    exit(1);
  }

  if( avcodec_open2( c, codec, NULL ) < 0 )
  {
    fprintf(stderr, "Could not open codec\n");
    exit(1);
  }

  frame = avcodec_alloc_frame();
  if( !frame )
  {
    fprintf(stderr, "Could not allocate video frame\n");
    exit(1);
  }

  memset( inBuff, 0, sizeof( inBuff ) );
  frame_count = 0;
  gettimeofday( &tv1, NULL );
  for( i = 0; i <= 234; i++ )
  {
    sprintf( inFileName, "/boot/carplay/AnnexB/carplay_video_%03d.raw", i );
    fIn = fopen( inFileName, "rb" );
    if( !fIn )
    {
      fprintf(stderr, "Could not open %s\n", inFileName);
      exit(1);
    }

    avpkt.size = fread( inBuff, 1, INBUF_SIZE, fIn );
    if( avpkt.size == 0 )
    {
      fprintf(stderr, "File read error\n");
      break;
    }

    avpkt.data = inBuff;
    if( avpkt.size > 0 )
    {
      decode_write_frame( c, frame, &frame_count, &avpkt, 0 );
    }

    fclose( fIn );
  }
  gettimeofday( &tv2, NULL );
  elapsedTime = tv2.tv_sec - tv1.tv_sec;
  elapsedTime += ( tv2.tv_usec - tv1.tv_usec ) / 1000000.0f;
  fprintf(stderr, "Decoded %d frames in %5.3f sec = %5.2f fps\n", i, elapsedTime, i/elapsedTime );

  avcodec_close( c );
  av_free( c );
  avcodec_free_frame( &frame );
}

//-----------------------------------------------------------------------------
static void decodeHw( void )
{
  char            inFileName[50];
  FILE           *fIn;
  size_t          index;
  status_t        res;
  BufferInfo      info;
  int             i, frame_count;
  struct timeval  tv1, tv2;
  double          elapsedTime;

  memset( inBuff, 0, sizeof( inBuff ) );
  frame_count = 0;
  gettimeofday( &tv1, NULL );
  for( i = 1; i <= 234; i++ )
  {
    sprintf( inFileName, "/boot/carplay/AnnexB/carplay_video_%03d.raw", i );
    fIn = fopen( inFileName, "rb" );
    if( !fIn )
    {
      fprintf(stderr, "Could not open %s\n", inFileName);
      exit(1);
    }

    inBuffSize = fread( inBuff, 1, INBUF_SIZE, fIn );
    if( inBuffSize <= 0 )
    {
      fprintf(stderr, "File read error\n");
      break;
    }

    if( inBuffSize > 0 )
    {
      do{
        res = gCodec->dequeueInputBuffer( &index, 1ll );
        if( res == OK )
        {
          const sp<ABuffer> &buffer = gInBuffers.itemAt( index );

          buffer->setRange( 0, inBuffSize );
          memcpy( (uint8_t*)buffer->data(), inBuff, inBuffSize );

          gCodec->queueInputBuffer( index,
                                    buffer->offset(),
                                    buffer->size(),
                                    0ll,
                                    0 );
          break;
        }

        res = gCodec->dequeueOutputBuffer( &info.mIndex,
                                         &info.mOffset,
                                         &info.mSize,
                                         &info.mPresentationTimeUs,
                                         &info.mFlags);
        if( res == OK )
        {
          if( bSurface )
            gCodec->renderOutputBufferAndRelease( info.mIndex );
          else
            gCodec->releaseOutputBuffer( info.mIndex );
          frame_count++;
        }
      }while( true );
    }

    fclose( fIn );
  }
  gettimeofday( &tv2, NULL );
  elapsedTime = tv2.tv_sec - tv1.tv_sec;
  elapsedTime += ( tv2.tv_usec - tv1.tv_usec ) / 1000000.0f;
  fprintf(stderr, "Decoded %d frames in %5.3f sec = %5.2f fps\n", i, elapsedTime, i/elapsedTime );
}

//--------------------------------------------------------------------------
static void usage(FILE * fp, int argc, char ** argv)
{
  fprintf( fp,
           "Usage: %s [options]\n"
           "Options:\n"
           "-a | --all      Benchmark all modules\n"
           "-s | --soft     Benchmark software decoder\n"
           "-h | --hard     Benchmark hardware decoder\n"
           "-c | --csc      Benchmark color space conversion\n"
           "-d | --display  Benchmark display\n"
           "",
           argv[0] );
}

//--------------------------------------------------------------------------
static const char short_options[] = "hsacd";
static const struct option long_options[] =
{
  { "hard",     no_argument, NULL, 'h' },
  { "soft",     no_argument, NULL, 's' },
  { "all",      no_argument, NULL, 'a' },
  { "csc",      no_argument, NULL, 'c' },
  { "display",  no_argument, NULL, 'd' },
  { 0, 0, 0, 0 }
};

//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  bSDecoder = false;
  bHDecoder = false;
  bCSC      = false;
  bSurface  = false;

  if( argc == 1 )
  {
    usage(stderr, argc, argv);
    exit(EXIT_FAILURE);
  }

  for(;;)
  {
    int index;
    int c;

    c = getopt_long(argc, argv, short_options, long_options, &index);
    if(-1 == c)
      break;

    switch(c)
    {
      case 'a':
        bSDecoder = true;
        bCSC      = true;
        bSurface  = true;
        break;

      case 's':
        bSDecoder = true;
        break;

      case 'h':
        bHDecoder = true;
        break;

      case 'c':
        bCSC = true;
        break;

      case 'd':
        bSurface = true;
        break;

      default:
        usage(stderr, argc, argv);
        exit(EXIT_FAILURE);
        break;
    }
  }

  if( !bSDecoder && !bHDecoder && !bCSC && !bSurface )
  {
    bSDecoder = true;
    bCSC      = true;
    bSurface  = true;
  }

  ProcessState::self()->startThreadPool();
  DataSource::RegisterDefaultSniffers();

  initOutputSurface();
  if( bHDecoder )
  {
    initHwCodec();
    decodeHw();
  }
  else
  {
    decode();
  }

  exit(EXIT_SUCCESS);
}
