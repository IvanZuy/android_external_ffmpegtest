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
#include <sys/time.h>
#include <binder/IServiceManager.h>
#include <gui/ISurfaceComposer.h>
#include <gui/SurfaceComposerClient.h>
#include <gui/Surface.h>
#include <ui/DisplayInfo.h>
#include <android/native_window.h>

#define UINT64_C(x) (x ## ULL)
extern "C" {
#include <libavcodec/avcodec.h>
}

using namespace android;

//--------------------------------------------------------------------------
#define INBUF_SIZE              0x20000
#define NUM_OF_G2D_BUFFERS      3

//--------------------------------------------------------------------------
uint8_t inBuff[INBUF_SIZE + FF_INPUT_BUFFER_PADDING_SIZE];
sp<SurfaceComposerClient> gComposerClient;
sp<SurfaceControl>        gControl;
sp<Surface>               gSurface;
struct g2d_buf*           gBuffers[ NUM_OF_G2D_BUFFERS ];
bool                      bDecoder,
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
  fprintf( stderr, "Screen surface created\n" );
  fprintf( stderr, "Screen surface %s\n", (gSurface->isValid(gSurface)?"valid":"invalid") );
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
  int   len,
        got_frame;

  if( bDecoder || ( *frame_count < 1 ) )
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

//--------------------------------------------------------------------------
static void usage(FILE * fp, int argc, char ** argv)
{
  fprintf( fp,
           "Usage: %s [options]\n"
           "Options:\n"
           "-h | --help Print this message\n"
           "-a | --all      Benchmark all modules\n"
           "-d | --decoder  Benchmark decoder\n"
           "-c | --csc      Benchmark color space conversion\n"
           "-s | --surface  Benchmark android surface blitter\n"
           "",
           argv[0] );
}

//--------------------------------------------------------------------------
static const char short_options[] = "hadcs";
static const struct option long_options[] =
{
  { "help",     no_argument, NULL, 'h' },
  { "all",      no_argument, NULL, 'a' },
  { "decoder",  no_argument, NULL, 'd' },
  { "csc",      no_argument, NULL, 'c' },
  { "surface",  no_argument, NULL, 's' },
  { 0, 0, 0, 0 }
};

//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  bDecoder  = false;
  bCSC      = false;
  bSurface  = false;

  for(;;)
  {
    int index;
    int c;

    c = getopt_long(argc, argv, short_options, long_options, &index);
    if(-1 == c)
      break;

    switch(c)
    {
      case 0:
        break;

      case 'a':
        bDecoder  = true;
        bCSC      = true;
        bSurface  = true;
        break;

      case 'd':
        bDecoder = true;
        break;

      case 'c':
        bCSC = true;
        break;

      case 's':
        bSurface = true;
        break;

      case 'h':
      default:
        usage(stderr, argc, argv);
        exit(EXIT_FAILURE);
        break;
    }
  }

  if( !bDecoder && !bCSC && !bSurface )
  {
    bDecoder  = true;
    bCSC      = true;
    bSurface  = true;
  }

  initOutputSurface();
  decode();

  exit(EXIT_SUCCESS);
}
