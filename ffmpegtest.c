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
#include <getopt.h>
#include <sys/time.h>
#include <libavcodec/avcodec.h>

//--------------------------------------------------------------------------
#define INBUF_SIZE   0x20000

//--------------------------------------------------------------------------
uint8_t inBuff[INBUF_SIZE + FF_INPUT_BUFFER_PADDING_SIZE];

//--------------------------------------------------------------------------
static void usage(FILE * fp, int argc, char ** argv)
{
  fprintf( fp,
           "Usage: %s [options]\n"
           "Options:\n"
           "-h | --help Print this message\n"
           "",
           argv[0] );
}

//--------------------------------------------------------------------------
static const char short_options[] = "h";
static const struct option long_options[] =
{
  { "help",       no_argument, NULL, 'h' },
  { 0, 0, 0, 0 }
};

//-----------------------------------------------------------------------------
static int decode_write_frame( FILE           *fOut,
                               AVCodecContext *avctx,
                               AVFrame        *frame,
                               int            *frame_count,
                               AVPacket       *pkt,
                               int             last )
{
  int   len,
        got_frame;
  char  buf[1024];

  len = avcodec_decode_video2( avctx, frame, &got_frame, pkt );
  if( len < 0)
  {
    fprintf(stderr, "Error while decoding frame %d\n", *frame_count);
    return len;
  }

  if( got_frame )
  {
    fprintf(stderr, "Saving frame %3d: %08X %dx%d, %d, %d, %d\n", *frame_count,
                                                                   frame->format,
                                                                   frame->width,
                                                                   frame->height,
                                                                   frame->linesize[0],
                                                                   frame->linesize[1],
                                                                   frame->linesize[2] );
    fflush(stderr);
    fwrite( frame->data[0], 1, frame->linesize[0] * frame->height, fOut );
    fwrite( frame->data[1], 1, frame->linesize[1] * frame->height / 2, fOut );
    fwrite( frame->data[2], 1, frame->linesize[2] * frame->height / 2, fOut );
    fflush( fOut );
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
  char            inFileName[50],
                  outFileName[50];
  FILE           *fIn,
                 *fOut;
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
      sprintf( outFileName, "/boot/carplay/out/carplay_video_%03d.yuv", i );
      fOut = fopen( outFileName, "a+b" );
      if( !fOut )
      {
        fprintf(stderr, "Could not open %s\n", outFileName);
        exit(1);
      }

      decode_write_frame( fOut, c, frame, &frame_count, &avpkt, 0 );
      fclose( fOut );
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
int main(int argc, char ** argv)
{
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

      case 'h':
      default:
        usage(stderr, argc, argv);
        exit(EXIT_FAILURE);
        break;
    }
  }

  decode();

  exit(EXIT_SUCCESS);
}
