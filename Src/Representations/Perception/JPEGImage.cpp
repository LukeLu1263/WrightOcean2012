/**
 * @file JPEGImage.cpp
 *
 * Implementation of class JPEGImage
 */

#include "Platform/BHAssert.h"
#include "JPEGImage.h"
#include <mmintrin.h>
#include "Tools/Debugging/Stopwatch.h"

JPEGImage::JPEGImage(const Image& image)
{
  *this = image;
}

JPEGImage& JPEGImage::operator=(const Image& src)
{
  STOP_TIME_ON_REQUEST("jpeg-compression",
  {
    cameraInfo = src.cameraInfo;
    timeStamp = src.timeStamp;

    ASSERT(cameraInfo.resolutionWidth == 320 && cameraInfo.resolutionHeight == 240);
    unsigned char* aiboAlignedImage = new unsigned char[cameraResolutionWidth * cameraResolutionHeight * 3];
    toAiboAlignment((const unsigned char*)src.image, aiboAlignedImage);

    jpeg_compress_struct cInfo;
    jpeg_error_mgr jem;
    cInfo.err = jpeg_std_error(&jem);
    jpeg_create_compress(&cInfo);

    if(!cInfo.dest)
      cInfo.dest = (jpeg_destination_mgr*)
                   (*cInfo.mem->alloc_small)((j_common_ptr) &cInfo, JPOOL_PERMANENT, sizeof(jpeg_destination_mgr));
    cInfo.dest->init_destination = onDestIgnore;
    cInfo.dest->empty_output_buffer = onDestEmpty;
    cInfo.dest->term_destination = onDestIgnore;
    cInfo.dest->next_output_byte = (JOCTET*) image;
    cInfo.dest->free_in_buffer = cameraResolutionWidth * cameraResolutionHeight;

    cInfo.image_width = cameraInfo.resolutionWidth * 3;
    cInfo.image_height = cameraInfo.resolutionHeight;
    cInfo.input_components = 1;
    cInfo.in_color_space = JCS_GRAYSCALE;
    cInfo.jpeg_color_space = JCS_GRAYSCALE;
    jpeg_set_defaults(&cInfo);
    cInfo.dct_method = JDCT_FASTEST;
    jpeg_set_quality(&cInfo, 75, true);

    jpeg_start_compress(&cInfo, true);

    while(cInfo.next_scanline < cInfo.image_height)
    {
      JSAMPROW rowPointer = const_cast<JSAMPROW>(&aiboAlignedImage[cInfo.next_scanline * cInfo.image_width]);
      jpeg_write_scanlines(&cInfo, &rowPointer, 1);
    }

    jpeg_finish_compress(&cInfo);
    size = (char unsigned*) cInfo.dest->next_output_byte - (unsigned char*) image;
    jpeg_destroy_compress(&cInfo);
    delete[] aiboAlignedImage;
  });
  return *this;
}

void JPEGImage::toImage(Image& dest) const
{
  dest.cameraInfo = cameraInfo;
  dest.timeStamp = timeStamp;

  jpeg_decompress_struct cInfo;
  jpeg_error_mgr jem;
  cInfo.err = jpeg_std_error(&jem);

  jpeg_create_decompress(&cInfo);

  if(!cInfo.src)
    cInfo.src = (jpeg_source_mgr*)
                (*cInfo.mem->alloc_small)((j_common_ptr) &cInfo, JPOOL_PERMANENT,
                                          sizeof(jpeg_source_mgr));
  cInfo.src->init_source       = onSrcIgnore;
  cInfo.src->fill_input_buffer = onSrcEmpty;
  cInfo.src->skip_input_data   = onSrcSkip;
  cInfo.src->resync_to_restart = jpeg_resync_to_restart;
  cInfo.src->term_source       = onSrcIgnore;
  cInfo.src->bytes_in_buffer   = cameraResolutionWidth * cameraResolutionHeight;
  cInfo.src->next_input_byte = (const JOCTET*) &image[0];

  jpeg_read_header(&cInfo, true);
  jpeg_start_decompress(&cInfo);
  if(cInfo.num_components == 4) // old JPEG-compression (for compatibility with old logfiles)
  {
    // setup rows
    while(cInfo.output_scanline < cInfo.output_height)
    {
      JSAMPROW rowPointer = (unsigned char*)(&dest.image[cInfo.output_scanline]);
      (void) jpeg_read_scanlines(&cInfo, &rowPointer, 1);
    }
  }
  else if(cInfo.num_components == 1) // new JPEG-compression
  {
    ASSERT(cameraInfo.resolutionWidth == 320 && cameraInfo.resolutionHeight == 240);
    unsigned char* aiboAlignedImage = new unsigned char[cameraInfo.resolutionWidth * cameraInfo.resolutionHeight * 3];

    // setup rows
    while(cInfo.output_scanline < cInfo.output_height)
    {
      JSAMPROW rowPointer = &aiboAlignedImage[cInfo.output_scanline * cInfo.output_width];
      (void) jpeg_read_scanlines(&cInfo, &rowPointer, 1);
    }

    fromAiboAlignment(aiboAlignedImage, (unsigned char*)dest.image);
    delete[] aiboAlignedImage;
  }
  else
  {
    ASSERT(false);
  }

  // finish decompress
  jpeg_finish_decompress(&cInfo);
  jpeg_destroy_decompress(&cInfo);
}

int JPEGImage::onDestEmpty(j_compress_ptr)
{
  ASSERT(false);
  return false;
}

void JPEGImage::onDestIgnore(j_compress_ptr)
{
}

void JPEGImage::onSrcSkip(j_decompress_ptr, long)
{
}

int JPEGImage::onSrcEmpty(j_decompress_ptr)
{
  ASSERT(false);
  return false;
}

void JPEGImage::onSrcIgnore(j_decompress_ptr)
{
}

void JPEGImage::serialize(In* in, Out* out)
{
  int& resolutionWidth(cameraInfo.resolutionWidth);
  int& resolutionHeight(cameraInfo.resolutionHeight);
  STREAM_REGISTER_BEGIN();
  STREAM(resolutionWidth);
  STREAM(resolutionHeight);
  STREAM(timeStamp);
  STREAM(size);
  if(in)
    in->read(image, size);
  else
    out->write(image, size);
  STREAM_REGISTER_FINISH();

  if(in)
    cameraInfo = CameraInfo();
}

void JPEGImage::toAiboAlignment(const unsigned char* src, unsigned char* dst)
{
  const int width = 320;
  const int height = 240;
  for(int y = 0; y < height; y++)
  {
    const __m64* pSrc = (__m64*)src + y * width;
    __m64* pDst = (__m64*)dst + (y * width / 8 * 3);
    for(int x = 0; x < width / 8; x++)
    {
      __m64 mm0 = pSrc[0];
      __m64 mm1 = pSrc[1];
      __m64 mm2 = _mm_srli_si64(mm0, 32);
      __m64 mm3 = _mm_srli_si64(mm1, 32);
      mm0 = _mm_unpacklo_pi8(mm0, mm2);
      mm1 = _mm_unpacklo_pi8(mm1, mm3);
      mm2 = _mm_unpacklo_pi16(mm0, mm1);
      mm0 = _mm_unpackhi_pi16(mm0, mm1);
      mm3 = pSrc[2];
      mm1 = pSrc[3];
      __m64 mm4 = _mm_srli_si64(mm3, 32);
      __m64 mm5 = _mm_srli_si64(mm1, 32);
      mm3 = _mm_unpacklo_pi8(mm3, mm4);
      mm1 = _mm_unpacklo_pi8(mm1, mm5);
      mm4 = _mm_unpacklo_pi16(mm3, mm1);
      mm3 = _mm_unpackhi_pi16(mm3, mm1);
      mm2 = _mm_unpackhi_pi32(mm2, mm4);
      mm5 = _mm_unpacklo_pi32(mm0, mm3);
      mm0 = _mm_unpackhi_pi32(mm0, mm3);
      pDst[0] = mm2;
      pDst[40] = mm5;
      pDst[80] = mm0; // TODO not hardcoded
      pSrc += 4;
      pDst++;
    }
  }
  _mm_empty();
}

void JPEGImage::fromAiboAlignment(const unsigned char* src, unsigned char* dst)
{
  ASSERT(cameraResolutionWidth == 320 && cameraResolutionHeight == 240);
  const int width = 320;
  const int height = 240;
  for(int y = 0; y < height; y++)
  {
    const unsigned char* pSrc = src + y * width * 3;
    unsigned char* pDst = dst + y * width * 4 * 2;
    for(int x = 0; x < width; x++)
    {
      pDst[1] = pSrc[0];
      pDst[0] = pDst[2] = pSrc[320];
      pDst[3] = pSrc[640];
      pSrc++;
      pDst += 4;
    }
  }
}
