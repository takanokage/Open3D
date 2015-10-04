// ----------------------------------------------------------------------------
// -                       Open3DV: www.open3dv.org                           -
// ----------------------------------------------------------------------------
// The MIT License (MIT)
//
// Copyright (c) 2015 Qianyi Zhou <Qianyi.Zhou@gmail.com>
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.
// ----------------------------------------------------------------------------

#include "ImageIO.h"

#include <External/libpng/png.h>
#include <External/zlib/zlib.h>

namespace three{

namespace {

void SetPNGImageFromImage(const Image &image, png_image &pngimage)
{
	pngimage.width = image.width_;
	pngimage.height = image.height_;
	pngimage.format = 0;
	if (image.bytes_per_channel_ == 2) {
		pngimage.format |= PNG_FORMAT_FLAG_LINEAR;
	}
	if (image.num_of_channels_ == 3) {
		pngimage.format |= PNG_FORMAT_FLAG_COLOR;
	}
}

}	// unnamed namespace

bool ReadImageFromPNG(const std::string &filename, Image &image)
{
	png_image pngimage;
	memset(&pngimage, 0, sizeof(pngimage));
	pngimage.version = PNG_IMAGE_VERSION;
	if (png_image_begin_read_from_file(&pngimage, filename.c_str()) == 0) {
		PrintDebug("Read PNG failed: unable to parse header.\n");
		return false;
	}
	
	// We only support two channel type: gray, and RGB.
	// There is no alpha channel.
	// bytes_per_channel is determined by PNG_FORMAT_FLAG_LINEAR flag.
	image.bytes_per_channel_ = (pngimage.format & PNG_FORMAT_FLAG_LINEAR) ?
			2 : 1;
	image.num_of_channels_ = (pngimage.format & PNG_FORMAT_FLAG_COLOR) ?
			3 : 1;
	image.width_ = pngimage.width;
	image.height_ = pngimage.height;
	image.AllocateDataBuffer();

	SetPNGImageFromImage(image, pngimage);
	if (png_image_finish_read(&pngimage, NULL, image.data_.data(), 
			0, NULL) == 0) {
		PrintDebug("Read PNG failed: unable to read file.\n");
		return false;
	}
	return true;
}

bool WriteImageToPNG(const std::string &filename, const Image &image)
{
	if (image.HasData() == false) {
		PrintDebug("Write PNG failed: image has no data.\n");
		return false;
	}
	png_image pngimage;
	memset(&pngimage, 0, sizeof(pngimage));
	pngimage.version = PNG_IMAGE_VERSION;
	SetPNGImageFromImage(image, pngimage);
	if (png_image_write_to_file(&pngimage, filename.c_str(), NULL, 
			image.data_.data(), 0, NULL) == 0) {
		PrintDebug("Write PNG failed: unable to write file.\n");
		return false;
	}
	return true;
}

}	// namespace three
