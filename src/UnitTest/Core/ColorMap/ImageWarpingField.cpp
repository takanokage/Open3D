// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// The MIT License (MIT)
//
// Copyright (c) 2018 www.open3d.org
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

#include "UnitTest.h"

#include "Core/ColorMap/ImageWarpingField.h"

using namespace Eigen;
using namespace open3d;
using namespace std;
using namespace unit_test;

// ----------------------------------------------------------------------------
//
// ----------------------------------------------------------------------------
TEST(ImageWarpingField, Constructor)
{
    vector<double> ref =
    {
           0.00,   0.00,  16.00,   0.00,  32.00,   0.00,  48.00,   0.00,
          64.00,   0.00,  80.00,   0.00,  96.00,   0.00, 112.00,   0.00,
         128.00,   0.00, 144.00,   0.00, 160.00,   0.00, 176.00,   0.00,
         192.00,   0.00, 208.00,   0.00, 224.00,   0.00, 240.00,   0.00,
         256.00,   0.00, 272.00,   0.00, 288.00,   0.00, 304.00,   0.00,
         320.00,   0.00,   0.00,  16.00,  16.00,  16.00,  32.00,  16.00,
          48.00,  16.00,  64.00,  16.00,  80.00,  16.00,  96.00,  16.00,
         112.00,  16.00, 128.00,  16.00, 144.00,  16.00, 160.00,  16.00,
         176.00,  16.00, 192.00,  16.00, 208.00,  16.00, 224.00,  16.00,
         240.00,  16.00, 256.00,  16.00, 272.00,  16.00, 288.00,  16.00,
         304.00,  16.00, 320.00,  16.00,   0.00,  32.00,  16.00,  32.00,
          32.00,  32.00,  48.00,  32.00,  64.00,  32.00,  80.00,  32.00,
          96.00,  32.00, 112.00,  32.00, 128.00,  32.00, 144.00,  32.00,
         160.00,  32.00, 176.00,  32.00, 192.00,  32.00, 208.00,  32.00,
         224.00,  32.00, 240.00,  32.00, 256.00,  32.00, 272.00,  32.00,
         288.00,  32.00, 304.00,  32.00, 320.00,  32.00,   0.00,  48.00,
          16.00,  48.00,  32.00,  48.00,  48.00,  48.00,  64.00,  48.00,
          80.00,  48.00,  96.00,  48.00, 112.00,  48.00, 128.00,  48.00,
         144.00,  48.00, 160.00,  48.00, 176.00,  48.00, 192.00,  48.00,
         208.00,  48.00, 224.00,  48.00, 240.00,  48.00, 256.00,  48.00,
         272.00,  48.00, 288.00,  48.00, 304.00,  48.00, 320.00,  48.00,
           0.00,  64.00,  16.00,  64.00,  32.00,  64.00,  48.00,  64.00,
          64.00,  64.00,  80.00,  64.00,  96.00,  64.00, 112.00,  64.00,
         128.00,  64.00, 144.00,  64.00, 160.00,  64.00, 176.00,  64.00,
         192.00,  64.00, 208.00,  64.00, 224.00,  64.00, 240.00,  64.00,
         256.00,  64.00, 272.00,  64.00, 288.00,  64.00, 304.00,  64.00,
         320.00,  64.00,   0.00,  80.00,  16.00,  80.00,  32.00,  80.00,
          48.00,  80.00,  64.00,  80.00,  80.00,  80.00,  96.00,  80.00,
         112.00,  80.00, 128.00,  80.00, 144.00,  80.00, 160.00,  80.00,
         176.00,  80.00, 192.00,  80.00, 208.00,  80.00, 224.00,  80.00,
         240.00,  80.00, 256.00,  80.00, 272.00,  80.00, 288.00,  80.00,
         304.00,  80.00, 320.00,  80.00,   0.00,  96.00,  16.00,  96.00,
          32.00,  96.00,  48.00,  96.00,  64.00,  96.00,  80.00,  96.00,
          96.00,  96.00, 112.00,  96.00, 128.00,  96.00, 144.00,  96.00,
         160.00,  96.00, 176.00,  96.00, 192.00,  96.00, 208.00,  96.00,
         224.00,  96.00, 240.00,  96.00, 256.00,  96.00, 272.00,  96.00,
         288.00,  96.00, 304.00,  96.00, 320.00,  96.00,   0.00, 112.00,
          16.00, 112.00,  32.00, 112.00,  48.00, 112.00,  64.00, 112.00,
          80.00, 112.00,  96.00, 112.00, 112.00, 112.00, 128.00, 112.00,
         144.00, 112.00, 160.00, 112.00, 176.00, 112.00, 192.00, 112.00,
         208.00, 112.00, 224.00, 112.00, 240.00, 112.00, 256.00, 112.00,
         272.00, 112.00, 288.00, 112.00, 304.00, 112.00, 320.00, 112.00,
           0.00, 128.00,  16.00, 128.00,  32.00, 128.00,  48.00, 128.00,
          64.00, 128.00,  80.00, 128.00,  96.00, 128.00, 112.00, 128.00,
         128.00, 128.00, 144.00, 128.00, 160.00, 128.00, 176.00, 128.00,
         192.00, 128.00, 208.00, 128.00, 224.00, 128.00, 240.00, 128.00,
         256.00, 128.00, 272.00, 128.00, 288.00, 128.00, 304.00, 128.00,
         320.00, 128.00,   0.00, 144.00,  16.00, 144.00,  32.00, 144.00,
          48.00, 144.00,  64.00, 144.00,  80.00, 144.00,  96.00, 144.00,
         112.00, 144.00, 128.00, 144.00, 144.00, 144.00, 160.00, 144.00,
         176.00, 144.00, 192.00, 144.00, 208.00, 144.00, 224.00, 144.00,
         240.00, 144.00, 256.00, 144.00, 272.00, 144.00, 288.00, 144.00,
         304.00, 144.00, 320.00, 144.00,   0.00, 160.00,  16.00, 160.00,
          32.00, 160.00,  48.00, 160.00,  64.00, 160.00,  80.00, 160.00,
          96.00, 160.00, 112.00, 160.00, 128.00, 160.00, 144.00, 160.00,
         160.00, 160.00, 176.00, 160.00, 192.00, 160.00, 208.00, 160.00,
         224.00, 160.00, 240.00, 160.00, 256.00, 160.00, 272.00, 160.00,
         288.00, 160.00, 304.00, 160.00, 320.00, 160.00,   0.00, 176.00,
          16.00, 176.00,  32.00, 176.00,  48.00, 176.00,  64.00, 176.00,
          80.00, 176.00,  96.00, 176.00, 112.00, 176.00, 128.00, 176.00,
         144.00, 176.00, 160.00, 176.00, 176.00, 176.00, 192.00, 176.00,
         208.00, 176.00, 224.00, 176.00, 240.00, 176.00, 256.00, 176.00,
         272.00, 176.00, 288.00, 176.00, 304.00, 176.00, 320.00, 176.00,
           0.00, 192.00,  16.00, 192.00,  32.00, 192.00,  48.00, 192.00,
          64.00, 192.00,  80.00, 192.00,  96.00, 192.00, 112.00, 192.00,
         128.00, 192.00, 144.00, 192.00, 160.00, 192.00, 176.00, 192.00,
         192.00, 192.00, 208.00, 192.00, 224.00, 192.00, 240.00, 192.00,
         256.00, 192.00, 272.00, 192.00, 288.00, 192.00, 304.00, 192.00,
         320.00, 192.00,   0.00, 208.00,  16.00, 208.00,  32.00, 208.00,
          48.00, 208.00,  64.00, 208.00,  80.00, 208.00,  96.00, 208.00,
         112.00, 208.00, 128.00, 208.00, 144.00, 208.00, 160.00, 208.00,
         176.00, 208.00, 192.00, 208.00, 208.00, 208.00, 224.00, 208.00,
         240.00, 208.00, 256.00, 208.00, 272.00, 208.00, 288.00, 208.00,
         304.00, 208.00, 320.00, 208.00,   0.00, 224.00,  16.00, 224.00,
          32.00, 224.00,  48.00, 224.00,  64.00, 224.00,  80.00, 224.00,
          96.00, 224.00, 112.00, 224.00, 128.00, 224.00, 144.00, 224.00,
         160.00, 224.00, 176.00, 224.00, 192.00, 224.00, 208.00, 224.00,
         224.00, 224.00, 240.00, 224.00, 256.00, 224.00, 272.00, 224.00,
         288.00, 224.00, 304.00, 224.00, 320.00, 224.00,   0.00, 240.00,
          16.00, 240.00,  32.00, 240.00,  48.00, 240.00,  64.00, 240.00,
          80.00, 240.00,  96.00, 240.00, 112.00, 240.00, 128.00, 240.00,
         144.00, 240.00, 160.00, 240.00, 176.00, 240.00, 192.00, 240.00,
         208.00, 240.00, 224.00, 240.00, 240.00, 240.00, 256.00, 240.00,
         272.00, 240.00, 288.00, 240.00, 304.00, 240.00, 320.00, 240.00
    };

    int width = 320;
    int height = 240;
    int nr_anchors = 16;

    ImageWarpingField field(width, height, nr_anchors);

    EXPECT_EQ(21, field.anchor_w_);
    EXPECT_EQ(16, field.anchor_h_);

    EXPECT_NEAR(16, field.anchor_step_, THRESHOLD_1E_6);

    EXPECT_EQ(672, field.flow_.size());
    for (size_t i = 0; i < field.flow_.size(); i++)
        EXPECT_NEAR(ref[i], field.flow_[i], THRESHOLD_1E_6);
}

// ----------------------------------------------------------------------------
// Same as ImageWarpingField.Constructor.
// ----------------------------------------------------------------------------
TEST(ImageWarpingField, DISABLED_InitializeWarpingFields)
{
    unit_test::NotImplemented();
}

// ----------------------------------------------------------------------------
//
// ----------------------------------------------------------------------------
TEST(ImageWarpingField, QueryFlow)
{
    vector<int> x =
    {
             5,     6,     7,    10,    13,    16,    18,    20,    20,    20,
            22,    38,    41,    42,    47,    55,    58,    60,    72,    75,
            76,   105,   109,   111,   114,   129,   130,   138,   138,   140,
           145,   171,   171,   176,   180,   184,   188,   193,   193,   205,
           209,   218,   219,   223,   248,   264,   281,   284,   304,   314
    };

    vector<int> y =
    {
            81,   109,     0,    15,     5,    12,    11,     4,     4,    56,
             0,     8,    12,    97,     7,    10,     0,     7,   213,   232,
             3,    54,     1,   164,    10,     3,     0,     0,     0,   210,
            15,     0,     3,     2,     0,     7,   157,     3,     3,     0,
           205,     3,   228,   135,     0,    79,   198,    83,   141,     0
    };

    vector<Vector2d> ref_output =
    {
        {    0.00,    0.00 }, {    0.00,    0.00 }, {  112.00,    0.00 },
        {  160.00,  240.00 }, {  208.00,   80.00 }, {  256.00,  192.00 },
        {  288.00,  176.00 }, {  320.00,   64.00 }, {  320.00,   64.00 },
        {    0.00,    0.00 }, {   16.00,   16.00 }, {  272.00,  144.00 },
        {  320.00,  208.00 }, {    0.00,    0.00 }, {   80.00,  144.00 },
        {  208.00,  192.00 }, {  256.00,   32.00 }, {  288.00,  144.00 },
        {    0.00,    0.00 }, {    0.00,    0.00 }, {  208.00,   96.00 },
        {    0.00,    0.00 }, {   64.00,   96.00 }, {    0.00,    0.00 },
        {  144.00,  240.00 }, {   48.00,  144.00 }, {   64.00,   96.00 },
        {  192.00,   96.00 }, {  192.00,   96.00 }, {    0.00,    0.00 },
        {    0.00,    0.00 }, {   48.00,  128.00 }, {   48.00,  176.00 },
        {  128.00,  160.00 }, {  192.00,  128.00 }, {  256.00,  240.00 },
        {    0.00,    0.00 }, {   64.00,  192.00 }, {   64.00,  192.00 },
        {  256.00,  144.00 }, {    0.00,    0.00 }, {  128.00,  208.00 },
        {    0.00,    0.00 }, {    0.00,    0.00 }, {  272.00,  176.00 },
        {    0.00,    0.00 }, {    0.00,    0.00 }, {    0.00,    0.00 },
        {    0.00,    0.00 }, {  320.00,  224.00 },
    };

    int width = 320;
    int height = 240;
    int nr_anchors = 16;

    ImageWarpingField field(width, height, nr_anchors);

    for (size_t i = 0; i < ref_output.size(); i++)
        ExpectEQ(ref_output[i], field.QueryFlow(x[i], y[i]));
}

// ----------------------------------------------------------------------------
//
// ----------------------------------------------------------------------------
TEST(ImageWarpingField, GetImageWarpingField)
{
    vector<int> x =
    {
             5,     6,     7,    10,    13,    16,    18,    20,    20,    20,
            22,    38,    41,    42,    47,    55,    58,    60,    72,    75,
            76,   105,   109,   111,   114,   129,   130,   138,   138,   140,
           145,   171,   171,   176,   180,   184,   188,   193,   193,   205,
           209,   218,   219,   223,   248,   264,   281,   284,   304,   314
    };

    vector<int> y =
    {
            81,   109,     0,    15,     5,    12,    11,     4,     4,    56,
             0,     8,    12,    97,     7,    10,     0,     7,   213,   232,
             3,    54,     1,   164,    10,     3,     0,     0,     0,   210,
            15,     0,     3,     2,     0,     7,   157,     3,     3,     0,
           205,     3,   228,   135,     0,    79,   198,    83,   141,     0
    };

    vector<Vector2d> ref_output =
    {
        {    5.00,   81.00 }, {    6.00,  109.00 }, {    7.00,    0.00 },
        {   10.00,   15.00 }, {   13.00,    5.00 }, {   16.00,   12.00 },
        {   18.00,   11.00 }, {   20.00,    4.00 }, {   20.00,    4.00 },
        {   20.00,   56.00 }, {   22.00,    0.00 }, {   38.00,    8.00 },
        {   41.00,   12.00 }, {   42.00,   97.00 }, {   47.00,    7.00 },
        {   55.00,   10.00 }, {   58.00,    0.00 }, {   60.00,    7.00 },
        {   72.00,  213.00 }, {   75.00,  232.00 }, {   76.00,    3.00 },
        {  105.00,   54.00 }, {  109.00,    1.00 }, {  111.00,  164.00 },
        {  114.00,   10.00 }, {  129.00,    3.00 }, {  130.00,    0.00 },
        {  138.00,    0.00 }, {  138.00,    0.00 }, {  140.00,  210.00 },
        {  145.00,   15.00 }, {  171.00,    0.00 }, {  171.00,    3.00 },
        {  176.00,    2.00 }, {  180.00,    0.00 }, {  184.00,    7.00 },
        {  188.00,  157.00 }, {  193.00,    3.00 }, {  193.00,    3.00 },
        {  205.00,    0.00 }, {  209.00,  205.00 }, {  218.00,    3.00 },
        {  219.00,  228.00 }, {  223.00,  135.00 }, {  248.00,    0.00 },
        {  264.00,   79.00 }, {  281.00,  198.00 }, {  284.00,   83.00 },
        {  304.00,  141.00 }, {  314.00,    0.00 }
    };

    int width = 320;
    int height = 240;
    int nr_anchors = 16;

    ImageWarpingField field(width, height, nr_anchors);

    for (size_t i = 0; i < ref_output.size(); i++)
    {
        Vector2d elem = field.GetImageWarpingField(x[i], y[i]);

        ExpectEQ(ref_output[i], elem);
    }
}
