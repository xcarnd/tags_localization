// Copyright 2018 Chen, Zhuhui
//
// Permission is hereby granted, free of charge, to any person
// obtaining a copy of this software and associated documentation
// files (the "Software"), to deal in the Software without
// restriction, including without limitation the rights to use, copy,
// modify, merge, publish, distribute, sublicense, and/or sell copies
// of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be
// included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
// EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
// NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
// BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
// ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
// CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <unistd.h>

#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

using namespace cv;
using namespace cv::aruco;
using namespace std;

const string premble =
    "<?xml version=\"1.0\" standalone=\"no\"?>\n"
    "<!DOCTYPE svg PUBLIC \"-//W3C//DTD SVG 1.1//EN\" \n"
    "\"http://www.w3.org/Graphics/SVG/1.1/DTD/svg11.dtd\">\n";
const string svg_leading =
    "<svg width=\"%.2fmm\" height=\"%.2fmm\" version=\"1.1\" xmlns=\"http://www.w3.org/2000/svg\">\n"
    "<line x1=\"%.2fmm\" y1=\"%.2fmm\" x2=\"%.2fmm\" y2=\"%.2fmm\" style=\"stroke:rgb(0,0,0);stroke-width:1mm;\"/>\n"
    "<line x1=\"%.2fmm\" y1=\"%.2fmm\" x2=\"%.2fmm\" y2=\"%.2fmm\" style=\"stroke:rgb(0,0,0);stroke-width:1mm;\"/>\n"
    "<text x=\"%.2fmm\" y=\"%.2fmm\" font-size=\"16mm\">X</text>\n"
    "<text x=\"%.2fmm\" y=\"%.2fmm\" font-size=\"16mm\">Y</text>\n";
const string svg_ending = "</svg>";
const string rect_0 = "<rect x=\"%.2fmm\" y=\"%.2fmm\" width=\"%.2fmm\" height=\"%.2fmm\" style=\"fill:rgb(0,0,0);stroke-width:0;\"/>";
const string rect_1 = "<rect x=\"%.2fmm\" y=\"%.2fmm\" width=\"%.2fmm\" height=\"%.2fmm\" style=\"fill:rgb(255,255,255);stroke-width:0;\"/>";

inline void print_usage(const string &program_name) {
  cerr << "Usage: " << program_name << " -f marker_family -x num_markers_x -y num_markers_y -s marker_size -g marker_gap -o output" << endl;
  cerr << "Predefined marker family:" << endl;
  cerr << "  0 - 4x4_50" << endl;
  cerr << "  1 - 4x4_100" << endl;
  cerr << "  2 - 4x4_250" << endl;
  cerr << "  3 - 4x4_1000" << endl;
  cerr << "  4 - 5x5_50" << endl;
  cerr << "  5 - 5x5_100" << endl;
  cerr << "  6 - 5x5_250" << endl;
  cerr << "  7 - 5x5_1000" << endl;
  cerr << "  8 - 6x6_50" << endl;
  cerr << "  9 - 6x6_100" << endl;
  cerr << " 10 - 6x6_250" << endl;
  cerr << " 11 - 6x6_1000" << endl;
  cerr << " 12 - 7x7_50" << endl;
  cerr << " 13 - 7x7_100" << endl;
  cerr << " 14 - 7x7_250" << endl;
  cerr << " 15 - 7x7_1000" << endl;
}

static const int CAPACITY[] = {
  50, 100, 250, 1000,
  50, 100, 250, 1000,
  50, 100, 250, 1000,
  50, 100, 250, 1000
};

int main(int argc, char **argv) {
  int marker_family;
  int num_markers_x, num_markers_y;
  float marker_size, marker_gap;
  std::string output_file_name;

  int opt;
  bool noopt = true;

  while ((opt = getopt(argc, argv, "x:y:s:g:f:o:")) != -1) {
    noopt = false;
    switch (opt) {
    case 'x':
      num_markers_x = atoi(optarg);
      break;
    case 'y':
      num_markers_y = atoi(optarg);
      break;
    case 's':
      marker_size = atof(optarg);
      break;
    case 'g':
      marker_gap = atof(optarg);
      break;
    case 'f':
      marker_family = atoi(optarg);
      break;
    case 'o':
      output_file_name = optarg;
      break;
    default:
      print_usage(argv[0]);
      return -1;
    }
  }

  if (noopt) {
    print_usage(argv[0]);
    return -1;
  }

  const Dictionary &dict = getPredefinedDictionary(marker_family);

  int markerSize = dict.markerSize;
  Mat bytesList = dict.bytesList.clone();

  constexpr float margin_left = 20.0f;
  constexpr float margin_right = 20.0f;
  constexpr float margin_top = 20.0f;
  constexpr float margin_bottom = 20.0f;

  int rows = num_markers_y;
  int cols = num_markers_x;

  float width = num_markers_x * marker_size + (num_markers_x - 1) * marker_gap + margin_left + margin_right;
  float height = num_markers_y * marker_size + (num_markers_y - 1) * marker_gap + margin_top + margin_bottom;

  int total = num_markers_x * num_markers_y;
  if (total > CAPACITY[marker_family]) {
    cerr<<"Selected marker family (contains "<< CAPACITY[marker_family] <<" in total) is not capable of generating "<<total<<" markers board."<<endl;
    return -2;
  }

  clog << "(Minimum) Page width: " << width << endl;
  clog << "(Minimum) Page weight: " << height << endl;
  clog << "Margin left/right: " << margin_left << endl;
  clog << "Margin top/bottom: " << margin_top << endl;
  clog << "Marker family: "<< marker_family << endl;
  clog << "Num markers X: "<< num_markers_x << endl;
  clog << "Num markers Y: "<< num_markers_y << endl;
  clog << "Marker size: "<< marker_size << endl;  
  clog << "Marker gap: "<< marker_gap << endl;
  clog << "Total markers: "<< num_markers_x * num_markers_y << endl;

  ofstream board(output_file_name);

  char buf[2048];
  board << premble << endl;
  float start_mark_x = margin_left - 10;
  float end_mark_y = margin_top + rows * marker_size + (rows - 1) * marker_gap + 10;
  sprintf(buf, svg_leading.c_str(), width, height,
          start_mark_x - 10, end_mark_y,
          start_mark_x + 90, end_mark_y,
          start_mark_x, end_mark_y + 10,
          start_mark_x, end_mark_y - 90,
          start_mark_x + 92, end_mark_y + 5,
          start_mark_x - 5, end_mark_y - 92);
  board << buf << endl;

  float x = margin_left;
  float y = margin_top;

  for (int i = 0; i < rows * cols; ++i) {
    int row_idx = i / cols;
    int col_idx = i - row_idx * cols;

    x = margin_left + (col_idx * marker_size) + col_idx * marker_gap;
    y = margin_top + (row_idx * marker_size) + row_idx * marker_gap;

    Mat markerBits = Dictionary::getBitsFromByteList(
        bytesList.rowRange(i, i + 1),
        markerSize);
    Mat tag(markerSize + 2, markerSize + 2, CV_8UC1, Scalar::all(0));
    Mat inner = tag.rowRange(1, tag.rows - 1).colRange(1, tag.cols - 1);
    markerBits.copyTo(inner);

    float sqr_size = marker_size / tag.cols;

    float dx = 0;
    float dy = 0;
    for (int m = 0; m < tag.rows; ++m) {
      for (int n = 0; n < tag.cols; ++n) {
        if (tag.at<char>(m, n) == 0) {
          sprintf(buf, rect_0.c_str(), x + dx, y + dy, sqr_size, sqr_size);
        }
        else {
          sprintf(buf, rect_1.c_str(), x + dx, y + dy, sqr_size, sqr_size);
        }
        board << buf << endl;
        dx += sqr_size;
      }
      dx = 0;
      dy += sqr_size;
    }
  }

  board << svg_ending << endl;

  board.close();

  cout<<"Board generated and output into file: "<<output_file_name<<"."<<endl;

  return 0;
}
