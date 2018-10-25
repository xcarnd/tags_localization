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
#include <sstream>

#include "utils.hpp"

namespace aruco_track {

    geometry_msgs::Transform parseTransformString(const std::string& str) {
        std::istringstream iss(str);
        double tx, ty, tz, r, p, y;
        iss>>tx>>ty>>tz>>r>>p>>y;

        geometry_msgs::Transform ret;
        fillTransform(
            ret,
            tx, ty, tz,
            makeTf2QuaternionFromRPYDegree(r, p, y));
        return ret;
    }

    void parseTransformStringInto(geometry_msgs::Transform& out, const std::string& str) {
        std::istringstream iss(str);
        double tx, ty, tz, r, p, y;
        iss>>tx>>ty>>tz>>r>>p>>y;
        
        fillTransform(out,
                      tx, ty, tz,
                      makeTf2QuaternionFromRPYDegree(r, p, y));
    }

}