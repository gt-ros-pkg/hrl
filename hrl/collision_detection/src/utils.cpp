/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/*
  Author: Daniel Hennes
 */

#include "inverse_dynamics/utils.h"

namespace utils {

  struct timeval starttic;
  
  void tic() {
    gettimeofday(&starttic, NULL);
  }

  void toc() {
    struct timeval end;
    long mtime, seconds, useconds;
    gettimeofday(&end, NULL);
    seconds  = end.tv_sec  - starttic.tv_sec;
    useconds = end.tv_usec - starttic.tv_usec;
    mtime = ((seconds) * 1000 + useconds/1000.0) + 0.5;
    printf("Elapsed time: %ld milliseconds\n", mtime);
  }

  bool writeMatrix(std::string fname, Eigen::MatrixXd * matrix) {
    std::ofstream file;
    std::ostream * out;
    file.open(fname.c_str());
    if (!file) {
      fprintf(stderr, "Error: can't open file %s.\n", fname.c_str());
      return false;
    }
    out = &file;
    *out << *matrix << std::endl;
    file.close();
    return true;
  }

  bool readMatrix(std::string fname, Eigen::MatrixXd * matrix) {
    // open file
    std::ifstream file;
    std::istream * in;
    file.open(fname.c_str());
    if (!file) {
      fprintf(stderr, "Error: can't open file %s.\n", fname.c_str());
      return false;
    }
    in = &file;

    std::vector<double> data;
    std::vector<double>::iterator it;
    size_t cols(0);
    size_t rows(0);
    for (; *in; rows++) {
      std::vector<double> row;

      std::string line;
      getline(*in, line);
      std::istringstream ls(line);

      double value;
      while (ls >> value) {
        row.push_back(value);
      }
    
      if (0 == row.size())
        break;
      
      // check if column number agrees      
      if (!cols) {
        cols = row.size();
      } else if (cols != row.size()) {
        fprintf(stderr, "Error reading matrix from \"%s\", line: %zu. Expected %zu columns but got %zu.\n", fname.c_str(), rows+1, cols, row.size());
        return false;
      }

      // add row
      for (it = row.begin(); it < row.end(); it++)
        data.push_back(*it); 
    }

    if (0 == rows)
      return false;

    // Eigen::MatrixXd m = *matrix;
    matrix->resize(rows, cols);
    int i = 0;
    for (it = data.begin(); it < data.end(); it++) {
      (*matrix)(i / cols, i % cols) = *it;
      i++;
    }
    file.close();
    return true;
  }

} // end namespace utils
