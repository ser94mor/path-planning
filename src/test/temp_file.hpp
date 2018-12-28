//
// Created by aoool on 14.11.18.
//

#ifndef PATH_PLANNING_TEMPFILE_HPP
#define PATH_PLANNING_TEMPFILE_HPP

#include <cstdlib>
#include <fstream>
#include <cstring>
#include <unistd.h>

class TempFile {
public:

  TempFile();

  virtual ~TempFile();

  std::ofstream& GetOfstream();

  const char* GetName();

private:
  char* name_;
  int fd_;
  std::ofstream ofstream_;
};


#endif //PATH_PLANNING_TEMPFILE_HPP
