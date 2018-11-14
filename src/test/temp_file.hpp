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

  TempFile() {
    name = strdup("/tmp/path_plannerXXXXXX");
    fd = mkstemp(name);
    ofstream.open(name);
  }

  virtual ~TempFile() {
    ofstream.close();
    close(fd);
    remove(name);
    delete name;
  }

  std::ofstream &GetOfstream() {
    return ofstream;
  }

  const char* GetName() {
    return name;
  }

private:
  char* name;
  int fd;
  std::ofstream ofstream;
};


#endif //PATH_PLANNING_TEMPFILE_HPP
