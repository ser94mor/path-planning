//
// Created by aoool on 14.11.18.
//

#include "temp_file.hpp"

TempFile::TempFile() {
  name_ = strdup("/tmp/path_plannerXXXXXX");
  fd_ = mkstemp(name_);
  ofstream_.open(name_);
}

TempFile::~TempFile() {
  ofstream_.close();
  close(fd_);
  remove(name_);
  delete name_;
}

std::ofstream& TempFile::GetOfstream() {
  return ofstream_;
}

const char* TempFile::GetName() {
  return name_;
}
