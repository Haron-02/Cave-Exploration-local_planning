#include "file_utils/file_rw.h"

namespace file_utils
{
  void createDirectory(const std::string &dir_path)
  {
    if (!fs::exists(dir_path))
    {
      if (!fs::create_directory(dir_path))
      {
        std::cerr << "create directory " << dir_path.c_str() << " fail!" << std::endl;
      }
    }
  }
}