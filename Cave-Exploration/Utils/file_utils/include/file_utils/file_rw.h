#ifndef FILE_UTILS_H
#define FILE_UTILS_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <iomanip>
#include <boost/filesystem.hpp>

namespace fs = boost::filesystem;

namespace file_utils
{
  void createDirectory(const std::string &dir_path);

  template <typename... Args>
  void writeToFileByTrunc(const std::string &file_name, const std::string &delimiter, Args &&...args)
  {
    std::ofstream fout(file_name, std::ios_base::out | std::ios_base::trunc);
    if (fout.is_open())
    {
      std::ostringstream oss;
      ((oss << std::forward<Args>(args) << delimiter), ...);
      std::string line = oss.str();
      line.pop_back();
      fout << line << std::endl;
      fout.close();
      std::cout << "File written successfully." << std::endl;
    }
    else
    {
      std::cerr << "Failed to open the file: " << file_name << std::endl;
    }
  }

  template <typename... Args>
  void writeToFileByAdd(const std::string &file_name, const std::string &delimiter, Args &&...args)
  {
    std::ofstream fout(file_name, std::ios_base::out | std::ios_base::app);
    if (fout.is_open())
    {
      std::ostringstream oss;
      ((oss << std::setw(4) << std::setprecision(4) << std::fixed << std::forward<Args>(args) << delimiter), ...);
      std::string line = oss.str();
      line.pop_back();
      fout << line << std::endl;
      fout.close();
      std::cout << "File written successfully." << std::endl;
    }
    else
    {
      std::cerr << "Failed to open the file: " << file_name << std::endl;
    }
  }
}

#endif