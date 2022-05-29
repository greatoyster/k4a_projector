#ifndef K4A_PROJECTOR_UTILITY_HPP_
#define K4A_PROJECTOR_UTILITY_HPP_

#include <boost/filesystem.hpp>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <ros/package.h>
#include <string>
#include <yaml-cpp/yaml.h>

namespace colorful_char {

std::string info(std::string input_str) {
  return "\033[1;32m>> " + input_str + " \033[0m";
}

std::string warning(std::string input_str) {
  return "\033[1;35m>> WARNING: " + input_str + " \033[0m";
}

std::string error(std::string input_str) {
  return "\033[1;31m>> ERROR: " + input_str + " \033[0m";
}

}  // namespace colorful_char

namespace fs = boost::filesystem;

// Check whether the directory/file path is absolute path or relative path, as well as its validness.
bool directory_path_check(std::string & path) {
  if (path.back() != '/')
    path += '/';
  if (!fs::exists(path)) {
    if (path.front() != '/')
      path = '/' + path;
    path = ros::package::getPath("k4a_projector") + path;
  }
  if (!fs::exists(path)) {
    std::cerr << colorful_char::error("Invalid directory path: " + path) << std::endl;
    return false;
  }
  return true;
}

bool file_path_check(std::string & path) {
  if (!fs::exists(path)) {
    if (path.front() != '/')
      path = '/' + path;
    path = ros::package::getPath("k4a_projector") + path;
  }
  if (!fs::exists(path)) {
    std::cerr << colorful_char::error("Invalid file path: " + path) << std::endl;
    return false;
  }
  return true;
}

#endif  // K4A_PROJECTOR_UTILITY_HPP_
