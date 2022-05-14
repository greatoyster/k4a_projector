#include <Eigen/Eigen>
#include <chrono>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <k4a/k4a.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <string>

#define USE_CUSTOM_PARAM

namespace
{
std::vector<uint8_t> raw_calib = {
  123, 34,  67,  97,  108, 105, 98,  114, 97,  116, 105, 111, 110, 73,  110, 102, 111, 114, 109, 97,  116, 105, 111, 110, 34,  58,  123, 34,  67,  97,  109, 101, 114, 97,  115, 34,  58,  91,  123,
  34,  73,  110, 116, 114, 105, 110, 115, 105, 99,  115, 34,  58,  123, 34,  77,  111, 100, 101, 108, 80,  97,  114, 97,  109, 101, 116, 101, 114, 67,  111, 117, 110, 116, 34,  58,  49,  52,  44,
  34,  77,  111, 100, 101, 108, 80,  97,  114, 97,  109, 101, 116, 101, 114, 115, 34,  58,  91,  48,  46,  53,  48,  50,  50,  57,  50,  55,  53,  50,  50,  54,  53,  57,  51,  48,  49,  56,  44,
  48,  46,  53,  48,  52,  53,  50,  57,  56,  51,  51,  55,  57,  51,  54,  52,  48,  49,  52,  44,  48,  46,  52,  57,  50,  55,  52,  55,  57,  54,  50,  52,  55,  52,  56,  50,  51,  44,  48,
  46,  52,  57,  50,  56,  52,  53,  50,  54,  55,  48,  53,  55,  52,  49,  56,  56,  50,  44,  53,  46,  51,  54,  51,  50,  57,  55,  52,  54,  50,  52,  54,  51,  51,  55,  56,  57,  44,  51,
  46,  51,  53,  52,  57,  51,  49,  53,  57,  50,  57,  52,  49,  50,  56,  52,  50,  44,  48,  46,  49,  54,  51,  48,  51,  50,  51,  57,  55,  54,  50,  55,  56,  51,  48,  53,  49,  44,  53,
  46,  54,  57,  49,  50,  51,  54,  52,  57,  53,  57,  55,  49,  54,  56,  44,  53,  46,  49,  51,  52,  51,  54,  56,  56,  57,  54,  52,  56,  52,  51,  55,  53,  44,  48,  46,  56,  56,  55,
  56,  54,  48,  54,  53,  53,  55,  56,  52,  54,  48,  54,  57,  51,  44,  48,  44,  48,  44,  45,  49,  46,  52,  51,  56,  54,  56,  50,  51,  55,  53,  48,  49,  48,  49,  48,  51,  69,  45,
  53,  44,  45,  49,  46,  48,  56,  49,  54,  55,  54,  54,  57,  49,  54,  56,  51,  50,  52,  50,  49,  69,  45,  53,  93,  44,  34,  77,  111, 100, 101, 108, 84,  121, 112, 101, 34,  58,  34,
  67,  65,  76,  73,  66,  82,  65,  84,  73,  79,  78,  95,  76,  101, 110, 115, 68,  105, 115, 116, 111, 114, 116, 105, 111, 110, 77,  111, 100, 101, 108, 66,  114, 111, 119, 110, 67,  111, 110,
  114, 97,  100, 121, 34,  125, 44,  34,  76,  111, 99,  97,  116, 105, 111, 110, 34,  58,  34,  67,  65,  76,  73,  66,  82,  65,  84,  73,  79,  78,  95,  67,  97,  109, 101, 114, 97,  76,  111,
  99,  97,  116, 105, 111, 110, 68,  48,  34,  44,  34,  80,  117, 114, 112, 111, 115, 101, 34,  58,  34,  67,  65,  76,  73,  66,  82,  65,  84,  73,  79,  78,  95,  67,  97,  109, 101, 114, 97,
  80,  117, 114, 112, 111, 115, 101, 68,  101, 112, 116, 104, 34,  44,  34,  77,  101, 116, 114, 105, 99,  82,  97,  100, 105, 117, 115, 34,  58,  49,  46,  55,  51,  57,  57,  57,  57,  55,  55,
  49,  49,  49,  56,  49,  54,  52,  49,  44,  34,  82,  116, 34,  58,  123, 34,  82,  111, 116, 97,  116, 105, 111, 110, 34,  58,  91,  49,  44,  48,  44,  48,  44,  48,  44,  49,  44,  48,  44,
  48,  44,  48,  44,  49,  93,  44,  34,  84,  114, 97,  110, 115, 108, 97,  116, 105, 111, 110, 34,  58,  91,  48,  44,  48,  44,  48,  93,  125, 44,  34,  83,  101, 110, 115, 111, 114, 72,  101,
  105, 103, 104, 116, 34,  58,  49,  48,  50,  52,  44,  34,  83,  101, 110, 115, 111, 114, 87,  105, 100, 116, 104, 34,  58,  49,  48,  50,  52,  44,  34,  83,  104, 117, 116, 116, 101, 114, 34,
  58,  34,  67,  65,  76,  73,  66,  82,  65,  84,  73,  79,  78,  95,  83,  104, 117, 116, 116, 101, 114, 84,  121, 112, 101, 85,  110, 100, 101, 102, 105, 110, 101, 100, 34,  44,  34,  84,  104,
  101, 114, 109, 97,  108, 65,  100, 106, 117, 115, 116, 109, 101, 110, 116, 80,  97,  114, 97,  109, 115, 34,  58,  123, 34,  80,  97,  114, 97,  109, 115, 34,  58,  91,  48,  44,  48,  44,  48,
  44,  48,  44,  48,  44,  48,  44,  48,  44,  48,  44,  48,  44,  48,  44,  48,  44,  48,  93,  125, 125, 44,  123, 34,  73,  110, 116, 114, 105, 110, 115, 105, 99,  115, 34,  58,  123, 34,  77,
  111, 100, 101, 108, 80,  97,  114, 97,  109, 101, 116, 101, 114, 67,  111, 117, 110, 116, 34,  58,  49,  52,  44,  34,  77,  111, 100, 101, 108, 80,  97,  114, 97,  109, 101, 116, 101, 114, 115,
  34,  58,  91,  48,  46,  53,  48,  48,  52,  49,  50,  53,  50,  51,  55,  52,  54,  52,  57,  48,  52,  56,  44,  48,  46,  53,  48,  53,  51,  53,  54,  49,  51,  50,  57,  56,  52,  49,  54,
  49,  51,  56,  44,  48,  46,  52,  55,  49,  48,  54,  54,  53,  51,  52,  53,  49,  57,  49,  57,  53,  53,  54,  44,  48,  46,  54,  50,  55,  57,  48,  50,  48,  51,  48,  57,  52,  52,  56,
  50,  52,  50,  50,  44,  48,  46,  57,  48,  50,  52,  57,  53,  50,  48,  53,  52,  48,  50,  51,  55,  52,  50,  55,  44,  45,  50,  46,  57,  54,  51,  56,  50,  57,  55,  53,  53,  55,  56,
  51,  48,  56,  49,  49,  44,  49,  46,  54,  48,  48,  57,  57,  51,  55,  53,  50,  52,  55,  57,  53,  53,  51,  50,  44,  48,  46,  55,  56,  49,  50,  53,  49,  52,  57,  48,  49,  49,  54,
  49,  49,  57,  51,  56,  44,  45,  50,  46,  56,  48,  57,  50,  51,  56,  52,  51,  51,  56,  51,  55,  56,  57,  48,  54,  44,  49,  46,  53,  52,  50,  50,  56,  53,  54,  56,  48,  55,  55,
  48,  56,  55,  52,  44,  48,  44,  48,  44,  45,  48,  46,  48,  48,  48,  52,  55,  55,  50,  51,  56,  50,  57,  54,  50,  57,  56,  51,  49,  48,  49,  54,  44,  48,  46,  48,  48,  48,  52,
  51,  50,  54,  54,  49,  55,  51,  52,  53,  50,  49,  51,  56,  57,  93,  44,  34,  77,  111, 100, 101, 108, 84,  121, 112, 101, 34,  58,  34,  67,  65,  76,  73,  66,  82,  65,  84,  73,  79,
  78,  95,  76,  101, 110, 115, 68,  105, 115, 116, 111, 114, 116, 105, 111, 110, 77,  111, 100, 101, 108, 66,  114, 111, 119, 110, 67,  111, 110, 114, 97,  100, 121, 34,  125, 44,  34,  76,  111,
  99,  97,  116, 105, 111, 110, 34,  58,  34,  67,  65,  76,  73,  66,  82,  65,  84,  73,  79,  78,  95,  67,  97,  109, 101, 114, 97,  76,  111, 99,  97,  116, 105, 111, 110, 80,  86,  48,  34,
  44,  34,  80,  117, 114, 112, 111, 115, 101, 34,  58,  34,  67,  65,  76,  73,  66,  82,  65,  84,  73,  79,  78,  95,  67,  97,  109, 101, 114, 97,  80,  117, 114, 112, 111, 115, 101, 80,  104,
  111, 116, 111, 86,  105, 100, 101, 111, 34,  44,  34,  77,  101, 116, 114, 105, 99,  82,  97,  100, 105, 117, 115, 34,  58,  48,  44,  34,  82,  116, 34,  58,  123, 34,  82,  111, 116, 97,  116,
  105, 111, 110, 34,  58,  91,  48,  46,  57,  57,  57,  57,  57,  57,  49,  48,  53,  57,  51,  48,  51,  50,  56,  51,  55,  44,  48,  46,  48,  48,  48,  53,  48,  52,  50,  52,  48,  55,  52,
  50,  56,  56,  52,  53,  55,  54,  51,  50,  44,  45,  48,  46,  48,  48,  49,  50,  52,  56,  56,  52,  51,  53,  48,  50,  50,  53,  51,  50,  57,  52,  44,  45,  48,  46,  48,  48,  48,  51,
  55,  55,  57,  53,  48,  51,  52,  53,  56,  54,  52,  53,  49,  57,  52,  56,  44,  48,  46,  57,  57,  53,  48,  55,  51,  55,  51,  53,  55,  49,  51,  57,  53,  56,  55,  52,  44,  48,  46,
  48,  57,  57,  49,  51,  55,  48,  55,  53,  50,  52,  53,  51,  56,  48,  52,  44,  48,  46,  48,  48,  49,  50,  57,  50,  54,  56,  48,  51,  57,  48,  49,  55,  57,  49,  53,  55,  51,  44,
  45,  48,  46,  48,  57,  57,  49,  51,  54,  53,  49,  54,  52,  53,  49,  56,  51,  53,  54,  51,  50,  44,  48,  46,  57,  57,  53,  48,  55,  51,  48,  50,  48,  52,  53,  56,  50,  50,  49,
  52,  52,  93,  44,  34,  84,  114, 97,  110, 115, 108, 97,  116, 105, 111, 110, 34,  58,  91,  45,  48,  46,  48,  51,  50,  48,  48,  53,  51,  54,  53,  57,  51,  55,  57,  52,  56,  50,  50,
  55,  44,  45,  48,  46,  48,  48,  49,  56,  52,  51,  50,  53,  52,  56,  57,  54,  56,  49,  50,  49,  52,  48,  57,  44,  48,  46,  48,  48,  52,  48,  53,  50,  51,  55,  48,  51,  50,  49,
  48,  48,  53,  53,  56,  50,  56,  93,  125, 44,  34,  83,  101, 110, 115, 111, 114, 72,  101, 105, 103, 104, 116, 34,  58,  51,  48,  55,  50,  44,  34,  83,  101, 110, 115, 111, 114, 87,  105,
  100, 116, 104, 34,  58,  52,  48,  57,  54,  44,  34,  83,  104, 117, 116, 116, 101, 114, 34,  58,  34,  67,  65,  76,  73,  66,  82,  65,  84,  73,  79,  78,  95,  83,  104, 117, 116, 116, 101,
  114, 84,  121, 112, 101, 85,  110, 100, 101, 102, 105, 110, 101, 100, 34,  44,  34,  84,  104, 101, 114, 109, 97,  108, 65,  100, 106, 117, 115, 116, 109, 101, 110, 116, 80,  97,  114, 97,  109,
  115, 34,  58,  123, 34,  80,  97,  114, 97,  109, 115, 34,  58,  91,  48,  44,  48,  44,  48,  44,  48,  44,  48,  44,  48,  44,  48,  44,  48,  44,  48,  44,  48,  44,  48,  44,  48,  93,  125,
  125, 93,  44,  34,  73,  110, 101, 114, 116, 105, 97,  108, 83,  101, 110, 115, 111, 114, 115, 34,  58,  91,  123, 34,  66,  105, 97,  115, 84,  101, 109, 112, 101, 114, 97,  116, 117, 114, 101,
  77,  111, 100, 101, 108, 34,  58,  91,  45,  48,  46,  48,  48,  55,  52,  56,  57,  56,  50,  50,  51,  51,  57,  50,  54,  54,  53,  51,  56,  54,  44,  48,  44,  48,  44,  48,  44,  48,  46,
  48,  51,  50,  53,  55,  53,  51,  51,  53,  51,  53,  51,  54,  49,  50,  57,  44,  48,  44,  48,  44,  48,  44,  45,  48,  46,  48,  48,  51,  48,  51,  55,  57,  54,  50,  53,  57,  54,  56,
  54,  51,  53,  48,  56,  50,  44,  48,  44,  48,  44,  48,  93,  44,  34,  66,  105, 97,  115, 85,  110, 99,  101, 114, 116, 97,  105, 110, 116, 121, 34,  58,  91,  57,  46,  57,  57,  57,  57,
  57,  57,  55,  52,  55,  51,  55,  56,  55,  53,  49,  54,  69,  45,  53,  44,  57,  46,  57,  57,  57,  57,  57,  57,  55,  52,  55,  51,  55,  56,  55,  53,  49,  54,  69,  45,  53,  44,  57,
  46,  57,  57,  57,  57,  57,  57,  55,  52,  55,  51,  55,  56,  55,  53,  49,  54,  69,  45,  53,  93,  44,  34,  73,  100, 34,  58,  34,  67,  65,  76,  73,  66,  82,  65,  84,  73,  79,  78,
  95,  73,  110, 101, 114, 116, 105, 97,  108, 83,  101, 110, 115, 111, 114, 73,  100, 95,  76,  83,  77,  54,  68,  83,  77,  34,  44,  34,  77,  105, 120, 105, 110, 103, 77,  97,  116, 114, 105,
  120, 84,  101, 109, 112, 101, 114, 97,  116, 117, 114, 101, 77,  111, 100, 101, 108, 34,  58,  91,  48,  46,  57,  57,  57,  50,  51,  54,  56,  56,  49,  55,  51,  50,  57,  52,  48,  54,  55,
  44,  48,  44,  48,  44,  48,  44,  48,  46,  48,  48,  51,  50,  49,  53,  54,  52,  52,  48,  52,  50,  57,  51,  56,  57,  52,  55,  55,  44,  48,  44,  48,  44,  48,  44,  48,  46,  48,  48,
  52,  53,  51,  55,  56,  57,  56,  53,  56,  49,  52,  55,  53,  48,  49,  57,  53,  44,  48,  44,  48,  44,  48,  44,  48,  46,  48,  48,  51,  50,  50,  53,  50,  50,  49,  53,  51,  49,  52,
  54,  53,  54,  52,  57,  54,  44,  48,  44,  48,  44,  48,  44,  48,  46,  57,  57,  54,  50,  53,  50,  53,  51,  54,  55,  55,  51,  54,  56,  49,  54,  52,  44,  48,  44,  48,  44,  48,  44,
  45,  48,  46,  48,  48,  49,  57,  50,  48,  48,  50,  53,  52,  52,  54,  52,  53,  50,  50,  48,  48,  52,  44,  48,  44,  48,  44,  48,  44,  48,  46,  48,  48,  52,  53,  54,  56,  55,  49,
  49,  56,  53,  52,  53,  49,  55,  52,  54,  44,  48,  44,  48,  44,  48,  44,  45,  48,  46,  48,  48,  49,  57,  50,  55,  51,  50,  50,  52,  55,  53,  50,  51,  57,  54,  51,  52,  53,  44,
  48,  44,  48,  44,  48,  44,  48,  46,  57,  57,  50,  52,  57,  49,  48,  54,  54,  52,  53,  53,  56,  52,  49,  48,  54,  44,  48,  44,  48,  44,  48,  93,  44,  34,  77,  111, 100, 101, 108,
  84,  121, 112, 101, 77,  97,  115, 107, 34,  58,  49,  54,  44,  34,  78,  111, 105, 115, 101, 34,  58,  91,  48,  46,  48,  48,  48,  57,  53,  48,  48,  48,  48,  48,  49,  54,  48,  49,  56,
  55,  52,  56,  50,  56,  44,  48,  46,  48,  48,  48,  57,  53,  48,  48,  48,  48,  48,  49,  54,  48,  49,  56,  55,  52,  56,  50,  56,  44,  48,  46,  48,  48,  48,  57,  53,  48,  48,  48,
  48,  48,  49,  54,  48,  49,  56,  55,  52,  56,  50,  56,  44,  48,  44,  48,  44,  48,  93,  44,  34,  82,  116, 34,  58,  123, 34,  82,  111, 116, 97,  116, 105, 111, 110, 34,  58,  91,  48,
  46,  48,  49,  52,  56,  50,  50,  49,  49,  56,  57,  49,  53,  54,  49,  55,  52,  54,  54,  44,  48,  46,  49,  48,  57,  54,  49,  50,  56,  56,  50,  49,  51,  55,  50,  57,  56,  53,  56,
  44,  45,  48,  46,  57,  57,  51,  56,  54,  51,  56,  50,  49,  48,  50,  57,  54,  54,  51,  48,  57,  44,  45,  48,  46,  57,  57,  57,  56,  56,  56,  57,  53,  54,  53,  52,  54,  55,  56,
  51,  52,  53,  44,  48,  46,  48,  48,  51,  49,  55,  52,  51,  53,  56,  57,  55,  56,  56,  54,  55,  53,  51,  48,  56,  44,  45,  48,  46,  48,  49,  52,  53,  54,  49,  56,  55,  53,  55,
  50,  51,  51,  48,  50,  51,  54,  52,  44,  48,  46,  48,  48,  49,  53,  53,  56,  55,  49,  49,  50,  56,  54,  54,  51,  52,  50,  48,  54,  56,  44,  48,  46,  57,  57,  51,  57,  54,  57,
  50,  54,  49,  54,  52,  54,  50,  55,  48,  55,  53,  44,  48,  46,  49,  48,  57,  54,  52,  55,  55,  53,  56,  51,  48,  53,  48,  55,  50,  55,  56,  93,  44,  34,  84,  114, 97,  110, 115,
  108, 97,  116, 105, 111, 110, 34,  58,  91,  48,  44,  48,  44,  48,  93,  125, 44,  34,  83,  101, 99,  111, 110, 100, 79,  114, 100, 101, 114, 83,  99,  97,  108, 105, 110, 103, 34,  58,  91,
  48,  44,  48,  44,  48,  44,  48,  44,  48,  44,  48,  44,  48,  44,  48,  44,  48,  93,  44,  34,  83,  101, 110, 115, 111, 114, 84,  121, 112, 101, 34,  58,  34,  67,  65,  76,  73,  66,  82,
  65,  84,  73,  79,  78,  95,  73,  110, 101, 114, 116, 105, 97,  108, 83,  101, 110, 115, 111, 114, 84,  121, 112, 101, 95,  71,  121, 114, 111, 34,  44,  34,  84,  101, 109, 112, 101, 114, 97,
  116, 117, 114, 101, 66,  111, 117, 110, 100, 115, 34,  58,  91,  53,  44,  54,  48,  93,  44,  34,  84,  101, 109, 112, 101, 114, 97,  116, 117, 114, 101, 67,  34,  58,  48,  125, 44,  123, 34,
  66,  105, 97,  115, 84,  101, 109, 112, 101, 114, 97,  116, 117, 114, 101, 77,  111, 100, 101, 108, 34,  58,  91,  48,  46,  48,  53,  57,  50,  51,  57,  48,  55,  56,  51,  49,  51,  49,  49,
  50,  50,  53,  57,  44,  48,  44,  48,  44,  48,  44,  48,  46,  48,  53,  51,  54,  50,  49,  51,  48,  55,  48,  49,  53,  52,  49,  57,  48,  48,  54,  44,  48,  44,  48,  44,  48,  44,  45,
  48,  46,  48,  54,  57,  57,  51,  49,  57,  51,  57,  50,  52,  52,  50,  55,  48,  51,  50,  53,  44,  48,  44,  48,  44,  48,  93,  44,  34,  66,  105, 97,  115, 85,  110, 99,  101, 114, 116,
  97,  105, 110, 116, 121, 34,  58,  91,  48,  46,  48,  48,  57,  57,  57,  57,  57,  57,  57,  55,  55,  54,  52,  56,  50,  53,  56,  50,  49,  44,  48,  46,  48,  48,  57,  57,  57,  57,  57,
  57,  57,  55,  55,  54,  52,  56,  50,  53,  56,  50,  49,  44,  48,  46,  48,  48,  57,  57,  57,  57,  57,  57,  57,  55,  55,  54,  52,  56,  50,  53,  56,  50,  49,  93,  44,  34,  73,  100,
  34,  58,  34,  67,  65,  76,  73,  66,  82,  65,  84,  73,  79,  78,  95,  73,  110, 101, 114, 116, 105, 97,  108, 83,  101, 110, 115, 111, 114, 73,  100, 95,  76,  83,  77,  54,  68,  83,  77,
  34,  44,  34,  77,  105, 120, 105, 110, 103, 77,  97,  116, 114, 105, 120, 84,  101, 109, 112, 101, 114, 97,  116, 117, 114, 101, 77,  111, 100, 101, 108, 34,  58,  91,  48,  46,  57,  57,  49,
  49,  54,  50,  56,  57,  54,  49,  53,  54,  51,  49,  49,  44,  48,  44,  48,  44,  48,  44,  48,  46,  48,  48,  48,  50,  48,  50,  54,  50,  56,  49,  52,  51,  53,  57,  55,  51,  57,  52,
  50,  51,  44,  48,  44,  48,  44,  48,  44,  45,  48,  46,  48,  48,  50,  57,  51,  52,  53,  53,  55,  56,  53,  49,  52,  48,  51,  57,  53,  49,  54,  44,  48,  44,  48,  44,  48,  44,  48,
  46,  48,  48,  48,  50,  48,  48,  54,  50,  53,  55,  49,  50,  55,  53,  48,  52,  55,  57,  53,  56,  44,  48,  44,  48,  44,  48,  44,  49,  46,  48,  48,  49,  48,  52,  54,  56,  57,  53,
  57,  56,  48,  56,  51,  53,  44,  48,  44,  48,  44,  48,  44,  48,  46,  48,  48,  48,  49,  51,  55,  52,  55,  51,  51,  54,  57,  50,  53,  48,  48,  55,  52,  48,  51,  44,  48,  44,  48,
  44,  48,  44,  45,  48,  46,  48,  48,  50,  57,  50,  52,  52,  53,  53,  48,  57,  54,  57,  52,  53,  49,  54,  54,  54,  44,  48,  44,  48,  44,  48,  44,  48,  46,  48,  48,  48,  49,  51,
  56,  51,  54,  55,  52,  56,  50,  53,  55,  55,  52,  53,  48,  53,  55,  44,  48,  44,  48,  44,  48,  44,  48,  46,  57,  57,  52,  53,  56,  54,  56,  56,  52,  57,  55,  53,  52,  51,  51,
  51,  53,  44,  48,  44,  48,  44,  48,  93,  44,  34,  77,  111, 100, 101, 108, 84,  121, 112, 101, 77,  97,  115, 107, 34,  58,  53,  54,  44,  34,  78,  111, 105, 115, 101, 34,  58,  91,  48,
  46,  48,  49,  48,  55,  48,  48,  48,  48,  48,  52,  53,  48,  48,  49,  53,  48,  54,  56,  44,  48,  46,  48,  49,  48,  55,  48,  48,  48,  48,  48,  52,  53,  48,  48,  49,  53,  48,  54,
  56,  44,  48,  46,  48,  49,  48,  55,  48,  48,  48,  48,  48,  52,  53,  48,  48,  49,  53,  48,  54,  56,  44,  48,  44,  48,  44,  48,  93,  44,  34,  82,  116, 34,  58,  123, 34,  82,  111,
  116, 97,  116, 105, 111, 110, 34,  58,  91,  48,  46,  48,  48,  53,  48,  56,  55,  55,  48,  57,  53,  50,  48,  48,  49,  50,  49,  52,  44,  48,  46,  49,  48,  52,  52,  49,  51,  51,  51,
  56,  48,  48,  53,  53,  52,  50,  55,  54,  44,  45,  48,  46,  57,  57,  52,  53,  50,  48,  57,  54,  50,  50,  51,  56,  51,  49,  49,  55,  55,  44,  45,  48,  46,  57,  57,  57,  57,  56,
  54,  53,  50,  57,  51,  53,  48,  50,  56,  48,  55,  54,  44,  48,  46,  48,  48,  49,  53,  52,  57,  53,  48,  50,  49,  51,  54,  49,  56,  53,  55,  54,  53,  51,  44,  45,  48,  46,  48,
  48,  52,  57,  53,  50,  57,  56,  57,  57,  56,  52,  51,  48,  51,  55,  49,  50,  56,  44,  48,  46,  48,  48,  49,  48,  50,  51,  56,  53,  52,  49,  50,  57,  48,  49,  54,  51,  57,  57,
  52,  44,  48,  46,  57,  57,  52,  53,  51,  50,  55,  54,  51,  57,  53,  55,  57,  55,  55,  50,  57,  44,  48,  46,  49,  48,  52,  52,  49,  57,  56,  50,  48,  48,  49,  48,  54,  54,  50,
  48,  56,  93,  44,  34,  84,  114, 97,  110, 115, 108, 97,  116, 105, 111, 110, 34,  58,  91,  45,  48,  46,  48,  53,  49,  48,  53,  54,  50,  49,  51,  54,  55,  54,  57,  50,  57,  52,  55,
  52,  44,  48,  46,  48,  48,  51,  49,  55,  48,  48,  48,  57,  55,  48,  50,  52,  52,  52,  48,  55,  54,  53,  44,  48,  46,  48,  48,  49,  54,  53,  57,  55,  51,  48,  50,  49,  54,  52,
  56,  56,  50,  52,  50,  49,  93,  125, 44,  34,  83,  101, 99,  111, 110, 100, 79,  114, 100, 101, 114, 83,  99,  97,  108, 105, 110, 103, 34,  58,  91,  48,  44,  48,  44,  48,  44,  48,  44,
  48,  44,  48,  44,  48,  44,  48,  44,  48,  93,  44,  34,  83,  101, 110, 115, 111, 114, 84,  121, 112, 101, 34,  58,  34,  67,  65,  76,  73,  66,  82,  65,  84,  73,  79,  78,  95,  73,  110,
  101, 114, 116, 105, 97,  108, 83,  101, 110, 115, 111, 114, 84,  121, 112, 101, 95,  65,  99,  99,  101, 108, 101, 114, 111, 109, 101, 116, 101, 114, 34,  44,  34,  84,  101, 109, 112, 101, 114,
  97,  116, 117, 114, 101, 66,  111, 117, 110, 100, 115, 34,  58,  91,  53,  44,  54,  48,  93,  44,  34,  84,  101, 109, 112, 101, 114, 97,  116, 117, 114, 101, 67,  34,  58,  48,  125, 93,  44,
  34,  77,  101, 116, 97,  100, 97,  116, 97,  34,  58,  123, 34,  83,  101, 114, 105, 97,  108, 73,  100, 34,  58,  34,  48,  48,  48,  50,  49,  50,  50,  57,  50,  57,  49,  50,  34,  44,  34,
  70,  97,  99,  116, 111, 114, 121, 67,  97,  108, 68,  97,  116, 101, 34,  58,  34,  55,  47,  49,  56,  47,  50,  48,  49,  57,  32,  49,  48,  58,  52,  55,  58,  50,  48,  32,  80,  77,  32,
  71,  77,  84,  34,  44,  34,  86,  101, 114, 115, 105, 111, 110, 34,  58,  123, 34,  77,  97,  106, 111, 114, 34,  58,  49,  44,  34,  77,  105, 110, 111, 114, 34,  58,  50,  125, 44,  34,  68,
  101, 118, 105, 99,  101, 78,  97,  109, 101, 34,  58,  34,  65,  122, 117, 114, 101, 75,  105, 110, 101, 99,  116, 45,  80,  86,  34,  44,  34,  78,  111, 116, 101, 115, 34,  58,  34,  80,  86,
  48,  95,  109, 97,  120, 95,  114, 97,  100, 105, 117, 115, 95,  105, 110, 118, 97,  108, 105, 100, 34,  125, 125, 125, 0,
};

void showCameraInfo( k4a_calibration_t *calib, bool color_cam = true )
{
  int H, W;

  Eigen::Matrix3d             camera_mtx;
  Eigen::Matrix<double, 8, 1> dist_coef;
  Eigen::Matrix3d             rect_mtx;
  Eigen::Matrix3d             new_camera_mtx;

  cv::Mat dist;
  cv::Mat camera_matrix;
  cv::Mat rect_matrix;

  camera_mtx.setZero();
  dist_coef.setZero();
  rect_mtx = Eigen::Matrix3d::Identity();
  if ( color_cam )
    {
      H = calib->color_camera_calibration.resolution_height;
      W = calib->color_camera_calibration.resolution_width;

      camera_mtx( 0, 0 ) = calib->color_camera_calibration.intrinsics.parameters.param.fx;
      camera_mtx( 0, 2 ) = calib->color_camera_calibration.intrinsics.parameters.param.cx;
      camera_mtx( 1, 1 ) = calib->color_camera_calibration.intrinsics.parameters.param.fy;
      camera_mtx( 1, 2 ) = calib->color_camera_calibration.intrinsics.parameters.param.cy;
      camera_mtx( 2, 2 ) = 1;

      dist_coef( 0, 0 ) = calib->color_camera_calibration.intrinsics.parameters.param.k1;
      dist_coef( 1, 0 ) = calib->color_camera_calibration.intrinsics.parameters.param.k2;
      dist_coef( 2, 0 ) = calib->color_camera_calibration.intrinsics.parameters.param.p1;
      dist_coef( 3, 0 ) = calib->color_camera_calibration.intrinsics.parameters.param.p2;
      dist_coef( 4, 0 ) = calib->color_camera_calibration.intrinsics.parameters.param.k3;
      dist_coef( 5, 0 ) = calib->color_camera_calibration.intrinsics.parameters.param.k4;
      dist_coef( 6, 0 ) = calib->color_camera_calibration.intrinsics.parameters.param.k5;
      dist_coef( 7, 0 ) = calib->color_camera_calibration.intrinsics.parameters.param.k6;

      // dist_coef.setZero();

      cv::eigen2cv( camera_mtx, camera_matrix );
      cv::eigen2cv( dist_coef, dist );
      cv::eigen2cv( rect_mtx, rect_matrix );

      cv::Mat new_camera_matrix = cv::getOptimalNewCameraMatrix( camera_matrix, dist, cv::Size( W, H ), 1, cv::Size( W, H ) );

      cv::cv2eigen( new_camera_matrix, new_camera_mtx );
    }
  else
    {
      H = calib->depth_camera_calibration.resolution_height;
      W = calib->depth_camera_calibration.resolution_width;

      camera_mtx( 0, 0 ) = calib->depth_camera_calibration.intrinsics.parameters.param.fx;
      camera_mtx( 0, 2 ) = calib->depth_camera_calibration.intrinsics.parameters.param.cx;
      camera_mtx( 1, 1 ) = calib->depth_camera_calibration.intrinsics.parameters.param.fy;
      camera_mtx( 1, 2 ) = calib->depth_camera_calibration.intrinsics.parameters.param.cy;
      camera_mtx( 2, 2 ) = 1;

      dist_coef( 0, 0 ) = calib->depth_camera_calibration.intrinsics.parameters.param.k1;
      dist_coef( 1, 0 ) = calib->depth_camera_calibration.intrinsics.parameters.param.k2;
      dist_coef( 2, 0 ) = calib->depth_camera_calibration.intrinsics.parameters.param.p1;
      dist_coef( 3, 0 ) = calib->depth_camera_calibration.intrinsics.parameters.param.p2;
      dist_coef( 4, 0 ) = calib->depth_camera_calibration.intrinsics.parameters.param.k3;
      dist_coef( 5, 0 ) = calib->depth_camera_calibration.intrinsics.parameters.param.k4;
      dist_coef( 6, 0 ) = calib->depth_camera_calibration.intrinsics.parameters.param.k5;
      dist_coef( 7, 0 ) = calib->depth_camera_calibration.intrinsics.parameters.param.k6;

      // dist_coef.setZero();

      cv::eigen2cv( camera_mtx, camera_matrix );
      cv::eigen2cv( dist_coef, dist );
      cv::eigen2cv( rect_mtx, rect_matrix );

      cv::Mat new_camera_matrix = cv::getOptimalNewCameraMatrix( camera_matrix, dist, cv::Size( W, H ), 1, cv::Size( W, H ) );

      cv::cv2eigen( new_camera_matrix, new_camera_mtx );
    }

  std::cout << "========" << std::endl;
  std::cout << "camera model: " << calib->color_camera_calibration.intrinsics.type << std::endl;
  std::cout << "Height: " << H << ",Width: " << W << std::endl;
  std::cout << "camera_matrix:\n" << camera_mtx << std::endl;
  std::cout << "dist_coef:\n" << dist_coef << std::endl;
  std::cout << "rectification_matrix:\n" << rect_mtx << std::endl;
  std::cout << "new_camera_matrix:\n" << new_camera_mtx << std::endl;
  std::cout << "========" << std::endl;
}
} // namespace

namespace k4a
{
cv::Mat get_mat( k4a::image &src, bool deep_copy = true )
{
  assert( src.get_size() != 0 );

  cv::Mat       mat;
  const int32_t width  = src.get_width_pixels();
  const int32_t height = src.get_height_pixels();

  const k4a_image_format_t format = src.get_format();
  switch ( format )
    {
      case k4a_image_format_t::K4A_IMAGE_FORMAT_COLOR_MJPG: {
        // NOTE: this is slower than other formats.
        std::vector<uint8_t> buffer( src.get_buffer(), src.get_buffer() + src.get_size() );
        mat = cv::imdecode( buffer, cv::IMREAD_ANYCOLOR );
        cv::cvtColor( mat, mat, cv::COLOR_BGR2BGRA );
        break;
      }
      case k4a_image_format_t::K4A_IMAGE_FORMAT_COLOR_NV12: {
        cv::Mat nv12 = cv::Mat( height + height / 2, width, CV_8UC1, src.get_buffer() ).clone();
        cv::cvtColor( nv12, mat, cv::COLOR_YUV2BGRA_NV12 );
        break;
      }
      case k4a_image_format_t::K4A_IMAGE_FORMAT_COLOR_YUY2: {
        cv::Mat yuy2 = cv::Mat( height, width, CV_8UC2, src.get_buffer() ).clone();
        cv::cvtColor( yuy2, mat, cv::COLOR_YUV2BGRA_YUY2 );
        break;
      }
      case k4a_image_format_t::K4A_IMAGE_FORMAT_COLOR_BGRA32: {
        mat = deep_copy ? cv::Mat( height, width, CV_8UC4, src.get_buffer() ).clone() : cv::Mat( height, width, CV_8UC4, src.get_buffer() );
        break;
      }
    case k4a_image_format_t::K4A_IMAGE_FORMAT_DEPTH16:
      case k4a_image_format_t::K4A_IMAGE_FORMAT_IR16: {
        mat = deep_copy ? cv::Mat( height, width, CV_16UC1, reinterpret_cast<uint16_t *>( src.get_buffer() ) ).clone()
                        : cv::Mat( height, width, CV_16UC1, reinterpret_cast<uint16_t *>( src.get_buffer() ) );
        break;
      }
      case k4a_image_format_t::K4A_IMAGE_FORMAT_CUSTOM8: {
        mat = cv::Mat( height, width, CV_8UC1, src.get_buffer() ).clone();
        break;
      }
      case k4a_image_format_t::K4A_IMAGE_FORMAT_CUSTOM: {
        // NOTE: This is opencv_viz module format (cv::viz::WCloud).
        const int16_t *buffer = reinterpret_cast<int16_t *>( src.get_buffer() );
        mat                   = cv::Mat( height, width, CV_32FC3, cv::Vec3f::all( std::numeric_limits<float>::quiet_NaN() ) );
        mat.forEach<cv::Vec3f>( [&]( cv::Vec3f &point, const int32_t *position ) {
          const int32_t index = ( position[0] * width + position[1] ) * 3;
          point               = cv::Vec3f( buffer[index + 0], buffer[index + 1], buffer[index + 2] );
        } );
        break;
      }
    default:
      throw k4a::error( "Failed to convert this format!" );
      break;
    }

  return mat;
}
} // namespace k4a

cv::Mat k4a_get_mat( k4a_image_t &src, bool deep_copy = true )
{
  k4a_image_reference( src );
  k4a::image img = k4a::image( src );
  return k4a::get_mat( img, deep_copy );
}

int main( int argc, char **argv )
{
  std::string bag_in_path;
  std::string bag_out_path;

  std::string depth_topic_in;
  std::string depth_topic_out;

  int target_height;
  int target_width;

  int kinect_height;
  int kinect_width;

  std::vector<float> target_K;
  std::vector<float> target_D;

  std::vector<float> kinect_rgb_to_target;

  ros::init( argc, argv, "k4a_projector_node" );
  ros::NodeHandle nh;

  ros::param::get( "/config/bag_in", bag_in_path );
  ros::param::get( "/config/bag_out", bag_out_path );

  ros::param::get( "/config/depth_topic_in", depth_topic_in );
  ros::param::get( "/config/depth_topic_out", depth_topic_out );

  ros::param::get( "/target/kinect_rgb_to_target", kinect_rgb_to_target );

  ros::param::get( "/target/image_height", target_height );
  ros::param::get( "/target/image_width", target_width );

  ros::param::get( "/target/camera_matrix/data", target_K );
  ros::param::get( "/target/distortion_coefficients/data", target_D );

  bool undistort_color = false;
  bool nearest_interp  = false;

  std::string rgb_topic_in;
  std::string rgb_topic_out;
  cv::Mat     map1, map2;

  ros::param::get( "/config/undistort_color", undistort_color );
  ros::param::get( "/config/rgb_topic_in", rgb_topic_in );
  ros::param::get( "/config/rgb_topic_out", rgb_topic_out );

  /* Configuation */
  k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
  config.camera_fps                 = K4A_FRAMES_PER_SECOND_30;
  config.color_format               = K4A_IMAGE_FORMAT_COLOR_BGRA32;
  config.color_resolution           = K4A_COLOR_RESOLUTION_720P;
  config.depth_mode                 = K4A_DEPTH_MODE_NFOV_UNBINNED;
  config.synchronized_images_only   = true;

  /* Get calibration info */
  k4a::calibration calib = k4a::calibration::get_from_raw( raw_calib, config.depth_mode, config.color_resolution );

  showCameraInfo( &calib, false );

  kinect_height = calib.depth_camera_calibration.resolution_height;
  kinect_width  = calib.depth_camera_calibration.resolution_width;


#ifdef USE_CUSTOM_PARAM

  calib.color_camera_calibration.intrinsics.type   = K4A_CALIBRATION_LENS_DISTORTION_MODEL_BROWN_CONRADY;
  calib.color_camera_calibration.resolution_height = target_height;
  calib.color_camera_calibration.resolution_width  = target_width;

  cv::Mat camera_mtx     = cv::Mat( 3, 3, CV_32F, target_K.data() ).clone();
  cv::Mat dist_coeff     = cv::Mat( target_D.size(), 1, CV_32F, target_D.data() ).clone();
  cv::Mat new_camera_mtx = cv::getOptimalNewCameraMatrix( camera_mtx, dist_coeff, cv::Size( target_width, target_height ), 0 );

  if ( undistort_color )
    {
      cv::Mat R           = cv::Mat::zeros( 3, 3, CV_32F );
      R.at<float>( 0, 0 ) = 1;
      R.at<float>( 1, 1 ) = 1;
      R.at<float>( 2, 2 ) = 1;
      cv::initUndistortRectifyMap( camera_mtx, dist_coeff, R, new_camera_mtx, cv::Size( target_width, target_height ), CV_32F, map1, map2 );
    }

  calib.color_camera_calibration.intrinsics.parameters.param.fx   = new_camera_mtx.at<float>( 0, 0 );
  calib.color_camera_calibration.intrinsics.parameters.param.fy   = new_camera_mtx.at<float>( 1, 1 );
  calib.color_camera_calibration.intrinsics.parameters.param.cx   = new_camera_mtx.at<float>( 0, 2 );
  calib.color_camera_calibration.intrinsics.parameters.param.cy   = new_camera_mtx.at<float>( 1, 2 );
  calib.color_camera_calibration.intrinsics.parameters.param.k1   = 0;
  calib.color_camera_calibration.intrinsics.parameters.param.k2   = 0;
  calib.color_camera_calibration.intrinsics.parameters.param.p1   = 0;
  calib.color_camera_calibration.intrinsics.parameters.param.p2   = 0;
  calib.color_camera_calibration.intrinsics.parameters.param.k3   = 0;
  calib.color_camera_calibration.intrinsics.parameters.param.k4   = 0;
  calib.color_camera_calibration.intrinsics.parameters.param.k5   = 0;
  calib.color_camera_calibration.intrinsics.parameters.param.k6   = 0;
  calib.color_camera_calibration.intrinsics.parameters.param.codx = 0;
  calib.color_camera_calibration.intrinsics.parameters.param.cody = 0;

#else
  std::cout << "Use original kinect rgb camera intrisics" << std::endl;
#endif
  /* Set extrinsics info */
  Eigen::Matrix4d T_target_kinect_depth, T_kinect_rgb_kinect_depth;
  Eigen::Matrix4d T_target_kinect_rgb;

  // clang-format off
  T_target_kinect_rgb << kinect_rgb_to_target[0], kinect_rgb_to_target[1], kinect_rgb_to_target[2], kinect_rgb_to_target[3] * 1000.f,
                         kinect_rgb_to_target[4], kinect_rgb_to_target[5], kinect_rgb_to_target[6], kinect_rgb_to_target[7] * 1000.f,
                         kinect_rgb_to_target[8], kinect_rgb_to_target[9], kinect_rgb_to_target[10], kinect_rgb_to_target[11] * 1000.f,
                         kinect_rgb_to_target[12], kinect_rgb_to_target[13], kinect_rgb_to_target[14], kinect_rgb_to_target[15];

  
  // T_kinect_rgb_kinect_depth << calib.extrinsics[K4A_CALIBRATION_TYPE_DEPTH][K4A_CALIBRATION_TYPE_COLOR].rotation[0],
  //               calib.extrinsics[K4A_CALIBRATION_TYPE_DEPTH][K4A_CALIBRATION_TYPE_COLOR].rotation[1],
  //               calib.extrinsics[K4A_CALIBRATION_TYPE_DEPTH][K4A_CALIBRATION_TYPE_COLOR].rotation[2],
  //               calib.extrinsics[K4A_CALIBRATION_TYPE_DEPTH][K4A_CALIBRATION_TYPE_COLOR].translation[0],
  //               calib.extrinsics[K4A_CALIBRATION_TYPE_DEPTH][K4A_CALIBRATION_TYPE_COLOR].rotation[3],
  //               calib.extrinsics[K4A_CALIBRATION_TYPE_DEPTH][K4A_CALIBRATION_TYPE_COLOR].rotation[4],
  //               calib.extrinsics[K4A_CALIBRATION_TYPE_DEPTH][K4A_CALIBRATION_TYPE_COLOR].rotation[5],
  //               calib.extrinsics[K4A_CALIBRATION_TYPE_DEPTH][K4A_CALIBRATION_TYPE_COLOR].translation[1],
  //               calib.extrinsics[K4A_CALIBRATION_TYPE_DEPTH][K4A_CALIBRATION_TYPE_COLOR].rotation[6],
  //               calib.extrinsics[K4A_CALIBRATION_TYPE_DEPTH][K4A_CALIBRATION_TYPE_COLOR].rotation[7],
  //               calib.extrinsics[K4A_CALIBRATION_TYPE_DEPTH][K4A_CALIBRATION_TYPE_COLOR].rotation[8],
  //               calib.extrinsics[K4A_CALIBRATION_TYPE_DEPTH][K4A_CALIBRATION_TYPE_COLOR].translation[2],
  //               0,0,0,1;
  T_kinect_rgb_kinect_depth <<0.9999983519720124, 0.0007095286521193374, -0.001671114104806111, -0.03128054306089928*1000.0,
                -0.0005461132760243128, 0.995397130621369, 0.0958345141952277, -0.001701648753243945*1000.0, 
                 0.001731419518548352, -0.09583344363966775, 0.9953958776620555, -0.01229668401143895*1000.0, 
                 0, 0, 0, 1;
  // clang-format on

  std::cout << "T_kinect_rgb_kinect_depth" << std::endl;
  std::cout << T_kinect_rgb_kinect_depth << std::endl;
  std::cout << "T_target_kinect_rgb" << std::endl;
  std::cout << T_target_kinect_rgb << std::endl;

  T_target_kinect_depth = T_target_kinect_rgb * T_kinect_rgb_kinect_depth;

  std::cout << "T_target_kinect_depth" << std::endl;
  std::cout << T_target_kinect_depth << std::endl;

  Eigen::Matrix4d T_target_kinect_depth_inv = T_target_kinect_depth.inverse();

  calib.extrinsics[K4A_CALIBRATION_TYPE_DEPTH][K4A_CALIBRATION_TYPE_COLOR].rotation[0]    = T_target_kinect_depth( 0, 0 );
  calib.extrinsics[K4A_CALIBRATION_TYPE_DEPTH][K4A_CALIBRATION_TYPE_COLOR].rotation[1]    = T_target_kinect_depth( 0, 1 );
  calib.extrinsics[K4A_CALIBRATION_TYPE_DEPTH][K4A_CALIBRATION_TYPE_COLOR].rotation[2]    = T_target_kinect_depth( 0, 2 );
  calib.extrinsics[K4A_CALIBRATION_TYPE_DEPTH][K4A_CALIBRATION_TYPE_COLOR].rotation[3]    = T_target_kinect_depth( 1, 0 );
  calib.extrinsics[K4A_CALIBRATION_TYPE_DEPTH][K4A_CALIBRATION_TYPE_COLOR].rotation[4]    = T_target_kinect_depth( 1, 1 );
  calib.extrinsics[K4A_CALIBRATION_TYPE_DEPTH][K4A_CALIBRATION_TYPE_COLOR].rotation[5]    = T_target_kinect_depth( 1, 2 );
  calib.extrinsics[K4A_CALIBRATION_TYPE_DEPTH][K4A_CALIBRATION_TYPE_COLOR].rotation[6]    = T_target_kinect_depth( 2, 0 );
  calib.extrinsics[K4A_CALIBRATION_TYPE_DEPTH][K4A_CALIBRATION_TYPE_COLOR].rotation[7]    = T_target_kinect_depth( 2, 1 );
  calib.extrinsics[K4A_CALIBRATION_TYPE_DEPTH][K4A_CALIBRATION_TYPE_COLOR].rotation[8]    = T_target_kinect_depth( 2, 2 );
  calib.extrinsics[K4A_CALIBRATION_TYPE_DEPTH][K4A_CALIBRATION_TYPE_COLOR].translation[0] = T_target_kinect_depth( 0, 3 );
  calib.extrinsics[K4A_CALIBRATION_TYPE_DEPTH][K4A_CALIBRATION_TYPE_COLOR].translation[1] = T_target_kinect_depth( 1, 3 );
  calib.extrinsics[K4A_CALIBRATION_TYPE_DEPTH][K4A_CALIBRATION_TYPE_COLOR].translation[2] = T_target_kinect_depth( 2, 3 );

  calib.extrinsics[K4A_CALIBRATION_TYPE_COLOR][K4A_CALIBRATION_TYPE_DEPTH].rotation[0]    = T_target_kinect_depth_inv( 0, 0 );
  calib.extrinsics[K4A_CALIBRATION_TYPE_COLOR][K4A_CALIBRATION_TYPE_DEPTH].rotation[1]    = T_target_kinect_depth_inv( 0, 1 );
  calib.extrinsics[K4A_CALIBRATION_TYPE_COLOR][K4A_CALIBRATION_TYPE_DEPTH].rotation[2]    = T_target_kinect_depth_inv( 0, 2 );
  calib.extrinsics[K4A_CALIBRATION_TYPE_COLOR][K4A_CALIBRATION_TYPE_DEPTH].rotation[3]    = T_target_kinect_depth_inv( 1, 0 );
  calib.extrinsics[K4A_CALIBRATION_TYPE_COLOR][K4A_CALIBRATION_TYPE_DEPTH].rotation[4]    = T_target_kinect_depth_inv( 1, 1 );
  calib.extrinsics[K4A_CALIBRATION_TYPE_COLOR][K4A_CALIBRATION_TYPE_DEPTH].rotation[5]    = T_target_kinect_depth_inv( 1, 2 );
  calib.extrinsics[K4A_CALIBRATION_TYPE_COLOR][K4A_CALIBRATION_TYPE_DEPTH].rotation[6]    = T_target_kinect_depth_inv( 2, 0 );
  calib.extrinsics[K4A_CALIBRATION_TYPE_COLOR][K4A_CALIBRATION_TYPE_DEPTH].rotation[7]    = T_target_kinect_depth_inv( 2, 1 );
  calib.extrinsics[K4A_CALIBRATION_TYPE_COLOR][K4A_CALIBRATION_TYPE_DEPTH].rotation[8]    = T_target_kinect_depth_inv( 2, 2 );
  calib.extrinsics[K4A_CALIBRATION_TYPE_COLOR][K4A_CALIBRATION_TYPE_DEPTH].translation[0] = T_target_kinect_depth_inv( 0, 3 );
  calib.extrinsics[K4A_CALIBRATION_TYPE_COLOR][K4A_CALIBRATION_TYPE_DEPTH].translation[1] = T_target_kinect_depth_inv( 1, 3 );
  calib.extrinsics[K4A_CALIBRATION_TYPE_COLOR][K4A_CALIBRATION_TYPE_DEPTH].translation[2] = T_target_kinect_depth_inv( 2, 3 );

  /* Create transformation */
  k4a::transformation tf = k4a::transformation( calib );

  /* Avoid stupid operation */
  assert( bag_in_path != bag_out_path );

  /* Create processed bag */
  rosbag::Bag bag_in( bag_in_path, rosbag::BagMode::Read );
  rosbag::Bag bag_out( bag_out_path, rosbag::BagMode::Write );

  assert( bag_in.isOpen() );
  assert( bag_out.isOpen() );

  for ( rosbag::MessageInstance const m : rosbag::View( bag_in ) )
    {
      if ( m.getTopic() == depth_topic_in )
        {
          auto msg         = m.instantiate<sensor_msgs::Image>();
          auto ptr         = cv_bridge::toCvShare( msg, msg->encoding );
          auto img_out     = cv_bridge::CvImage();
          img_out.header   = msg->header;
          img_out.encoding = msg->encoding;

          k4a::image k4a_in = k4a::image::create_from_buffer( K4A_IMAGE_FORMAT_DEPTH16, kinect_width, kinect_height, kinect_width * sizeof( uint16_t ), (uint8_t *)ptr->image.data,
                                                              kinect_width * kinect_height * sizeof( uint16_t ), nullptr, nullptr );

          k4a::image k4a_out = k4a::image::create( K4A_IMAGE_FORMAT_DEPTH16, target_width, target_height, target_width * sizeof( uint16_t ) );
          if ( nearest_interp )
            {
              auto pair     = tf.depth_image_to_color_camera_custom( k4a_in, k4a_out, K4A_TRANSFORMATION_INTERPOLATION_TYPE_NEAREST, 0 );
              img_out.image = k4a::get_mat( pair.first );
            }
          else
            {
              tf.depth_image_to_color_camera( k4a_in, &k4a_out );
              img_out.image = k4a::get_mat( k4a_out );
            }

          bag_out.write( depth_topic_out, msg->header.stamp, *img_out.toImageMsg() );
        }
      if ( m.getTopic() == rgb_topic_in && undistort_color )
        {
          auto msg         = m.instantiate<sensor_msgs::Image>();
          auto ptr         = cv_bridge::toCvShare( msg, msg->encoding );
          auto img_out     = cv_bridge::CvImage();
          img_out.header   = msg->header;
          img_out.encoding = msg->encoding;

          cv::remap( ptr->image, img_out.image, map1, map2, cv::INTER_LINEAR );

          bag_out.write( rgb_topic_out, msg->header.stamp, *img_out.toImageMsg() );
        }
    }

  bag_in.close();
  bag_out.close();

  return 0;
}