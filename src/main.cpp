#include <CameraCalibration.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cv_bridge/cv_bridge.h>
#include <k4a/k4a.hpp>
#include <opencv2/core/eigen.hpp>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>

using Image = sensor_msgs::Image;

// ------------------------------------------------------------------------- //

std::vector<uint8_t> raw_calibration_data = {
  123, 34,  67,  97,  108, 105, 98,  114, 97,  116, 105, 111, 110, 73,  110, 102, 111, 114, 109, 97,  116, 105, 111, 110, 34,  58,  123,
  34,  67,  97,  109, 101, 114, 97,  115, 34,  58,  91,  123, 34,  73,  110, 116, 114, 105, 110, 115, 105, 99,  115, 34,  58,  123, 34,
  77,  111, 100, 101, 108, 80,  97,  114, 97,  109, 101, 116, 101, 114, 67,  111, 117, 110, 116, 34,  58,  49,  52,  44,  34,  77,  111,
  100, 101, 108, 80,  97,  114, 97,  109, 101, 116, 101, 114, 115, 34,  58,  91,  48,  46,  53,  48,  50,  50,  57,  50,  55,  53,  50,
  50,  54,  53,  57,  51,  48,  49,  56,  44,  48,  46,  53,  48,  52,  53,  50,  57,  56,  51,  51,  55,  57,  51,  54,  52,  48,  49,
  52,  44,  48,  46,  52,  57,  50,  55,  52,  55,  57,  54,  50,  52,  55,  52,  56,  50,  51,  44,  48,  46,  52,  57,  50,  56,  52,
  53,  50,  54,  55,  48,  53,  55,  52,  49,  56,  56,  50,  44,  53,  46,  51,  54,  51,  50,  57,  55,  52,  54,  50,  52,  54,  51,
  51,  55,  56,  57,  44,  51,  46,  51,  53,  52,  57,  51,  49,  53,  57,  50,  57,  52,  49,  50,  56,  52,  50,  44,  48,  46,  49,
  54,  51,  48,  51,  50,  51,  57,  55,  54,  50,  55,  56,  51,  48,  53,  49,  44,  53,  46,  54,  57,  49,  50,  51,  54,  52,  57,
  53,  57,  55,  49,  54,  56,  44,  53,  46,  49,  51,  52,  51,  54,  56,  56,  57,  54,  52,  56,  52,  51,  55,  53,  44,  48,  46,
  56,  56,  55,  56,  54,  48,  54,  53,  53,  55,  56,  52,  54,  48,  54,  57,  51,  44,  48,  44,  48,  44,  45,  49,  46,  52,  51,
  56,  54,  56,  50,  51,  55,  53,  48,  49,  48,  49,  48,  51,  69,  45,  53,  44,  45,  49,  46,  48,  56,  49,  54,  55,  54,  54,
  57,  49,  54,  56,  51,  50,  52,  50,  49,  69,  45,  53,  93,  44,  34,  77,  111, 100, 101, 108, 84,  121, 112, 101, 34,  58,  34,
  67,  65,  76,  73,  66,  82,  65,  84,  73,  79,  78,  95,  76,  101, 110, 115, 68,  105, 115, 116, 111, 114, 116, 105, 111, 110, 77,
  111, 100, 101, 108, 66,  114, 111, 119, 110, 67,  111, 110, 114, 97,  100, 121, 34,  125, 44,  34,  76,  111, 99,  97,  116, 105, 111,
  110, 34,  58,  34,  67,  65,  76,  73,  66,  82,  65,  84,  73,  79,  78,  95,  67,  97,  109, 101, 114, 97,  76,  111, 99,  97,  116,
  105, 111, 110, 68,  48,  34,  44,  34,  80,  117, 114, 112, 111, 115, 101, 34,  58,  34,  67,  65,  76,  73,  66,  82,  65,  84,  73,
  79,  78,  95,  67,  97,  109, 101, 114, 97,  80,  117, 114, 112, 111, 115, 101, 68,  101, 112, 116, 104, 34,  44,  34,  77,  101, 116,
  114, 105, 99,  82,  97,  100, 105, 117, 115, 34,  58,  49,  46,  55,  51,  57,  57,  57,  57,  55,  55,  49,  49,  49,  56,  49,  54,
  52,  49,  44,  34,  82,  116, 34,  58,  123, 34,  82,  111, 116, 97,  116, 105, 111, 110, 34,  58,  91,  49,  44,  48,  44,  48,  44,
  48,  44,  49,  44,  48,  44,  48,  44,  48,  44,  49,  93,  44,  34,  84,  114, 97,  110, 115, 108, 97,  116, 105, 111, 110, 34,  58,
  91,  48,  44,  48,  44,  48,  93,  125, 44,  34,  83,  101, 110, 115, 111, 114, 72,  101, 105, 103, 104, 116, 34,  58,  49,  48,  50,
  52,  44,  34,  83,  101, 110, 115, 111, 114, 87,  105, 100, 116, 104, 34,  58,  49,  48,  50,  52,  44,  34,  83,  104, 117, 116, 116,
  101, 114, 34,  58,  34,  67,  65,  76,  73,  66,  82,  65,  84,  73,  79,  78,  95,  83,  104, 117, 116, 116, 101, 114, 84,  121, 112,
  101, 85,  110, 100, 101, 102, 105, 110, 101, 100, 34,  44,  34,  84,  104, 101, 114, 109, 97,  108, 65,  100, 106, 117, 115, 116, 109,
  101, 110, 116, 80,  97,  114, 97,  109, 115, 34,  58,  123, 34,  80,  97,  114, 97,  109, 115, 34,  58,  91,  48,  44,  48,  44,  48,
  44,  48,  44,  48,  44,  48,  44,  48,  44,  48,  44,  48,  44,  48,  44,  48,  44,  48,  93,  125, 125, 44,  123, 34,  73,  110, 116,
  114, 105, 110, 115, 105, 99,  115, 34,  58,  123, 34,  77,  111, 100, 101, 108, 80,  97,  114, 97,  109, 101, 116, 101, 114, 67,  111,
  117, 110, 116, 34,  58,  49,  52,  44,  34,  77,  111, 100, 101, 108, 80,  97,  114, 97,  109, 101, 116, 101, 114, 115, 34,  58,  91,
  48,  46,  53,  48,  48,  52,  49,  50,  53,  50,  51,  55,  52,  54,  52,  57,  48,  52,  56,  44,  48,  46,  53,  48,  53,  51,  53,
  54,  49,  51,  50,  57,  56,  52,  49,  54,  49,  51,  56,  44,  48,  46,  52,  55,  49,  48,  54,  54,  53,  51,  52,  53,  49,  57,
  49,  57,  53,  53,  54,  44,  48,  46,  54,  50,  55,  57,  48,  50,  48,  51,  48,  57,  52,  52,  56,  50,  52,  50,  50,  44,  48,
  46,  57,  48,  50,  52,  57,  53,  50,  48,  53,  52,  48,  50,  51,  55,  52,  50,  55,  44,  45,  50,  46,  57,  54,  51,  56,  50,
  57,  55,  53,  53,  55,  56,  51,  48,  56,  49,  49,  44,  49,  46,  54,  48,  48,  57,  57,  51,  55,  53,  50,  52,  55,  57,  53,
  53,  51,  50,  44,  48,  46,  55,  56,  49,  50,  53,  49,  52,  57,  48,  49,  49,  54,  49,  49,  57,  51,  56,  44,  45,  50,  46,
  56,  48,  57,  50,  51,  56,  52,  51,  51,  56,  51,  55,  56,  57,  48,  54,  44,  49,  46,  53,  52,  50,  50,  56,  53,  54,  56,
  48,  55,  55,  48,  56,  55,  52,  44,  48,  44,  48,  44,  45,  48,  46,  48,  48,  48,  52,  55,  55,  50,  51,  56,  50,  57,  54,
  50,  57,  56,  51,  49,  48,  49,  54,  44,  48,  46,  48,  48,  48,  52,  51,  50,  54,  54,  49,  55,  51,  52,  53,  50,  49,  51,
  56,  57,  93,  44,  34,  77,  111, 100, 101, 108, 84,  121, 112, 101, 34,  58,  34,  67,  65,  76,  73,  66,  82,  65,  84,  73,  79,
  78,  95,  76,  101, 110, 115, 68,  105, 115, 116, 111, 114, 116, 105, 111, 110, 77,  111, 100, 101, 108, 66,  114, 111, 119, 110, 67,
  111, 110, 114, 97,  100, 121, 34,  125, 44,  34,  76,  111, 99,  97,  116, 105, 111, 110, 34,  58,  34,  67,  65,  76,  73,  66,  82,
  65,  84,  73,  79,  78,  95,  67,  97,  109, 101, 114, 97,  76,  111, 99,  97,  116, 105, 111, 110, 80,  86,  48,  34,  44,  34,  80,
  117, 114, 112, 111, 115, 101, 34,  58,  34,  67,  65,  76,  73,  66,  82,  65,  84,  73,  79,  78,  95,  67,  97,  109, 101, 114, 97,
  80,  117, 114, 112, 111, 115, 101, 80,  104, 111, 116, 111, 86,  105, 100, 101, 111, 34,  44,  34,  77,  101, 116, 114, 105, 99,  82,
  97,  100, 105, 117, 115, 34,  58,  48,  44,  34,  82,  116, 34,  58,  123, 34,  82,  111, 116, 97,  116, 105, 111, 110, 34,  58,  91,
  48,  46,  57,  57,  57,  57,  57,  57,  49,  48,  53,  57,  51,  48,  51,  50,  56,  51,  55,  44,  48,  46,  48,  48,  48,  53,  48,
  52,  50,  52,  48,  55,  52,  50,  56,  56,  52,  53,  55,  54,  51,  50,  44,  45,  48,  46,  48,  48,  49,  50,  52,  56,  56,  52,
  51,  53,  48,  50,  50,  53,  51,  50,  57,  52,  44,  45,  48,  46,  48,  48,  48,  51,  55,  55,  57,  53,  48,  51,  52,  53,  56,
  54,  52,  53,  49,  57,  52,  56,  44,  48,  46,  57,  57,  53,  48,  55,  51,  55,  51,  53,  55,  49,  51,  57,  53,  56,  55,  52,
  44,  48,  46,  48,  57,  57,  49,  51,  55,  48,  55,  53,  50,  52,  53,  51,  56,  48,  52,  44,  48,  46,  48,  48,  49,  50,  57,
  50,  54,  56,  48,  51,  57,  48,  49,  55,  57,  49,  53,  55,  51,  44,  45,  48,  46,  48,  57,  57,  49,  51,  54,  53,  49,  54,
  52,  53,  49,  56,  51,  53,  54,  51,  50,  44,  48,  46,  57,  57,  53,  48,  55,  51,  48,  50,  48,  52,  53,  56,  50,  50,  49,
  52,  52,  93,  44,  34,  84,  114, 97,  110, 115, 108, 97,  116, 105, 111, 110, 34,  58,  91,  45,  48,  46,  48,  51,  50,  48,  48,
  53,  51,  54,  53,  57,  51,  55,  57,  52,  56,  50,  50,  55,  44,  45,  48,  46,  48,  48,  49,  56,  52,  51,  50,  53,  52,  56,
  57,  54,  56,  49,  50,  49,  52,  48,  57,  44,  48,  46,  48,  48,  52,  48,  53,  50,  51,  55,  48,  51,  50,  49,  48,  48,  53,
  53,  56,  50,  56,  93,  125, 44,  34,  83,  101, 110, 115, 111, 114, 72,  101, 105, 103, 104, 116, 34,  58,  51,  48,  55,  50,  44,
  34,  83,  101, 110, 115, 111, 114, 87,  105, 100, 116, 104, 34,  58,  52,  48,  57,  54,  44,  34,  83,  104, 117, 116, 116, 101, 114,
  34,  58,  34,  67,  65,  76,  73,  66,  82,  65,  84,  73,  79,  78,  95,  83,  104, 117, 116, 116, 101, 114, 84,  121, 112, 101, 85,
  110, 100, 101, 102, 105, 110, 101, 100, 34,  44,  34,  84,  104, 101, 114, 109, 97,  108, 65,  100, 106, 117, 115, 116, 109, 101, 110,
  116, 80,  97,  114, 97,  109, 115, 34,  58,  123, 34,  80,  97,  114, 97,  109, 115, 34,  58,  91,  48,  44,  48,  44,  48,  44,  48,
  44,  48,  44,  48,  44,  48,  44,  48,  44,  48,  44,  48,  44,  48,  44,  48,  93,  125, 125, 93,  44,  34,  73,  110, 101, 114, 116,
  105, 97,  108, 83,  101, 110, 115, 111, 114, 115, 34,  58,  91,  123, 34,  66,  105, 97,  115, 84,  101, 109, 112, 101, 114, 97,  116,
  117, 114, 101, 77,  111, 100, 101, 108, 34,  58,  91,  45,  48,  46,  48,  48,  55,  52,  56,  57,  56,  50,  50,  51,  51,  57,  50,
  54,  54,  53,  51,  56,  54,  44,  48,  44,  48,  44,  48,  44,  48,  46,  48,  51,  50,  53,  55,  53,  51,  51,  53,  51,  53,  51,
  54,  49,  50,  57,  44,  48,  44,  48,  44,  48,  44,  45,  48,  46,  48,  48,  51,  48,  51,  55,  57,  54,  50,  53,  57,  54,  56,
  54,  51,  53,  48,  56,  50,  44,  48,  44,  48,  44,  48,  93,  44,  34,  66,  105, 97,  115, 85,  110, 99,  101, 114, 116, 97,  105,
  110, 116, 121, 34,  58,  91,  57,  46,  57,  57,  57,  57,  57,  57,  55,  52,  55,  51,  55,  56,  55,  53,  49,  54,  69,  45,  53,
  44,  57,  46,  57,  57,  57,  57,  57,  57,  55,  52,  55,  51,  55,  56,  55,  53,  49,  54,  69,  45,  53,  44,  57,  46,  57,  57,
  57,  57,  57,  57,  55,  52,  55,  51,  55,  56,  55,  53,  49,  54,  69,  45,  53,  93,  44,  34,  73,  100, 34,  58,  34,  67,  65,
  76,  73,  66,  82,  65,  84,  73,  79,  78,  95,  73,  110, 101, 114, 116, 105, 97,  108, 83,  101, 110, 115, 111, 114, 73,  100, 95,
  76,  83,  77,  54,  68,  83,  77,  34,  44,  34,  77,  105, 120, 105, 110, 103, 77,  97,  116, 114, 105, 120, 84,  101, 109, 112, 101,
  114, 97,  116, 117, 114, 101, 77,  111, 100, 101, 108, 34,  58,  91,  48,  46,  57,  57,  57,  50,  51,  54,  56,  56,  49,  55,  51,
  50,  57,  52,  48,  54,  55,  44,  48,  44,  48,  44,  48,  44,  48,  46,  48,  48,  51,  50,  49,  53,  54,  52,  52,  48,  52,  50,
  57,  51,  56,  57,  52,  55,  55,  44,  48,  44,  48,  44,  48,  44,  48,  46,  48,  48,  52,  53,  51,  55,  56,  57,  56,  53,  56,
  49,  52,  55,  53,  48,  49,  57,  53,  44,  48,  44,  48,  44,  48,  44,  48,  46,  48,  48,  51,  50,  50,  53,  50,  50,  49,  53,
  51,  49,  52,  54,  53,  54,  52,  57,  54,  44,  48,  44,  48,  44,  48,  44,  48,  46,  57,  57,  54,  50,  53,  50,  53,  51,  54,
  55,  55,  51,  54,  56,  49,  54,  52,  44,  48,  44,  48,  44,  48,  44,  45,  48,  46,  48,  48,  49,  57,  50,  48,  48,  50,  53,
  52,  52,  54,  52,  53,  50,  50,  48,  48,  52,  44,  48,  44,  48,  44,  48,  44,  48,  46,  48,  48,  52,  53,  54,  56,  55,  49,
  49,  56,  53,  52,  53,  49,  55,  52,  54,  44,  48,  44,  48,  44,  48,  44,  45,  48,  46,  48,  48,  49,  57,  50,  55,  51,  50,
  50,  52,  55,  53,  50,  51,  57,  54,  51,  52,  53,  44,  48,  44,  48,  44,  48,  44,  48,  46,  57,  57,  50,  52,  57,  49,  48,
  54,  54,  52,  53,  53,  56,  52,  49,  48,  54,  44,  48,  44,  48,  44,  48,  93,  44,  34,  77,  111, 100, 101, 108, 84,  121, 112,
  101, 77,  97,  115, 107, 34,  58,  49,  54,  44,  34,  78,  111, 105, 115, 101, 34,  58,  91,  48,  46,  48,  48,  48,  57,  53,  48,
  48,  48,  48,  48,  49,  54,  48,  49,  56,  55,  52,  56,  50,  56,  44,  48,  46,  48,  48,  48,  57,  53,  48,  48,  48,  48,  48,
  49,  54,  48,  49,  56,  55,  52,  56,  50,  56,  44,  48,  46,  48,  48,  48,  57,  53,  48,  48,  48,  48,  48,  49,  54,  48,  49,
  56,  55,  52,  56,  50,  56,  44,  48,  44,  48,  44,  48,  93,  44,  34,  82,  116, 34,  58,  123, 34,  82,  111, 116, 97,  116, 105,
  111, 110, 34,  58,  91,  48,  46,  48,  49,  52,  56,  50,  50,  49,  49,  56,  57,  49,  53,  54,  49,  55,  52,  54,  54,  44,  48,
  46,  49,  48,  57,  54,  49,  50,  56,  56,  50,  49,  51,  55,  50,  57,  56,  53,  56,  44,  45,  48,  46,  57,  57,  51,  56,  54,
  51,  56,  50,  49,  48,  50,  57,  54,  54,  51,  48,  57,  44,  45,  48,  46,  57,  57,  57,  56,  56,  56,  57,  53,  54,  53,  52,
  54,  55,  56,  51,  52,  53,  44,  48,  46,  48,  48,  51,  49,  55,  52,  51,  53,  56,  57,  55,  56,  56,  54,  55,  53,  51,  48,
  56,  44,  45,  48,  46,  48,  49,  52,  53,  54,  49,  56,  55,  53,  55,  50,  51,  51,  48,  50,  51,  54,  52,  44,  48,  46,  48,
  48,  49,  53,  53,  56,  55,  49,  49,  50,  56,  54,  54,  51,  52,  50,  48,  54,  56,  44,  48,  46,  57,  57,  51,  57,  54,  57,
  50,  54,  49,  54,  52,  54,  50,  55,  48,  55,  53,  44,  48,  46,  49,  48,  57,  54,  52,  55,  55,  53,  56,  51,  48,  53,  48,
  55,  50,  55,  56,  93,  44,  34,  84,  114, 97,  110, 115, 108, 97,  116, 105, 111, 110, 34,  58,  91,  48,  44,  48,  44,  48,  93,
  125, 44,  34,  83,  101, 99,  111, 110, 100, 79,  114, 100, 101, 114, 83,  99,  97,  108, 105, 110, 103, 34,  58,  91,  48,  44,  48,
  44,  48,  44,  48,  44,  48,  44,  48,  44,  48,  44,  48,  44,  48,  93,  44,  34,  83,  101, 110, 115, 111, 114, 84,  121, 112, 101,
  34,  58,  34,  67,  65,  76,  73,  66,  82,  65,  84,  73,  79,  78,  95,  73,  110, 101, 114, 116, 105, 97,  108, 83,  101, 110, 115,
  111, 114, 84,  121, 112, 101, 95,  71,  121, 114, 111, 34,  44,  34,  84,  101, 109, 112, 101, 114, 97,  116, 117, 114, 101, 66,  111,
  117, 110, 100, 115, 34,  58,  91,  53,  44,  54,  48,  93,  44,  34,  84,  101, 109, 112, 101, 114, 97,  116, 117, 114, 101, 67,  34,
  58,  48,  125, 44,  123, 34,  66,  105, 97,  115, 84,  101, 109, 112, 101, 114, 97,  116, 117, 114, 101, 77,  111, 100, 101, 108, 34,
  58,  91,  48,  46,  48,  53,  57,  50,  51,  57,  48,  55,  56,  51,  49,  51,  49,  49,  50,  50,  53,  57,  44,  48,  44,  48,  44,
  48,  44,  48,  46,  48,  53,  51,  54,  50,  49,  51,  48,  55,  48,  49,  53,  52,  49,  57,  48,  48,  54,  44,  48,  44,  48,  44,
  48,  44,  45,  48,  46,  48,  54,  57,  57,  51,  49,  57,  51,  57,  50,  52,  52,  50,  55,  48,  51,  50,  53,  44,  48,  44,  48,
  44,  48,  93,  44,  34,  66,  105, 97,  115, 85,  110, 99,  101, 114, 116, 97,  105, 110, 116, 121, 34,  58,  91,  48,  46,  48,  48,
  57,  57,  57,  57,  57,  57,  57,  55,  55,  54,  52,  56,  50,  53,  56,  50,  49,  44,  48,  46,  48,  48,  57,  57,  57,  57,  57,
  57,  57,  55,  55,  54,  52,  56,  50,  53,  56,  50,  49,  44,  48,  46,  48,  48,  57,  57,  57,  57,  57,  57,  57,  55,  55,  54,
  52,  56,  50,  53,  56,  50,  49,  93,  44,  34,  73,  100, 34,  58,  34,  67,  65,  76,  73,  66,  82,  65,  84,  73,  79,  78,  95,
  73,  110, 101, 114, 116, 105, 97,  108, 83,  101, 110, 115, 111, 114, 73,  100, 95,  76,  83,  77,  54,  68,  83,  77,  34,  44,  34,
  77,  105, 120, 105, 110, 103, 77,  97,  116, 114, 105, 120, 84,  101, 109, 112, 101, 114, 97,  116, 117, 114, 101, 77,  111, 100, 101,
  108, 34,  58,  91,  48,  46,  57,  57,  49,  49,  54,  50,  56,  57,  54,  49,  53,  54,  51,  49,  49,  44,  48,  44,  48,  44,  48,
  44,  48,  46,  48,  48,  48,  50,  48,  50,  54,  50,  56,  49,  52,  51,  53,  57,  55,  51,  57,  52,  50,  51,  44,  48,  44,  48,
  44,  48,  44,  45,  48,  46,  48,  48,  50,  57,  51,  52,  53,  53,  55,  56,  53,  49,  52,  48,  51,  57,  53,  49,  54,  44,  48,
  44,  48,  44,  48,  44,  48,  46,  48,  48,  48,  50,  48,  48,  54,  50,  53,  55,  49,  50,  55,  53,  48,  52,  55,  57,  53,  56,
  44,  48,  44,  48,  44,  48,  44,  49,  46,  48,  48,  49,  48,  52,  54,  56,  57,  53,  57,  56,  48,  56,  51,  53,  44,  48,  44,
  48,  44,  48,  44,  48,  46,  48,  48,  48,  49,  51,  55,  52,  55,  51,  51,  54,  57,  50,  53,  48,  48,  55,  52,  48,  51,  44,
  48,  44,  48,  44,  48,  44,  45,  48,  46,  48,  48,  50,  57,  50,  52,  52,  53,  53,  48,  57,  54,  57,  52,  53,  49,  54,  54,
  54,  44,  48,  44,  48,  44,  48,  44,  48,  46,  48,  48,  48,  49,  51,  56,  51,  54,  55,  52,  56,  50,  53,  55,  55,  52,  53,
  48,  53,  55,  44,  48,  44,  48,  44,  48,  44,  48,  46,  57,  57,  52,  53,  56,  54,  56,  56,  52,  57,  55,  53,  52,  51,  51,
  51,  53,  44,  48,  44,  48,  44,  48,  93,  44,  34,  77,  111, 100, 101, 108, 84,  121, 112, 101, 77,  97,  115, 107, 34,  58,  53,
  54,  44,  34,  78,  111, 105, 115, 101, 34,  58,  91,  48,  46,  48,  49,  48,  55,  48,  48,  48,  48,  48,  52,  53,  48,  48,  49,
  53,  48,  54,  56,  44,  48,  46,  48,  49,  48,  55,  48,  48,  48,  48,  48,  52,  53,  48,  48,  49,  53,  48,  54,  56,  44,  48,
  46,  48,  49,  48,  55,  48,  48,  48,  48,  48,  52,  53,  48,  48,  49,  53,  48,  54,  56,  44,  48,  44,  48,  44,  48,  93,  44,
  34,  82,  116, 34,  58,  123, 34,  82,  111, 116, 97,  116, 105, 111, 110, 34,  58,  91,  48,  46,  48,  48,  53,  48,  56,  55,  55,
  48,  57,  53,  50,  48,  48,  49,  50,  49,  52,  44,  48,  46,  49,  48,  52,  52,  49,  51,  51,  51,  56,  48,  48,  53,  53,  52,
  50,  55,  54,  44,  45,  48,  46,  57,  57,  52,  53,  50,  48,  57,  54,  50,  50,  51,  56,  51,  49,  49,  55,  55,  44,  45,  48,
  46,  57,  57,  57,  57,  56,  54,  53,  50,  57,  51,  53,  48,  50,  56,  48,  55,  54,  44,  48,  46,  48,  48,  49,  53,  52,  57,
  53,  48,  50,  49,  51,  54,  49,  56,  53,  55,  54,  53,  51,  44,  45,  48,  46,  48,  48,  52,  57,  53,  50,  57,  56,  57,  57,
  56,  52,  51,  48,  51,  55,  49,  50,  56,  44,  48,  46,  48,  48,  49,  48,  50,  51,  56,  53,  52,  49,  50,  57,  48,  49,  54,
  51,  57,  57,  52,  44,  48,  46,  57,  57,  52,  53,  51,  50,  55,  54,  51,  57,  53,  55,  57,  55,  55,  50,  57,  44,  48,  46,
  49,  48,  52,  52,  49,  57,  56,  50,  48,  48,  49,  48,  54,  54,  50,  48,  56,  93,  44,  34,  84,  114, 97,  110, 115, 108, 97,
  116, 105, 111, 110, 34,  58,  91,  45,  48,  46,  48,  53,  49,  48,  53,  54,  50,  49,  51,  54,  55,  54,  57,  50,  57,  52,  55,
  52,  44,  48,  46,  48,  48,  51,  49,  55,  48,  48,  48,  57,  55,  48,  50,  52,  52,  52,  48,  55,  54,  53,  44,  48,  46,  48,
  48,  49,  54,  53,  57,  55,  51,  48,  50,  49,  54,  52,  56,  56,  50,  52,  50,  49,  93,  125, 44,  34,  83,  101, 99,  111, 110,
  100, 79,  114, 100, 101, 114, 83,  99,  97,  108, 105, 110, 103, 34,  58,  91,  48,  44,  48,  44,  48,  44,  48,  44,  48,  44,  48,
  44,  48,  44,  48,  44,  48,  93,  44,  34,  83,  101, 110, 115, 111, 114, 84,  121, 112, 101, 34,  58,  34,  67,  65,  76,  73,  66,
  82,  65,  84,  73,  79,  78,  95,  73,  110, 101, 114, 116, 105, 97,  108, 83,  101, 110, 115, 111, 114, 84,  121, 112, 101, 95,  65,
  99,  99,  101, 108, 101, 114, 111, 109, 101, 116, 101, 114, 34,  44,  34,  84,  101, 109, 112, 101, 114, 97,  116, 117, 114, 101, 66,
  111, 117, 110, 100, 115, 34,  58,  91,  53,  44,  54,  48,  93,  44,  34,  84,  101, 109, 112, 101, 114, 97,  116, 117, 114, 101, 67,
  34,  58,  48,  125, 93,  44,  34,  77,  101, 116, 97,  100, 97,  116, 97,  34,  58,  123, 34,  83,  101, 114, 105, 97,  108, 73,  100,
  34,  58,  34,  48,  48,  48,  50,  49,  50,  50,  57,  50,  57,  49,  50,  34,  44,  34,  70,  97,  99,  116, 111, 114, 121, 67,  97,
  108, 68,  97,  116, 101, 34,  58,  34,  55,  47,  49,  56,  47,  50,  48,  49,  57,  32,  49,  48,  58,  52,  55,  58,  50,  48,  32,
  80,  77,  32,  71,  77,  84,  34,  44,  34,  86,  101, 114, 115, 105, 111, 110, 34,  58,  123, 34,  77,  97,  106, 111, 114, 34,  58,
  49,  44,  34,  77,  105, 110, 111, 114, 34,  58,  50,  125, 44,  34,  68,  101, 118, 105, 99,  101, 78,  97,  109, 101, 34,  58,  34,
  65,  122, 117, 114, 101, 75,  105, 110, 101, 99,  116, 45,  80,  86,  34,  44,  34,  78,  111, 116, 101, 115, 34,  58,  34,  80,  86,
  48,  95,  109, 97,  120, 95,  114, 97,  100, 105, 117, 115, 95,  105, 110, 118, 97,  108, 105, 100, 34,  125, 125, 125, 0};

// ------------------------------------------------------------------------- //

void ShowCameraInfo(k4a_calibration_t * data) {
  cv::Size img_size = cv::Size(data->color_camera_calibration.resolution_width, data->color_camera_calibration.resolution_height);

  Eigen::Matrix3d cam_eigen_mtx;
  cam_eigen_mtx.setZero();
  cam_eigen_mtx(0, 0) = data->color_camera_calibration.intrinsics.parameters.param.fx;
  cam_eigen_mtx(1, 1) = data->color_camera_calibration.intrinsics.parameters.param.fy;
  cam_eigen_mtx(0, 2) = data->color_camera_calibration.intrinsics.parameters.param.cx;
  cam_eigen_mtx(1, 2) = data->color_camera_calibration.intrinsics.parameters.param.cy;
  cam_eigen_mtx(2, 2) = 1;
  cv::Mat cam_mtx;
  cv::eigen2cv(cam_eigen_mtx, cam_mtx);

  std::vector<double> dist_vec;
  dist_vec.emplace_back(data->color_camera_calibration.intrinsics.parameters.param.k1);
  dist_vec.emplace_back(data->color_camera_calibration.intrinsics.parameters.param.k2);
  dist_vec.emplace_back(data->color_camera_calibration.intrinsics.parameters.param.p1);
  dist_vec.emplace_back(data->color_camera_calibration.intrinsics.parameters.param.p2);
  dist_vec.emplace_back(data->color_camera_calibration.intrinsics.parameters.param.k3);
  dist_vec.emplace_back(data->color_camera_calibration.intrinsics.parameters.param.k4);
  dist_vec.emplace_back(data->color_camera_calibration.intrinsics.parameters.param.k5);
  dist_vec.emplace_back(data->color_camera_calibration.intrinsics.parameters.param.k6);
  cv::Mat dist_coeff = cv::Mat(1, dist_vec.size(), CV_64F, dist_vec.data()).clone();

  cv::Mat proj_mtx = cv::getOptimalNewCameraMatrix(cam_mtx, dist_coeff, img_size, 0);

  std::cout << colorful_char::info("Intrinisc for Kinect Depth Camera: ") << std::endl << std::endl;
  std::cout << colorful_char::info("image_width: ") << img_size.width << std::endl;
  std::cout << colorful_char::info("image_height: ") << img_size.height << std::endl;
  std::cout << colorful_char::info("camera_name: ") + "rgbd_depth_camera" << std::endl;
  std::cout << colorful_char::info("camera_matrix: ") << std::endl << cam_mtx << std::endl;
  std::cout << colorful_char::info("camera_model: ") + "brown_conrady" << std::endl;
  std::cout << colorful_char::info("distortion_coefficients: ") << std::endl << dist_coeff << std::endl;
  std::cout << colorful_char::info("projection_matrix: ") << std::endl << proj_mtx << std::endl;
}

// The following function is created and modified based on UnaNancyOwen/AzureKinectSample.
// Please refer to https://github.com/UnaNancyOwen/AzureKinectSample
cv::Mat get_mat(k4a::image & src, bool deep_copy = true) {
  cv::Mat                  mat;
  const int32_t            width  = src.get_width_pixels();
  const int32_t            height = src.get_height_pixels();
  const k4a_image_format_t format = src.get_format();

  switch (format) {
    case k4a_image_format_t::K4A_IMAGE_FORMAT_COLOR_MJPG: {
      // NOTE: this is slower than other formats.
      std::vector<uint8_t> buffer(src.get_buffer(), src.get_buffer() + src.get_size());
      mat = cv::imdecode(buffer, cv::IMREAD_ANYCOLOR);
      cv::cvtColor(mat, mat, cv::COLOR_BGR2BGRA);
      break;
    }
    case k4a_image_format_t::K4A_IMAGE_FORMAT_COLOR_NV12: {
      cv::Mat nv12 = cv::Mat(height + height / 2, width, CV_8UC1, src.get_buffer()).clone();
      cv::cvtColor(nv12, mat, cv::COLOR_YUV2BGRA_NV12);
      break;
    }
    case k4a_image_format_t::K4A_IMAGE_FORMAT_COLOR_YUY2: {
      cv::Mat yuy2 = cv::Mat(height, width, CV_8UC2, src.get_buffer()).clone();
      cv::cvtColor(yuy2, mat, cv::COLOR_YUV2BGRA_YUY2);
      break;
    }
    case k4a_image_format_t::K4A_IMAGE_FORMAT_COLOR_BGRA32: {
      mat = deep_copy ? cv::Mat(height, width, CV_8UC4, src.get_buffer()).clone() : cv::Mat(height, width, CV_8UC4, src.get_buffer());
      break;
    }
    case k4a_image_format_t::K4A_IMAGE_FORMAT_DEPTH16:
    case k4a_image_format_t::K4A_IMAGE_FORMAT_IR16: {
      mat = deep_copy ? cv::Mat(height, width, CV_16UC1, reinterpret_cast<uint16_t *>(src.get_buffer())).clone() :
                        cv::Mat(height, width, CV_16UC1, reinterpret_cast<uint16_t *>(src.get_buffer()));
      break;
    }
    case k4a_image_format_t::K4A_IMAGE_FORMAT_CUSTOM8: {
      mat = cv::Mat(height, width, CV_8UC1, src.get_buffer()).clone();
      break;
    }
    case k4a_image_format_t::K4A_IMAGE_FORMAT_CUSTOM: {
      // NOTE: This is opencv_viz module format (cv::viz::WCloud).
      const int16_t * buffer = reinterpret_cast<int16_t *>(src.get_buffer());
      mat                    = cv::Mat(height, width, CV_32FC3, cv::Vec3f::all(std::numeric_limits<float>::quiet_NaN()));
      mat.forEach<cv::Vec3f>([&](cv::Vec3f & point, const int32_t * position) {
        const int32_t index = (position[0] * width + position[1]) * 3;
        point               = cv::Vec3f(buffer[index + 0], buffer[index + 1], buffer[index + 2]);
      });
      break;
    }
    default: throw k4a::error("Failed to convert this format!"); break;
  }
  return mat;
}

// ------------------------------------------------------------------------- //

int main(int argc, char ** argv) {
  ros::init(argc, argv, "k4a_projector");
  ros::NodeHandle nh;

  bool display_kinect_intrinsic;
  ros::param::get("/display_kinect_intrinsic", display_kinect_intrinsic);

  // Read each camera's calibration result from the input path.
  std::vector<CameraCalibration> cam_calib_vec;
  std::vector<std::string>       cam_calib_path_vec;
  ros::param::get("/target_camera_calibration_yaml_path", cam_calib_path_vec);
  for (std::string cam_calib_path : cam_calib_path_vec) {
    cam_calib_vec.emplace_back(cam_calib_path);
    if (!cam_calib_vec.back().status()) {
      ros::shutdown();
      return -1;
    }
  }
  int cam_num = cam_calib_vec.size();

  // Read Kinect's factory calibration results.
  k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
  config.color_format               = K4A_IMAGE_FORMAT_COLOR_BGRA32;
  config.color_resolution           = K4A_COLOR_RESOLUTION_720P;
  config.depth_mode                 = K4A_DEPTH_MODE_NFOV_UNBINNED;
  config.camera_fps                 = K4A_FRAMES_PER_SECOND_30;
  config.synchronized_images_only   = true;

  std::vector<k4a::transformation> k4a_tf_vec;
  for (size_t idx = 0; idx < cam_num; ++idx) {
    k4a::calibration k4a_calib = k4a::calibration::get_from_raw(raw_calibration_data, config.depth_mode, config.color_resolution);

    // Setup k4a calibration config (target camera intrinsics).
    k4a_calib.color_camera_calibration.intrinsics.parameters.param.fx   = cam_calib_vec[idx].projection_matrix().at<double>(0, 0);
    k4a_calib.color_camera_calibration.intrinsics.parameters.param.fy   = cam_calib_vec[idx].projection_matrix().at<double>(1, 1);
    k4a_calib.color_camera_calibration.intrinsics.parameters.param.cx   = cam_calib_vec[idx].projection_matrix().at<double>(0, 2);
    k4a_calib.color_camera_calibration.intrinsics.parameters.param.cy   = cam_calib_vec[idx].projection_matrix().at<double>(1, 2);
    k4a_calib.color_camera_calibration.intrinsics.parameters.param.k1   = 0;
    k4a_calib.color_camera_calibration.intrinsics.parameters.param.k2   = 0;
    k4a_calib.color_camera_calibration.intrinsics.parameters.param.p1   = 0;
    k4a_calib.color_camera_calibration.intrinsics.parameters.param.p2   = 0;
    k4a_calib.color_camera_calibration.intrinsics.parameters.param.k3   = 0;
    k4a_calib.color_camera_calibration.intrinsics.parameters.param.k4   = 0;
    k4a_calib.color_camera_calibration.intrinsics.parameters.param.k5   = 0;
    k4a_calib.color_camera_calibration.intrinsics.parameters.param.k6   = 0;
    k4a_calib.color_camera_calibration.intrinsics.parameters.param.codx = 0;
    k4a_calib.color_camera_calibration.intrinsics.parameters.param.cody = 0;
    k4a_calib.color_camera_calibration.intrinsics.type                  = K4A_CALIBRATION_LENS_DISTORTION_MODEL_BROWN_CONRADY;
    k4a_calib.color_camera_calibration.resolution_width                 = cam_calib_vec[idx].width();
    k4a_calib.color_camera_calibration.resolution_height                = cam_calib_vec[idx].height();

    // Setup k4a calibration config (kinect extrinsics). Note that k4a translation uses mm as unit.
    // clang-format off
    double T_color_depth_vec[16] = {
       0.9999983519720124   ,  0.0007095286521193374, -0.001671114104806111, -0.03128054306089928 ,
      -0.0005461132760243128,  0.995397130621369    ,  0.0958345141952277  , -0.001701648753243945,
       0.001731419518548352 , -0.09583344363966775  ,  0.9953958776620555  , -0.01229668401143895 ,
       0.0                  ,  0.0                  ,  0.0                 ,  1.0                          
    };
    // clang-format on
    cv::Mat T_color_depth = cv::Mat(4, 4, CV_64F, T_color_depth_vec).clone();
    cv::Mat T_target_color;
    cv::invert(cam_calib_vec[idx].extrinisc_matrix(), T_target_color);
    cv::Mat T_target_depth = T_target_color * T_color_depth;
    cv::Mat T_depth_target;
    cv::invert(T_target_depth, T_depth_target);
    k4a_calib.extrinsics[K4A_CALIBRATION_TYPE_DEPTH][K4A_CALIBRATION_TYPE_COLOR].rotation[0]    = T_target_depth.at<double>(0, 0);
    k4a_calib.extrinsics[K4A_CALIBRATION_TYPE_DEPTH][K4A_CALIBRATION_TYPE_COLOR].rotation[1]    = T_target_depth.at<double>(0, 1);
    k4a_calib.extrinsics[K4A_CALIBRATION_TYPE_DEPTH][K4A_CALIBRATION_TYPE_COLOR].rotation[2]    = T_target_depth.at<double>(0, 2);
    k4a_calib.extrinsics[K4A_CALIBRATION_TYPE_DEPTH][K4A_CALIBRATION_TYPE_COLOR].rotation[3]    = T_target_depth.at<double>(1, 0);
    k4a_calib.extrinsics[K4A_CALIBRATION_TYPE_DEPTH][K4A_CALIBRATION_TYPE_COLOR].rotation[4]    = T_target_depth.at<double>(1, 1);
    k4a_calib.extrinsics[K4A_CALIBRATION_TYPE_DEPTH][K4A_CALIBRATION_TYPE_COLOR].rotation[5]    = T_target_depth.at<double>(1, 2);
    k4a_calib.extrinsics[K4A_CALIBRATION_TYPE_DEPTH][K4A_CALIBRATION_TYPE_COLOR].rotation[6]    = T_target_depth.at<double>(2, 0);
    k4a_calib.extrinsics[K4A_CALIBRATION_TYPE_DEPTH][K4A_CALIBRATION_TYPE_COLOR].rotation[7]    = T_target_depth.at<double>(2, 1);
    k4a_calib.extrinsics[K4A_CALIBRATION_TYPE_DEPTH][K4A_CALIBRATION_TYPE_COLOR].rotation[8]    = T_target_depth.at<double>(2, 2);
    k4a_calib.extrinsics[K4A_CALIBRATION_TYPE_DEPTH][K4A_CALIBRATION_TYPE_COLOR].translation[0] = T_target_depth.at<double>(0, 3) * 1000;
    k4a_calib.extrinsics[K4A_CALIBRATION_TYPE_DEPTH][K4A_CALIBRATION_TYPE_COLOR].translation[1] = T_target_depth.at<double>(1, 3) * 1000;
    k4a_calib.extrinsics[K4A_CALIBRATION_TYPE_DEPTH][K4A_CALIBRATION_TYPE_COLOR].translation[2] = T_target_depth.at<double>(2, 3) * 1000;
    k4a_calib.extrinsics[K4A_CALIBRATION_TYPE_COLOR][K4A_CALIBRATION_TYPE_DEPTH].rotation[0]    = T_depth_target.at<double>(0, 0);
    k4a_calib.extrinsics[K4A_CALIBRATION_TYPE_COLOR][K4A_CALIBRATION_TYPE_DEPTH].rotation[1]    = T_depth_target.at<double>(0, 1);
    k4a_calib.extrinsics[K4A_CALIBRATION_TYPE_COLOR][K4A_CALIBRATION_TYPE_DEPTH].rotation[2]    = T_depth_target.at<double>(0, 2);
    k4a_calib.extrinsics[K4A_CALIBRATION_TYPE_COLOR][K4A_CALIBRATION_TYPE_DEPTH].rotation[3]    = T_depth_target.at<double>(1, 0);
    k4a_calib.extrinsics[K4A_CALIBRATION_TYPE_COLOR][K4A_CALIBRATION_TYPE_DEPTH].rotation[4]    = T_depth_target.at<double>(1, 1);
    k4a_calib.extrinsics[K4A_CALIBRATION_TYPE_COLOR][K4A_CALIBRATION_TYPE_DEPTH].rotation[5]    = T_depth_target.at<double>(1, 2);
    k4a_calib.extrinsics[K4A_CALIBRATION_TYPE_COLOR][K4A_CALIBRATION_TYPE_DEPTH].rotation[6]    = T_depth_target.at<double>(2, 0);
    k4a_calib.extrinsics[K4A_CALIBRATION_TYPE_COLOR][K4A_CALIBRATION_TYPE_DEPTH].rotation[7]    = T_depth_target.at<double>(2, 1);
    k4a_calib.extrinsics[K4A_CALIBRATION_TYPE_COLOR][K4A_CALIBRATION_TYPE_DEPTH].rotation[8]    = T_depth_target.at<double>(2, 2);
    k4a_calib.extrinsics[K4A_CALIBRATION_TYPE_COLOR][K4A_CALIBRATION_TYPE_DEPTH].translation[0] = T_depth_target.at<double>(0, 3) * 1000;
    k4a_calib.extrinsics[K4A_CALIBRATION_TYPE_COLOR][K4A_CALIBRATION_TYPE_DEPTH].translation[1] = T_depth_target.at<double>(1, 3) * 1000;
    k4a_calib.extrinsics[K4A_CALIBRATION_TYPE_COLOR][K4A_CALIBRATION_TYPE_DEPTH].translation[2] = T_depth_target.at<double>(2, 3) * 1000;

    k4a_tf_vec.emplace_back(k4a::transformation(k4a_calib));
    if (display_kinect_intrinsic && idx == 0)
      ShowCameraInfo(&k4a_calib);
  }

  // ----------------------------------------------------------------------- //

  // Read ROS-related parameters.
  std::string              source_bag_path_in, bag_path_out;
  std::vector<std::string> target_bag_path_in_vec;
  ros::param::get("/source_rosbag_path", source_bag_path_in);
  ros::param::get("/target_rosbag_path", target_bag_path_in_vec);
  if (!file_path_check(source_bag_path_in)) {
    ros::shutdown();
    return -1;
  }
  for (std::string & target_bag_path_in : target_bag_path_in_vec) {
    if (!file_path_check(target_bag_path_in)) {
      ros::shutdown();
      return -1;
    }
  }
  bag_path_out = source_bag_path_in.substr(0, source_bag_path_in.size() - 16) + "depth_reprojection.bag";

  // Read each ROS bag.
  size_t                              msg_size = 0;
  size_t                              bag_num  = cam_num + 1;
  rosbag::Bag                         bags_in[bag_num];
  rosbag::View                        bags_views[bag_num];
  std::vector<rosbag::View::iterator> bags_msg_ptr;
  std::vector<bool>                   are_bags_valid;
  for (size_t idx = 0; idx < bag_num; ++idx) {
    if (idx == bag_num - 1)
      bags_in[idx].open(source_bag_path_in, rosbag::bagmode::Read);
    else
      bags_in[idx].open(target_bag_path_in_vec[idx], rosbag::bagmode::Read);
    bags_views[idx].addQuery(bags_in[idx]);
    msg_size += bags_views[idx].size();
    bags_msg_ptr.emplace_back(bags_views[idx].begin());
    are_bags_valid.emplace_back(true);
  }

  // Read and process each ROS message from different ROS bags.
  rosbag::Bag bag_out;
  bag_out.open(bag_path_out, rosbag::bagmode::Write);
  size_t msg_idx = 0;
  while (bag_num != 1) {
    int    earliest_idx;
    double earliest_ts = DBL_MAX;
    for (size_t idx = 0; idx < cam_num + 1; ++idx) {
      if (are_bags_valid[idx] && earliest_ts > (*bags_msg_ptr[idx]).getTime().toSec()) {
        earliest_idx = idx;
        earliest_ts  = (*bags_msg_ptr[idx]).getTime().toSec();
      }
    }

    // Undistort the original message from the target ROS bags.
    if (bags_msg_ptr[earliest_idx]->getTopic() == "/camera/left/image_mono") {
      Image::Ptr img_msg = bags_msg_ptr[earliest_idx]->instantiate<Image>();
      cv::Mat    cv_img  = cv_bridge::toCvCopy(img_msg)->image;
      cv::remap(cv_img, cv_img, cam_calib_vec[earliest_idx].undistortion_map_x(), cam_calib_vec[earliest_idx].undistortion_map_y(),
                cv::INTER_LINEAR);
      Image::Ptr undistorted_img_msg = cv_bridge::CvImage(img_msg->header, img_msg->encoding, cv_img).toImageMsg();
      bag_out.write("/camera/left/image_mono_undistort", img_msg->header.stamp, undistorted_img_msg);
    }

    else if (bags_msg_ptr[earliest_idx]->getTopic() == "/camera/right/image_mono") {
      Image::Ptr img_msg = bags_msg_ptr[earliest_idx]->instantiate<Image>();
      cv::Mat    cv_img  = cv_bridge::toCvCopy(img_msg)->image;
      cv::remap(cv_img, cv_img, cam_calib_vec[earliest_idx].undistortion_map_x(), cam_calib_vec[earliest_idx].undistortion_map_y(),
                cv::INTER_LINEAR);
      Image::Ptr undistorted_img_msg = cv_bridge::CvImage(img_msg->header, img_msg->encoding, cv_img).toImageMsg();
      bag_out.write("/camera/right/image_mono_undistort", img_msg->header.stamp, undistorted_img_msg);
    }

    else if (bags_msg_ptr[earliest_idx]->getTopic() == "/prophesee/left/events") {
      // Event data will not be undistorted given its sparse nature.
    }

    else if (bags_msg_ptr[earliest_idx]->getTopic() == "/prophesee/right/events") {
      // Event data will not be undistorted given its sparse nature.
    }

    // Reproject the depth readings onto target frames.
    else if (bags_msg_ptr[earliest_idx]->getTopic() == "/kinect/depth_image") {
      for (size_t idx = 0; idx < cam_num; ++idx) {
        Image::Ptr img_msg     = bags_msg_ptr[earliest_idx]->instantiate<Image>();
        cv::Mat    cv_img      = cv_bridge::toCvCopy(img_msg)->image;
        k4a::image k4a_img_in  = k4a::image::create_from_buffer(K4A_IMAGE_FORMAT_DEPTH16, 640, 576, 640 * sizeof(uint16_t),
                                                               (uint8_t *) cv_img.data, 640 * 576 * sizeof(uint16_t), nullptr, nullptr);
        k4a::image k4a_img_out = k4a::image::create(K4A_IMAGE_FORMAT_DEPTH16, cam_calib_vec[idx].width(), cam_calib_vec[idx].height(),
                                                    cam_calib_vec[idx].width() * sizeof(uint16_t));
        k4a_tf_vec[idx].depth_image_to_color_camera(k4a_img_in, &k4a_img_out);
        Image::Ptr undistorted_img_msg = cv_bridge::CvImage(img_msg->header, img_msg->encoding, get_mat(k4a_img_out)).toImageMsg();
        if (cam_calib_vec[idx].name() == "left_regular_camera")
          bag_out.write("/camera/left/depth_image_undistort", img_msg->header.stamp, undistorted_img_msg);
        else if (cam_calib_vec[idx].name() == "right_regular_camera")
          bag_out.write("/camera/right/depth_image_undistort", img_msg->header.stamp, undistorted_img_msg);
        else if (cam_calib_vec[idx].name() == "left_event_camera")
          bag_out.write("/prophesee/left/depth_image_undistort", img_msg->header.stamp, undistorted_img_msg);
        else if (cam_calib_vec[idx].name() == "right_event_camera")
          bag_out.write("/prophesee/right/depth_image_undistort", img_msg->header.stamp, undistorted_img_msg);
        else
          ROS_WARN("%s", colorful_char::warning("Unrecognize Camera Name! Please customize this camera in the code!").c_str());
      }
    }

    else
      ROS_WARN("%s", colorful_char::warning("Unrecognize ROS topic! Please customize this topic in the code!").c_str());

    ++bags_msg_ptr[earliest_idx];
    if (bags_msg_ptr[earliest_idx] == bags_views[earliest_idx].end()) {
      are_bags_valid[earliest_idx] = false;
      --bag_num;
      bags_in[earliest_idx].close();
    }

    ++msg_idx;
    if (msg_idx % 100 == 0)
      ROS_INFO_STREAM(msg_idx << "/" << msg_size << " messages have been processed.");
  }
  ROS_INFO("%s", colorful_char::info("All messages have been processed! Enjoy!").c_str());
  bag_out.close();

  ros::shutdown();
  return 0;
}
