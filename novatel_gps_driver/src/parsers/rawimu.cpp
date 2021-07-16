// *****************************************************************************
//
// Copyright (c) 2021, Will Bryan and Southwest Research Institute® (SwRI®)
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Southwest Research Institute® (SwRI®) nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL SOUTHWEST RESEARCH INSTITUTE BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// *****************************************************************************

#include <sstream>

#include <novatel_gps_driver/parsers/rawimu.h>
#include <novatel_gps_driver/parsers/header.h>

const std::string novatel_gps_driver::RawImuParser::MESSAGE_NAME = "RAWIMUS";

uint32_t novatel_gps_driver::RawImuParser::GetMessageId() const
{
  return MESSAGE_ID;
}

const std::string novatel_gps_driver::RawImuParser::GetMessageName() const
{
  return MESSAGE_NAME;
}

novatel_gps_driver::RawImuParser::MessageType
novatel_gps_driver::RawImuParser::ParseBinary(const novatel_gps_driver::BinaryMessage& bin_msg) noexcept(false)
{
  if (bin_msg.data_.size() != BINARY_LENGTH)
  {
    std::stringstream error;
    error << "Unexpected rawimu message size: " << bin_msg.data_.size();
    throw ParseException(error.str());
  }
  auto ros_msg = std::make_unique<novatel_gps_msgs::msg::NovatelRawImu>();
  HeaderParser h_parser;
  ros_msg->novatel_msg_header = h_parser.ParseBinary(bin_msg);
  ros_msg->novatel_msg_header.message_name = "RAWIMUS";

  // Scale factors - Values for G320N IMU
  double DataRate = 125; // Hz (must be full rate to use for navigation)
  double ONE_G = 9.80665; // m/s/s
  double sf_az = (0.200/65536)*(ONE_G/1000)/(DataRate); // m/s/LSB
  double sf_ay = (0.200/65536)*(ONE_G/1000)/(DataRate); // m/s/LSB
  double sf_ax = (0.200/65536)*(ONE_G/1000)/(DataRate); // m/s/LSB
  double sf_wz = (0.008/65536)/(DataRate); // deg/LSB;
  double sf_wy = (0.008/65536)/(DataRate); // deg/LSB;
  double sf_wx = (0.008/65536)/(DataRate); // deg/LSB;
  const double pi = 3.14159265358979323846;

  // New parsing - Puts into m/s/s and rad/s
  ros_msg->gps_week_num = ParseUInt32(&bin_msg.data_[0]);
  ros_msg->gps_seconds = ParseDouble(&bin_msg.data_[4]);
  ros_msg->linear_acceleration.z = sf_az*DataRate*static_cast<double>(ParseInt32(&bin_msg.data_[16]));
  ros_msg->linear_acceleration.y = sf_ay*DataRate*static_cast<double>(ParseInt32(&bin_msg.data_[20]));
  ros_msg->linear_acceleration.x = sf_ax*DataRate*static_cast<double>(ParseInt32(&bin_msg.data_[24]));
  ros_msg->angular_velocity.z = sf_wz*DataRate*static_cast<double>(ParseInt32(&bin_msg.data_[28]))*pi/180.0;
  ros_msg->angular_velocity.y = sf_wy*DataRate*static_cast<double>(ParseInt32(&bin_msg.data_[32]))*pi/180.0;
  ros_msg->angular_velocity.x = sf_wx*DataRate*static_cast<double>(ParseInt32(&bin_msg.data_[36]))*pi/180.0;

  return ros_msg;
}

// DO NOT USE ASCII - it is not suitable for high frequency and will give raw values
//   not the values converted to accelerations and angular velocities.
novatel_gps_driver::RawImuParser::MessageType
novatel_gps_driver::RawImuParser::ParseAscii(const novatel_gps_driver::NovatelSentence& sentence) noexcept(false)
{
  if (sentence.body.size() != ASCII_FIELDS)
  {
    std::stringstream error;
    error << "Unexpected number of fields in RAWIMUS log: " << sentence.body.size();
    throw ParseException(error.str());
  }
  auto msg = std::make_unique<novatel_gps_msgs::msg::NovatelRawImu>();
  HeaderParser h_parser;
  msg->novatel_msg_header = h_parser.ParseAscii(sentence);

  bool valid = true;

  // Will only give raw measurements, not useful data
  valid &= ParseUInt32(sentence.body[0], msg->gps_week_num);
  valid &= ParseDouble(sentence.body[1], msg->gps_seconds);
  valid &= ParseDouble(sentence.body[3], msg->linear_acceleration.z);
  valid &= ParseDouble(sentence.body[4], msg->linear_acceleration.y);
  valid &= ParseDouble(sentence.body[5], msg->linear_acceleration.x);
  valid &= ParseDouble(sentence.body[6], msg->angular_velocity.z);
  valid &= ParseDouble(sentence.body[7], msg->angular_velocity.y);
  valid &= ParseDouble(sentence.body[8], msg->angular_velocity.x);

  if (!valid)
  {
    throw ParseException("Error parsing RAWIMUS log.");
  }

  return msg;
}
