/*********************************************************************************************************************
Copyright (c) 2020 RoboSense
All rights reserved

By downloading, copying, installing or using the software you agree to this license. If you do not agree to this
license, do not download, install, copy or use the software.

License Agreement
For RoboSense LiDAR SDK Library
(3-clause BSD License)

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the names of the RoboSense, nor Suteng Innovation Technology, nor the names of other contributors may be used
to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*********************************************************************************************************************/

#pragma once

#include <rs_driver/driver/decoder/decoder.hpp>
#include <rs_driver/driver/decoder/compress_algo.hpp>

#define EM4_SURFACE_NUM 4
#define EM4_PIXELS_PER_COLUMN 520
#define EM4_VECSELS_PER_COLUMN 26
#define EM4_PIXELS_PER_VCSEL 20
#define EM4_COMPRESS_MAX_LEN 1500*2
#define MSOP_TAIL_LEN 12
namespace robosense
{
namespace lidar
{
#pragma pack(push, 1)

typedef struct
{
  uint8_t id[8];
  uint8_t reserved_0[106];
  int8_t yaw_offset[26];
  int16_t pitch_angle[520];
  int16_t surface_pitch_offset[4];
  uint8_t reserved_1[110];
  uint16_t data_length;
  uint16_t counter;
  uint32_t data_id;
  uint32_t crc32;
} RSEM4DifopPkt;

typedef struct
{
  uint8_t id[4];
  uint8_t reserved0[63];
  uint8_t surface_id;
  uint8_t pixelCnt;  // 1 vcsel: 20 pixel
  uint8_t vcselCnt;  // 1 column: 26 vcsel
  int8_t yaw_offset[26];
  int16_t pitch_angle[520];
  int16_t surface_pitch_offset[4];
  int16_t roll_offset;
  uint8_t reserved1[4];
  uint16_t data_length;
  uint16_t counter;
  uint32_t data_id;
  uint32_t crc32;
} RSEM4Difop2Pkt;

typedef struct
{
  uint8_t id[4];
  uint8_t reserved_0[306];
  int8_t yaw_offset[26];
  int16_t pitch_angle[520];
  int16_t surface_pitch_offset[4];
  uint8_t reserved_1[6];
  uint16_t data_length;
  uint16_t counter;
  uint32_t data_id;
  uint32_t crc32;
} RSEM4DifopPkt0624;

typedef struct
{
  uint16_t distance;
  uint8_t intensity;
  uint8_t point_attribute;
} RSEM4Channel;  // 4-bytes

typedef struct
{
  RSEM4Channel channel[1];
} RSEM4Block;

typedef struct
{
  uint8_t id[4];
  uint16_t pkt_seq;
  uint16_t protocol_version;
  uint8_t return_mode;
  uint8_t time_mode;
  RSTimestampUTC timestamp;
  uint8_t fram_sync;
  uint8_t frame_rate;
  uint16_t column_num;
  int16_t yaw_angle;
  uint8_t pack_mode;
  uint8_t surface_id;
  uint16_t reserved;
  uint8_t lidar_type;
  uint8_t temperature;
} RSEM4MsopHeader;  // 32-bytes

typedef struct
{
  uint8_t id[4];
  uint16_t pkt_seq;
  uint8_t reserved[2];
} RSEM4MsopHeader2;  // 8-bytes

typedef struct
{
  RSEM4MsopHeader header;
  RSEM4Block blocks[260];
  uint16_t data_length;
  uint16_t counter;
  uint32_t data_id;
  uint32_t crc32;
} RSEM4MsopPkt;  // 1084-bytes

#pragma pack(pop)

class CurentPacketInfo
{
public:
  CurentPacketInfo(){}
  ~CurentPacketInfo()
  {
    clear();
  }
  void clear(){
    data_length = 0;
    received_packet_len = 0;
    pre_packet_seq = -1;
  }
  //Without packet splitting, receive all data
  void getPackedData(uint8_t* data, uint16_t len)
  {
    if (len > EM4_COMPRESS_MAX_LEN) {
      RS_ERROR << "Buffer overflow detected in CurentPacketInfo::getPackedData";
      return;
    }
    std::memcpy(packed_data, data, len);
    data_length += len;
    convertEndianness();
  }
  void convertEndianness(){
    for (uint16_t i = 0; i < data_length / 2; ++i) {
      uint16_t temp_val;
      std::memcpy(&temp_val, packed_data + i * 2, sizeof(uint16_t)); 
      packed_data_endian[i] = ntohs(temp_val);
    }
  }
  void receiveParticalPacket(const uint8_t* data, uint16_t len, uint16_t seq)
  {
    pre_packet_seq = seq;
    if (received_packet_len+len > EM4_COMPRESS_MAX_LEN) {
      RS_ERROR << "Buffer overflow detected in CurentPacketInfo::receiveParticalPacket";
      return;
    }
    std::memcpy(received_packet+received_packet_len, data, len);
    received_packet_len += len;
  }
  //
  uint8_t received_packet[EM4_COMPRESS_MAX_LEN] = {0};  // Received packet data
  uint16_t received_packet_len = 0;  // Length of the received packet
  uint16_t pre_packet_seq = -1;
  //
  uint8_t packed_data[EM4_COMPRESS_MAX_LEN] = {0}; //packed data segment
  uint16_t packed_data_endian[EM4_COMPRESS_MAX_LEN/2] = { 0 };
  uint16_t data_length = 0;
};

template <typename T_PointCloud>
class DecoderRSEM4 : public Decoder<T_PointCloud>
{
public:
  constexpr static double FRAME_DURATION = 0.1;
  constexpr static uint32_t SINGLE_PKT_NUM = 2400;

  virtual bool decodeMsopPkt(const uint8_t* pkt, size_t size) override;
  virtual ~DecoderRSEM4(){};

  virtual void decodeDifopPkt(const uint8_t* pkt, size_t size) override;

  explicit DecoderRSEM4(const RSDecoderParam& param);

private:
  std::array<int16_t, EM4_VECSELS_PER_COLUMN> yaw_offset_;
  std::array<int16_t, EM4_PIXELS_PER_COLUMN> pitch_angle_;
  std::array<int16_t, EM4_SURFACE_NUM> surface_pitch_offset_;
  std::array<uint16_t, 2 * EM4_PIXELS_PER_COLUMN> dual_return_pitch_index_;
  CurentPacketInfo packet_info_;

  static RSDecoderConstParam& getConstParam();
  bool decodeGeneralPkt(const uint8_t* pkt, size_t size);
  bool decodeCompPkt(const uint8_t* pkt, size_t size);

  SplitStrategyBySeq split_strategy_;

  void rleDecodeMethod(const uint8_t* udp_payload, size_t size, const double pkt_ts);
};
template <typename T_PointCloud>
inline RSDecoderConstParam& DecoderRSEM4<T_PointCloud>::getConstParam()
{
  static RSDecoderConstParam param = {
    1084  // msop len
    ,
    1310  // difop len
    ,
    3  // msop id len
    ,
    3  // difop id len
    ,
    { 0x55, 0xAA, 0x5A }  // msop id
    ,
    { 0xA5, 0xFF, 0x00, 0x5A, 0x11, 0x11, 0x55, 0x55 }  // difop id
    ,
    { 0x00, 0x00 },
    1  // laser number
    ,
    260  // blocks per packet
    ,
    1  // channels per block
    ,
    0.5f  // distance min
    ,
    350.0f  // distance max
    ,
    0.005f  // distance resolution
    ,
    80.0f  // initial value of temperature
  };

  return param;
}

template <typename T_PointCloud>
inline DecoderRSEM4<T_PointCloud>::DecoderRSEM4(const RSDecoderParam& param)
  : Decoder<T_PointCloud>(getConstParam(), param)
{
  this->packet_duration_ = FRAME_DURATION / SINGLE_PKT_NUM;
  this->bCheckMsopLen_ = false;
  this->bCheckDifopLen_ = false;

  this->yaw_offset_.fill(0);

  std::array<int16_t, EM4_PIXELS_PER_COLUMN> defaultAngle;
  constexpr int16_t START_ANGLE = -1300;
  constexpr int16_t ANGLE_STEP = 5;
  for (int i = 0; i < EM4_PIXELS_PER_COLUMN; ++i)
  {
    defaultAngle[i] = START_ANGLE + i * ANGLE_STEP;
  }
  this->pitch_angle_ .fill(0);

  this->surface_pitch_offset_.fill(0);

  for (int i = 0, j = 0; i < EM4_PIXELS_PER_COLUMN; i++)
  {
    this->dual_return_pitch_index_[j++] = i;
    this->dual_return_pitch_index_[j++] = i;
  }
}

template <typename T_PointCloud>
inline void DecoderRSEM4<T_PointCloud>::decodeDifopPkt(const uint8_t* packet, size_t size)
{
  if (packet == nullptr)
    return;

  const static uint16_t DIFOP1_LEN = sizeof(RSEM4DifopPkt);
  const static uint16_t DIFOP2_LEN = sizeof(RSEM4Difop2Pkt);
  const static uint16_t DIFOP0624_LEN = sizeof(RSEM4DifopPkt0624);

  auto processDifopPkt = [this](const auto& pkt) {
    for (int i = 0; i < EM4_VECSELS_PER_COLUMN; ++i)
    {
      this->yaw_offset_[i] = pkt.yaw_offset[i];
    }
    for (int i = 0; i < EM4_PIXELS_PER_COLUMN; ++i)
    {
      this->pitch_angle_[i] = RS_SWAP_INT16(pkt.pitch_angle[i]);
    }
    for (int i = 0; i < EM4_SURFACE_NUM; ++i)
    {
      this->surface_pitch_offset_[i] = RS_SWAP_INT16(pkt.surface_pitch_offset[i]);
    }

    this->angles_ready_ = true;
  };

  if (size == DIFOP1_LEN)
  {
    processDifopPkt(*reinterpret_cast<const RSEM4DifopPkt*>(packet));
  }
  else if (size == DIFOP2_LEN)
  {
    processDifopPkt(*reinterpret_cast<const RSEM4Difop2Pkt*>(packet));
  }
  else if(size ==DIFOP0624_LEN)
  {
    processDifopPkt(*reinterpret_cast<const RSEM4DifopPkt0624*>(packet));
  }
}

template <typename T_PointCloud>
inline bool DecoderRSEM4<T_PointCloud>::decodeMsopPkt(const uint8_t* packet, size_t size)
{
  static const uint8_t complete_pack_header[] = {0x55, 0xAA, 0x5A, 0xA5};
  if(memcmp(packet, complete_pack_header, sizeof(complete_pack_header)) != 0){
    const RSEM4MsopHeader2& header2 = *(RSEM4MsopHeader2*)packet;
    uint16_t pkt_seq = ntohs(header2.pkt_seq);
    if(pkt_seq!= packet_info_.pre_packet_seq){ //lost packet
      packet_info_.clear();
      return false;
    }
    packet_info_.receiveParticalPacket(packet+sizeof(RSEM4MsopHeader2), size- sizeof(RSEM4MsopHeader2),pkt_seq);
    return this->decodeCompPkt(packet_info_.received_packet, packet_info_.received_packet_len);
  }
  const RSEM4MsopHeader& header = *(RSEM4MsopHeader*)packet;
  uint8_t pack_mode = header.pack_mode & 0x03;
  if (pack_mode == 0x01 || pack_mode == 0x0)  // without compression
  {
    return this->decodeGeneralPkt(packet, size);
  }
  else if (pack_mode == 0x03)  // with compression
  {
    uint8_t split_pack_num = header.pack_mode >> 4;
    if(split_pack_num == 0x0){
      return this->decodeCompPkt(packet, size);
    }
    //split_pack_num == 0x1
    uint16_t pkt_seq = ntohs(header.pkt_seq);
    packet_info_.clear();
    packet_info_.receiveParticalPacket(packet, size,pkt_seq);
  }
  return false;
}

template <typename T_PointCloud>
inline bool DecoderRSEM4<T_PointCloud>::decodeCompPkt(const uint8_t* packet, size_t size)
{
  if (size < sizeof(RSEM4MsopHeader))
  {
    return false;
  }
  bool ret = false;
  const RSEM4MsopHeader& header = *(RSEM4MsopHeader*)packet;

  double pkt_ts = 0;
  if (this->param_.use_lidar_clock)
  {
    pkt_ts = parseTimeUTCWithUs(&header.timestamp) * 1e-6;
  }
  else
  {
    uint64_t ts = getTimeHost();

    // roll back to first block to approach lidar ts as near as possible.
    pkt_ts = getTimeHost() * 1e-6 - this->getPacketDuration();

    if (this->write_pkt_ts_)
    {
      createTimeUTCWithUs(ts, (RSTimestampUTC*)&header.timestamp);
    }
  }
  uint16_t pkt_seq = ntohs(header.pkt_seq);
  this->temperature_ = static_cast<float>((int)header.temperature - this->const_param_.TEMPERATURE_RES);

  if (split_strategy_.newPacket(pkt_seq))
  {
    this->cb_split_frame_(this->const_param_.LASER_NUM, this->cloudTs());
    this->first_point_ts_ = pkt_ts;
    ret = true;
  }
#ifdef BIG
  uint8_t* udp_payload_input = (uint8_t*)packet;
  packet_info_.getPackedData(udp_payload_input + sizeof(RSEM4MsopHeader), size - sizeof(RSEM4MsopHeader) - MSOP_TAIL_LEN);
  if (packet_info_.data_length == 0)
  {
   RS_WARNING << "RSEM4 decodeCompPkt: invalid data length: " << packet_info_.data_length << RS_REND;
   packet_info_.clear();
   return false;
  }
#endif
  rleDecodeMethod(packet, size, pkt_ts);
  packet_info_.clear();
  return ret;
}

template <typename T_PointCloud>
inline void DecoderRSEM4<T_PointCloud>::rleDecodeMethod(const uint8_t* udp_payload, size_t size, const double pkt_ts)
{
  constexpr uint8_t BYTE_MASK = 0xFF;
  //constexpr uint8_t FEATURE_SHIFT = 7;
  const auto& header = *reinterpret_cast<const RSEM4MsopHeader*>(udp_payload);

  int16_t yaw_angle = ntohs(header.yaw_angle);
  uint16_t depressed_data[EM4_PIXELS_PER_COLUMN*2] = { 0 };
  uint16_t* radius_decode_array = depressed_data;
  uint16_t* identity_decode_array = depressed_data + EM4_PIXELS_PER_COLUMN;

  // Convert surface_index from 1-based to 0-based index.
  uint8_t surface_index = header.surface_id - 1;
  if (surface_index >= EM4_SURFACE_NUM)
  {
    RS_WARNING << "Invalid surface index: " << (uint16_t)surface_index << ", set as 0" << RS_REND;
    surface_index = 0;
  }
#ifdef BIG
  CompressAlgo::RLenc_unpack_optimize(packet_info_.packed_data_endian, depressed_data, packet_info_.data_length/2,16);
#else
  int DATA_LEN = (size - sizeof(RSEM4MsopHeader) - MSOP_TAIL_LEN) / 2;
  CompressAlgo::RLenc_unpack_optimize(reinterpret_cast<const uint16_t*>(udp_payload+sizeof(RSEM4MsopHeader)), depressed_data, DATA_LEN,16);
#endif
  const int surface_pitch_offset = this->surface_pitch_offset_[surface_index];
  const uint16_t points_in_this_packet = this->const_param_.BLOCKS_PER_PKT * this->const_param_.CHANNELS_PER_BLOCK;
  this->point_cloud_->points.reserve(points_in_this_packet);
  // uint16_t wave_num = 1;
  // uint16_t pkt_seq = ntohs(header.pkt_seq)-1;
  if(header.return_mode == 0x0) // dual mode
  {
    // wave_num = pkt_seq %2 + 1; // 1 - first wave, 2 - second wave
  }
  for (int i = 0; i < EM4_PIXELS_PER_COLUMN; i++)
  {
    const int real_chan = i;
    const float distance = radius_decode_array[i] * this->const_param_.DISTANCE_RES;
    uint8_t intensity = (uint8_t)(identity_decode_array[i] & BYTE_MASK);
    const uint8_t feature = intensity & 0x01;  //only frost flag
    if (this->distance_section_.in(distance))
    {
      const int vecsel = real_chan / EM4_PIXELS_PER_VCSEL;
      const int yaw = yaw_angle + this->yaw_offset_[vecsel];
      const int pitch = this->pitch_angle_[real_chan] + surface_pitch_offset;
      float x = distance * COS(pitch) * COS(yaw);
      float y = distance * COS(pitch) * SIN(yaw);
      float z = distance * SIN(pitch);
      this->transformPoint(x, y, z);

      typename T_PointCloud::PointT point;

      setX(point, x);
      setY(point, y);
      setZ(point, z);
      setTimestamp(point, pkt_ts);
      setRing(point, real_chan);
      setIntensity(point, intensity);
      setFeature(point, feature);
      this->point_cloud_->points.emplace_back(point);
    }
    else if (!this->param_.dense_points)
    {
      typename T_PointCloud::PointT point;
      setX(point, NAN);
      setY(point, NAN);
      setZ(point, NAN);
      setIntensity(point, 0);
      setTimestamp(point, pkt_ts);
      setRing(point, real_chan);
      setFeature(point, feature);
      this->point_cloud_->points.emplace_back(point);
    }
    this->prev_point_ts_ = pkt_ts;
  }
  this->prev_pkt_ts_ = pkt_ts;
}

template <typename T_PointCloud>
inline bool DecoderRSEM4<T_PointCloud>::decodeGeneralPkt(const uint8_t* packet, size_t size)
{
  if (size < sizeof(RSEM4MsopPkt))
  {
    RS_WARNING << "decodeGeneralPkt: packet size < sizeof(RSEM4MsopPkt)" << RS_REND;
    return false;
  }
  const RSEM4MsopPkt& pkt = *(RSEM4MsopPkt*)packet;
  bool ret = false;
  this->temperature_ = static_cast<float>((int)pkt.header.temperature - this->const_param_.TEMPERATURE_RES);

  double pkt_ts = 0;
  if (this->param_.use_lidar_clock)
  {
    pkt_ts = parseTimeUTCWithUs(&pkt.header.timestamp) * 1e-6;
  }
  else
  {
    uint64_t ts = getTimeHost();

    // roll back to first block to approach lidar ts as near as possible.
    pkt_ts = getTimeHost() * 1e-6 - this->getPacketDuration();

    if (this->write_pkt_ts_)
    {
      createTimeUTCWithUs(ts, (RSTimestampUTC*)&pkt.header.timestamp);
    }
  }

  // Convert pkt_seq from 1-based to 0-based index.
  uint16_t pkt_seq = ntohs(pkt.header.pkt_seq) - 1;
  uint16_t return_mode = ntohs(pkt.header.return_mode);
  if (return_mode == 0x0) // dual mode
  {
    this->echo_mode_ = RSEchoMode::ECHO_DUAL;
  }else{
    this->echo_mode_ = RSEchoMode::ECHO_SINGLE;
  }
  if (split_strategy_.newPacket(pkt_seq))
  {
    this->cb_split_frame_(this->const_param_.LASER_NUM, this->cloudTs());
    this->first_point_ts_ = pkt_ts;
    ret = true;
  }

  constexpr uint16_t PIX_PER_COL_HALF = EM4_PIXELS_PER_COLUMN / 2;
  const uint16_t blocks_per_pkt = this->const_param_.BLOCKS_PER_PKT;
  const uint16_t channels_per_block = this->const_param_.CHANNELS_PER_BLOCK;
  const float distance_res = this->const_param_.DISTANCE_RES;
  const bool dense_points = this->param_.dense_points;
  // Convert surface_index from 1-based to 0-based index.
  uint8_t surface_index = pkt.header.surface_id - 1;
  if (surface_index >= EM4_SURFACE_NUM)
  {
    RS_WARNING << "Invalid surface index: " << (uint16_t)surface_index << ", set as 0" << RS_REND;
    surface_index = 0;
  }
  const int16_t yaw_base = RS_SWAP_INT16(pkt.header.yaw_angle);
  const int surface_pitch_offset = this->surface_pitch_offset_[surface_index];
  const uint16_t seq_mod = this->echo_mode_==RSEchoMode::ECHO_DUAL ? (pkt_seq % 4) : (pkt_seq % 2);
  // uint16_t wave_num = 1;
  for (uint16_t blk = 0; blk < blocks_per_pkt; ++blk)
  {
    const auto& block = pkt.blocks[blk];
    const double point_time = pkt_ts;
    uint16_t real_chan;
    if (this->echo_mode_==RSEchoMode::ECHO_DUAL)
    {
      if (blk % 2 == 0){
        // wave_num = 1;
      }
      else{
        // wave_num = 2;
      }
      real_chan = (uint16_t)(blk / 2) + seq_mod * (PIX_PER_COL_HALF / 2);
    } 
    else
    {
      real_chan = static_cast<uint16_t>(blk + seq_mod * PIX_PER_COL_HALF);
    }
    const int vecsel = real_chan / EM4_PIXELS_PER_VCSEL;
    const int yaw = yaw_base + this->yaw_offset_[vecsel];
    const int pitch = this->pitch_angle_[real_chan] + surface_pitch_offset;

    const float cos_pitch = COS(pitch);
    const float sin_pitch = SIN(pitch);
    const float cos_yaw = COS(yaw);
    const float sin_yaw = SIN(yaw);

    for (uint16_t chan = 0; chan < channels_per_block; ++chan)
    {
      const auto& channel = block.channel[chan];
      const float distance = ntohs(channel.distance) * distance_res;
      //const uint8_t feature = (channel.point_attribute >> 3) & 0x01;
      const uint8_t feature = channel.point_attribute;

      typename T_PointCloud::PointT point;
      if (this->distance_section_.in(distance))
      {
        float x = distance * cos_pitch * cos_yaw;
        float y = distance * cos_pitch * sin_yaw;
        float z = distance * sin_pitch;
        this->transformPoint(x, y, z);

        setX(point, x);
        setY(point, y);
        setZ(point, z);
        setIntensity(point, channel.intensity);
        setTimestamp(point, point_time);
        setRing(point, real_chan);
        setFeature(point, feature);
        this->point_cloud_->points.emplace_back(std::move(point));
      }
      else if (!dense_points)
      {
        setX(point, NAN);
        setY(point, NAN);
        setZ(point, NAN);
        setIntensity(point, 0);
        setTimestamp(point, point_time);
        setRing(point, real_chan);
        setFeature(point, feature);
        this->point_cloud_->points.emplace_back(std::move(point));
      }
    }
    this->prev_point_ts_ = pkt_ts;
  }
  this->prev_pkt_ts_ = pkt_ts;
  return ret;
}

}  // namespace lidar
}  // namespace robosense
