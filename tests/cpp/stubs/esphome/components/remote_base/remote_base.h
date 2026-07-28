#pragma once

#include <cstdint>
#include <vector>

namespace esphome {
namespace remote_base {

using RawTimings = std::vector<int32_t>;

class RemoteTransmitData {
 public:
  void set_data(const RawTimings &data) { this->data_ = data; }
  const RawTimings &get_data() const { return this->data_; }

 protected:
  RawTimings data_;
};

class RemoteReceiveData {
 public:
  explicit RemoteReceiveData(const RawTimings &data) : data_(data) {}
  const RawTimings &get_raw_data() const { return this->data_; }

 protected:
  RawTimings data_;
};

class RemoteReceiverListener {
 public:
  virtual ~RemoteReceiverListener() = default;
  virtual bool on_receive(RemoteReceiveData data) = 0;
};

}  // namespace remote_base
}  // namespace esphome
