#pragma once

#include "esphome/components/remote_base/remote_base.h"
#include "esphome/core/component.h"

#include <cstdint>

namespace esphome {
namespace remote_transmitter {

class RemoteTransmitterComponent : public Component {
 public:
  class TransmitCall {
   public:
    explicit TransmitCall(RemoteTransmitterComponent *parent) : parent_(parent) {}
    remote_base::RemoteTransmitData *get_data() { return &this->parent_->last_data; }
    void set_send_times(uint32_t times) { this->parent_->send_times = times; }
    void set_send_wait(uint32_t wait) { this->parent_->send_wait = wait; }
    void perform() { this->parent_->transmit_count++; }

   protected:
    RemoteTransmitterComponent *parent_;
  };

  TransmitCall transmit() { return TransmitCall(this); }

  remote_base::RemoteTransmitData last_data;
  uint32_t send_times{0};
  uint32_t send_wait{0};
  int transmit_count{0};
};

}  // namespace remote_transmitter
}  // namespace esphome
