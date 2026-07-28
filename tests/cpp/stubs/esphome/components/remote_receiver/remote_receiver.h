#pragma once

#include "esphome/components/remote_base/remote_base.h"
#include "esphome/core/component.h"

#include <vector>

namespace esphome {
namespace remote_receiver {

class RemoteReceiverComponent : public Component {
 public:
  void register_listener(remote_base::RemoteReceiverListener *listener) {
    this->listeners.push_back(listener);
  }

  std::vector<remote_base::RemoteReceiverListener *> listeners;
};

}  // namespace remote_receiver
}  // namespace esphome
