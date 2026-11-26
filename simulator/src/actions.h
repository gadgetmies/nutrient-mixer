// User actions from the UI (WebSocket messages) and scenario scripts, as JSON messages such as
// {"type": "button", "button": "encoder", "action": "click"}. Run on the firmware thread.
#pragma once

#include <string>

#include "json.h"

namespace sim {

// Returns an error message, or an empty string when the action was done
std::string performAction(const Json &message);

// Called on the server thread: speed and pause take effect immediately, everything else is passed to the
// firmware thread
void handleUiMessage(const Json &message);

}  // namespace sim
