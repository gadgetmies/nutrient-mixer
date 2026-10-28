// Small HTTP server for the UI: static files plus a WebSocket (/ws) that streams the simulator state and the
// display contents and receives the user's inputs.
#pragma once

#include <stdint.h>

#include <functional>
#include <string>
#include <vector>

#include "json.h"

namespace sim {

class Server {
 public:
  struct Mount {
    std::string urlPrefix;  // e.g. "/eagle/"
    std::string directory;
  };

  bool start(int port, const std::vector<Mount> &mounts);
  void run();  // Serves forever (call on its own thread)
  std::function<void(const Json &message)> onMessage;

 private:
  struct Client {
    int socket;
    std::string input;
    std::string output;
    bool webSocket = false;
    bool closeAfterWrite = false;
    uint64_t stateSequence = 0;
    uint64_t panelSequence = 0;
    uint64_t serialOffset = 0;
    std::string fragments;  // Partial WebSocket message
  };

  void accept();
  bool readFrom(Client &client);
  bool writeTo(Client &client);
  void handleHttp(Client &client);
  void handleWebSocketFrames(Client &client);
  void sendFrame(Client &client, int opcode, const std::string &payload);
  void sendUpdates(Client &client);
  void serveFile(Client &client, const std::string &path);
  void respond(Client &client, int status, const std::string &statusText, const std::string &contentType,
               const std::string &body);

  int listener = -1;
  std::vector<Mount> mounts;
  std::vector<Client> clients;
};

std::string sha1(const std::string &data);
std::string base64(const std::string &data);

}  // namespace sim
