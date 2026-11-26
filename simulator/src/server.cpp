#include "server.h"

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <signal.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#include <fstream>
#include <sstream>

#include "sim.h"

namespace sim {

namespace {

const size_t maximumBacklog = 2 * 1024 * 1024;  // Skip display updates for clients this far behind

void setNonBlocking(int socket) { fcntl(socket, F_SETFL, fcntl(socket, F_GETFL, 0) | O_NONBLOCK); }

std::string contentType(const std::string &path) {
  const size_t dot = path.rfind('.');
  const std::string extension = dot == std::string::npos ? "" : path.substr(dot + 1);
  if (extension == "html") return "text/html; charset=utf-8";
  if (extension == "js") return "text/javascript; charset=utf-8";
  if (extension == "css") return "text/css; charset=utf-8";
  if (extension == "svg") return "image/svg+xml";
  if (extension == "png") return "image/png";
  if (extension == "json") return "application/json";
  if (extension == "brd" || extension == "sch") return "application/xml";
  return "application/octet-stream";
}

std::string headerValue(const std::string &request, const std::string &name) {
  std::string lower = request;
  for (auto &c : lower) c = tolower((unsigned char)c);
  std::string key = "\r\n" + name + ":";
  for (auto &c : key) c = tolower((unsigned char)c);
  const size_t position = lower.find(key);
  if (position == std::string::npos) return "";
  size_t start = position + key.size();
  const size_t end = request.find("\r\n", start);
  while (start < end && request[start] == ' ') ++start;
  return request.substr(start, end - start);
}

}  // namespace

// SHA-1 and Base64 for the WebSocket handshake

std::string sha1(const std::string &data) {
  uint32_t h[5] = {0x67452301, 0xEFCDAB89, 0x98BADCFE, 0x10325476, 0xC3D2E1F0};
  std::string message = data;
  const uint64_t bitLength = (uint64_t)data.size() * 8;
  message += (char)0x80;
  while (message.size() % 64 != 56) message += (char)0;
  for (int i = 7; i >= 0; --i) message += (char)((bitLength >> (i * 8)) & 0xFF);
  auto rotate = [](uint32_t value, int bits) { return (value << bits) | (value >> (32 - bits)); };
  for (size_t chunk = 0; chunk < message.size(); chunk += 64) {
    uint32_t w[80];
    for (int i = 0; i < 16; ++i) {
      w[i] = ((uint8_t)message[chunk + i * 4] << 24) | ((uint8_t)message[chunk + i * 4 + 1] << 16)
             | ((uint8_t)message[chunk + i * 4 + 2] << 8) | (uint8_t)message[chunk + i * 4 + 3];
    }
    for (int i = 16; i < 80; ++i) w[i] = rotate(w[i - 3] ^ w[i - 8] ^ w[i - 14] ^ w[i - 16], 1);
    uint32_t a = h[0], b = h[1], c = h[2], d = h[3], e = h[4];
    for (int i = 0; i < 80; ++i) {
      uint32_t f, k;
      if (i < 20) {
        f = (b & c) | (~b & d);
        k = 0x5A827999;
      } else if (i < 40) {
        f = b ^ c ^ d;
        k = 0x6ED9EBA1;
      } else if (i < 60) {
        f = (b & c) | (b & d) | (c & d);
        k = 0x8F1BBCDC;
      } else {
        f = b ^ c ^ d;
        k = 0xCA62C1D6;
      }
      const uint32_t temp = rotate(a, 5) + f + e + k + w[i];
      e = d;
      d = c;
      c = rotate(b, 30);
      b = a;
      a = temp;
    }
    h[0] += a;
    h[1] += b;
    h[2] += c;
    h[3] += d;
    h[4] += e;
  }
  std::string digest;
  for (uint32_t value : h) {
    for (int i = 3; i >= 0; --i) digest += (char)((value >> (i * 8)) & 0xFF);
  }
  return digest;
}

std::string base64(const std::string &data) {
  static const char alphabet[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string out;
  size_t i = 0;
  while (i + 2 < data.size()) {
    const uint32_t n = ((uint8_t)data[i] << 16) | ((uint8_t)data[i + 1] << 8) | (uint8_t)data[i + 2];
    out += alphabet[(n >> 18) & 63];
    out += alphabet[(n >> 12) & 63];
    out += alphabet[(n >> 6) & 63];
    out += alphabet[n & 63];
    i += 3;
  }
  if (i + 1 == data.size()) {
    const uint32_t n = (uint8_t)data[i] << 16;
    out += alphabet[(n >> 18) & 63];
    out += alphabet[(n >> 12) & 63];
    out += "==";
  } else if (i + 2 == data.size()) {
    const uint32_t n = ((uint8_t)data[i] << 16) | ((uint8_t)data[i + 1] << 8);
    out += alphabet[(n >> 18) & 63];
    out += alphabet[(n >> 12) & 63];
    out += alphabet[(n >> 6) & 63];
    out += '=';
  }
  return out;
}

// SERVER

bool Server::start(int port, const std::vector<Mount> &newMounts) {
  mounts = newMounts;
  signal(SIGPIPE, SIG_IGN);
  listener = socket(AF_INET, SOCK_STREAM, 0);
  if (listener < 0) return false;
  int yes = 1;
  setsockopt(listener, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
  fcntl(listener, F_SETFD, FD_CLOEXEC);
  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_port = htons(port);
  address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  if (bind(listener, (sockaddr *)&address, sizeof(address)) < 0 || listen(listener, 16) < 0) {
    close(listener);
    listener = -1;
    return false;
  }
  setNonBlocking(listener);
  return true;
}

void Server::run() {
  for (;;) {
    std::vector<pollfd> descriptors;
    descriptors.push_back({listener, POLLIN, 0});
    for (auto &client : clients) {
      short events = POLLIN;
      if (!client.output.empty()) events |= POLLOUT;
      descriptors.push_back({client.socket, events, 0});
    }
    poll(descriptors.data(), descriptors.size(), 15);

    // Only the clients that were polled; new connections are accepted after them
    const size_t polled = clients.size();
    std::vector<bool> keep(polled, true);
    for (size_t i = 0; i < polled; ++i) {
      const short revents = descriptors[i + 1].revents;
      if (revents & (POLLERR | POLLHUP | POLLNVAL)) {
        keep[i] = false;
        continue;
      }
      if ((revents & POLLIN) && !readFrom(clients[i])) {
        keep[i] = false;
        continue;
      }
      if (clients[i].webSocket) sendUpdates(clients[i]);
      if (!clients[i].output.empty() && !writeTo(clients[i])) keep[i] = false;
      if (clients[i].output.empty() && clients[i].closeAfterWrite) keep[i] = false;
    }
    std::vector<Client> remaining;
    for (size_t i = 0; i < clients.size(); ++i) {
      if (i >= polled || keep[i]) {
        remaining.push_back(std::move(clients[i]));
      } else {
        close(clients[i].socket);
      }
    }
    clients.swap(remaining);
    if (descriptors[0].revents & POLLIN) accept();
  }
}

void Server::accept() {
  for (;;) {
    const int socket = ::accept(listener, nullptr, nullptr);
    if (socket < 0) return;
    setNonBlocking(socket);
    fcntl(socket, F_SETFD, FD_CLOEXEC);
    int yes = 1;
    setsockopt(socket, IPPROTO_TCP, TCP_NODELAY, &yes, sizeof(yes));
#ifdef SO_NOSIGPIPE
    setsockopt(socket, SOL_SOCKET, SO_NOSIGPIPE, &yes, sizeof(yes));
#endif
    Client client;
    client.socket = socket;
    clients.push_back(client);
  }
}

bool Server::readFrom(Client &client) {
  char buffer[16384];
  for (;;) {
    const ssize_t n = recv(client.socket, buffer, sizeof(buffer), 0);
    if (n > 0) {
      client.input.append(buffer, n);
      continue;
    }
    if (n == 0) return false;
    if (errno == EAGAIN || errno == EWOULDBLOCK) break;
    if (errno == EINTR) continue;
    return false;
  }
  if (client.webSocket) {
    handleWebSocketFrames(client);
  } else if (client.input.find("\r\n\r\n") != std::string::npos) {
    handleHttp(client);
  } else if (client.input.size() > 65536) {
    return false;
  }
  return true;
}

bool Server::writeTo(Client &client) {
  while (!client.output.empty()) {
    const ssize_t n = send(client.socket, client.output.data(), client.output.size(), 0);
    if (n > 0) {
      client.output.erase(0, n);
      continue;
    }
    if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) return true;
    if (n < 0 && errno == EINTR) continue;
    return false;
  }
  return true;
}

void Server::respond(Client &client, int status, const std::string &statusText, const std::string &type,
                     const std::string &body) {
  std::ostringstream response;
  response << "HTTP/1.1 " << status << " " << statusText << "\r\n"
           << "Content-Type: " << type << "\r\n"
           << "Content-Length: " << body.size() << "\r\n"
           << "Cache-Control: no-cache\r\n"
           << "Connection: close\r\n\r\n";
  client.output += response.str();
  client.output += body;
  client.closeAfterWrite = true;
}

void Server::handleHttp(Client &client) {
  const size_t end = client.input.find("\r\n\r\n");
  const std::string request = client.input.substr(0, end + 2);
  client.input.erase(0, end + 4);
  std::istringstream line(request);
  std::string method, target;
  line >> method >> target;
  const size_t query = target.find('?');
  const std::string path = query == std::string::npos ? target : target.substr(0, query);

  if (path == "/ws") {
    const std::string key = headerValue(request, "Sec-WebSocket-Key");
    if (key.empty()) {
      respond(client, 400, "Bad Request", "text/plain", "WebSocket key missing\n");
      return;
    }
    const std::string accept = base64(sha1(key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"));
    client.output += "HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n"
                     "Sec-WebSocket-Accept: " + accept + "\r\n\r\n";
    client.webSocket = true;
    sendFrame(client, 1, simulator().helloText());
    return;
  }
  if (method != "GET") {
    respond(client, 405, "Method Not Allowed", "text/plain", "Only GET is supported\n");
    return;
  }
  serveFile(client, path == "/" ? "/index.html" : path);
}

void Server::serveFile(Client &client, const std::string &path) {
  if (path.find("..") != std::string::npos) {
    respond(client, 403, "Forbidden", "text/plain", "Forbidden\n");
    return;
  }
  for (const auto &mount : mounts) {
    if (path.compare(0, mount.urlPrefix.size(), mount.urlPrefix) != 0) continue;
    const std::string file = mount.directory + "/" + path.substr(mount.urlPrefix.size());
    std::ifstream input(file, std::ios::binary);
    if (!input) continue;
    std::stringstream contents;
    contents << input.rdbuf();
    respond(client, 200, "OK", contentType(file), contents.str());
    return;
  }
  respond(client, 404, "Not Found", "text/plain", "Not found: " + path + "\n");
}

void Server::sendFrame(Client &client, int opcode, const std::string &payload) {
  std::string frame;
  frame += (char)(0x80 | opcode);
  const size_t length = payload.size();
  if (length < 126) {
    frame += (char)length;
  } else if (length < 65536) {
    frame += (char)126;
    frame += (char)((length >> 8) & 0xFF);
    frame += (char)(length & 0xFF);
  } else {
    frame += (char)127;
    for (int i = 7; i >= 0; --i) frame += (char)(((uint64_t)length >> (i * 8)) & 0xFF);
  }
  client.output += frame;
  client.output += payload;
}

void Server::handleWebSocketFrames(Client &client) {
  for (;;) {
    std::string &in = client.input;
    if (in.size() < 2) return;
    const uint8_t first = in[0], second = in[1];
    const bool final = first & 0x80;
    const int opcode = first & 0x0F;
    const bool masked = second & 0x80;
    uint64_t length = second & 0x7F;
    size_t offset = 2;
    if (length == 126) {
      if (in.size() < 4) return;
      length = ((uint8_t)in[2] << 8) | (uint8_t)in[3];
      offset = 4;
    } else if (length == 127) {
      if (in.size() < 10) return;
      length = 0;
      for (int i = 0; i < 8; ++i) length = (length << 8) | (uint8_t)in[2 + i];
      offset = 10;
    }
    if (length > 16 * 1024 * 1024) {
      client.closeAfterWrite = true;
      in.clear();
      return;
    }
    const size_t maskOffset = offset;
    if (masked) offset += 4;
    if (in.size() < offset + length) return;
    std::string payload = in.substr(offset, length);
    if (masked) {
      for (size_t i = 0; i < payload.size(); ++i) payload[i] ^= in[maskOffset + (i & 3)];
    }
    in.erase(0, offset + length);

    if (opcode == 8) {  // Close
      sendFrame(client, 8, "");
      client.closeAfterWrite = true;
      return;
    }
    if (opcode == 9) {  // Ping
      sendFrame(client, 10, payload);
      continue;
    }
    if (opcode == 10) continue;
    client.fragments += payload;
    if (!final) continue;
    std::string message;
    message.swap(client.fragments);
    Json json;
    if (Json::parse(message, json) && onMessage) onMessage(json);
  }
}

void Server::sendUpdates(Client &client) {
  Simulator &s = simulator();
  const std::string serial = s.serialSince(client.serialOffset);
  if (!serial.empty()) {
    std::string message = "{\"type\":\"serial\",\"text\":";
    Json::escape(serial, message);
    message += "}";
    sendFrame(client, 1, message);
  }
  std::string state;
  std::string panel;
  {
    std::lock_guard<std::mutex> lock(s.publishMutex);
    if (s.published.sequence != client.stateSequence && client.output.size() < maximumBacklog) {
      client.stateSequence = s.published.sequence;
      state = s.published.state;
    }
    if (s.published.panelSequence != client.panelSequence && client.output.size() < maximumBacklog) {
      client.panelSequence = s.published.panelSequence;
      panel.reserve(2 + s.published.panel.size() * 2);
      panel += (char)1;  // Message type: display contents
      panel += (char)(s.published.backlight ? 1 : 0);
      for (uint16_t pixel : s.published.panel) {
        panel += (char)(pixel & 0xFF);
        panel += (char)(pixel >> 8);
      }
    }
  }
  if (!state.empty()) sendFrame(client, 1, state);
  if (!panel.empty()) sendFrame(client, 2, panel);
}

}  // namespace sim
