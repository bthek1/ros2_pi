#include "pimesh_dashboard/web_server.hpp"

#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <fstream>
#include <sstream>
#include <utility>

#include "pimesh_dashboard/websocket.hpp"

namespace pimesh_dashboard
{
namespace
{

/// Lower-case a header name for comparison. HTTP header names are
/// case-insensitive and browsers do not agree on the case they send
/// `Sec-WebSocket-Key` in.
std::string lower(std::string text)
{
  std::transform(
    text.begin(), text.end(), text.begin(),
    [](unsigned char c) {return static_cast<char>(::tolower(c));});
  return text;
}

std::string trim(const std::string & text)
{
  const auto begin = text.find_first_not_of(" \t\r\n");
  if (begin == std::string::npos) {return "";}
  const auto end = text.find_last_not_of(" \t\r\n");
  return text.substr(begin, end - begin + 1);
}

}  // namespace

std::string content_type_for(const std::string & path)
{
  const auto dot = path.find_last_of('.');
  const std::string ext = (dot == std::string::npos) ? "" : lower(path.substr(dot));
  if (ext == ".html") {return "text/html; charset=utf-8";}
  if (ext == ".css") {return "text/css; charset=utf-8";}
  if (ext == ".js") {return "application/javascript; charset=utf-8";}
  if (ext == ".json") {return "application/json";}
  if (ext == ".png") {return "image/png";}
  if (ext == ".jpg" || ext == ".jpeg") {return "image/jpeg";}
  if (ext == ".svg") {return "image/svg+xml";}
  if (ext == ".ico") {return "image/x-icon";}
  // Unknown types download rather than run. A dashboard's asset directory should
  // contain nothing surprising, and if it does, the browser should not execute it.
  return "application/octet-stream";
}

std::string resolve_path(const std::string & root, const std::string & request_target)
{
  if (root.empty()) {return "";}

  // The target may carry a query string and a fragment; neither is part of the
  // path. Stripping them here rather than in the caller keeps every path that
  // reaches the filesystem through this one function.
  std::string target = request_target;
  const auto cut = target.find_first_of("?#");
  if (cut != std::string::npos) {target = target.substr(0, cut);}

  if (target.empty() || target[0] != '/') {return "";}
  if (target.find('\0') != std::string::npos) {return "";}
  if (target == "/") {target = "/index.html";}

  // Resolve the segments by hand rather than trusting the filesystem. A
  // realpath() check on the result would be correct too, but it answers the
  // question *after* a path has been built out of untrusted text, and it cannot
  // run at all for a file that does not exist — which is exactly the case a
  // probing request produces.
  std::vector<std::string> parts;
  std::stringstream stream(target.substr(1));
  std::string segment;
  while (std::getline(stream, segment, '/')) {
    if (segment.empty() || segment == ".") {continue;}
    if (segment == "..") {
      // **Refused, not popped.** Popping would silently serve a different file
      // than the one asked for, which makes the log and the behaviour disagree;
      // and a request containing `..` is never a request this dashboard's own
      // page makes.
      return "";
    }
    parts.push_back(segment);
  }
  if (parts.empty()) {return "";}

  std::string path = root;
  for (const std::string & part : parts) {path += "/" + part;}
  return path;
}

WebServer::WebServer(const Config & config)
: config_(config) {}

WebServer::~WebServer()
{
  stop();
}

bool WebServer::start()
{
  listen_fd_ = ::socket(AF_INET, SOCK_STREAM | SOCK_CLOEXEC, 0);
  if (listen_fd_ < 0) {
    error_ = std::string("socket: ") + std::strerror(errno);
    return false;
  }

  int yes = 1;
  ::setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

  sockaddr_in address {};
  address.sin_family = AF_INET;
  // INADDR_ANY: the point of this page is that a phone on the same network can
  // watch it. It serves a read-only view of a robot's own telemetry on a LAN; it
  // is not on the internet and it holds no credentials.
  address.sin_addr.s_addr = htonl(INADDR_ANY);
  address.sin_port = htons(static_cast<std::uint16_t>(config_.port));
  if (::bind(listen_fd_, reinterpret_cast<sockaddr *>(&address), sizeof(address)) != 0) {
    error_ = "bind port " + std::to_string(config_.port) + ": " + std::strerror(errno);
    ::close(listen_fd_);
    listen_fd_ = -1;
    return false;
  }
  if (::listen(listen_fd_, 16) != 0) {
    error_ = std::string("listen: ") + std::strerror(errno);
    ::close(listen_fd_);
    listen_fd_ = -1;
    return false;
  }
  ::fcntl(listen_fd_, F_SETFL, O_NONBLOCK);

  if (::pipe(wake_fds_) != 0) {
    error_ = std::string("pipe: ") + std::strerror(errno);
    ::close(listen_fd_);
    listen_fd_ = -1;
    return false;
  }
  ::fcntl(wake_fds_[0], F_SETFL, O_NONBLOCK);

  running_ = true;
  thread_ = std::thread([this] {this->serve();});
  return true;
}

void WebServer::stop()
{
  if (!running_.exchange(false)) {return;}
  if (wake_fds_[1] >= 0) {
    const char byte = 'x';
    // The return is deliberately ignored: this is a wake-up, and the only reason
    // it can fail is that the reader has already gone, which is the state being
    // asked for.
    [[maybe_unused]] const auto ignored = ::write(wake_fds_[1], &byte, 1);
  }
  if (thread_.joinable()) {thread_.join();}

  std::lock_guard<std::mutex> lock(mutex_);
  for (Client & client : clients_list_) {
    if (client.fd >= 0) {::close(client.fd);}
  }
  clients_list_.clear();
  clients_ = 0;
  for (int & fd : wake_fds_) {
    if (fd >= 0) {::close(fd);}
    fd = -1;
  }
  if (listen_fd_ >= 0) {
    ::close(listen_fd_);
    listen_fd_ = -1;
  }
}

void WebServer::broadcast(Channel channel, const std::uint8_t * data, std::size_t size)
{
  // Framed once, for everybody. A server frame is not masked, so the bytes are
  // identical per client and building them per client would be the same work N
  // times — which matters at 4 MB of mesh.
  std::vector<std::uint8_t> payload;
  payload.reserve(size + 1);
  payload.push_back(static_cast<std::uint8_t>(channel));
  payload.insert(payload.end(), data, data + size);
  const std::vector<std::uint8_t> frame =
    build_frame(Opcode::Binary, payload.data(), payload.size());

  std::lock_guard<std::mutex> lock(mutex_);
  bool any = false;
  for (Client & client : clients_list_) {
    if (!client.upgraded) {continue;}
    // **The pacing rule, and it is the whole point of this class.** A browser
    // that has been backgrounded stops reading; its socket buffer fills; and
    // without this the queue here would grow until the process died, with the
    // pipeline slowing down first. Dropping is the correct answer for every
    // channel this server has — each one is a *latest value*, not a stream where
    // a missing frame means anything.
    if (client.outgoing.size() + frame.size() > config_.send_limit_bytes) {
      dropped_.fetch_add(1, std::memory_order_relaxed);
      continue;
    }
    client.outgoing.insert(client.outgoing.end(), frame.begin(), frame.end());
    any = true;
  }
  if (any) {
    sent_.fetch_add(1, std::memory_order_relaxed);
    if (wake_fds_[1] >= 0) {
      const char byte = 'x';
      [[maybe_unused]] const auto ignored = ::write(wake_fds_[1], &byte, 1);
    }
  }
}

void WebServer::broadcast(Channel channel, const std::string & text)
{
  broadcast(channel, reinterpret_cast<const std::uint8_t *>(text.data()), text.size());
}

void WebServer::accept_one()
{
  const int fd = ::accept(listen_fd_, nullptr, nullptr);
  if (fd < 0) {return;}
  ::fcntl(fd, F_SETFL, O_NONBLOCK);
  // Nagle off: every message here is a whole frame that someone is waiting to
  // draw, so coalescing small writes trades latency for bandwidth nobody needs.
  int yes = 1;
  ::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &yes, sizeof(yes));

  std::lock_guard<std::mutex> lock(mutex_);
  if (clients_list_.size() >= config_.max_clients) {
    ::close(fd);
    return;
  }
  Client client;
  client.fd = fd;
  clients_list_.push_back(std::move(client));
  clients_ = clients_list_.size();
}

bool WebServer::handle_http(Client & client)
{
  // A request ends at the blank line. Anything before that is a partial read,
  // which on a stream socket is ordinary rather than an error.
  const std::string text(client.incoming.begin(), client.incoming.end());
  const auto end = text.find("\r\n\r\n");
  if (end == std::string::npos) {
    // A header block this large is not a browser.
    return client.incoming.size() <= 16384;
  }

  std::istringstream stream(text.substr(0, end));
  std::string line;
  std::getline(stream, line);
  std::istringstream request_line(line);
  std::string method;
  std::string target;
  request_line >> method >> target;

  std::map<std::string, std::string> headers;
  while (std::getline(stream, line)) {
    const auto colon = line.find(':');
    if (colon == std::string::npos) {continue;}
    headers[lower(trim(line.substr(0, colon)))] = trim(line.substr(colon + 1));
  }
  client.incoming.erase(client.incoming.begin(), client.incoming.begin() +
    static_cast<long>(end + 4));

  const auto upgrade = headers.find("upgrade");
  const auto key = headers.find("sec-websocket-key");
  if (upgrade != headers.end() && lower(upgrade->second) == "websocket" &&
    key != headers.end())
  {
    const std::string response =
      "HTTP/1.1 101 Switching Protocols\r\n"
      "Upgrade: websocket\r\n"
      "Connection: Upgrade\r\n"
      "Sec-WebSocket-Accept: " + accept_key(key->second) + "\r\n\r\n";
    client.outgoing.insert(client.outgoing.end(), response.begin(), response.end());
    client.upgraded = true;
    return true;
  }

  // The one write path. POST only: a GET that changes state is a state change a
  // browser will make on its own, by prefetching a link.
  //
  // **Parked, not run.** This function is called with `mutex_` held, and the
  // handler waits on a ROS service — see run_pending_actions() for the deadlock
  // that taught this and for the quieter bug behind it.
  static const std::string kActionPrefix = "/action/";
  if (target.rfind(kActionPrefix, 0) == 0) {
    std::string name = target.substr(kActionPrefix.size());
    const auto cut = name.find_first_of("?#");
    if (cut != std::string::npos) {name = name.substr(0, cut);}
    client.pending_action = name;
    client.pending_was_post = (method == "POST");
    client.has_pending = true;
    return true;
  }

  if (method != "GET") {
    const std::string response = "HTTP/1.1 405 Method Not Allowed\r\nContent-Length: 0\r\n"
      "Connection: close\r\n\r\n";
    client.outgoing.insert(client.outgoing.end(), response.begin(), response.end());
    return false;
  }

  const std::string path = resolve_path(config_.root, target);
  std::vector<std::uint8_t> body;
  bool found = false;
  if (!path.empty()) {
    std::ifstream file(path, std::ios::binary);
    if (file) {
      body.assign(
        std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>());
      found = true;
    }
  }

  std::string head;
  if (found) {
    head = "HTTP/1.1 200 OK\r\nContent-Type: " + content_type_for(path) +
      "\r\nContent-Length: " + std::to_string(body.size()) +
      // No caching: the page is a live view and the assets change when the
      // package is rebuilt. A stale index.html against a new protocol is a blank
      // screen with nothing in the console.
      "\r\nCache-Control: no-store\r\nConnection: keep-alive\r\n\r\n";
  } else {
    const std::string message = "not found\n";
    body.assign(message.begin(), message.end());
    head = "HTTP/1.1 404 Not Found\r\nContent-Type: text/plain\r\nContent-Length: " +
      std::to_string(body.size()) + "\r\nConnection: keep-alive\r\n\r\n";
  }
  client.outgoing.insert(client.outgoing.end(), head.begin(), head.end());
  client.outgoing.insert(client.outgoing.end(), body.begin(), body.end());
  return true;
}

void WebServer::handle_frames(Client & client)
{
  // The browser sends very little: a pong, and a close. Anything else is read and
  // discarded — this is a read-only view, and the two buttons call ROS services
  // over HTTP rather than inventing a command channel here.
  for (;;) {
    Frame frame;
    std::size_t consumed = 0;
    const Parse status = parse_frame(client.incoming, frame, consumed);
    if (status == Parse::Incomplete) {return;}
    if (status == Parse::Invalid) {
      client.fd = -1;
      return;
    }
    client.incoming.erase(
      client.incoming.begin(), client.incoming.begin() + static_cast<long>(consumed));

    if (frame.opcode == Opcode::Close) {
      const auto close = build_frame(Opcode::Close, nullptr, 0);
      client.outgoing.insert(client.outgoing.end(), close.begin(), close.end());
      client.upgraded = false;
      return;
    }
    if (frame.opcode == Opcode::Ping) {
      const auto pong = build_frame(
        Opcode::Pong, frame.payload.data(), frame.payload.size());
      client.outgoing.insert(client.outgoing.end(), pong.begin(), pong.end());
    }
  }
}

void WebServer::flush(Client & client)
{
  while (!client.outgoing.empty()) {
    // The deque is not contiguous, so a chunk at a time. 64 kB is comfortably
    // more than a socket buffer will take in one go, which means the loop exits
    // on EAGAIN rather than on running out of chunk.
    const std::size_t chunk = std::min<std::size_t>(client.outgoing.size(), 65536);
    std::vector<std::uint8_t> buffer(
      client.outgoing.begin(), client.outgoing.begin() + static_cast<long>(chunk));
    const ssize_t written = ::send(client.fd, buffer.data(), buffer.size(), MSG_NOSIGNAL);
    if (written <= 0) {
      // EAGAIN is the ordinary state of a client that is not keeping up: leave
      // the rest queued and come back when poll says it is writable. Anything
      // else is a dead socket.
      if (written < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {return;}
      client.fd = -1;
      return;
    }
    client.outgoing.erase(
      client.outgoing.begin(), client.outgoing.begin() + static_cast<long>(written));
  }
}

void WebServer::run_pending_actions()
{
  // Three short critical sections instead of one long one: collect, run with no
  // lock at all, then queue the replies. A broadcast from a ROS callback waits
  // only for the first and the third, each of which is a few pointer moves.
  struct Pending
  {
    int fd;
    std::string name;
    bool was_post;
  };
  std::vector<Pending> todo;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (Client & client : clients_list_) {
      if (!client.has_pending) {continue;}
      todo.push_back(Pending{client.fd, client.pending_action, client.pending_was_post});
      client.has_pending = false;
      client.pending_action.clear();
    }
  }
  if (todo.empty()) {return;}

  std::function<std::string(const std::string &)> handler;
  {
    std::lock_guard<std::mutex> lock(action_mutex_);
    handler = action_;
  }

  for (const Pending & item : todo) {
    std::string body;
    if (!item.was_post) {
      body = "actions are POST only\n";
    } else {
      body = handler ? handler(item.name) : std::string("no action handler\n");
    }
    const std::string response =
      "HTTP/1.1 200 OK\r\nContent-Type: text/plain; charset=utf-8\r\nContent-Length: " +
      std::to_string(body.size()) + "\r\nCache-Control: no-store\r\n"
      "Connection: keep-alive\r\n\r\n" + body;

    // By fd, not by index: the client list may have moved while the handler ran,
    // and the socket may have gone entirely — a page reloaded during a ten-second
    // save is an ordinary thing to do.
    std::lock_guard<std::mutex> lock(mutex_);
    for (Client & client : clients_list_) {
      if (client.fd != item.fd) {continue;}
      client.outgoing.insert(client.outgoing.end(), response.begin(), response.end());
      break;
    }
  }
}

void WebServer::close_client(std::size_t index)
{
  if (clients_list_[index].fd >= 0) {::close(clients_list_[index].fd);}
  clients_list_.erase(clients_list_.begin() + static_cast<long>(index));
  clients_ = clients_list_.size();
}

void WebServer::serve()
{
  while (running_.load()) {
    std::vector<pollfd> fds;
    fds.push_back(pollfd{listen_fd_, POLLIN, 0});
    fds.push_back(pollfd{wake_fds_[0], POLLIN, 0});
    {
      std::lock_guard<std::mutex> lock(mutex_);
      for (const Client & client : clients_list_) {
        short events = POLLIN;
        if (!client.outgoing.empty()) {events = static_cast<short>(events | POLLOUT);}
        fds.push_back(pollfd{client.fd, events, 0});
      }
    }

    // 100 ms, which only ever expires when nothing at all is happening: a
    // broadcast writes the wake pipe, so a queued frame is sent within
    // microseconds rather than at the next tick.
    const int ready = ::poll(fds.data(), fds.size(), 100);
    if (ready < 0 && errno != EINTR) {break;}
    if (!running_.load()) {break;}

    if (fds[0].revents & POLLIN) {accept_one();}
    if (fds[1].revents & POLLIN) {
      char drain[256];
      while (::read(wake_fds_[0], drain, sizeof(drain)) > 0) {}
    }

    {
    std::lock_guard<std::mutex> lock(mutex_);
    // Backwards, so erasing does not move an index this loop has yet to reach.
    for (std::size_t i = clients_list_.size(); i-- > 0; ) {
      const std::size_t slot = i + 2;
      if (slot >= fds.size()) {continue;}
      Client & client = clients_list_[i];
      const short revents = fds[slot].revents;

      if (revents & (POLLHUP | POLLERR | POLLNVAL)) {
        close_client(i);
        continue;
      }
      if (revents & POLLIN) {
        std::uint8_t buffer[16384];
        const ssize_t got = ::recv(client.fd, buffer, sizeof(buffer), 0);
        if (got == 0) {
          close_client(i);
          continue;
        }
        if (got > 0) {
          client.incoming.insert(client.incoming.end(), buffer, buffer + got);
          if (!client.upgraded) {
            if (!handle_http(client)) {
              flush(client);
              close_client(i);
              continue;
            }
            // A browser sends the upgrade request and the first frames in
            // separate packets, but a test client may send them together —
            // so once upgraded, whatever is left in the buffer is frames.
            if (client.upgraded && !client.incoming.empty()) {handle_frames(client);}
          } else {
            handle_frames(client);
          }
        } else if (errno != EAGAIN && errno != EWOULDBLOCK) {
          close_client(i);
          continue;
        }
      }
      if (client.fd < 0) {
        clients_list_.erase(clients_list_.begin() + static_cast<long>(i));
        clients_ = clients_list_.size();
        continue;
      }
      flush(client);
      if (client.fd < 0) {
        clients_list_.erase(clients_list_.begin() + static_cast<long>(i));
        clients_ = clients_list_.size();
      }
    }
    }

    // Outside the lock, and outside the loop. A reply queued here goes out on the
    // next pass — at most 100 ms later, and immediately in practice because the
    // wake pipe is written below.
    run_pending_actions();
    if (wake_fds_[1] >= 0) {
      std::lock_guard<std::mutex> lock(mutex_);
      for (const Client & client : clients_list_) {
        if (!client.outgoing.empty()) {
          const char byte = 'x';
          [[maybe_unused]] const auto ignored = ::write(wake_fds_[1], &byte, 1);
          break;
        }
      }
    }
  }
}

}  // namespace pimesh_dashboard
