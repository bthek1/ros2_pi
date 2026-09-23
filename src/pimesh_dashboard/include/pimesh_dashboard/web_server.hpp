#ifndef PIMESH_DASHBOARD__WEB_SERVER_HPP_
#define PIMESH_DASHBOARD__WEB_SERVER_HPP_

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace pimesh_dashboard
{

/// One byte at the front of every WebSocket payload saying what it is.
///
/// **Every message is a binary frame, JSON included**, which is one code path in
/// the browser instead of two. The alternative — text frames for JSON, binary for
/// images — means the client has to branch on the frame type *and* on a channel
/// field inside the JSON, and the two can disagree. A leading byte cannot.
///
/// It is emphatically not base64: a JPEG costs 33% more encoded that way, for a
/// browser that can take an `ArrayBuffer` and make a `Blob` out of it directly.
enum class Channel : std::uint8_t
{
  Stats = 1,   ///< JSON, ~10 Hz — one object per stage, exactly as published.
  Rgb = 2,     ///< JPEG bytes: the annotated keypoint frame.
  Depth = 3,   ///< JPEG bytes: the colour-mapped depth.
  Pose = 4,    ///< JSON: the current pose and a tail of the trajectory.
  Mesh = 5,    ///< Binary: vertex and colour arrays, whole replacement. **No
               ///< indices** — see mesh_payload.hpp for the layout and why.
};

/// What a static request should be answered with.
struct Asset
{
  std::string content_type;
  std::vector<std::uint8_t> body;
  bool found {false};
};

/// The path a request line names, resolved under `root`, or empty if it escapes.
///
/// **Refusing `..` is the whole of this function.** A dashboard serves files from
/// a directory by name, which is the oldest hole in the oldest kind of server:
/// `GET /../../../../etc/passwd` is a request a browser will not send and
/// `curl --path-as-is` will. It is on the LAN by design — a phone on the same
/// network is meant to be able to watch — so "nobody can reach it" is not true
/// here and never was.
///
/// Returns the resolved absolute path, or an empty string for anything that
/// leaves `root`, contains a NUL, or is not an ordinary relative file path.
std::string resolve_path(const std::string & root, const std::string & request_target);

/// The media type for a file extension. Unknown extensions get
/// `application/octet-stream`, which browsers download rather than execute.
std::string content_type_for(const std::string & path);

/// An HTTP + WebSocket server on one thread, serving one directory and
/// broadcasting to whoever is connected.
///
/// **The one rule it exists to keep: nothing a client does may slow the pipeline
/// down.** `broadcast()` is called from a ROS callback and must not block, must
/// not do a syscall, and must not grow without bound when a browser is
/// backgrounded and stops reading. So it appends to a per-client buffer under a
/// mutex, and **drops and counts** when that buffer is over `send_limit_bytes`.
/// The socket writes happen on this server's own thread, where blocking costs
/// nothing.
///
/// That is the difference between a monitoring tool and a load: a dashboard that
/// can apply backpressure to a TSDF is a dashboard that changes the thing it is
/// measuring, and the gate for this phase is built entirely around proving it
/// does not.
class WebServer
{
public:
  struct Config
  {
    int port {8080};
    /// Directory the static assets are served from.
    std::string root;
    /// Per-client outgoing bytes past which frames are dropped rather than
    /// queued. One mesh (~4 MB) plus a little: enough that a normal client never
    /// hits it, small enough that a dead one cannot cost the process memory.
    std::size_t send_limit_bytes {8u * 1024u * 1024u};
    /// Ceiling on connected clients. A dashboard has a handful of viewers; a
    /// bound is what stops a port scanner becoming a memory question.
    std::size_t max_clients {8};
  };

  explicit WebServer(const Config & config);
  ~WebServer();

  WebServer(const WebServer &) = delete;
  WebServer & operator=(const WebServer &) = delete;

  /// Start listening. Returns false and leaves `error()` set if the port is
  /// taken — which is a thing to report rather than to retry: a second dashboard
  /// on one machine is two dashboards drawing one pipeline.
  bool start();
  void stop();

  /// Queue a payload for every connected client. Never blocks, never allocates
  /// per client beyond the copy, and drops rather than queueing past the limit.
  void broadcast(Channel channel, const std::uint8_t * data, std::size_t size);
  void broadcast(Channel channel, const std::string & text);

  /// What `POST /action/<name>` should do, and what to tell the page.
  ///
  /// **The only write path this server has**, and it is deliberately not on the
  /// WebSocket: a command channel multiplexed into the same socket as five
  /// telemetry streams is a place for a stray frame to become an action. An HTTP
  /// POST to a named path is one request, one answer, and it shows up in any
  /// proxy or log between here and the browser as what it is.
  ///
  /// Called on the server's own thread, so it may block — which it does: it
  /// calls a ROS service and waits for the reply. That stalls other clients for
  /// the duration, and a button somebody pressed is exactly the case where that
  /// is the right trade.
  void set_action_handler(std::function<std::string(const std::string & name)> handler)
  {
    std::lock_guard<std::mutex> lock(action_mutex_);
    action_ = std::move(handler);
  }

  std::size_t clients() const {return clients_.load(std::memory_order_relaxed);}
  /// Frames dropped because a client was not reading fast enough. **This is the
  /// number the panel shows**, and it is the evidence that the pacing rule is
  /// being kept rather than merely intended.
  std::uint64_t dropped() const {return dropped_.load(std::memory_order_relaxed);}
  std::uint64_t sent() const {return sent_.load(std::memory_order_relaxed);}
  const std::string & error() const {return error_;}
  int port() const {return config_.port;}

private:
  struct Client
  {
    int fd {-1};
    bool upgraded {false};
    std::vector<std::uint8_t> incoming;
    std::deque<std::uint8_t> outgoing;
    /// An action this client asked for, to be **run outside the lock**. See
    /// run_pending_actions().
    std::string pending_action;
    bool pending_was_post {false};
    bool has_pending {false};
  };

  void serve();
  void accept_one();
  bool handle_http(Client & client);
  void handle_frames(Client & client);
  void flush(Client & client);
  /// Run whatever `handle_http` parked, with **no lock held**, and queue the
  /// replies afterwards.
  ///
  /// **This exists because the first version called the handler inline and
  /// deadlocked.** `serve()` holds `mutex_` across its client loop, and the
  /// handler took the same non-recursive mutex to read itself — so the very first
  /// action request wedged the server and every later HTTP request timed out with
  /// no log line anywhere.
  ///
  /// Removing that second lock would have fixed the deadlock and left something
  /// worse: the handler calls a ROS service and waits up to ten seconds for it,
  /// and running that while `mutex_` is held blocks `broadcast()` — which is
  /// called from the ROS callbacks. **A button press would have applied
  /// backpressure to the pipeline**, which is the one thing this whole class is
  /// arranged to make impossible. A deadlock is a loud bug; that would have been
  /// a quiet one.
  void run_pending_actions();
  void close_client(std::size_t index);

  Config config_;
  int listen_fd_ {-1};
  /// The self-pipe the server thread wakes on. Without it `stop()` would have to
  /// wait out the poll timeout, which is the difference between a viewer closing
  /// in 20 ms and in a second.
  int wake_fds_[2] {-1, -1};
  std::thread thread_;
  std::atomic<bool> running_ {false};
  std::atomic<std::size_t> clients_ {0};
  std::atomic<std::uint64_t> dropped_ {0};
  std::atomic<std::uint64_t> sent_ {0};
  std::string error_;

  mutable std::mutex mutex_;
  std::vector<Client> clients_list_;
  /// Under its own mutex, so reading it never contends with a broadcast.
  mutable std::mutex action_mutex_;
  std::function<std::string(const std::string & name)> action_;
};

}  // namespace pimesh_dashboard

#endif  // PIMESH_DASHBOARD__WEB_SERVER_HPP_
