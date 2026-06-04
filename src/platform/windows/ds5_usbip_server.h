/**
 * @file src/platform/windows/ds5_usbip_server.h
 * @brief Embedded USB/IP server that emulates a Sony DualSense as a genuine USB
 *        device. The Microsoft-signed usbip-win2 vhci client attaches to this
 *        server on 127.0.0.1:3240, so the controller enumerates as a real
 *        USB\VID_054C&PID_0CE6 device — which strict games (Ratchet & Clank)
 *        accept for full output (adaptive triggers / rumble / lightbar).
 *
 * Replaces the old root-HID UMDF driver + HidD feature-report transport, which
 * worked for input but whose mandatory vendor feature reports made strict games
 * treat it as an unstable controller and withhold output. Proven end-to-end via
 * the ds5-usbip/usbip_ds5.py de-risk (374 output reports, rumble live).
 */
#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

namespace platf::ds5usbip {

  // 64-byte game-facing input report ([0]=report-id 0x01, [1..63]=DS5 body).
  constexpr int INPUT_REPORT_LEN = 64;
  // 47-byte DS5 output effects payload (the 0x02 report body, report-id stripped).
  constexpr int OUTPUT_PAYLOAD_LEN = 47;

  /**
   * @brief Single-controller virtual DualSense over USB/IP. Lifecycle: start() to
   *        listen, set_input() each frame, output_cb fires when the game writes an
   *        output report, stop() to tear down.
   */
  class server_t {
  public:
    /// Called from the server thread with the 47-byte effects payload (after the
    /// 0x02 report-id) whenever the game sends a DualSense output report.
    using output_cb = std::function<void(const uint8_t *payload)>;

    server_t() = default;
    ~server_t();

    server_t(const server_t &) = delete;
    server_t &operator=(const server_t &) = delete;

    /// Bind+listen on 127.0.0.1:3240 and spawn the accept/serve thread.
    /// Returns false if Winsock or bind/listen fails (e.g. port in use).
    bool start(output_cb cb);

    /// Stop the server, close sockets, join threads. Safe to call repeatedly.
    void stop();

    bool is_running() const { return running_.load(); }

    /// True once the vhci client has imported the device and is exchanging URBs.
    bool is_attached() const { return attached_.load(); }

    /// Update the live 64-byte input report delivered on the next interrupt-IN.
    void set_input(const uint8_t report[INPUT_REPORT_LEN]);

  private:
    void accept_loop();
    void serve_session(uintptr_t client_sock);

    std::atomic<bool> running_ {false};
    std::atomic<bool> attached_ {false};
    std::atomic<bool> stop_ {false};

    uintptr_t listen_sock_ {~uintptr_t(0)};            // INVALID_SOCKET
    std::atomic<uintptr_t> active_client_ {~uintptr_t(0)};  // closed by stop() to unblock recv
    std::thread accept_thread_;

    output_cb output_cb_;

    std::mutex input_mtx_;
    uint8_t input_report_[INPUT_REPORT_LEN] {};
  };

}  // namespace platf::ds5usbip
