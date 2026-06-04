/**
 * @file src/platform/windows/ds5_usbip_server.cpp
 * @brief USB/IP DualSense device emulation. C++ port of the proven reference
 *        ds5-usbip/usbip_ds5.py. Standard Linux USB/IP protocol (version 0x0111),
 *        the dialect the Microsoft-signed usbip-win2 vhci client speaks.
 */
// clang-format off
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
// clang-format on

#include <chrono>
#include <cstring>
#include <deque>
#include <thread>

#include "ds5_usbip_server.h"
#include "src/logging.h"

using namespace std::literals;

namespace platf::ds5usbip {

  namespace {

    constexpr uint16_t USBIP_VERSION = 0x0111;
    constexpr uint16_t OP_REQ_DEVLIST = 0x8005;
    constexpr uint16_t OP_REP_DEVLIST = 0x0005;
    constexpr uint16_t OP_REQ_IMPORT = 0x8003;
    constexpr uint16_t OP_REP_IMPORT = 0x0003;

    constexpr uint32_t CMD_SUBMIT = 0x0001;
    constexpr uint32_t RET_SUBMIT = 0x0003;
    constexpr uint32_t CMD_UNLINK = 0x0002;
    constexpr uint32_t RET_UNLINK = 0x0004;

    constexpr uint32_t DIR_OUT = 0;
    constexpr uint32_t DIR_IN = 1;

    const char BUSID[] = "1-1";

    // ---- DualSense descriptors (real DS5, VID 054C / PID 0CE6) -------------
    const uint8_t DEVICE_DESC[18] = {
      0x12, 0x01, 0x00, 0x02, 0x00, 0x00, 0x00, 0x40,
      0x4C, 0x05, 0xE6, 0x0C, 0x00, 0x01, 0x01, 0x02, 0x00, 0x01,
    };

    const uint8_t CONFIG_DESC[41] = {
      0x09, 0x02, 0x29, 0x00, 0x01, 0x01, 0x00, 0xC0, 0xFA,
      0x09, 0x04, 0x00, 0x00, 0x02, 0x03, 0x00, 0x00, 0x00,
      0x09, 0x21, 0x11, 0x01, 0x00, 0x01, 0x22, 0x11, 0x01,
      0x07, 0x05, 0x84, 0x03, 0x40, 0x00, 0x06,
      0x07, 0x05, 0x03, 0x03, 0x40, 0x00, 0x06,
    };

    // Real 273-byte DS5 HID report descriptor (no vendor reports — clean).
    const uint8_t HID_REPORT_DESC[273] = {
      0x05, 0x01, 0x09, 0x05, 0xA1, 0x01, 0x85, 0x01, 0x09, 0x30, 0x09, 0x31, 0x09, 0x32, 0x09, 0x35,
      0x09, 0x33, 0x09, 0x34, 0x15, 0x00, 0x26, 0xFF, 0x00, 0x75, 0x08, 0x95, 0x06, 0x81, 0x02, 0x06,
      0x00, 0xFF, 0x09, 0x20, 0x95, 0x01, 0x81, 0x02, 0x05, 0x01, 0x09, 0x39, 0x15, 0x00, 0x25, 0x07,
      0x35, 0x00, 0x46, 0x3B, 0x01, 0x65, 0x14, 0x75, 0x04, 0x95, 0x01, 0x81, 0x42, 0x65, 0x00, 0x05,
      0x09, 0x19, 0x01, 0x29, 0x0F, 0x15, 0x00, 0x25, 0x01, 0x75, 0x01, 0x95, 0x0F, 0x81, 0x02, 0x06,
      0x00, 0xFF, 0x09, 0x21, 0x95, 0x0D, 0x81, 0x02, 0x06, 0x00, 0xFF, 0x09, 0x22, 0x15, 0x00, 0x26,
      0xFF, 0x00, 0x75, 0x08, 0x95, 0x34, 0x81, 0x02, 0x85, 0x02, 0x09, 0x23, 0x95, 0x2F, 0x91, 0x02,
      0x85, 0x05, 0x09, 0x33, 0x95, 0x28, 0xB1, 0x02, 0x85, 0x08, 0x09, 0x34, 0x95, 0x2F, 0xB1, 0x02,
      0x85, 0x09, 0x09, 0x24, 0x95, 0x13, 0xB1, 0x02, 0x85, 0x0A, 0x09, 0x25, 0x95, 0x1A, 0xB1, 0x02,
      0x85, 0x20, 0x09, 0x26, 0x95, 0x3F, 0xB1, 0x02, 0x85, 0x21, 0x09, 0x27, 0x95, 0x04, 0xB1, 0x02,
      0x85, 0x22, 0x09, 0x40, 0x95, 0x3F, 0xB1, 0x02, 0x85, 0x80, 0x09, 0x28, 0x95, 0x3F, 0xB1, 0x02,
      0x85, 0x81, 0x09, 0x29, 0x95, 0x3F, 0xB1, 0x02, 0x85, 0x82, 0x09, 0x2A, 0x95, 0x09, 0xB1, 0x02,
      0x85, 0x83, 0x09, 0x2B, 0x95, 0x3F, 0xB1, 0x02, 0x85, 0x84, 0x09, 0x2C, 0x95, 0x3F, 0xB1, 0x02,
      0x85, 0x85, 0x09, 0x2D, 0x95, 0x02, 0xB1, 0x02, 0x85, 0xA0, 0x09, 0x2E, 0x95, 0x01, 0xB1, 0x02,
      0x85, 0xE0, 0x09, 0x2F, 0x95, 0x3F, 0xB1, 0x02, 0x85, 0xF0, 0x09, 0x30, 0x95, 0x3F, 0xB1, 0x02,
      0x85, 0xF1, 0x09, 0x31, 0x95, 0x3F, 0xB1, 0x02, 0x85, 0xF2, 0x09, 0x32, 0x95, 0x0F, 0xB1, 0x02,
      0x85, 0xF4, 0x09, 0x35, 0x95, 0x3F, 0xB1, 0x02, 0x85, 0xF5, 0x09, 0x36, 0x95, 0x03, 0xB1, 0x02,
      0xC0,
    };

    // Genuine feature reports captured from a real DualSense (all 64 bytes).
    const uint8_t FEAT_05[64] = {
      0x05,0xFC,0xFF,0xF3,0xFF,0xFC,0xFF,0x99,0x22,0x5F,0xDD,0x8C,0x22,0x59,0xDD,0xBC,
      0x22,0x3E,0xDD,0x1C,0x02,0x1C,0x02,0x15,0x20,0x15,0xE0,0xB5,0x1F,0xDD,0xDF,0xFD,
      0x1F,0xFE,0xDF,0x06,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
      0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    };
    const uint8_t FEAT_09[64] = {
      0x09,0x0F,0xB1,0xA9,0xC1,0xBC,0xD0,0x08,0x25,0x00,0xB2,0xE5,0xBE,0x21,0xE4,0x00,
      0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
      0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
      0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    };
    const uint8_t FEAT_20[64] = {
      0x20,0x4A,0x75,0x6E,0x20,0x32,0x34,0x20,0x32,0x30,0x32,0x34,0x31,0x31,0x3A,0x31,
      0x36,0x3A,0x32,0x31,0x02,0x00,0x04,0x00,0x13,0x03,0x00,0x00,0x00,0x00,0x0F,0x01,
      0x41,0x0A,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x20,0x05,0x00,0x00,
      0x2A,0x00,0x01,0x00,0x0A,0x00,0x02,0x00,0x06,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    };
    const uint8_t FEAT_81[64] = {0x81};

    // ---- big-endian write helpers -----------------------------------------
    void put32(std::vector<uint8_t> &v, uint32_t x) {
      v.push_back((x >> 24) & 0xFF); v.push_back((x >> 16) & 0xFF);
      v.push_back((x >> 8) & 0xFF);  v.push_back(x & 0xFF);
    }
    void put16(std::vector<uint8_t> &v, uint16_t x) {
      v.push_back((x >> 8) & 0xFF); v.push_back(x & 0xFF);
    }
    uint32_t rd32(const uint8_t *p) {
      return (uint32_t(p[0]) << 24) | (uint32_t(p[1]) << 16) | (uint32_t(p[2]) << 8) | p[3];
    }

    // little-endian read (USB setup packet fields)
    uint16_t le16(const uint8_t *p) { return uint16_t(p[0]) | (uint16_t(p[1]) << 8); }

  }  // namespace

  // ===========================================================================
  //  Per-session state machine
  // ===========================================================================
  namespace {

    struct session_t {
      SOCKET sock;
      std::mutex send_mtx;
      std::mutex pend_mtx;
      std::deque<std::pair<uint32_t, uint32_t>> pending_in;  // (seqnum, devid)
      std::atomic<bool> stop {false};
      std::mutex *input_mtx;
      const uint8_t *input_report;       // shared 64-byte buffer (guarded by input_mtx)
      server_t::output_cb *out_cb;

      bool send_all(const uint8_t *buf, int len) {
        std::lock_guard<std::mutex> lk(send_mtx);
        int off = 0;
        while (off < len) {
          int n = ::send(sock, (const char *) buf + off, len - off, 0);
          if (n <= 0) return false;
          off += n;
        }
        return true;
      }

      bool recv_n(uint8_t *buf, int n) {
        int off = 0;
        while (off < n) {
          int r = ::recv(sock, (char *) buf + off, n - off, 0);
          if (r <= 0) return false;
          off += r;
        }
        return true;
      }

      void ret_submit(uint32_t seqnum, uint32_t devid, uint32_t direction, uint32_t ep,
                      int32_t status, const uint8_t *payload, int paylen) {
        std::vector<uint8_t> v;
        v.reserve(48 + (direction == DIR_IN ? paylen : 0));
        put32(v, RET_SUBMIT); put32(v, seqnum); put32(v, devid);
        put32(v, direction);  put32(v, ep);
        put32(v, (uint32_t) status);
        put32(v, (uint32_t) (direction == DIR_IN ? paylen : 0));
        put32(v, 0); put32(v, 0); put32(v, 0);     // start_frame, npkts, error_count
        for (int i = 0; i < 8; i++) v.push_back(0); // padding
        if (direction == DIR_IN && paylen > 0)
          v.insert(v.end(), payload, payload + paylen);
        send_all(v.data(), (int) v.size());
      }

      void ret_unlink(uint32_t seqnum, uint32_t devid, int32_t status) {
        std::vector<uint8_t> v;
        put32(v, RET_UNLINK); put32(v, seqnum); put32(v, devid);
        put32(v, 0); put32(v, 0);
        put32(v, (uint32_t) status);
        for (int i = 0; i < 24; i++) v.push_back(0);
        send_all(v.data(), (int) v.size());
      }

      // EP0 control transfer.
      void handle_control(uint32_t seqnum, uint32_t devid, uint32_t direction,
                          const uint8_t *setup, const uint8_t *out_data, int out_len) {
        uint8_t bmRequestType = setup[0];
        uint8_t bRequest = setup[1];
        uint16_t wValue = le16(setup + 2);
        uint16_t wLength = le16(setup + 6);
        uint8_t rtype = (bmRequestType >> 5) & 0x3;  // 0=std 1=class 2=vendor

        if (rtype == 0) {                            // standard
          if (bRequest == 0x06) {                    // GET_DESCRIPTOR
            uint8_t dtype = wValue >> 8, dindex = wValue & 0xFF;
            const uint8_t *data = nullptr; int dlen = 0;
            static const uint8_t lang[] = {0x04, 0x03, 0x09, 0x04};
            static const uint8_t s_mfr[] = {0x3E, 0x03, 'S',0,'o',0,'n',0,'y',0,' ',0,'I',0,
              'n',0,'t',0,'e',0,'r',0,'a',0,'c',0,'t',0,'i',0,'v',0,'e',0,' ',0,'E',0,'n',0,
              't',0,'e',0,'r',0,'t',0,'a',0,'i',0,'n',0,'m',0,'e',0,'n',0,'t',0};
            static const uint8_t s_prod[] = {0x28, 0x03, 'W',0,'i',0,'r',0,'e',0,'l',0,'e',0,
              's',0,'s',0,' ',0,'C',0,'o',0,'n',0,'t',0,'r',0,'o',0,'l',0,'l',0,'e',0,'r',0};
            if (dtype == 0x01) { data = DEVICE_DESC; dlen = sizeof(DEVICE_DESC); }
            else if (dtype == 0x02) { data = CONFIG_DESC; dlen = sizeof(CONFIG_DESC); }
            else if (dtype == 0x22) { data = HID_REPORT_DESC; dlen = sizeof(HID_REPORT_DESC); }
            else if (dtype == 0x21) { data = CONFIG_DESC + 18; dlen = 9; }
            else if (dtype == 0x03) {                // string
              if (dindex == 0) { data = lang; dlen = sizeof(lang); }
              else if (dindex == 1) { data = s_mfr; dlen = sizeof(s_mfr); }
              else if (dindex == 2) { data = s_prod; dlen = sizeof(s_prod); }
              else { ret_submit(seqnum, devid, direction, 0, 0, nullptr, 0); return; }
            }
            int n = (dlen < (int) wLength) ? dlen : (int) wLength;
            ret_submit(seqnum, devid, direction, 0, 0, data, n);
            return;
          }
          ret_submit(seqnum, devid, direction, 0, 0, nullptr, 0);  // SET_CONFIG/IFACE/etc
          return;
        }
        if (rtype == 1) {                            // HID class
          if (bRequest == 0x01) {                    // GET_REPORT (feature)
            uint8_t rid = wValue & 0xFF;
            const uint8_t *f = nullptr;
            uint8_t zero[64] = {0};
            if (rid == 0x05) f = FEAT_05;
            else if (rid == 0x09) f = FEAT_09;
            else if (rid == 0x20) f = FEAT_20;
            else if (rid == 0x81) f = FEAT_81;
            else { zero[0] = rid; f = zero; }
            int n = (64 < (int) wLength) ? 64 : (int) wLength;
            ret_submit(seqnum, devid, direction, 0, 0, f, n);
            return;
          }
          if (bRequest == 0x09) {                    // SET_REPORT (output via EP0)
            if (out_len > 1 && out_data[0] == 0x02 && out_cb && *out_cb)
              (*out_cb)(out_data + 1);
            ret_submit(seqnum, devid, direction, 0, 0, nullptr, 0);
            return;
          }
          ret_submit(seqnum, devid, direction, 0, 0, nullptr, 0);  // SET_IDLE / SET_PROTOCOL
          return;
        }
        ret_submit(seqnum, devid, direction, 0, 0, nullptr, 0);
      }

      // ~250 Hz interrupt-IN sender: deliver the current input report to one
      // pending IN URB per tick (paces the stream like a real full-speed device).
      void input_sender() {
        while (!stop.load()) {
          std::pair<uint32_t, uint32_t> job; bool have = false;
          {
            std::lock_guard<std::mutex> lk(pend_mtx);
            if (!pending_in.empty()) { job = pending_in.front(); pending_in.pop_front(); have = true; }
          }
          if (have) {
            uint8_t rep[INPUT_REPORT_LEN];
            { std::lock_guard<std::mutex> lk(*input_mtx);
              std::memcpy(rep, input_report, INPUT_REPORT_LEN); }
            ret_submit(job.first, job.second, DIR_IN, 4, 0, rep, INPUT_REPORT_LEN);
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(4));
        }
      }

      void run() {
        std::thread sender(&session_t::input_sender, this);
        uint8_t hdr[48];
        while (!stop.load()) {
          if (!recv_n(hdr, 48)) break;
          uint32_t command = rd32(hdr);
          uint32_t seqnum = rd32(hdr + 4), devid = rd32(hdr + 8);
          uint32_t direction = rd32(hdr + 12), ep = rd32(hdr + 16);
          if (command == CMD_SUBMIT) {
            uint32_t xfer_len = rd32(hdr + 24);
            const uint8_t *setup = hdr + 40;
            std::vector<uint8_t> out_data;
            if (direction == DIR_OUT && xfer_len > 0) {
              out_data.resize(xfer_len);
              if (!recv_n(out_data.data(), (int) xfer_len)) break;
            }
            if (ep == 0) {
              handle_control(seqnum, devid, direction, setup,
                             out_data.data(), (int) out_data.size());
            } else if (direction == DIR_IN) {
              std::lock_guard<std::mutex> lk(pend_mtx);
              pending_in.emplace_back(seqnum, devid);
            } else {                                 // interrupt OUT — game output
              if (out_data.size() > 1 && out_data[0] == 0x02 && out_cb && *out_cb)
                (*out_cb)(out_data.data() + 1);
              ret_submit(seqnum, devid, direction, ep, 0, nullptr, 0);
            }
          } else if (command == CMD_UNLINK) {
            uint32_t unlink_seq = rd32(hdr + 20);
            { std::lock_guard<std::mutex> lk(pend_mtx);
              for (auto it = pending_in.begin(); it != pending_in.end();)
                it = (it->first == unlink_seq) ? pending_in.erase(it) : it + 1; }
            ret_unlink(seqnum, devid, 0);
          } else {
            break;
          }
        }
        stop.store(true);
        sender.join();
      }
    };

    // usbip_usb_device wire struct (big-endian) for DEVLIST / IMPORT replies.
    void append_usb_device(std::vector<uint8_t> &v, bool with_iface) {
      uint8_t path[256] = {0}; std::strcpy((char *) path, "/sys/devices/usbip/1-1");
      uint8_t busid[32] = {0}; std::strcpy((char *) busid, BUSID);
      v.insert(v.end(), path, path + 256);
      v.insert(v.end(), busid, busid + 32);
      put32(v, 1); put32(v, 1); put32(v, 2);       // busnum, devnum, speed(2=full)
      put16(v, 0x054C); put16(v, 0x0CE6); put16(v, 0x0100);
      v.push_back(0x00); v.push_back(0x00); v.push_back(0x00);  // dev class/sub/proto
      v.push_back(0x01); v.push_back(0x01); v.push_back(0x01);  // cfgval, nconf, nif
      if (with_iface) { v.push_back(0x03); v.push_back(0x00); v.push_back(0x00); v.push_back(0x00); }
    }

  }  // namespace

  // ===========================================================================
  //  server_t
  // ===========================================================================
  server_t::~server_t() { stop(); }

  bool server_t::start(output_cb cb) {
    if (running_.load()) return true;
    output_cb_ = std::move(cb);
    for (int i = 0; i < INPUT_REPORT_LEN; i++) input_report_[i] = 0;
    input_report_[0] = 0x01;
    input_report_[1] = input_report_[2] = input_report_[3] = input_report_[4] = 0x80;

    WSADATA wsa;
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
      BOOST_LOG(error) << "ds5usbip: WSAStartup failed"sv;
      return false;
    }
    SOCKET s = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (s == INVALID_SOCKET) { BOOST_LOG(error) << "ds5usbip: socket() failed"sv; return false; }
    int yes = 1;
    setsockopt(s, SOL_SOCKET, SO_REUSEADDR, (const char *) &yes, sizeof(yes));
    sockaddr_in addr {};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(3240);
    InetPtonA(AF_INET, "127.0.0.1", &addr.sin_addr);
    if (::bind(s, (sockaddr *) &addr, sizeof(addr)) != 0 || ::listen(s, 2) != 0) {
      BOOST_LOG(error) << "ds5usbip: bind/listen on 127.0.0.1:3240 failed (port in use?)"sv;
      ::closesocket(s);
      return false;
    }
    listen_sock_ = (uintptr_t) s;
    stop_.store(false);
    running_.store(true);
    accept_thread_ = std::thread(&server_t::accept_loop, this);
    BOOST_LOG(info) << "ds5usbip: virtual DualSense USB/IP server on 127.0.0.1:3240"sv;
    return true;
  }

  void server_t::accept_loop() {
    while (!stop_.load()) {
      SOCKET cs = ::accept((SOCKET) listen_sock_, nullptr, nullptr);
      if (cs == INVALID_SOCKET) break;
      int one = 1;
      setsockopt(cs, IPPROTO_TCP, TCP_NODELAY, (const char *) &one, sizeof(one));
      active_client_.store((uintptr_t) cs);
      serve_session((uintptr_t) cs);
      active_client_.store(~uintptr_t(0));
      ::closesocket(cs);
      attached_.store(false);
    }
  }

  void server_t::serve_session(uintptr_t client_sock) {
    SOCKET cs = (SOCKET) client_sock;
    uint8_t hdr[8];
    int off = 0;
    while (off < 8) {
      int r = ::recv(cs, (char *) hdr + off, 8 - off, 0);
      if (r <= 0) return;
      off += r;
    }
    uint16_t code = (uint16_t(hdr[2]) << 8) | hdr[3];

    if (code == OP_REQ_DEVLIST) {
      std::vector<uint8_t> v;
      put16(v, USBIP_VERSION); put16(v, OP_REP_DEVLIST); put32(v, 0);
      put32(v, 1);
      append_usb_device(v, true);
      ::send(cs, (const char *) v.data(), (int) v.size(), 0);
      return;
    }
    if (code == OP_REQ_IMPORT) {
      uint8_t busid[32]; int boff = 0;
      while (boff < 32) { int r = ::recv(cs, (char *) busid + boff, 32 - boff, 0); if (r <= 0) return; boff += r; }
      bool ok = std::strncmp((char *) busid, BUSID, 3) == 0;
      std::vector<uint8_t> v;
      put16(v, USBIP_VERSION); put16(v, OP_REP_IMPORT); put32(v, ok ? 0 : 1);
      if (ok) append_usb_device(v, false);
      ::send(cs, (const char *) v.data(), (int) v.size(), 0);
      if (!ok) return;

      BOOST_LOG(info) << "ds5usbip: vhci attached — serving virtual DualSense"sv;
      attached_.store(true);
      session_t sess;
      sess.sock = cs;
      sess.input_mtx = &input_mtx_;
      sess.input_report = input_report_;
      sess.out_cb = &output_cb_;
      sess.run();
      BOOST_LOG(info) << "ds5usbip: vhci session ended"sv;
      return;
    }
  }

  void server_t::set_input(const uint8_t report[INPUT_REPORT_LEN]) {
    std::lock_guard<std::mutex> lk(input_mtx_);
    std::memcpy(input_report_, report, INPUT_REPORT_LEN);
  }

  void server_t::stop() {
    if (!running_.exchange(false)) return;
    stop_.store(true);
    if (listen_sock_ != ~uintptr_t(0)) {
      ::closesocket((SOCKET) listen_sock_);
      listen_sock_ = ~uintptr_t(0);
    }
    // Close any in-flight client socket too, so a session blocked in recv() unblocks
    // and the accept thread can be joined without hanging.
    uintptr_t ac = active_client_.exchange(~uintptr_t(0));
    if (ac != ~uintptr_t(0)) {
      ::closesocket((SOCKET) ac);
    }
    if (accept_thread_.joinable()) accept_thread_.join();
    attached_.store(false);
  }

}  // namespace platf::ds5usbip
