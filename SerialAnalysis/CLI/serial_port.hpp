#pragma once
#include <cstdint>
#include <stdexcept>
#include <string>
#ifdef _WIN32
#include <windows.h>
#else
#include <cerrno>
#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>
#endif

// Minimal 115200 8N1 serial port. read() waits up to 100 ms for data and
// returns 0 on timeout; write() fails after 200 ms without progress.
class Port {
#ifdef _WIN32
  HANDLE h = INVALID_HANDLE_VALUE;
#else
  int fd = -1;
#endif

  void closeHandle() {
#ifdef _WIN32
    if (h != INVALID_HANDLE_VALUE) CloseHandle(h);
    h = INVALID_HANDLE_VALUE;
#else
    if (fd >= 0) close(fd);
    fd = -1;
#endif
  }

 public:
  explicit Port(std::string name) {
    // A constructor that throws never runs the destructor, so release the
    // handle ourselves if configuration fails after the port was opened.
    try {
#ifdef _WIN32
      if (name.rfind("COM", 0) == 0) name = "\\\\.\\" + name;
      h = CreateFileA(name.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr, OPEN_EXISTING, 0, nullptr);
      if (h == INVALID_HANDLE_VALUE)
        throw std::runtime_error("cannot open serial port (Windows error " + std::to_string(GetLastError()) + ")");
      DCB d{};
      d.DCBlength = sizeof(d);
      if (!GetCommState(h, &d)) throw std::runtime_error("GetCommState failed");
      d.BaudRate = 115200;
      d.ByteSize = 8;
      d.Parity = NOPARITY;
      d.StopBits = ONESTOPBIT;
      d.fBinary = TRUE;
      // Native USB CDC boards (UNO R4 Minima) only transmit while DTR is asserted.
      d.fDtrControl = DTR_CONTROL_ENABLE;
      d.fOutxCtsFlow = FALSE;
      d.fOutxDsrFlow = FALSE;
      d.fOutX = FALSE;
      d.fInX = FALSE;
      d.fRtsControl = RTS_CONTROL_ENABLE;
      if (!SetCommState(h, &d)) throw std::runtime_error("SetCommState failed");
      COMMTIMEOUTS t{};
      t.ReadIntervalTimeout = MAXDWORD;  // return buffered bytes immediately,
      t.ReadTotalTimeoutConstant = 100;  // otherwise wait up to 100 ms for one
      t.WriteTotalTimeoutConstant = 200;
      if (!SetCommTimeouts(h, &t)) throw std::runtime_error("SetCommTimeouts failed");
#else
      fd = open(name.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
      if (fd < 0) throw std::runtime_error("cannot open serial port " + name);
      termios t{};
      if (tcgetattr(fd, &t)) throw std::runtime_error("tcgetattr failed");
      cfmakeraw(&t);
      cfsetispeed(&t, B115200);
      cfsetospeed(&t, B115200);
      t.c_cflag |= CLOCAL | CREAD;
      t.c_cflag &= ~tcflag_t(CRTSCTS);
      if (tcsetattr(fd, TCSANOW, &t)) throw std::runtime_error("tcsetattr failed");
#endif
    } catch (...) {
      closeHandle();
      throw;
    }
  }
  Port(const Port&) = delete;
  Port& operator=(const Port&) = delete;
  ~Port() { closeHandle(); }

  int read(uint8_t* b, int n) {
#ifdef _WIN32
    DWORD got = 0;
    if (!ReadFile(h, b, DWORD(n), &got, nullptr)) throw std::runtime_error("serial read failed");
    return int(got);
#else
    pollfd p{fd, POLLIN, 0};
    const int ready = ::poll(&p, 1, 100);  // sleeps instead of spinning on an idle port
    if (ready < 0) {
      if (errno == EINTR) return 0;
      throw std::runtime_error("serial poll failed");
    }
    if (ready == 0) return 0;
    const ssize_t got = ::read(fd, b, size_t(n));
    if (got < 0) {
      if (errno == EAGAIN || errno == EINTR) return 0;
      throw std::runtime_error("serial read failed");
    }
    return int(got);
#endif
  }

  void write(const uint8_t* b, int n) {
#ifdef _WIN32
    int total = 0;
    while (total < n) {
      DWORD sent = 0;
      if (!WriteFile(h, b + total, DWORD(n - total), &sent, nullptr))
        throw std::runtime_error("serial write failed (Windows error " + std::to_string(GetLastError()) +
                                 ", sent " + std::to_string(total) + "/" + std::to_string(n) + " bytes)");
      if (!sent)
        throw std::runtime_error("serial write timed out (sent " + std::to_string(total) + "/" +
                                 std::to_string(n) + " bytes)");
      total += int(sent);
    }
#else
    int total = 0;
    while (total < n) {
      const ssize_t v = ::write(fd, b + total, size_t(n - total));
      if (v > 0) {
        total += int(v);
      } else if (v < 0 && (errno == EAGAIN || errno == EINTR)) {
        pollfd p{fd, POLLOUT, 0};
        if (::poll(&p, 1, 200) == 0)
          throw std::runtime_error("serial write timed out (sent " + std::to_string(total) + "/" +
                                   std::to_string(n) + " bytes)");
      } else {
        throw std::runtime_error("serial write failed");
      }
    }
#endif
  }
};
