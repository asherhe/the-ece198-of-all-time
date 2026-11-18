#pragma once
#include <cstdint>
#include <stdexcept>
#include <string>
#ifdef _WIN32
#include <windows.h>
#else
#include <cerrno>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#endif

class Port {
#ifdef _WIN32
  HANDLE h = INVALID_HANDLE_VALUE;
#else
  int fd = -1;
#endif
public:
  explicit Port(std::string name) {
#ifdef _WIN32
    if (name.rfind("COM", 0) == 0) name = "\\\\.\\" + name;
    h = CreateFileA(name.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr, OPEN_EXISTING, 0, nullptr);
    if (h == INVALID_HANDLE_VALUE) throw std::runtime_error("cannot open serial port (Windows error " + std::to_string(GetLastError()) + ")");
    DCB d{}; d.DCBlength = sizeof(d);
    if (!GetCommState(h, &d)) throw std::runtime_error("GetCommState failed");
    d.BaudRate = 115200; d.ByteSize = 8; d.Parity = NOPARITY; d.StopBits = ONESTOPBIT;
    d.fBinary = TRUE; d.fDtrControl = DTR_CONTROL_ENABLE;
    d.fOutxCtsFlow = FALSE; d.fOutxDsrFlow = FALSE;
    d.fOutX = FALSE; d.fInX = FALSE; d.fRtsControl = RTS_CONTROL_ENABLE;
    if (!SetCommState(h, &d)) throw std::runtime_error("SetCommState failed");
    COMMTIMEOUTS t{}; t.ReadIntervalTimeout = MAXDWORD; t.ReadTotalTimeoutConstant = 100;
    t.WriteTotalTimeoutConstant = 1000;
    if (!SetCommTimeouts(h, &t)) throw std::runtime_error("SetCommTimeouts failed");
#else
    fd = open(name.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) throw std::runtime_error("cannot open serial port");
    termios t{}; if (tcgetattr(fd, &t)) throw std::runtime_error("tcgetattr failed");
    cfmakeraw(&t); cfsetispeed(&t, B115200); cfsetospeed(&t, B115200);
    t.c_cflag |= CLOCAL | CREAD; t.c_cflag &= ~CRTSCTS;
    if (tcsetattr(fd, TCSANOW, &t)) throw std::runtime_error("tcsetattr failed");
#endif
  }
  ~Port() {
#ifdef _WIN32
    if (h != INVALID_HANDLE_VALUE) CloseHandle(h);
#else
    if (fd >= 0) close(fd);
#endif
  }
  int read(uint8_t* b, int n) {
#ifdef _WIN32
    DWORD got = 0; if (!ReadFile(h, b, n, &got, nullptr)) throw std::runtime_error("serial read failed"); return int(got);
#else
    int got = ::read(fd, b, n); if (got < 0 && (errno == EAGAIN || errno == EINTR)) return 0;
    if (got < 0) throw std::runtime_error("serial read failed"); return got;
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
      if (!sent) throw std::runtime_error("serial write timed out (sent " + std::to_string(total) +
                                          "/" + std::to_string(n) + " bytes)");
      total += int(sent);
    }
#else
    int sent = 0; while (sent < n) { int v = ::write(fd, b + sent, n - sent);
      if (v > 0) sent += v; else if (errno != EAGAIN && errno != EINTR) throw std::runtime_error("serial write failed"); }
#endif
  }
};
