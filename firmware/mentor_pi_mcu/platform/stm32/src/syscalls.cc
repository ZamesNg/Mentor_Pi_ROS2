// Copyright 2026 Mentor Pi contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include <sys/stat.h>
#include <sys/types.h>

#include <cerrno>
#include <cstddef>

extern "C" void Error_Handler();

namespace {

int Unsupported() {
  errno = ENOSYS;
  return -1;
}

}  // namespace

extern "C" int _close(int file) {
  static_cast<void>(file);
  return Unsupported();
}

extern "C" int _fstat(int file, struct stat* status) {
  static_cast<void>(file);
  static_cast<void>(status);
  return Unsupported();
}

extern "C" int _getpid() { return 1; }

extern "C" int _isatty(int file) {
  static_cast<void>(file);
  return Unsupported();
}

extern "C" int _kill(int process_id, int signal) {
  static_cast<void>(process_id);
  static_cast<void>(signal);
  return Unsupported();
}

extern "C" off_t _lseek(int file, off_t offset, int origin) {
  static_cast<void>(file);
  static_cast<void>(offset);
  static_cast<void>(origin);
  Unsupported();
  return static_cast<off_t>(-1);
}

extern "C" int _read(int file, void* buffer, std::size_t length) {
  static_cast<void>(file);
  static_cast<void>(buffer);
  static_cast<void>(length);
  return Unsupported();
}

extern "C" int _write(int file, const void* buffer, std::size_t length) {
  static_cast<void>(file);
  static_cast<void>(buffer);
  static_cast<void>(length);
  return Unsupported();
}

extern "C" [[noreturn]] void _exit(int status) {
  static_cast<void>(status);
  Error_Handler();
  __builtin_unreachable();
}
