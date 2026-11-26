// Arduino Stream class (the parts used for reading text with a timeout)
#pragma once

#include "Print.h"

class Stream : public Print {
 public:
  virtual int available() = 0;
  virtual int read() = 0;
  virtual int peek() = 0;

  void setTimeout(unsigned long timeout) { timeoutMilliseconds = timeout; }
  unsigned long getTimeout() const { return timeoutMilliseconds; }

  String readString();
  String readStringUntil(char terminator);
  size_t readBytes(char *buffer, size_t length);
  size_t readBytesUntil(char terminator, char *buffer, size_t length);
  long parseInt();
  float parseFloat();

 protected:
  // Waits up to the timeout for the next character, -1 on timeout
  int timedRead();
  int timedPeek();
  unsigned long timeoutMilliseconds = 1000;
};
