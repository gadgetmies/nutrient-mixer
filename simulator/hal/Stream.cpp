#include "Stream.h"

#include <stdlib.h>

#include "Arduino.h"

int Stream::timedRead() {
  const unsigned long startedAt = millis();
  do {
    const int c = read();
    if (c >= 0) return c;
    delay(1);
  } while (millis() - startedAt < timeoutMilliseconds);
  return -1;
}

int Stream::timedPeek() {
  const unsigned long startedAt = millis();
  do {
    const int c = peek();
    if (c >= 0) return c;
    delay(1);
  } while (millis() - startedAt < timeoutMilliseconds);
  return -1;
}

String Stream::readString() {
  String result;
  int c;
  while ((c = timedRead()) >= 0) result += (char)c;
  return result;
}

String Stream::readStringUntil(char terminator) {
  String result;
  int c;
  while ((c = timedRead()) >= 0 && c != terminator) result += (char)c;
  return result;
}

size_t Stream::readBytes(char *buffer, size_t length) {
  size_t count = 0;
  while (count < length) {
    const int c = timedRead();
    if (c < 0) break;
    buffer[count++] = (char)c;
  }
  return count;
}

size_t Stream::readBytesUntil(char terminator, char *buffer, size_t length) {
  size_t count = 0;
  while (count < length) {
    const int c = timedRead();
    if (c < 0 || c == terminator) break;
    buffer[count++] = (char)c;
  }
  return count;
}

long Stream::parseInt() {
  String number;
  int c;
  while ((c = timedPeek()) >= 0 && !(c == '-' || (c >= '0' && c <= '9'))) read();
  while ((c = timedPeek()) >= 0 && (c == '-' || (c >= '0' && c <= '9'))) number += (char)read();
  return number.toInt();
}

float Stream::parseFloat() {
  String number;
  int c;
  while ((c = timedPeek()) >= 0 && !(c == '-' || c == '.' || (c >= '0' && c <= '9'))) read();
  while ((c = timedPeek()) >= 0 && (c == '-' || c == '.' || (c >= '0' && c <= '9'))) number += (char)read();
  return number.toFloat();
}
