#pragma once
#ifdef __cplusplus
extern "C" {
#endif
char *dtostrf(double number, signed char width, unsigned char precision, char *buffer);
char *itoa(int value, char *buffer, int base);
char *ltoa(long value, char *buffer, int base);
char *utoa(unsigned int value, char *buffer, int base);
char *ultoa(unsigned long value, char *buffer, int base);
#ifdef __cplusplus
}
#endif
