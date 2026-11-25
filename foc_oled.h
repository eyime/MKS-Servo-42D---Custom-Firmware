#ifndef __OLED_H__
#define __OLED_H__

#include <stdint.h>

// The width of the screen can be set using this
// define. The default value is 128. This program use 40
#define SSD1306_WIDTH           128

// The height can be changed as well if necessary.
// It can be 32, 64 or 128. The default value is 64. This program use 72
#define SSD1306_HEIGHT          64

// If your screen horizontal axis does not start
// in column 0 you can use this define to
// adjust the horizontal offset
#define SSD1306_X_OFFSET        0

void oled_init();

void oled_write(int line, char* buffer);

void oled_clearLine(int line);

void oled_clearAll();

#endif
