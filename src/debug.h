#pragma once

#define DEBUG 1    // SET TO 0 OUT TO REMOVE TRACES

#if DEBUG
#define D_print(...)    Serial.print(__VA_ARGS__)
#define D_write(...)    Serial.print(__VA_ARGS__)
#define D_println(...)  Serial.println(__VA_ARGS__)
#define D_printf(...)  Serial.printf(__VA_ARGS__)
#else
#define D_print(...)
#define D_write(...)
#define D_println(...)
#define D_printf(...)
#endif