#ifndef ERROR_H
#define ERROR_H

#include <stdint.h>

void fatal_error_trap(uint8_t err_code, const char* file, uint16_t line);

#define FATAL_ERROR(code) fatal_error_trap((code), __FILE__, __LINE__)

#endif