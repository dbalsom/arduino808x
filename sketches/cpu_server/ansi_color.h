/*
    ArduinoX86 Copyright 2022-2025 Daniel Balsom
    https://github.com/dbalsom/arduinoX86

    Permission is hereby granted, free of charge, to any person obtaining a
    copy of this software and associated documentation files (the “Software”),
    to deal in the Software without restriction, including without limitation
    the rights to use, copy, modify, merge, publish, distribute, sublicense,
    and/or sell copies of the Software, and to permit persons to whom the
    Software is furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER   
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
    FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
    DEALINGS IN THE SOFTWARE.
*/

// This module has some helpers for emitting ANSI color on the debugging 
// serial port on the Arduino DUE, Serial1

#ifndef _ARDUINO_ANSI_COLOR_H
#define _ARDUINO_ANSI_COLOR_H

#include <Arduino.h>

namespace ansi {

constexpr const char* reset  = "\x1b[0m";
constexpr const char* black  = "\x1b[30m";
constexpr const char* red    = "\x1b[31m";
constexpr const char* green  = "\x1b[32m";
constexpr const char* yellow = "\x1b[33m";
constexpr const char* blue   = "\x1b[34m";
constexpr const char* magenta= "\x1b[35m";
constexpr const char* cyan   = "\x1b[36m";
constexpr const char* white  = "\x1b[37m";

// Bright colors
constexpr const char* bright_black   = "\x1b[90m";
constexpr const char* bright_red     = "\x1b[91m";
constexpr const char* bright_green   = "\x1b[92m";
constexpr const char* bright_yellow  = "\x1b[93m";
constexpr const char* bright_blue    = "\x1b[94m";
constexpr const char* bright_magenta = "\x1b[95m";
constexpr const char* bright_cyan    = "\x1b[96m";
constexpr const char* bright_white   = "\x1b[97m";
}

// Print color with string
inline void debugPrintColor(const char* color, const char* text) {
  Serial1.print(color);
  Serial1.print(text);
  Serial1.print(ansi::reset);
}

// Print color with value (numeric or otherwise)
template<typename T>
inline void debugPrintColor(const char* color, T value) {
  Serial1.print(color);
  Serial1.print(value);
  Serial1.print(ansi::reset);
}

// Print color with value and base (for integers)
template<typename T>
inline void debugPrintColor(const char* color, T value, int base) {
  Serial1.print(color);
  Serial1.print(value, base);
  Serial1.print(ansi::reset);
}

// Println color with value
template<typename T>
inline void debugPrintlnColor(const char* color, T value) {
  Serial1.print(color);
  Serial1.println(value);
  Serial1.print(ansi::reset);
}

// Println color with value and base
template<typename T>
inline void debugPrintlnColor(const char* color, T value, int base) {
  Serial1.print(color);
  Serial1.println(value, base);
  Serial1.print(ansi::reset);
}

// Println color with string
inline void debugPrintlnColor(const char* color, const char* text) {
  Serial1.print(color);
  Serial1.println(text);
  Serial1.print(ansi::reset);
}

// Println color with no arguments (just color + newline)
inline void debugPrintlnColor(const char* color) {
  Serial1.print(color);
  Serial1.println();
  Serial1.print(ansi::reset);
}


#endif //_ARDUINO_ANSI_COLOR_H