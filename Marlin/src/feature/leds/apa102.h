/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */
#pragma once
#ifndef _APA102_INCLUDE_
#define _APA102_INCLUDE_

/**
 * APA102 Support
 * TODO: #1 Update this to use the SPI interface instead of bit-banging
 */

#ifndef _INC_FROM_LEDS_H_
#error "Always include 'leds.h' and not 'apa102.h' directly."
#endif

// ------------------------
// Includes
// ------------------------

#include <stdint.h>

#include <numeric>
#include <vector>

#include "../../inc/MarlinConfig.h"

// ------------------------
// Defines
// ------------------------
#define WRITE_BIT(B)           \
    WRITE(APA102_DATA_PIN, B); \
    tick();

#define PIXEL_COLOR_BITS 0x00FFFFFF
#define PIXEL_BRIGHTNESS_BITS 0x1F000000

// ------------------------
// Types
// ------------------------
typedef uint16_t pindex_t;  // Type of "index into d_pixels vector"
typedef uint8_t brightness_t;
typedef uint32_t pixel_t;
typedef pixel_t color_t;

// ------------------------
// Classes
// ------------------------

class Marlin_APA102 {
   public:
    // "Static class"
    Marlin_APA102() = delete;

    static void init();

    // Set All
    inline static void set_all_color(const color_t color) {
        for (auto& pixel : d_pixels) set_bits(pixel, PIXEL_COLOR_BITS, color);
    }
    inline static void set_all_brightness(const brightness_t brightness) {
        for (auto& pixel : d_pixels) set_bits(pixel, PIXEL_BRIGHTNESS_BITS, brightness, 24);
    }
    inline static void set_all_pixels(const pixel_t value) {
        for (auto& pixel : d_pixels) pixel = value;
    }
    inline static void set_all_pixels(const brightness_t brightness, const color_t color) {
        for (auto& pixel : d_pixels) {
            set_bits(pixel, PIXEL_BRIGHTNESS_BITS, brightness, 24);
            set_bits(pixel, PIXEL_COLOR_BITS, color);
        }
    }

    // Set One
    inline static void set_pixel_color(const pindex_t index, const color_t color) {
        if (index < d_pixels.size()) set_bits(d_pixels[index], PIXEL_COLOR_BITS, color);
    }
    inline static void set_pixel_brightness(const pindex_t index, const brightness_t brightness) {
        if (index < d_pixels.size()) set_bits(d_pixels[index], PIXEL_BRIGHTNESS_BITS, brightness, 24);
    }
    inline static void set_pixel(const pindex_t index, const pixel_t value) {
        if (index < d_pixels.size()) d_pixels[index] = value;
    }
    inline static void set_pixel(const pindex_t index, const brightness_t brightness, const color_t color) {
        set_bits(d_pixels[index], PIXEL_BRIGHTNESS_BITS, brightness, 24);
        set_bits(d_pixels[index], PIXEL_COLOR_BITS, color);
    }

    // Write pixel data out to APA102 LEDs
    static void show();

    // Accessors
    static pixel_t pixel(const pindex_t index);
    static color_t pixel_color(const pindex_t index);
    static brightness_t pixel_brightness(const pindex_t index);

    static const std::vector<pixel_t>& pixels();
    static void pixel_colors(std::vector<color_t>& out);
    static void pixel_brightnesses(std::vector<brightness_t>& out);

   private:
    // Colors are stored in the lower 24 bits of an integer,
    // in 8-bit BGR format
    static std::vector<uint32_t> d_pixels;

    // Pulse Clock high then low
    static void tick(unsigned int cycles = 1);
    // Set Data and Clock pins low
    inline static void reset_pins() {
        WRITE(APA102_CLOCK_PIN, LOW);
        WRITE(APA102_DATA_PIN, LOW);
    }

    // Overwrite any bits in 'out' that are set in `bit`, with optional left shift
    template <class O, class B, class V, uint32_t bit_size = (sizeof(O) * 8)>
    static void set_bits(O& out, const B& bits, V& val, const uint32_t shift = 0) {
        // if (shift >= bit_size) {
        //     throw std::invalid_argument("Attempting to left shift " + std::to_string(shift) + " times on a " + std::to_string(bit_size) + " bit number");
        // }
        if (shift >= sizeof(O) * 8) {
            std::printf("Attempted to left-shift %ld times on a %ld-bit number\n", shift, bit_size);
            return;
        }
        O new_val = static_cast<O>(val) << shift;
        out &= ~bits;
        out |= new_val & bits;
    }

    // Get one bit of data from the specified bit position
    template <class T, uint32_t bit_size = sizeof(T)*8, T mask = (static_cast<T>(1) << ((sizeof(T)*8)-1))>
    static uint8_t get_bit(const T data, const uint32_t pos = 0) {
        // if (pos >= bit_size) throw std::out_of_range("Attempted to left shift " + std::to_string(shift) + " times on a " + std::to_string(bit_size) + " bit number");
        if (pos >= bit_size) return 0;
        switch (data) {
            case 0:
                return LOW;
            case std::numeric_limits<T>::max():
                return HIGH;
            default:
                return (data << pos) & mask ? HIGH : LOW;
        }
    }

    // Send a frame of arbitrary data, which will be left-packed to 32 bits
    static void data_frame(pixel_t data = LOW, uint8_t size = 32);
    // Send a start frame: 32 zeros
    inline static void start_frame() {
        data_frame(LOW, 32);
    }
    // Send an end frame: 32 + (pixels / 2) zeroes
    // See: https://cpldcpu.wordpress.com/2016/12/13/sk9822-a-clone-of-the-apa102/
    inline static void end_frame(){
    // Send 32 zeros (SK9822 "reset" frame)
    data_frame(LOW, 32);
    // Send "at least (n/2) bits of 0" end frame to clock through remaining LEDs
    data_frame(LOW, d_pixels.size() / 2);
}
};

#endif