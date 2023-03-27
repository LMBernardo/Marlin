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

/**
 * Marlin APA102 Addressable LED general support
 */

#include "../../inc/MarlinConfig.h"

#if ENABLED(APA102)

#include "leds.h"

#if ENABLED(APA102_STARTUP_TEST)
#include "../../core/utility.h"
#endif

// Initialize members
std::vector<uint32_t> Marlin_APA102::d_pixels{};


// Public methods
void Marlin_APA102::init() {
    // It would be best to set the pins LOW before setting them to OUTPUT...
    reset_pins();
    SET_OUTPUT(APA102_CLOCK_PIN);
    SET_OUTPUT(APA102_DATA_PIN);
    // ... but I'm not sure if that will work on all platforms.
    reset_pins();

    // Initialize all to off, and set startup brightness
    for (int i = 0; i < APA102_PIXELS; ++i) d_pixels.push_back(0);
    set_all_brightness(APA102_STARTUP_BRIGHTNESS);  //  0 .. 255 range

#if ENABLED(APA102_STARTUP_TEST)
    set_all_color(0xFF0000);
    show();
    safe_delay(500);
    set_all_color(0x00FF00);
    show();
    safe_delay(500);
    set_all_color(0x0000FF);
    show();
    safe_delay(500);
#endif

    set_all_color(APA102_COLOR);  // 24 bit RGB value
    show();
}

void Marlin_APA102::show() {
    start_frame();
    for (const auto& pixel : d_pixels) data_frame(pixel, 32);
    end_frame();
}

pixel_t Marlin_APA102::pixel(const pindex_t index) {
    return d_pixels[index];
}

color_t Marlin_APA102::pixel_color(const pindex_t index) {
    if (index > d_pixels.size()) return 0;
    return d_pixels[index] & PIXEL_COLOR_BITS;
}

brightness_t Marlin_APA102::pixel_brightness(const pindex_t index) {
    if (index > d_pixels.size()) return 0;
    return (d_pixels[index] & PIXEL_BRIGHTNESS_BITS) >> 24;
}

const std::vector<pixel_t>& Marlin_APA102::pixels() {
    return d_pixels;
}

void Marlin_APA102::pixel_colors(std::vector<color_t>& out) {
    std::transform(d_pixels.cbegin(), d_pixels.cend(), std::back_inserter(out), [](pixel_t pixel){
        return pixel & PIXEL_COLOR_BITS;
    });
};
void Marlin_APA102::pixel_brightnesses(std::vector<brightness_t>& out) {
    std::transform(d_pixels.cbegin(), d_pixels.cend(), std::back_inserter(out), [](pixel_t pixel){
        return (pixel & PIXEL_BRIGHTNESS_BITS) >> 24;
    });
};


// Private methods
void Marlin_APA102::tick(unsigned int cycles/* =1 */) {
    while (cycles-- > 0) {
        /*
         * 2µs per cycle limits us to 500MHz refresh, but
         * the APA102 has a 15 MHz max clock and the datasheet
         * recommends < 2MHz for "lighting applications" (??)
         * Therefore, we need some way to limit the clock speed.
         * ( The SK9822 supports up to 30 MHz clock speed )
         * There are probably faster ways to do this, but really
         * we should just use the SPI interface anyway (TODO#1)
        */
        delayMicroseconds(1);
        WRITE(APA102_CLOCK_PIN, HIGH);
        delayMicroseconds(1);
        WRITE(APA102_CLOCK_PIN, LOW);
    }
}

void Marlin_APA102::data_frame(pixel_t data /* =LOW */, uint8_t size /* =32 */) {
    if (size > 32) size = 32;
    reset_pins();
    // We'll send MSB-first, so we need to shift out any unused upper bits
    data <<= 32 - size;
    // Send bits of data value
    for (uint8_t i = 0; i < size; ++i) WRITE_BIT(get_bit(data, i));
}

#endif  // APA102_LED
