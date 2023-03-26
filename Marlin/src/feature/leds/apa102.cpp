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

Marlin_APA102 apa102;

void Marlin_APA102::init(const pindex_t num_pixels) {
    if (d_init) return;

    // TODO: We probably want to set pins low first, or maybe after. We probably don't need to do it first AND after
    reset_pins();
    SET_OUTPUT(APA102_CLOCK_PIN);
    SET_OUTPUT(APA102_DATA_PIN);
    reset_pins();

    // Initialize all to off
    for (int i = 0; i < APA102_PIXELS; ++i) d_pixels.push_back(0);

    set_all_brightness(APA102_BRIGHTNESS);  //  0 .. 255 range
    show();

#if ENABLED(APA102_STARTUP_TEST)
    set_all_color(0xFF0000);
    safe_delay(500);
    set_all_color(0x00FF00);
    safe_delay(500);
    set_all_color(0x0000FF);
    safe_delay(500);
#endif

    set_all_color(APA102_COLOR);  // 24 bit RGB value
    show();

    d_init = true;
}

void Marlin_APA102::tick(unsigned int i) {
    WRITE(APA102_CLOCK_PIN, LOW);
    while (i-- > 0) {
        delayMicroseconds(1);
        WRITE(APA102_CLOCK_PIN, HIGH);
        delayMicroseconds(1);
        WRITE(APA102_CLOCK_PIN, LOW);
    }
}

template <class O, class B, class V>
void Marlin_APA102::set_bits(O& out, const B& bits, V& val, uint8_t shift) {
    // if (shift >= sizeof(out) * 8) {
    //     throw std::invalid_argument("Attempting to left shift " + std::to_string(shift) + " times on a " + std::to_string(sizeof(out) * 8) + " bit number");
    // }
    if (shift >= sizeof(out)*8){
      std::printf("Attempted to left-shift %d times on a %d-bit number\n", shift, sizeof(out)*8);
      return;
    }
    O new_val = static_cast<O>(val) << shift;
    out &= ~bits;
    out |= new_val & bits;
}

template <class T>
uint8_t Marlin_APA102::get_data_bit(const T data, const uint8_t shift) {
    switch (data) {
        case 0:
            return LOW;
        case 1:
            return HIGH;
        default:
            const int bits = sizeof(data) * 8;
            // if (shift >= bits) throw std::out_of_range("Attempted to left shift " + std::to_string(shift) + " times on a " + std::to_string(bits) + " bit number");
            if (shift >= bits) return 0;
            return (data << shift) & (1 << (bits - 1)) ? HIGH : LOW;
    }
}

void Marlin_APA102::data_frame(pixel_t data, uint8_t size) {
    if (size > 32) size = 32;
    reset_pins();
    // We'll send MSB-first, so we need to shift out any unused upper bits
    data <<= 32 - size;
    // Send bits of data value
    for (uint8_t i = 0; i < size; ++i) WRITE_BIT(get_data_bit(data, i));
}

#endif  // APA102_LED
