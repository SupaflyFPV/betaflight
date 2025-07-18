/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

// DEFIO_PORT_<port>_USED_MASK is bitmask of used pins on target
// DEFIO_PORT_<port>_USED_COUNT is count of used pins on target

#if defined(RP2350A)
#define DEFIO_USED_COUNT 30
#elif defined(RP2350B)
#define DEFIO_USED_COUNT 48
#else
#error "Unsupported target MCU type for PICO"
#endif

#define DEFIO_TAG__PA0    DEFIO_TAG_MAKE(0,0)
#define DEFIO_TAG__PA1    DEFIO_TAG_MAKE(0,1)
#define DEFIO_TAG__PA2    DEFIO_TAG_MAKE(0,2)
#define DEFIO_TAG__PA3    DEFIO_TAG_MAKE(0,3)
#define DEFIO_TAG__PA4    DEFIO_TAG_MAKE(0,4)
#define DEFIO_TAG__PA5    DEFIO_TAG_MAKE(0,5)
#define DEFIO_TAG__PA6    DEFIO_TAG_MAKE(0,6)
#define DEFIO_TAG__PA7    DEFIO_TAG_MAKE(0,7)
#define DEFIO_TAG__PA8    DEFIO_TAG_MAKE(0,8)
#define DEFIO_TAG__PA9    DEFIO_TAG_MAKE(0,9)
#define DEFIO_TAG__PA10   DEFIO_TAG_MAKE(0,10)
#define DEFIO_TAG__PA11   DEFIO_TAG_MAKE(0,11)
#define DEFIO_TAG__PA12   DEFIO_TAG_MAKE(0,12)
#define DEFIO_TAG__PA13   DEFIO_TAG_MAKE(0,13)
#define DEFIO_TAG__PA14   DEFIO_TAG_MAKE(0,14)
#define DEFIO_TAG__PA15   DEFIO_TAG_MAKE(0,15)
#define DEFIO_TAG__PA16   DEFIO_TAG_MAKE(0,16)
#define DEFIO_TAG__PA17   DEFIO_TAG_MAKE(0,17)
#define DEFIO_TAG__PA18   DEFIO_TAG_MAKE(0,18)
#define DEFIO_TAG__PA19   DEFIO_TAG_MAKE(0,19)
#define DEFIO_TAG__PA20   DEFIO_TAG_MAKE(0,20)
#define DEFIO_TAG__PA21   DEFIO_TAG_MAKE(0,21)
#define DEFIO_TAG__PA22   DEFIO_TAG_MAKE(0,22)
#define DEFIO_TAG__PA23   DEFIO_TAG_MAKE(0,23)
#define DEFIO_TAG__PA24   DEFIO_TAG_MAKE(0,24)
#define DEFIO_TAG__PA25   DEFIO_TAG_MAKE(0,25)
#define DEFIO_TAG__PA26   DEFIO_TAG_MAKE(0,26)
#define DEFIO_TAG__PA27   DEFIO_TAG_MAKE(0,27)
#define DEFIO_TAG__PA28   DEFIO_TAG_MAKE(0,28)
#define DEFIO_TAG__PA29   DEFIO_TAG_MAKE(0,29)

#if defined(RP2350B)
#define DEFIO_TAG__PA30   DEFIO_TAG_MAKE(0,30)
#define DEFIO_TAG__PA31   DEFIO_TAG_MAKE(0,31)
#define DEFIO_TAG__PA32   DEFIO_TAG_MAKE(0,32)
#define DEFIO_TAG__PA33   DEFIO_TAG_MAKE(0,33)
#define DEFIO_TAG__PA34   DEFIO_TAG_MAKE(0,34)
#define DEFIO_TAG__PA35   DEFIO_TAG_MAKE(0,35)
#define DEFIO_TAG__PA36   DEFIO_TAG_MAKE(0,36)
#define DEFIO_TAG__PA37   DEFIO_TAG_MAKE(0,37)
#define DEFIO_TAG__PA38   DEFIO_TAG_MAKE(0,38)
#define DEFIO_TAG__PA39   DEFIO_TAG_MAKE(0,39)
#define DEFIO_TAG__PA40   DEFIO_TAG_MAKE(0,40)
#define DEFIO_TAG__PA41   DEFIO_TAG_MAKE(0,41)
#define DEFIO_TAG__PA42   DEFIO_TAG_MAKE(0,42)
#define DEFIO_TAG__PA43   DEFIO_TAG_MAKE(0,43)
#define DEFIO_TAG__PA44   DEFIO_TAG_MAKE(0,44)
#define DEFIO_TAG__PA45   DEFIO_TAG_MAKE(0,45)
#define DEFIO_TAG__PA46   DEFIO_TAG_MAKE(0,46)
#define DEFIO_TAG__PA47   DEFIO_TAG_MAKE(0,47)
#endif

#define DEFIO_TAG_E__PA0  DEFIO_TAG__PA0
#define DEFIO_TAG_E__PA1  DEFIO_TAG__PA1
#define DEFIO_TAG_E__PA2  DEFIO_TAG__PA2
#define DEFIO_TAG_E__PA3  DEFIO_TAG__PA3
#define DEFIO_TAG_E__PA4  DEFIO_TAG__PA4
#define DEFIO_TAG_E__PA5  DEFIO_TAG__PA5
#define DEFIO_TAG_E__PA6  DEFIO_TAG__PA6
#define DEFIO_TAG_E__PA7  DEFIO_TAG__PA7
#define DEFIO_TAG_E__PA8  DEFIO_TAG__PA8
#define DEFIO_TAG_E__PA9  DEFIO_TAG__PA9
#define DEFIO_TAG_E__PA10 DEFIO_TAG__PA10
#define DEFIO_TAG_E__PA11 DEFIO_TAG__PA11
#define DEFIO_TAG_E__PA12 DEFIO_TAG__PA12
#define DEFIO_TAG_E__PA13 DEFIO_TAG__PA13
#define DEFIO_TAG_E__PA14 DEFIO_TAG__PA14
#define DEFIO_TAG_E__PA15 DEFIO_TAG__PA15
#define DEFIO_TAG_E__PA16 DEFIO_TAG__PA16
#define DEFIO_TAG_E__PA17 DEFIO_TAG__PA17
#define DEFIO_TAG_E__PA18 DEFIO_TAG__PA18
#define DEFIO_TAG_E__PA19 DEFIO_TAG__PA19
#define DEFIO_TAG_E__PA20 DEFIO_TAG__PA20
#define DEFIO_TAG_E__PA21 DEFIO_TAG__PA21
#define DEFIO_TAG_E__PA22 DEFIO_TAG__PA22
#define DEFIO_TAG_E__PA23 DEFIO_TAG__PA23
#define DEFIO_TAG_E__PA24 DEFIO_TAG__PA24
#define DEFIO_TAG_E__PA25 DEFIO_TAG__PA25
#define DEFIO_TAG_E__PA26 DEFIO_TAG__PA26
#define DEFIO_TAG_E__PA27 DEFIO_TAG__PA27
#define DEFIO_TAG_E__PA28 DEFIO_TAG__PA28
#define DEFIO_TAG_E__PA29 DEFIO_TAG__PA29

#if defined(RP2350B)
#define DEFIO_TAG_E__PA30 DEFIO_TAG__PA30
#define DEFIO_TAG_E__PA31 DEFIO_TAG__PA31
#define DEFIO_TAG_E__PA32 DEFIO_TAG__PA32
#define DEFIO_TAG_E__PA33 DEFIO_TAG__PA33
#define DEFIO_TAG_E__PA34 DEFIO_TAG__PA34
#define DEFIO_TAG_E__PA35 DEFIO_TAG__PA35
#define DEFIO_TAG_E__PA36 DEFIO_TAG__PA36
#define DEFIO_TAG_E__PA37 DEFIO_TAG__PA37
#define DEFIO_TAG_E__PA38 DEFIO_TAG__PA38
#define DEFIO_TAG_E__PA39 DEFIO_TAG__PA39
#define DEFIO_TAG_E__PA40 DEFIO_TAG__PA40
#define DEFIO_TAG_E__PA41 DEFIO_TAG__PA41
#define DEFIO_TAG_E__PA42 DEFIO_TAG__PA42
#define DEFIO_TAG_E__PA43 DEFIO_TAG__PA43
#define DEFIO_TAG_E__PA44 DEFIO_TAG__PA44
#define DEFIO_TAG_E__PA45 DEFIO_TAG__PA45
#define DEFIO_TAG_E__PA46 DEFIO_TAG__PA46
#define DEFIO_TAG_E__PA47 DEFIO_TAG__PA47
#else
#define DEFIO_TAG_E__PA30 DEFIO_TAG__NONE
#define DEFIO_TAG_E__PA31 DEFIO_TAG__NONE
#define DEFIO_TAG_E__PA32 DEFIO_TAG__NONE
#define DEFIO_TAG_E__PA33 DEFIO_TAG__NONE
#define DEFIO_TAG_E__PA34 DEFIO_TAG__NONE
#define DEFIO_TAG_E__PA35 DEFIO_TAG__NONE
#define DEFIO_TAG_E__PA36 DEFIO_TAG__NONE
#define DEFIO_TAG_E__PA37 DEFIO_TAG__NONE
#define DEFIO_TAG_E__PA38 DEFIO_TAG__NONE
#define DEFIO_TAG_E__PA39 DEFIO_TAG__NONE
#define DEFIO_TAG_E__PA40 DEFIO_TAG__NONE
#define DEFIO_TAG_E__PA41 DEFIO_TAG__NONE
#define DEFIO_TAG_E__PA42 DEFIO_TAG__NONE
#define DEFIO_TAG_E__PA43 DEFIO_TAG__NONE
#define DEFIO_TAG_E__PA44 DEFIO_TAG__NONE
#define DEFIO_TAG_E__PA45 DEFIO_TAG__NONE
#define DEFIO_TAG_E__PA46 DEFIO_TAG__NONE
#define DEFIO_TAG_E__PA47 DEFIO_TAG__NONE
#endif

#define DEFIO_REC__PA0    DEFIO_REC_INDEXED(0)
#define DEFIO_REC__PA1    DEFIO_REC_INDEXED(1)
#define DEFIO_REC__PA2    DEFIO_REC_INDEXED(2)
#define DEFIO_REC__PA3    DEFIO_REC_INDEXED(3)
#define DEFIO_REC__PA4    DEFIO_REC_INDEXED(4)
#define DEFIO_REC__PA5    DEFIO_REC_INDEXED(5)
#define DEFIO_REC__PA6    DEFIO_REC_INDEXED(6)
#define DEFIO_REC__PA7    DEFIO_REC_INDEXED(7)
#define DEFIO_REC__PA8    DEFIO_REC_INDEXED(8)
#define DEFIO_REC__PA9    DEFIO_REC_INDEXED(9)
#define DEFIO_REC__PA10   DEFIO_REC_INDEXED(10)
#define DEFIO_REC__PA11   DEFIO_REC_INDEXED(11)
#define DEFIO_REC__PA12   DEFIO_REC_INDEXED(12)
#define DEFIO_REC__PA13   DEFIO_REC_INDEXED(13)
#define DEFIO_REC__PA14   DEFIO_REC_INDEXED(14)
#define DEFIO_REC__PA15   DEFIO_REC_INDEXED(15)
#define DEFIO_REC__PA16   DEFIO_REC_INDEXED(16)
#define DEFIO_REC__PA17   DEFIO_REC_INDEXED(17)
#define DEFIO_REC__PA18   DEFIO_REC_INDEXED(18)
#define DEFIO_REC__PA19   DEFIO_REC_INDEXED(19)
#define DEFIO_REC__PA20   DEFIO_REC_INDEXED(20)
#define DEFIO_REC__PA21   DEFIO_REC_INDEXED(21)
#define DEFIO_REC__PA22   DEFIO_REC_INDEXED(22)
#define DEFIO_REC__PA23   DEFIO_REC_INDEXED(23)
#define DEFIO_REC__PA24   DEFIO_REC_INDEXED(24)
#define DEFIO_REC__PA25   DEFIO_REC_INDEXED(25)
#define DEFIO_REC__PA26   DEFIO_REC_INDEXED(26)
#define DEFIO_REC__PA27   DEFIO_REC_INDEXED(27)
#define DEFIO_REC__PA28   DEFIO_REC_INDEXED(28)
#define DEFIO_REC__PA29   DEFIO_REC_INDEXED(29)

#if defined(RP2350B)
#define DEFIO_REC__PA30   DEFIO_REC_INDEXED(30)
#define DEFIO_REC__PA31   DEFIO_REC_INDEXED(31)
#define DEFIO_REC__PA32   DEFIO_REC_INDEXED(32)
#define DEFIO_REC__PA33   DEFIO_REC_INDEXED(33)
#define DEFIO_REC__PA34   DEFIO_REC_INDEXED(34)
#define DEFIO_REC__PA35   DEFIO_REC_INDEXED(35)
#define DEFIO_REC__PA36   DEFIO_REC_INDEXED(36)
#define DEFIO_REC__PA37   DEFIO_REC_INDEXED(37)
#define DEFIO_REC__PA38   DEFIO_REC_INDEXED(38)
#define DEFIO_REC__PA39   DEFIO_REC_INDEXED(39)
#define DEFIO_REC__PA40   DEFIO_REC_INDEXED(40)
#define DEFIO_REC__PA41   DEFIO_REC_INDEXED(41)
#define DEFIO_REC__PA42   DEFIO_REC_INDEXED(42)
#define DEFIO_REC__PA43   DEFIO_REC_INDEXED(43)
#define DEFIO_REC__PA44   DEFIO_REC_INDEXED(44)
#define DEFIO_REC__PA45   DEFIO_REC_INDEXED(45)
#define DEFIO_REC__PA46   DEFIO_REC_INDEXED(46)
#define DEFIO_REC__PA47   DEFIO_REC_INDEXED(47)
#endif

// DEFIO_IO_USED_COUNT is number of io pins supported on target
#if defined(RP2350A)
#define DEFIO_IO_USED_COUNT DEFIO_USED_COUNT
#elif defined(RP2350B)
#define DEFIO_IO_USED_COUNT DEFIO_USED_COUNT
#endif

// DEFIO_PORT_USED_LIST - comma separated list of bitmask for all used ports.
// DEFIO_PORT_OFFSET_LIST - comma separated list of port offsets (count of pins before this port)
// unused ports on end of list are skipped
// NB the following are not required in src/main
#define DEFIO_PORT_USED_COUNT   1
#define DEFIO_PORT_USED_LIST
#define DEFIO_PORT_OFFSET_LIST  0
#define DEFIO_PIN_USED_COUNT    DEFIO_USED_COUNT
