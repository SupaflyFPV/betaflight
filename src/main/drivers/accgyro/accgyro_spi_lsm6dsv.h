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

#include <stdbool.h>
#include <stdint.h>

#include "drivers/accgyro/accgyro.h"
#include "drivers/bus_spi.h"

// LSM6DSV variant identifiers derived from WHO_AM_I.
typedef enum {
    LSM6DSV_VARIANT_16X = 0,
    LSM6DSV_VARIANT_320X,
    LSM6DSV_VARIANT_DSK320X,
    LSM6DSV_VARIANT_COUNT
} lsm6dsvVariantId_e;

// Discovery functions
uint8_t lsm6dsvSpiDetect(const extDevice_t *dev);
bool lsm6dsvSpiAccDetect(accDev_t *acc);
bool lsm6dsvSpiGyroDetect(gyroDev_t *gyro);
