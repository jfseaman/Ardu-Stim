/* vim: set syntax=c expandtab sw=2 softtabstop=2 autoindent smartindent smarttab : */
/*
 * Arbritrary wheel pattern generator wheel definitions
 *
 * copyright 2014 David J. Andruczyk
 * 
 * Ardu-Stim software is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ArduStim software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with any ArduStim software.  If not, see http://www.gnu.org/licenses/
 *
 */
#ifndef __DEFINES_H__
#define __DEFINES_H__
 
/* defines */
#define SWEEP_ISR_RATE 1000
#define TMP_RPM_SHIFT 5 /* x16, 0-32768 RPM via pot */
#define TMP_RPM_CAP 32768 /* MAX RPM control from TunerStudio */
#define FACTOR_THRESHOLD 1000000
#define MORE_LINEAR_SWEEP 1
#define SUI_NO_INCLUDE_EXTRA_SAFETYCHECKS
#define LOG_2 0.30102999566

#define EEPROM_LAST_MODE  100

//#define DEBUG_PIN         7   /* Debug pin for Saleae to track sweep ISR execution speed */
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  #define PRIMARY_CRANK_PIN 53
  #define PRIMARY_CAM_PIN   52
#else
  #define PRIMARY_CRANK_PIN 8
  #define PRIMARY_CAM_PIN   9

#endif
//#define KNOCK_SIGNAL_PIN  10 // Knock signal for seank, ony on LS1 pattern, NOT IMPL YET
#endif
