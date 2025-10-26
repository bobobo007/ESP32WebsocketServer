/**
 * @author  Bohumir Coufal
 * @email   bohumir.coufal@arcor.de
 * @version v1.0
 * @ide     Arduino IDE 2.3.6
 * @license MIT
 * @brief   Library for Functions.h hardware
 *
\verbatim
   ----------------------------------------------------------------------
    Copyright (c) 2024 Bohumir Coufal

    Permission is hereby granted, free of charge, to any person
    obtaining a copy of this software and associated documentation
    files (the "Software"), to deal in the Software without restriction,
    including without limitation the rights to use, copy, modify, merge,
    publish, distribute, sublicense, and/or sell copies of the Software,
    and to permit persons to whom the Software is furnished to do so,
    subject to the following conditions:

    The above copyright notice and this permission notice shall be
    included in all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
    EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
    OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE
    AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
    HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
    WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
    FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
    OTHER DEALINGS IN THE SOFTWARE.
   ----------------------------------------------------------------------
\endverbatim
*/

#include "V07_001.h"

#define DRAIN_VALVE_TIME 120000     //number of miliseconds before it will defecate
#define MAX_PREHEAT_TIME 3600000    //maximal preheat time in miliseconds = 1hour

//definite temperatures at the valve
//******************
#define MINTEMPERATURE 1.5          //if the valve temperature does not drop below this temperature for 24 hours the valve will not heat up 1.5°C
#define PREHEAT_TEMPERATURE 8       //the valve will heat up to this temperature in the winter period 8°C
#define ABSOLUT_TEMPERATURE -5      //if the temperature drops below this value the valve will not defrost (-5°C)

//PCB temperatures
#define PCB_MIN_TEMPERATURE 5
#define PCB_MAX_TEMPERATURE 10
#define MAX_PCB_PREHEAT_TIME 1800000   //maximal preheat time in miliseconds (1/2hour = 1800000)

// Function definitions
//*******************************************
#ifndef Functions
#define Functions
class Cleaning
{
  public:
  /**
   * @brief Function to control the drain valve
   * @param noting
   * @retval noting
   */
   void funDrainVentil(void);
  /**
   * @brief Function for controlling the three-way valve to the clening or recirculation position
   * @param noting
   * @retval noting
   */
   void funkCleaningVentil(void);
  /**
   * @brief Function for checking temperature below MINTEMPERATURE
   * @param noting
   * @retval false - Aspoň v jednej hodine za posledných 24 hodín klesla teplota pod MINTEMPERATURE
   */
   bool funTemperatureStatus(void);
   /**
   * @brief Heating control function
   * @param Teplota na PCB doske
   * @retval noting
   */
   void controlPCBHeating(float pcbTemperature);
};
#endif  //Functions
