/**************************************************************************/
/*!
@file     MQ135.cpp
@author   G.Krocker (Mad Frog Labs)
@license  GNU GPLv3

First version of an Arduino Library for the MQ135 gas sensor
TODO: Review the correction factor calculation. This currently relies on
the datasheet but the information there seems to be wrong.

@section  HISTORY

v1.0 - First release
*/
/**************************************************************************/

#include <driver/adc.h>
#include <esp_adc_cal.h>
#include <esp32-hal.h>
#include <esp32-hal-gpio.h>
#include "MQ135.h"

static const char* TAG = "MQ135";

/**************************************************************************/
/*!
@brief  Default constructor

@param[in] pin  The analog input pin for the readout of the sensor
*/
/**************************************************************************/

static esp_adc_cal_characteristics_t adc1_chars;
static esp_adc_cal_characteristics_t adc2_chars;

MQ135::MQ135(uint8_t pin) {
  _pin = pin;
}

/**************************************************************************/
/*!
@brief  Get the correction factor to correct for temperature and humidity

@param[in] t  The ambient air temperature
@param[in] h  The relative humidity

@return The calculated correction factor
*/
/**************************************************************************/
float MQ135::getCorrectionFactor(float t, float h) {
  return CORA * t * t - CORB * t + CORC - (h-33.)*CORD;
}

/**************************************************************************/
/*!
@brief  Get the resistance of the sensor, ie. the measurement value

@return The sensor resistance in kOhm
*/
/**************************************************************************/
float MQ135::getResistance() {
  float val = analogRead(_pin) * 2.450 / 4095.;
  float ref_val = analogRead(33) * 2.450 / 4095.;
  // Rs = 3*Vr*R/val -2
  float Rs = 3. * ref_val * 10.0 / val - 2.;
  ESP_LOGI(TAG, "analogRead = %5.3f, ref = %5.3f, Rs = %5.2f", val, ref_val, Rs);
  return Rs;
}

/**************************************************************************/
/*!
@brief  Get the resistance of the sensor, ie. the measurement value corrected
        for temp/hum

@param[in] t  The ambient air temperature
@param[in] h  The relative humidity

@return The corrected sensor resistance kOhm
*/
/**************************************************************************/
float MQ135::getCorrectedResistance(float t, float h) {
  return getResistance()/getCorrectionFactor(t, h);
}

/**************************************************************************/
/*!
@brief  Get the ppm of CO2 sensed (assuming only CO2 in the air)

@return The ppm of CO2 in the air
*/
/**************************************************************************/
float MQ135::getPPM() {
  return PARA * pow((getResistance()/_rzero), -PARB);
}

/**************************************************************************/
/*!
@brief  Get the ppm of CO2 sensed (assuming only CO2 in the air), corrected
        for temp/hum

@param[in] t  The ambient air temperature
@param[in] h  The relative humidity

@return The ppm of CO2 in the air
*/
/**************************************************************************/
float MQ135::getCorrectedPPM(float t, float h) {
  return PARA * pow((getCorrectedResistance(t, h)/_rzero), -PARB);
}

/**************************************************************************/
/*!
@brief  Get the resistance RZero of the sensor for calibration purposes

@return The sensor resistance RZero in kOhm
*/
/**************************************************************************/
float MQ135::getRZero() {
  return getResistance() * pow((ATMOCO2/PARA), (1./PARB));
}

/**************************************************************************/
/*!
@brief  Get the corrected resistance RZero of the sensor for calibration
        purposes

@param[in] t  The ambient air temperature
@param[in] h  The relative humidity

@return The corrected sensor resistance RZero in kOhm
*/
/**************************************************************************/
float MQ135::getCorrectedRZero(float t, float h) {
  return getCorrectedResistance(t, h) * pow((ATMOCO2/PARA), (1./PARB));
}

bool MQ135::calibrateRZero(float cal_ppm, float t, float h) {
  auto measurement = getCorrectedPPM(t, h);
  auto step = (measurement > cal_ppm) ? -.1 : +.1;
  int it;
  for (it =0; it<10000 ; it++) {
    _rzero += step;
    measurement = getCorrectedPPM(t,h);
    if (step <0) {
      if (measurement <= cal_ppm)
        break;
    } else {
      if (measurement >= cal_ppm)
        break;
    }
    if (it % 30 == 0) {
      vTaskDelay(1); // let other tasks run
    }
  }
  if ( it < 1000 ) {
    ESP_LOGI(TAG, "Calibrated in %d iterations, RZero = %5.2f", it, _rzero);
    return true;
  } else {
    ESP_LOGE(TAG, "Failed to calibrate in %d iterations", it);
    return false;
  }
}
