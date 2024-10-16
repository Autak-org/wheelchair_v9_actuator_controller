#include "Potentiometer.h"
#include <Arduino.h>
#include <cmath>
Potentiometer::Potentiometer(int pin_number = -1){
  this->pin_number = pin_number;
  this->num_of_resets = 0;
}

void Potentiometer::set_pin_number(int pin_number){
  this->pin_number = pin_number;
}

void Potentiometer::set_number_of_resets(int num_of_resets){
  this->num_of_resets = num_of_resets;
}

int Potentiometer::get_pin_number(){
  return this->pin_number;
}

int Potentiometer::get_number_of_resets(){
  return this->num_of_resets;
}

float Potentiometer::get_voltage_level(){
  return analogRead(pin_number);
}

float Potentiometer::get_angle_degrees(){
  float current_voltage = get_voltage_level();
  if(current_voltage < 0.5 * 4095 / 3.3 && this->voltage > 2.5 * 4095 / 3.3) set_number_of_resets(this->num_of_resets + 1);
  else if(current_voltage > 2.5 * 4095 / 3.3 &&  this->voltage < 0.5 * 4095 / 3.3) set_number_of_resets(this->num_of_resets - 1);

  this->voltage = current_voltage;

  if(get_number_of_resets() % 2 == 0) return fmod(current_voltage * 60 * 3.3 / 4095, 180);
  else{
    float angle = current_voltage * 60 * 3.3 / 4095;
    angle = fmod(angle, 180.0);
    return fmod(180 + angle, 360.0);
  }

}