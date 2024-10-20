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

void Potentiometer::increase_number_of_resets(){
  this->num_of_resets++;
}

void Potentiometer::decrease_number_of_resets(){
  this->num_of_resets--;
}

float Potentiometer::get_voltage_level(){
  return analogRead(pin_number);
}

float Potentiometer::get_angle_degrees(){
  float current_voltage = get_voltage_level();
  /*if(current_voltage < 0.5 * 4095 / 3.3 && this->voltage > 2.5 * 4095 / 3.3) set_number_of_resets(this->num_of_resets + 1);
  else if(current_voltage > 2.5 * 4095 / 3.3 &&  this->voltage < 0.5 * 4095 / 3.3) set_number_of_resets(this->num_of_resets - 1);*/

  this->voltage = current_voltage;

  /*if(get_number_of_resets() % 2 == 0) return fmod(current_voltage * 60 * 3.3 / 4095, 180);
  else{
    float angle = current_voltage * 60 * 3.3 / 4095;
    angle = fmod(angle, 180.0);
    return fmod(180 + angle, 360.0);
  }*/

 return current_voltage * 360 / 4095;
}

void Potentiometer::set_degree_offset(float offset){
  this->degree_offset = offset;
}
float Potentiometer::get_degree_offset(){
  return this->degree_offset;
}

void Potentiometer::set_last_angle(float angle){
  this->last_angle = angle;
}
float Potentiometer::get_last_angle(){
  return this->last_angle;
}

void Potentiometer::reset_degree_offset(){
  this->reset = true;
  float angle = get_angle_degrees();
  this->degree_offset = angle;
  /*if(get_angle_degrees() - this->degree_offset > 360*0.2){
    Serial.println("Negative init");
    this->num_of_resets = -1;
  }*/
  this->last_angle = 0;
  this->num_of_resets = 0;
}

//Watchout, this function also changes the reset. No pure getter
bool Potentiometer::was_reset(){
  bool val = this->reset;
  if(val) this->reset = false;
  return val;
}

//includes offset
float Potentiometer::get_clamped_angle(){
  float angle = this->get_angle_degrees();
  angle = angle-this->get_degree_offset();
  if(angle > 360) angle -= 360;
  if(angle < 0) angle += 360;

  return angle;
}

//Potis need to be updated in each cycle to handle wrap-arounds at high speeds
void Potentiometer::update(){
  //Serial.println("update");
  float angle = this->get_clamped_angle();
  float angle_old = this->get_last_angle();

  //use upper and lower 20% as margins for wrap-around
  if(angle_old > 360*0.8 && angle < 360*0.2){
    this->increase_number_of_resets();
  }
  if(angle_old < 360*0.2 && angle > 360*0.8){
    this->decrease_number_of_resets();
  }

  this->set_last_angle(angle);
}

//does not include 2:1 translation
float Potentiometer::get_tot_angle(){
  return this->get_last_angle()+this->get_number_of_resets()*360;
}

float Potentiometer::get_assembly_angle(){
  return this->get_tot_angle()/2;
}