#ifndef POTENTIOMETER_H
#define POTENTIOMETER_H
class Potentiometer{
  private: 
  int pin_number;
  int num_of_resets = 0;
  float voltage;

  public:
  Potentiometer(int pin_number);
  void set_pin_number(int pin_number);
  int get_pin_number();
  void set_number_of_resets(int num_of_resets);
  int get_number_of_resets();
  float get_voltage_level();
  float get_angle_degrees();
};
#endif