#ifndef POTENTIOMETER_H
#define POTENTIOMETER_H
class Potentiometer{
  private: 
  int pin_number;
  int num_of_resets = 0;
  float voltage;
  float degree_offset = 0;
  float last_angle = 0;
  bool reset = false;

  public:
  Potentiometer(int pin_number);
  void set_pin_number(int pin_number);
  int get_pin_number();
  void set_number_of_resets(int num_of_resets);
  int get_number_of_resets();
  void increase_number_of_resets();
  void decrease_number_of_resets();
  float get_voltage_level();
  float get_angle_degrees();
  void set_degree_offset(float offset);
  float get_degree_offset();
  void set_last_angle(float angle);
  float get_last_angle();

  void reset_degree_offset();
  bool was_reset();
  float get_clamped_angle();
  void update();
  float get_tot_angle();
  float get_assembly_angle();
};
#endif