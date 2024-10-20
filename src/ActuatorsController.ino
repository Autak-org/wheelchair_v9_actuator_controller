#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "driver/twai.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include "Potentiometer.h"

//Interupt and LED pins not used
#define INTERRUPT_PIN -1
#define LED_PIN -1

// Define the control pins for the first motor driver
#define IN1 27
#define IN2 14
#define IN3 12 
#define IN4 13

// Define the control pins for the second motor driver
#define IN5 25
#define IN6 26
#define POTENTIOMETER_1 33
#define POTENTIOMETER_2 32
#define POTENTIOMETER_3 39 //is 35 still available??? (31 is no ADC pin)

#define BAT1 35
#define BAT2 34

#define TEMP_SENSOR_PIN 4

#define TWAI_ID 99 //This is the TWAI communication identifier of this mictocontroller

//TWAI configuration settings 
twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_18, GPIO_NUM_5, TWAI_MODE_NORMAL);
twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

twai_message_t received_message;
uint32_t startingTime = 0;

MPU6050 mpu;
//Create instances of OneWire and DallasTemperature objects
OneWire oneWire(TEMP_SENSOR_PIN);
DallasTemperature temp_sensor(&oneWire);

/*
float current_length = 205.0;
float max_length = 205.0;

//--These variables are used in case that the mathematical model for the actuator self leveling is used within correctSeatPosition() method.
*/

//Control flags for voltage level and temperature data transmission over TWAI bus
bool tempTransmit = false, voltage1Transmit = true, voltage2Transmit = false;

//Enumerates the actuators that are used within the code
enum ACTUATOR{
  FOOTREST_ACTUATOR = 0,
  BACKREST_ACTUATOR = 1,
  SEAT_ACTUATOR = 2
};

//Enumerator to describe desired actuator action
enum ACTUATOR_ACTION{
  ACTUATOR_EXTEND,
  ACTUATOR_RETRACT,
  ACTUATOR_STOP
};

//Creates Potentiometer instances
Potentiometer left_potentiometer(POTENTIOMETER_2), right_potentiometer(POTENTIOMETER_1), rear_potentiometer(POTENTIOMETER_3);

/*---- These variables were created and are used according to the "MPU6050_6Axis_MotionApps20.h" library example "MPU6050_DMP6". 
Refer library's documentation for more details.*/
bool blinkState = false;

// MPU control/staturs vars
bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

//Orientation/motion vars
Quaternion q;
VectorInt16 aa;
VectorInt16 aaReal;
VectorInt16 aaWorld;
VectorFloat gravity;
float* ypr;

volatile bool mpuInterrupt = false;
void dmpDataReady() {
  mpuInterrupt = true;
}
/*-------------------------------------------------------------------------------------------------------------------*/

ACTUATOR_ACTION footrest_state = ACTUATOR_STOP;
ACTUATOR_ACTION backrest_state = ACTUATOR_STOP;
ACTUATOR_ACTION seat_state = ACTUATOR_STOP;

#define QUEUE_CAPACITY 40  // Define the capacity of the queue

// Define the structure for the circular queue
typedef struct {
    uint8_t items[QUEUE_CAPACITY];  // Queue array to store elements
    int front;                      // Front index
    int rear;                       // Rear index
    int size;                       // Current size of the queue
} CircularQueue;

void initializeQueue(CircularQueue* q);
bool isFull(CircularQueue* q);
bool isEmpty(CircularQueue* q);
bool enqueue(CircularQueue* q, uint8_t value);
uint8_t dequeue(CircularQueue* q);
uint8_t average(CircularQueue* q);

// Function to initialize the queue
void initializeQueue(CircularQueue* q) {
    q->front = 0;
    q->rear = 0;
    q->size = 0;
}

// Check if the queue is full
bool isFull(CircularQueue* q) {
    return q->size == QUEUE_CAPACITY;
}

// Check if the queue is empty
bool isEmpty(CircularQueue* q) {
    return q->size == 0;
}

// Enqueue an element to the queue
bool enqueue(CircularQueue* q, uint8_t value) {
    if (isFull(q)) {
        return false;
    }

    q->items[q->rear] = value;
    q->rear = (q->rear + 1) % QUEUE_CAPACITY;  // Wrap around
    q->size++;
    return true;
}

// Dequeue an element from the queue
uint8_t dequeue(CircularQueue* q) {
    if (isEmpty(q)) {
        return 0;
    }

    uint8_t value = q->items[q->front];
    q->front = (q->front + 1) % QUEUE_CAPACITY;  // Wrap around
    q->size--;
    return value;
}

uint8_t average(CircularQueue* q) {
    if (isEmpty(q)) {
        return 0.0;
    }

    int sum = 0;
    int index = q->front;
    for (int i = 0; i < q->size; i++) {
        sum += q->items[index];
        index = (index + 1) % QUEUE_CAPACITY;  // Wrap around
    }

    return (uint8_t)((float)sum / q->size);
}

/*-- getYawRoll() is a based on the "MPU6050_6Axis_MotionApps20.h" example "MPU6050_DMP6", and was modified according to
 the following YouTube video: "https://youtu.be/k5i-vE5rZR0?si=voIdb-SDxjW27zNN" by user "Maker's Wharf" --*/
float* getYawRoll(Quaternion& q){ 
  static float ypr[3];
  float q0 = q.w;
  float q1 = q.x;
  float q2 = q.y;
  float q3 = q.z;

  ypr[0] = -atan2(-2 * q1 * q2 + 2 * q0 * q3, q2 * q2 - q3 * q3 - q1 * q1 + q0 * q0);
  ypr[1] = asin(2 * q2 * q3 + 2 * q0 * q1);
  ypr[2] = atan2(-2 * q1 * q3 + 2 * q0 * q2, q3 * q3 - q2 * q2 - q1 * q1 + q0 * q0);
  return ypr;
}
/*-----------------------------------------------------------------------------------------------------------------*/

void setActuatorAction(ACTUATOR actuator, ACTUATOR_ACTION action){
  /* This method set's a given actuators action.
     Arguments:
        - enum ACTUATOR actuator: The actuator we want to control
        - enum ACTUATOR_ACTIOn action: The action that we want to perform with that acuator
     Returs:
        - void
  */
  int ina, inb;

  if(actuator == FOOTREST_ACTUATOR){
    ina = IN1; inb = IN2;
  }
  else if(actuator == BACKREST_ACTUATOR){
    ina = IN3; inb = IN4;
  }
  else if(actuator == SEAT_ACTUATOR){
    //ina = IN5; inb = IN6;
  }

  //This lazy-update version doesn't change the bucking behavior of the actuators
  if(action == ACTUATOR_EXTEND){
    if(actuator == FOOTREST_ACTUATOR && footrest_state != ACTUATOR_EXTEND || actuator == BACKREST_ACTUATOR && backrest_state != ACTUATOR_EXTEND){
      digitalWrite(ina, HIGH);
      digitalWrite(inb, LOW);

      if(actuator == FOOTREST_ACTUATOR) footrest_state = ACTUATOR_EXTEND;
      if(actuator == BACKREST_ACTUATOR) backrest_state = ACTUATOR_EXTEND;
    }
  }
  else if (action == ACTUATOR_RETRACT){
    if(actuator == FOOTREST_ACTUATOR && footrest_state != ACTUATOR_RETRACT || actuator == BACKREST_ACTUATOR && backrest_state != ACTUATOR_RETRACT || actuator == SEAT_ACTUATOR && seat_state != ACTUATOR_RETRACT){
      digitalWrite(ina, LOW);
      digitalWrite(inb, HIGH);

      if(actuator == FOOTREST_ACTUATOR) footrest_state = ACTUATOR_RETRACT;
      if(actuator == BACKREST_ACTUATOR) backrest_state = ACTUATOR_RETRACT;
      if(actuator == SEAT_ACTUATOR) seat_state = ACTUATOR_RETRACT;
    }
  }
  else if(action == ACTUATOR_STOP){
    digitalWrite(ina, LOW);
    digitalWrite(inb, LOW);

    if(actuator == FOOTREST_ACTUATOR) footrest_state = ACTUATOR_STOP;
    if(actuator == BACKREST_ACTUATOR) backrest_state = ACTUATOR_STOP;
    if(actuator == SEAT_ACTUATOR) seat_state = ACTUATOR_STOP;
  }
};

void correctSeatPosition(float pitch){
  /* This method is used to correct the seat's actuator position so the seat is always level. Limits can be set within
     the method's definition. Then an action for the actuator is decided depending on the relative value of the pitch, to the limits.
    Arguments:
      - float pitch: The value that corresponds to the pitch value of the MPU6050 sensor
    Returns: 
      - void
  */
  
  
  // This mathematical model might not be optimal, however keep the code commented in case it is needed later
  /*
   float h0 = 204.76;
  // float W = 60;
  // float d = 10;
  // float angle = pitch;
  // float speed  = 10.0 / 1000;
  // Serial.print(F("Pitch: "));
  // Serial.println(pitch);
  // float target_length = sqrt(pow(h0 - (W-d) * sin(angle), 2)+pow(d + (W-d) * (1 - cos(angle)), 2));
  // target_length = min(target_length, max_length);
  // ACTUATOR_ACTION action = (target_length < current_length) ? ACTUATOR_RETRACT : ACTUATOR_EXTEND;
  // if(target_length == current_length) action = ACTUATOR_STOP;
  // float travel_distance = abs(target_length - current_length);
  // float travel_time = travel_distance / speed;

  // setActuatorAction(0, action);
  // delay(travel_time);
  // setActuatorAction(0, ACTUATOR_STOP);
  // current_length = target_length;
  // Serial.println(target_length); 
  */
  float upper_incline_threshold = 5.0;
  float lower_incline_threshold = -5.0;

  float deg = pitch * 180/M_PI;
  if(deg > upper_incline_threshold){ setActuatorAction(SEAT_ACTUATOR, ACTUATOR_RETRACT); Serial.println("SEAT ACTUATOR RETRACTING");}
  else if(deg < lower_incline_threshold){ setActuatorAction(SEAT_ACTUATOR, ACTUATOR_EXTEND); Serial.println("SEAT ACTUATOR EXTENDING");}
  else {setActuatorAction(SEAT_ACTUATOR, ACTUATOR_STOP); Serial.println("SEAT ACTUATOR STOPPED");}
};

void process_twai_message(twai_message_t message){
  /* This method processes a TWAI message and set's the footrest and backrest's actions accordingly. More info on
     the message's context can be found on the documentation on GitHub.
    Arguments:
      - twai_message_t message: The TWAI message to be processed
    Returns:
      - void
  */
  if(message.identifier != TWAI_ID) return;
  uint8_t actuators_command = message.data[0];
  Serial.print(message.data[0]); //8 or 4: 1000 0100
  Serial.print(" ");
  Serial.println(message.data[1]);
  uint8_t footrest_command, backrest_command;
  footrest_command = actuators_command & 0b11; //mask footrest part of message
  actuators_command = actuators_command >> 2;
  backrest_command = actuators_command & 0b11;

  uint8_t seat_command = message.data[1];
  seat_command = seat_command & 0b11;

  if(footrest_command == 0b01){
    setActuatorAction(FOOTREST_ACTUATOR, ACTUATOR_EXTEND);
    Serial.println("Extending footrest");
  }else if(footrest_command == 0b10){
    setActuatorAction(FOOTREST_ACTUATOR, ACTUATOR_RETRACT);
    Serial.println("Retracting footrest");
  }else{
    setActuatorAction(FOOTREST_ACTUATOR, ACTUATOR_STOP);
    //Serial.println("Stopping footrest");
  }
  /*if(backrest_command == 0b01){
    setActuatorAction(BACKREST_ACTUATOR, ACTUATOR_EXTEND);
    Serial.println("Extending backrest");
  }else if(backrest_command == 0b10){
    setActuatorAction(BACKREST_ACTUATOR, ACTUATOR_RETRACT);
    Serial.println("Retracting backrest");
  }else{
    setActuatorAction(BACKREST_ACTUATOR, ACTUATOR_STOP);
    Serial.println("Stopping backrest");
  }*/
  //Problem with this: Multiple commands can be send in 1 message, but only the backrest will actually be processed

  //seat actuator stuff

  if(seat_command != 0){
    Serial.println("Moving seat");
  }

  if(seat_command == 0b01){
    //setActuatorAction(FOOTREST_ACTUATOR, ACTUATOR_EXTEND);
    //Serial.println("Extending footrest");
  }else if(seat_command == 0b10){
    setActuatorAction(SEAT_ACTUATOR, ACTUATOR_RETRACT);
    //Serial.println("Retracting footrest");
  }else{
    setActuatorAction(SEAT_ACTUATOR, ACTUATOR_STOP);
    //Serial.println("Stopping footrest");
  }
}

twai_message_t construct_transmitted_message(int id, int value){
  /* This method construct a TWAI message for transmission. The message's template is base on the official documentation from Espressif's website
  "https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/twai.html".
  Arguments:
    - int id: The identifier of the message.
    - int value: The value that we want to transmit
  Returns:
    - twai_message_t: The constructed message
  */
  twai_message_t constructed_message= {
    //For some reason, compiler is unable to initialize union
    /*
    .extd = 1,
    .rtr = 0,
    .ss = 0,
    .self = 0,
    .dlc_non_comp = 0,*/
    .flags = (1 << 0),
    .identifier = 99 + id,
    .data_length_code = 4,
    .data = {((int32_t)value >> 24) & 0xFF, ((int32_t)value >> 16) & 0xFF, ((int32_t)value >> 8) & 0xFF, ((int32_t)value) & 0xFF},
  };

  //ID 1 means battery 1 status, ID 2 means battery 2 status and ID 3 means temperature sensor status
  return constructed_message;
}

void getVoltages(float *bat1, float *bat2){
  int analogReadVal1 = analogRead(BAT1);
  int analogReadVal2 = analogRead(BAT2);

  //All calculations derived from measurements done on the voltage translator (plus offset seen on esp)
  float translatedValue1 = analogReadVal1*3.3f/4095+0.02f;
  float voltage_1 = translatedValue1*25/3.125;

  float translatedValue2 = analogReadVal2*3.3f/4095+0.08f;
  float voltage_2 = translatedValue2*51/3.09-voltage_1;

  Serial.print("Voltage: ");
  Serial.print(translatedValue1);
  Serial.print(", ");
  Serial.print(translatedValue2);
  Serial.print(" ");
  *bat1 = voltage_1;
  *bat2 = voltage_2;
}

uint8_t getBatteryCharge(float voltage){
  //Implements curve from tattu battery documentation
  if(voltage > 23.7){
    //upper linearized slope
    return 80+(int8_t)((voltage-23.7)/0.075f);
  }else if(voltage > 22.7){
    //middle linearized slope
    return 50+(int8_t)((voltage-22.7)/0.03333f);
  }else if(voltage > 21.8){
    //lower linearized slope
    return 10+(int8_t)((voltage-21.8)/0.0225f);
  }else{
    //forbidden zone
    return 0;
  }
}

void getBatteryCharges(uint8_t *charge1, uint8_t *charge2){
  float v1, v2;
  getVoltages(&v1, &v2);

  Serial.print("Voltages: ");
  Serial.print(v1);
  Serial.print(", ");
  Serial.print(v2);
  Serial.print(" ");

  //bat1
  *charge1 = getBatteryCharge(v1);
  *charge2 = getBatteryCharge(v2);
}

void setup() {
  //Initialize the pins and set them to LOW to avoid unexpected behavior
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(IN5, OUTPUT);
  pinMode(IN6, OUTPUT);

  pinMode(POTENTIOMETER_1, INPUT);
  pinMode(POTENTIOMETER_2, INPUT);

  //for battery charge reading
  pinMode(BAT1, INPUT);
  pinMode(BAT2, INPUT);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  digitalWrite(IN5, LOW);
  digitalWrite(IN6, LOW);
  
  //Join I2C bus
  Wire.begin();
  Wire.setClock(400000);

  //Initialize Serial Monitor
  Serial.begin(115200);
  while(!Serial);
  
  CircularQueue bat1_charges;
  CircularQueue bat2_charges;
  initializeQueue(&bat1_charges);
  initializeQueue(&bat2_charges);

  uint8_t c1_avg = 0;
  uint8_t c2_avg = 0;

  /*while(true){
    uint8_t c1, c2;
    getBatteryCharges(&c1, &c2);
    if(!isFull(&bat1_charges)){
      enqueue(&bat1_charges, c1);
      if(bat1_charges.size == 1){
        c1_avg = c1;
      }else{
        //c1_avg = c1_avg*(bat1_charges.size-1)/((float)bat1_charges.size)+c1/(float)bat1_charges.size;
        Serial.println(c1_avg);
      }
    }
    if(!isFull(&bat2_charges)){
      enqueue(&bat2_charges, c2);
      if(bat2_charges.size == 1){
        c2_avg = c2;
      }else{
        //c2_avg = c2_avg*(bat2_charges.size-1)/((float)bat2_charges.size)+c2/(float)bat2_charges.size;
        Serial.println(c2_avg);
      }
    }
    if(isFull(&bat1_charges) && isFull(&bat2_charges)){
      uint8_t old_1 = dequeue(&bat1_charges);
      uint8_t old_2 = dequeue(&bat2_charges);
      enqueue(&bat1_charges, c1);
      enqueue(&bat2_charges, c2);

      //c1_avg = (uint8_t)((float)c1_avg-old_1/20.0f+c1/20.0f);
      //c2_avg = (uint8_t)((float)c2_avg-old_2/20.0f+c2/20.0f);

      Serial.print("Charges: ");
      Serial.print(average(&bat1_charges));
      Serial.print(", ");
      Serial.println(average(&bat2_charges));
    }
  }*/

  //Initialize temperature sensor
  temp_sensor.begin();

  /* --- This section of code was taken from the "MPU6050_6Axis_MotionApps20.h" library example "MPU6050_DMP6" and initialized the MPU6050 ---*/
  //Initializing Device
  /*Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();*/
  pinMode(INTERRUPT_PIN, INPUT);

  //verify connection
  /*Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection()? F("MPU6050 connection succesful") : F("MPU6050 connection failed"));

  //Load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  //Set gyro offsets
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);*/

  //Make sure it worked
  if(devStatus == 0){
    // Calibration Time: generate offsets and calibrate our MPU6050
    /*mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();*/
  }
  else{
    Serial.print(F("DMP Initialization failed (code"));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
  /*-------------------------------------------------------------------------------------------*/

  //Initialize the TWAI driver and start the TWAI bus
  if(twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) Serial.println(F("TWAI driver installed successfully"));
  else Serial.println(F("Failed to install TWAI driver"));

  if(twai_start() == ESP_OK) Serial.println(F("TWAI driver started successfully"));
  else Serial.println(F("Failed to start TWAI driver"));

  //Init potis to not start in some crazy state
  left_potentiometer.reset_degree_offset();
  right_potentiometer.reset_degree_offset();
  rear_potentiometer.reset_degree_offset();
  Serial.println("Initialized potis");
  /*Serial.print("Left: ");
  Serial.print(left_potentiometer.get_assembly_angle());
  Serial.print("Right: ");
  Serial.print(right_potentiometer.get_assembly_angle());*/
}

void printBin8(uint8_t aByte) {
  for (int8_t aBit = 7; aBit > 0; aBit--)
    Serial.write(bitRead(aByte, aBit) ? '1' : '0');
  Serial.println(bitRead(aByte, 0) ? '1' : '0');
}

void printBin32(uint32_t aByte) {
  for (int8_t aBit = 31; aBit > 0; aBit--)
    Serial.write(bitRead(aByte, aBit) ? '1' : '0');
  Serial.println(bitRead(aByte, 0) ? '1' : '0');
}

uint16_t float32_to_float16(float f) {
    uint32_t f32 = *(uint32_t*)&f;  // Interpret float as a 32-bit unsigned integer

    uint32_t sign = (f32 >> 31) & 0x1;
    uint32_t exponent = (f32 >> 23) & 0xFF;
    uint32_t mantissa = f32 & 0x7FFFFF;

    // Handle special cases (NaN, Inf, Zero)
    if (exponent == 255) { // Inf or NaN
        if (mantissa == 0) {
            // Infinity
            return (sign << 15) | (0x1F << 10);
        } else {
            // NaN
            return (sign << 15) | (0x1F << 10) | (mantissa >> 13);
        }
    }

    if (exponent == 0) { // Zero or subnormal number
        return sign << 15; // Return signed zero
    }

    // Normalized number
    int16_t new_exponent = exponent - 127 + 15; // Rebase the exponent
    if (new_exponent >= 0x1F) { // Overflow, represent as infinity
        return (sign << 15) | (0x1F << 10);
    } else if (new_exponent <= 0) { // Underflow, represent as zero
        return sign << 15;
    }

    uint16_t new_mantissa = mantissa >> 13; // Truncate mantissa to 10 bits
    return (sign << 15) | (new_exponent << 10) | new_mantissa;
}

void loop() {
  if(dmpReady){
    //read the packet from FIFO
    if(mpu.dmpGetCurrentFIFOPacket(fifoBuffer)){
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      ypr = getYawRoll(q);
      //correctSeatPosition(ypr[1]); //Correct the seat's position based on it's incline. 
    }
  }

  //Read and display the temperature
  temp_sensor.requestTemperatures();
  float temp_celcius = temp_sensor.getTempCByIndex(0);
  /*Serial.print("Temperature: ");
  Serial.print(temp_celcius);
  Serial.println("°C");*/

  //Read the TWAI status to make sure that it runs OK. If not, troubleshoot and recover the TWAI bus if it is an a non-running state
  twai_status_info_t status_info;
  twai_get_status_info(&status_info);
  if(status_info.state != TWAI_STATE_RUNNING){
    Serial.print("TEC: ");
    Serial.print(status_info.tx_error_counter);
    Serial.print(" | REC: ");
    Serial.println(status_info.rx_error_counter);
    switch (status_info.state){
      case TWAI_STATE_STOPPED:
        Serial.println("ERROR CODE: TWAI_STATE_STOPPED");
        while(1);
      case TWAI_STATE_BUS_OFF:
        Serial.println("ERROR CODE: TWAI_STATE_BUS_OFF");
      case TWAI_STATE_RECOVERING: 
        Serial.println("ERROR CODE: TWAI_STATE_RECOVERING");
    }
    Serial.println(F("Initiating recovery"));
    //Avoiding recovery as a hotfix for the ERRATA error
    //twai_initiate_recovery();
    twai_driver_uninstall();
    twai_driver_install(&g_config, &t_config, &f_config);
    if(twai_start() == ESP_OK) Serial.println(F("TWAI driver started succesfully"));
    else Serial.println(F("Failed to start TWAI driver"));
  }

  //Read the messages in the TWAI bus and process accordingly
  esp_err_t receiveResult = twai_receive(&received_message, pdMS_TO_TICKS(40));
  if(receiveResult == ESP_OK){
    if(received_message.identifier == 99){
      //Serial.print(received_message.identifier);
      //Serial.print("-");
      //Serial.print(received_message.data_length_code);
      //Serial.print("-");
      //Serial.println(received_message.data[0]);
      /*Serial.print(received_message.data[1]);
      Serial.print(received_message.data[2]);
      Serial.println(received_message.data[3]);*/

      process_twai_message(received_message);
    }else if(received_message.identifier == 66){
      //reset offset message received
      left_potentiometer.reset_degree_offset();
      right_potentiometer.reset_degree_offset();
      rear_potentiometer.reset_degree_offset();
      Serial.println("Reset potis");
    }
    else{
      /*Serial.println(received_message.identifier);
      printBin32(received_message.identifier);
      printBin8(received_message.data[0]);*/
      //printBin8(received_message.data[1]);
      //printBin8(received_message.data[3]);
      //printBin8(received_message.data[4]);
    }

    //Send ready message
    twai_message_t transmitted_message = construct_transmitted_message(4, 4);
    if(twai_transmit(&transmitted_message, pdMS_TO_TICKS(10)) == ESP_OK){
      //Serial.println("Ready");
    }
  }else{
    //Serial.print("Could not receive TWAI message, error: ");
    //Serial.println(receiveResult);
    setActuatorAction(FOOTREST_ACTUATOR, ACTUATOR_STOP);
    setActuatorAction(BACKREST_ACTUATOR, ACTUATOR_STOP);

    /*twai_driver_uninstall();
    twai_driver_install(&g_config, &t_config, &f_config);
    if(twai_start() == ESP_OK) Serial.println(F("TWAI driver started succesfully"));
    else Serial.println(F("Failed to start TWAI driver"));*/
  }

  //Transmit TWAI messages for battery voltage levels and temperature. The messages get sent in a circular queue, every 200ms
  if(millis() - startingTime > 200){

    //transmit angles
    float l_angle = -left_potentiometer.get_assembly_angle(true); //Invert left angle to make both measure "forward"
    float r_angle = right_potentiometer.get_assembly_angle(true);
    float re_angle = rear_potentiometer.get_assembly_angle(false); //has no transmission, is 1:1
    Serial.print("Rear: ");
    Serial.println(re_angle);
    //Serial.println(right_potentiometer.get_angle_degrees());
    //Serial.println(right_potentiometer.get_voltage_level());
    //Serial.print("Right: ");
    //Serial.println(r_angle);

    unsigned short l_angle_16 = float32_to_float16(l_angle);
    unsigned short r_angle_16 = float32_to_float16(r_angle);
    unsigned short re_angle_16 = float32_to_float16(re_angle);

    //Old 4 byte message
    /*twai_message_t angle_message= {
      //For some reason, compiler is unable to initialize union
      /*
      .extd = 1,
      .rtr = 0,
      .ss = 0,
      .self = 0,
      .dlc_non_comp = 0,
      .flags = (1 << 0),
      .identifier = 99 + 7,
      .data_length_code = 4,
      .data = {(r_angle_16 >> 8) & 0xFF, (r_angle_16) & 0xFF, (l_angle_16 >> 8) & 0xFF, (l_angle_16) & 0xFF},
    };*/

    twai_message_t angle_message= {
      //For some reason, compiler is unable to initialize union
      .flags = (1 << 0),
      .identifier = 99 + 7,
      .data_length_code = 8,
      .data = {(r_angle_16 >> 8) & 0xFF, (r_angle_16) & 0xFF, (l_angle_16 >> 8) & 0xFF, (l_angle_16) & 0xFF, (re_angle_16 >> 8) & 0xFF, (re_angle_16) & 0xFF, 0, 0}, //Fill upper bytes later with battery data
    };


    if(twai_transmit(&angle_message, pdMS_TO_TICKS(10)) == ESP_OK){
      //Serial.println("Sent angles");
    }

    //Send ready message only if nothing else is to be sent
    /*twai_message_t transmitted_message = construct_transmitted_message(4, 4);
    if(twai_transmit(&transmitted_message, pdMS_TO_TICKS(10)) == ESP_OK){
      //Serial.println("Ready");
    }*/

    /*if(voltage1Transmit && !voltage2Transmit && !tempTransmit){
      twai_message_t transmitted_message = construct_transmitted_message(1, 20);
      if(twai_transmit(&transmitted_message, pdMS_TO_TICKS(10)) == ESP_OK){
        //Serial.println("Battery 1 status transmitted");
        voltage1Transmit = false;
        voltage2Transmit = true;
        tempTransmit = false;
      }
    }
    else if(!voltage1Transmit && voltage2Transmit && !tempTransmit){
      twai_message_t transmitted_message = construct_transmitted_message(2, 5);
      if(twai_transmit(&transmitted_message, pdMS_TO_TICKS(10)) == ESP_OK){
        //Serial.println("Battery 2 status transmitted");
        voltage1Transmit = false;
        voltage2Transmit = false;
        tempTransmit = true;
      }
    }
    else if(!voltage1Transmit && !voltage2Transmit && tempTransmit){
      twai_message_t transmitted_message = construct_transmitted_message(3, round(temp_celcius));
      if(twai_transmit(&transmitted_message, pdMS_TO_TICKS(10)) == ESP_OK){
        //Serial.println("Temperature status transmitted");
        voltage1Transmit = true;
        voltage2Transmit = false;
        tempTransmit = false;
      }
    }*/

    startingTime = millis();
  }

  updatePotis();
}

void updatePotis(){
  left_potentiometer.update();
  right_potentiometer.update();
  rear_potentiometer.update();
}