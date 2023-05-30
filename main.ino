#include "thingProperties.h"
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Arduino_PMIC.h>
#include <SD.h>
#include "algorithm.h"
#include "MAX30105.h"
#include <TinyGPS++.h>
#include <MPU6050_tockn.h>

// buffer_iot variable is used to store the IoT related data
int buffer_iot = 0;

// value_ and voltage, perc variables are used to store the battery level data
int value_ = 0;
float voltage, perc;
int battery_buffer = 0;
// float_map is used to map the voltage level to percentage level
long float_map(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// timerb, timera variables are used for timer related operations
float timerb, timera;

// chipSelect, name and myFile variables are used for SD reader operations
const int chipSelect = 5;
String name;
File myFile;
int sd_buffer = 0;

// test_rhr and rhr_sum, rhr_i variables are used for RHR (Resting Heart Rate) operations
int test_rhr = 0;
float rhr_sum = 0, rhr_i = 0;
int i_rhr=0;

// spo2_sum, spo2_i and spo2_comment_average are used for SPO2 (Blood Oxygen Saturation) operations
float spo2_sum = 0, spo2_i = 0, spo2_comment_average;

// aun_ir_buffer, aun_red_buffer, n_heart_rate, n_spo2, numSamples, count_tmp are used for SPO2, heart rate, and temperature operations
MAX30105 sensor;
float aun_ir_buffer[RFA_BUFFER_SIZE]; //
float aun_red_buffer[RFA_BUFFER_SIZE]; //
int32_t n_heart_rate;
float n_spo2;
int numSamples = 0, count_tmp = 0;
const int THRESHOLD = 3000;  //
static int32_t last_valid_heart_rate = 0; //
static float last_valid_spo2 = 0.0; //
#define THRESHOLD_PEAKS 2000 //
#define SAMPLE_RATE 100 //
const int AVERAGE_COUNT = 10; // 
float heartRateBuffer[AVERAGE_COUNT] = {0}; // 
uint8_t heartRateBufferIndex = 0; // 
const int AVERAGE_COUNT_VALID = 5; //
float validHeartRateBuffer[AVERAGE_COUNT_VALID] = {0}; // 
uint8_t heartRateBufferIndexValid = 0; // 
#define FINAL_HR_BUFFER_SIZE 500 // 
#define FINAL_SPO2_BUFFER_SIZE 500 //
float finalHRBuffer[FINAL_HR_BUFFER_SIZE] = {0}; //
uint16_t finalHRBufferIndex = 0; //
uint16_t finalHRCount = 0; //
unsigned long startTime = 0; //
float finalSpO2Buffer[FINAL_SPO2_BUFFER_SIZE]; //
uint8_t finalSpO2BufferIndex = 0; //
uint16_t finalSpO2Count = 0; //
int ss=0; ////


// GPSserial, GPSBaud and gps are used for GPS related operations
#define GPSserial Serial1
static const uint32_t GPSBaud = 9600;
TinyGPSPlus gps;
int gps_buffer = 0;
int gps_n = 0;
int speed_counter = 0;
int speed_max = 0;
int speed_max_determine = 0;
float lat=0;
float lng=0;

// steps, distanceinonestep, distance, timer and acc are used for MPU-6050 related operations
int steps = 0;
float distanceinonestep = 71, distance = 0;
long timer = 0;
double acc;
MPU6050 mpu6050(Wire);

// measurment_buffer is used for measurement related data
int measurment_buffer = 0;

// hr_pres and vo2_pres are used for VO2 (Oxygen Consumption) operations
float hr_pres = 0, vo2_pres = 0;

void setup()
{
  Serial.begin(115200);
  Wire.begin(); //

  //IOT initialization
  initProperties();
  ArduinoCloud.begin(ArduinoIoTPreferredConnection);

  //SPO2_HR_TEMP initialization
  sensor.begin(Wire, I2C_SPEED_FAST);
  //sensor.setup(0x7F, 4, 2, 200, 411, 16384);
  sensor.setup(0x7F, 8, 2, 800, 411, 16384); //
  sensor.getINT1();
  sensor.getINT2();
  
 
    //MPU-6050 initialization
  mpu6050.begin();
    mpu6050.calcGyroOffsets();

    //GPS initialization
    GPSserial.begin(GPSBaud);

    //SD Reader initialization
    SD.begin(chipSelect);
 

}

// IOT update loop
void loop()
{
  if (buffer_iot == 10000)
  {
    // Check if the buffer for IOT updates reaches 10,000/
    // If it does, perform an update on the Arduino cloud/
    ArduinoCloud.update();
    // Reset the buffer for IOT updates
    buffer_iot = 0;
  }

  // Operations for when device is turned off
  if (onoff_x == false)
  {
    // Record the start time
    timerb = millis();
    // Update for total distance and steps graphs/ (done)
    totaldistance_x = distance / 1000;
    totalsteps_x = steps;
    // Update for average speed and max speed graphs/
    speedavg_x = speed_counter / gps_n;
    speedmax_x = speed_max;
    speed_counter = 0;
    gps_n = 0;
    startTime=0;
  }

  // Operations for when device is turned on
  if (onoff_x == true)
  {
    // Record elapsed time since device was turned on (done)
    timera = (millis() - timerb);
   // startTime=timera; //
    
    // Convert elapsed time to seconds and minutes (done)
    timers_x = timera / 1000;
    timerm_x = timers_x / 60;

    // Assign file name for SD Reader (done)
    name = sd_x;

    // Perform measurement calculations once (done)
    if (measurment_buffer == 0)
    {
      bmi_x = (weight_x) / (pow(height_x / 100, 2));

      if (sex_x == "male")
      {
        bmr_x = (10 * weight_x) + (6.25 * height_x) - (5 * age_x) + 5;
      }
      else
      {
        bmr_x = (10 * weight_x) + (6.25 * height_x) - (5 * age_x) - 161;
      }

      mhr_x = 220 - age_x;
      measurment_buffer++;
    }

    // Perform battery level calculation (done)
      battery_level();

    // Perform SPO2, HR, and temp measurement/ (done)
    SPO2_HR_TEMP_measurment();

    // Perform steps and distance measurements (done)
    mpu6050_measurement();

    //GPS measurments/ (done)
        while (GPSserial.available() > 0)
        if (gps.encode(GPSserial.read()))
            displayInfo();

    //Write data to SD card (done)
        sD_Reader_measurment();
  }

  // Increment IOT buffer
  buffer_iot++;

}

// Check battery level every second
void battery_level()
{
  if (millis() - battery_buffer >= 2000)
  {
    // Read analog voltage value of battery and convert it to percentage
    value_ = analogRead(ADC_BATTERY);
    perc = float_map(value_ * 4.3 / 1023.0, 3.0, 4.2, 0, 100);
    batterylevel_x = perc;
    battery_buffer = millis();  // Update buffer timestamp
  }
}

//declare function SPO2_HR_TEMP_measurement
void SPO2_HR_TEMP_measurment()
{
  //declare local variables for SPO2, heart rate, and temperature measurement
  float ratio, correl;
  int8_t ch_spo2_valid;
  int8_t ch_hr_valid;
  bool fingerDetected = false; // 
  static uint16_t samplesAdded = 0; //
  static uint16_t samplesAddedValid = 0; //
  static float previousWeightedHeartRate = 60.0; //

  //check sensor status
  sensor.check();

  //loop while sensor data is available
  while (sensor.available())
  {
    //store IR and red values in buffer arrays
    aun_red_buffer[numSamples] = sensor.getFIFOIR();
    aun_ir_buffer[numSamples] = sensor.getFIFORed();
    
    // Check if the index finger is there or not //
    if (aun_red_buffer[numSamples] > THRESHOLD && aun_ir_buffer[numSamples] > THRESHOLD) { //
      fingerDetected = true; //
    } else { //
      fingerDetected = false; //
    } //

    //increment number of samples and move to next sample
    numSamples++;
    sensor.nextSample();

    //if number of samples reaches buffer size
    if (numSamples == RFA_BUFFER_SIZE)
    {
      //increment counter for temperature measurement
      count_tmp++;
      
//
      if(fingerDetected) {
      //calculate SPO2 and heart rate
      heart_rate_and_oxygen_saturation(aun_ir_buffer, RFA_BUFFER_SIZE, aun_red_buffer, &n_spo2, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid, &ratio, &correl);
      
      
        finalSpO2Buffer[finalSpO2BufferIndex] = n_spo2; 
        finalSpO2BufferIndex = (finalSpO2BufferIndex + 1) % FINAL_SPO2_BUFFER_SIZE;
        finalSpO2Count++;
        float currentHRValue;
        if (ch_hr_valid) {
          // Store the heart rate value in the buffer
          
          
          
          if (samplesAddedValid > 0) { //Reset the heartRateBuffer if HR valid found
            float averageHeartRate = computeMovingAverage(heartRateBuffer, AVERAGE_COUNT, samplesAdded);
            float heart_rate_valid;
            float difference;
            if(samplesAdded > 9){
              heart_rate_valid = n_heart_rate*0.5 + previousWeightedHeartRate*0.3 + averageHeartRate*0.2; // current 0.5, previous 0.3 , average previous 0.2
              difference = heart_rate_valid - previousWeightedHeartRate;
            }
            else {
              heart_rate_valid = n_heart_rate*0.7 + previousWeightedHeartRate*0.15 + averageHeartRate*0.15; // current 0.7, previous 0.15 , average previous 0.15
              difference = heart_rate_valid - previousWeightedHeartRate;
            }
        
            // Serial.print("Difference : ");
            // Serial.println(difference);
            if (difference > 20.0) {
              heart_rate_valid = previousWeightedHeartRate + 20.0;
            } else if (difference < -20.0) {
              heart_rate_valid = previousWeightedHeartRate - 20.0;
            }
            currentHRValue = heart_rate_valid;

            previousWeightedHeartRate = heart_rate_valid;
            validHeartRateBuffer[heartRateBufferIndexValid] = heart_rate_valid;
          } else {
            validHeartRateBuffer[heartRateBufferIndexValid] = n_heart_rate;
            currentHRValue = n_heart_rate;
          }
          heartRateBufferIndexValid = (heartRateBufferIndexValid + 1) % AVERAGE_COUNT_VALID;
          samplesAddedValid++;
          
        } 
        else {
          if (samplesAddedValid > 0) {
        
            n_heart_rate = calculateHeartRateZeroCrossing(aun_ir_buffer, RFA_BUFFER_SIZE, SAMPLE_RATE)*0.75;
            float averageHeartRateValid = computeMovingAverage(validHeartRateBuffer, AVERAGE_COUNT_VALID, samplesAddedValid);
            
            float averageHeartRate = computeMovingAverage(heartRateBuffer, AVERAGE_COUNT, samplesAdded);
            
            float weightedHeartRate = averageHeartRateValid * 0.5 + averageHeartRate* 0.3 + n_heart_rate*0.2; // average_valid 0.5,average_nonpeak_algo 0.3, current 0.2

        
            
            float difference = weightedHeartRate - previousWeightedHeartRate;
            // Serial.print("Difference : ");
            // Serial.println(difference);
            if (difference > 40.0) {
              weightedHeartRate = previousWeightedHeartRate + 40.0;
            } else if (difference < -40.0) {
              weightedHeartRate = previousWeightedHeartRate - 40.0;
            }

            heartRateBuffer[heartRateBufferIndex] = weightedHeartRate;
            heartRateBufferIndex = (heartRateBufferIndex + 1) % AVERAGE_COUNT;
            samplesAdded++;

            // Update the previous weightedHeartRate
            previousWeightedHeartRate = weightedHeartRate;
            currentHRValue = weightedHeartRate;
          } else {
            n_heart_rate = calculateHeartRateZeroCrossing(aun_ir_buffer, RFA_BUFFER_SIZE, SAMPLE_RATE)*0.75;
            // Store the heart rate value in the buffer
            heartRateBuffer[heartRateBufferIndex] = n_heart_rate;
            heartRateBufferIndex = (heartRateBufferIndex + 1) % AVERAGE_COUNT;
            samplesAdded++;

            // Compute the moving average of the heart rate values
            float averageHeartRate = computeMovingAverage(heartRateBuffer, AVERAGE_COUNT, samplesAdded);
            currentHRValue = averageHeartRate;
          }
        }
        // Store the current heart rate value in the finalHRBuffer
        finalHRBuffer[finalHRBufferIndex] = currentHRValue;
        finalHRBufferIndex = (finalHRBufferIndex + 1) % FINAL_HR_BUFFER_SIZE;
        finalHRCount++;

        // Check if 15 seconds have passed
     //   if (millis() - startTime >= 15000) {
      if (timera - startTime >= 15000) {
        
          
          
          float averageFinalSpO2 = computeMovingAverage(finalSpO2Buffer, FINAL_SPO2_BUFFER_SIZE, finalSpO2Count);
          spo2_x=averageFinalSpO2;
        ////  spo2_sum += averageFinalSpO2;
         //// spo2_i++;
          spo2_comment();



          // Calculate the average heart rate for the last 15 seconds
          float averageFinalHR = computeMovingAverage(finalHRBuffer, FINAL_HR_BUFFER_SIZE, finalHRCount);
          hr_x =averageFinalHR;
          if(i_rhr>1){
        rhr_sum += averageFinalHR;
        rhr_i++;
          }
          i_rhr++;
        heartrate_comment();
        VO2_current();
      
        

          float temperature = sensor.readTemperature();
          temp_x = temperature;
          temp_comment();
    

          // Reset the finalHRBuffer and finalSpO2Buffer index and count
          Serial.println("");
          finalHRBufferIndex = 0;
          finalHRCount = 0;
          finalSpO2BufferIndex = 0;
          finalSpO2Count = 0;
          //startTime = millis();
          startTime = timera; ////
         
        }
      }
      
//

//
else { 
       // Serial.println("Index finger not detected.");
        samplesAdded = 0;
        samplesAddedValid = 0;
        heartRateBufferIndex = 0;
        heartRateBufferIndexValid = 0;
        previousWeightedHeartRate = 60.0;
        finalHRBufferIndex = 0;
        finalHRCount = 0;
        finalSpO2BufferIndex = 0;
        finalSpO2Count = 0;
      }
//

numSamples = 0; //

//calculate resting heart rate after 120 seconds
    if (timers_x >= 70 && test_rhr == 0)
    {
      rhr_x = rhr_sum / rhr_i;
      vo2max_x = 15 * (mhr_x / rhr_x);
      test_rhr++;
    }


/*
      //print heart rate and check validity
      Serial.print(", Pulse ");
      if (ch_hr_valid)
      {
        Serial.print(n_heart_rate);
        hr_x = n_heart_rate;
        Serial.println();
        //sum for averaging heart rate for resting heart rate
        rhr_sum += n_heart_rate;
        rhr_i++;
      }

      //print SPO2 and check validity
      Serial.print("SP02 ");
      if (ch_spo2_valid)
      {
        Serial.print(n_spo2);
        spo2_x = n_spo2;
        Serial.println();
        //sum for averaging SPO2
        spo2_sum += n_spo2;
        spo2_i++;
      }

      //call functions for comments on SPO2 and heart rate
      spo2_comment();
      heartrate_comment();

      //Call function to measure current VO2
      VO2_current();

      //check if it's time for temperature measurement
      if (count_tmp == 12)
      {
        //read temperature and call function for comment
        temp_x = sensor.readTemperature();
        
        count_tmp = 0;
      }

      //reset number of samples
      numSamples = 0;
    }

    //calculate resting heart rate after 60 seconds
    if (timers_x >= 60 && test_rhr == 0)
    {
      rhr_x = rhr_sum / rhr_i;
      vo2max_x = 15 * (mhr_x / rhr_x);
      test_rhr++;
    }
    */
    
    
  }
}
}


//
float calculateHeartRateZeroCrossing(float *irBuffer, uint16_t bufferSize, float sampleRate) {
  uint16_t zeroCrossingCount = 0;
  uint32_t lastCrossingIndex = 0;

  // Calculate the mean of the IR buffer
  float mean = 0.0;
  for (uint16_t i = 0; i < bufferSize; i++) {
    mean += irBuffer[i];
  }
  mean /= bufferSize;

  // Count zero crossings
  for (uint16_t i = 1; i < bufferSize; i++) {
    if ((irBuffer[i] > mean && irBuffer[i - 1] <= mean) || (irBuffer[i] < mean && irBuffer[i - 1] >= mean)) {
      if (lastCrossingIndex > 0) {
        zeroCrossingCount++;
      }
      lastCrossingIndex = i;
    }
  }

  // Calculate the heart rate based on the number of zero crossings
  float heartRate = 60.0 * zeroCrossingCount * sampleRate / (2 * bufferSize);
  return heartRate;
}
//

//
float computeMovingAverage(float *buffer, uint16_t bufferSize, uint16_t numSamples) {
  float sum = 0.0;
  uint16_t divisor = numSamples < bufferSize ? numSamples : bufferSize;

  for (uint16_t i = 0; i < divisor; i++) {
    sum += buffer[i];
  }
  return sum / divisor;
}
//




void VO2_current()
{
  // Check if the time passed is greater than or equal to 70
  if (timers_x >= 95)
  {
    // Calculate the current heart rate percentage
    hr_pres = ((hr_x) / (mhr_x)) * 100;
    // Calculate the current VO2 percentage
    vo2_pres = (hr_pres - 37) / (0.64);
    // Calculate the current VO2
    vo2_x = abs((vo2_pres / 100) * vo2max_x);
  }
}

void heartrate_comment()
{
  //if the selected training type is endurance and timer is >= 60

  if (choice_x == "endurance" && timers_x >= 60)
  {
    //warm up (when timer <= 5.0)
    if (timerm_x <= 5.0)
    {
      phase_x = "Warm up";
      
      if(hr_x>0.90*mhr_x){
        led_hr_comment="Warning! Heart rate extremely high";
        led_hr_decision = "Stop exercise and take rest";
        led_hr_red = true;  //set green LED off
        led_hr_green = false; //set red LED on
      }
      
      if (hr_x >= (0.50 * mhr_x) && hr_x <= (0.65 * mhr_x))
      {
        led_hr_green = true;  //set green LED on
        led_hr_red = false; //set red LED off
        led_hr_comment = "Heart rate within normal range";  //comment
        led_hr_decision = "Keep going!";
      }
      
      else if (hr_x < (0.50 * mhr_x))
      {
        led_hr_green = true;  //set green LED on
        led_hr_red = true;  //set red LED on
        led_hr_comment = "Heart rate below normal range"; //comment
        led_hr_decision = "Increase intensity for effective warm-up";
      }
      
      
      else
      {
        led_hr_green = false; //set green LED off
        led_hr_red = true;  //set red LED on
        led_hr_comment = "Heart rate above normal range"; //comment
        led_hr_decision = "Decrease intensity for effective warm-up";
      }
    }

    //end of warm up

    //start of endurance training (when timer > 5.0)
    if (timerm_x > 5.0)
    {
      if(hr_x>0.90*mhr_x){
        led_hr_comment="Warning! Heart rate extremely high";
        led_hr_decision = "Stop exercise and take rest";
        led_hr_red = true;  //set green LED off
        led_hr_green = false; //set red LED on
      }
      
      phase_x = "Training";
      
      if (hr_x >= (0.70 * mhr_x) && hr_x <= (0.75 * mhr_x))
      {
        led_hr_green = true;  //set green LED on
        led_hr_red = false; //set red LED off
        led_hr_comment = "Heart rate within normal range";  //comment
        led_hr_decision = "Continue for some time and slow rhythm for recovery";
      }
      else if (hr_x >= (0.65 * mhr_x) && hr_x < (0.70 * mhr_x))
      {
        led_hr_green = true;  //set green LED on
        led_hr_red = true;  //set red LED on
        led_hr_comment = "Heart rate within recovery phase";  //comment
        led_hr_decision = "Continue for some time and increase rhythm for endurance";
      }
      else
      {
        led_hr_green = false; //set green LED off
        led_hr_red = true;  //set red LED on
        led_hr_comment = "Heart rate out normal range"; //comment
        led_hr_decision = "Slow/increase rhythm";
      }
    }
  }

  //end of endurance training

  //if the selected training type is stamina and timer is >= 45
  if (choice_x == "stamina" && timers_x >= 45)
  {
    //warm up (when timer <= 5.0)
    if (timerm_x <= 5.0)
    {
      phase_x = "Warm up";
      
      if(hr_x>0.90*mhr_x){
        led_hr_comment="Warning! Heart rate extremely high";
        led_hr_decision = "Stop exercise and take rest";
        led_hr_red = true;  //set green LED off
        led_hr_green = false; //set red LED on
      }
      
      if (hr_x >= (0.50 * mhr_x) && hr_x <= (0.65 * mhr_x))
      {
        led_hr_green = true;  //set green LED on
        led_hr_red = false; //set red LED off
        led_hr_comment = "Heart rate within normal range";  //comment
        led_hr_decision = "Keep going!";
      }
      else if (hr_x < (0.50 * mhr_x))
      {
         led_hr_green = true; //set green LED on
        led_hr_red = true;  //set red LED on
        led_hr_comment = "Heart rate below normal range"; //comment
        led_hr_decision = "Increase intensity for effective warm-up";     
      }
      else
      {
      led_hr_green = false; //set green LED off
        led_hr_red = true;  //set red LED on
        led_hr_comment = "Heart rate above normal range"; //comment
        led_hr_decision = "Decrease intensity for effective warm-up";
      }
    }

    //end of warm up

    //Start of stamina traning (When timer > 5.0)
    if (timerm_x > 5.0)
    {
      phase_x = "Training";
      
      if(hr_x>0.90*mhr_x){
   led_hr_comment="Warning! Heart rate extremely high";
        led_hr_decision = "Stop exercise and take rest";
        led_hr_red = true;  //set green LED off
        led_hr_green = false; //set red LED on
        
      }
      
      
      if (hr_x >= (0.75 * mhr_x) && hr_x <= (0.85 * mhr_x))
      {
        led_hr_green = true;  //set green LED on
        led_hr_red = false; //set red LED off
        led_hr_comment = "Heart rate within normal range";  //comment
      led_hr_decision="Continue for some time up incline and go down for recovery";
        
      }
      else if (hr_x >= (0.70 * mhr_x) && hr_x < (0.75 * mhr_x))
      {
        led_hr_green = true;  //set green LED on
        led_hr_red = true;  //set red LED on
        led_hr_comment = "Heart rate within recovery phase";  //comment
     led_hr_decision = "Continue for some time down incline and go up for stamina";
      }
      else
      {
        led_hr_green = false; //set green LED off
        led_hr_red = true;  //set red LED on
        led_hr_comment = "Heart rate out normal range"; //comment
     led_hr_decision = "Slow/increase rhythm";
      }
    }

    //end of Stamina training
  }
}

void temp_comment()
{
  //Check if the temperature is within the range of 28°C to 37°C
  if (temp_x >= 28 && temp_x <= 37)
  {
    led_temp_green = true;  //set green LED on
    led_temp_red = false; //set red LED off
    led_temp_comment = "Temp within normal range";  //comment
 led_temp_decision = "Keep going!";
  }

  //If temperature is not within the range
  else
  {
    led_temp_green = false; //set green LED off
    led_temp_red = true;  //set red LED on
    led_temp_comment = "Temp out of normal range";  //comment
 led_temp_decision = "Stop exercise and take rest in well-ventilated area";
  }
}

void spo2_comment()
{
  //check if the counter of spo2 readings reached 12
 //// if (spo2_i == 12)
 //// {
    //calculate the average of spo2 readings
 ////   spo2_comment_average = spo2_sum / spo2_i;
    //assign the average value to spo2_comment_x
    spo2_comment_x = spo2_x;
    //check if spo2_comment_x is between 95 and 100
    if (spo2_comment_x >= 95 && spo2_comment_x <= 100)
    {
      led_spo2_green = true;  //turn green LED on
      led_spo2_red = false; //turn red LED off
      led_spo2_comment = "SpO2 within normal range";  //comment
   led_spo2_decision= "Keep going!";
    }

    //check if spo2_comment_x is between 92 and 95
    else if (spo2_comment_x >= 92 && spo2_comment_x < 95)
    {
      led_spo2_green = true;  //turn green LED on
      led_spo2_red = true;  //turn red LED on
      led_spo2_comment = "SpO2 below normal range"; //comment
   led_spo2_decision= "Try breathing from nose";
    }

    //if neither of the conditions above are met
    else
    {
      led_spo2_green = false; //turn green LED off
      led_spo2_red = true;  //turn red LED on
      led_spo2_comment = "Warning! SpO2 exceedingly below normal range";  //comment
    led_spo2_decision= "Stop exercise and take rest";
    }

    //reset the counter and sum of spo2 readings
   // spo2_i = 0;
  ////  spo2_sum = 0;
 //// }
}

void mpu6050_measurement()
{
  //check if 1000 milliseconds have passed since last measurement
  if (millis() - timer >= 1000)
  {
    //update the mpu-6050 to get latest data
    mpu6050.update();

    //check if acceleration in Y direction is greater than 1
    if (abs(mpu6050.getAccY()) > 0.80)
    {
      //increment the number of steps
      steps += 1;
    }

    steps_x = steps;
    //calculate distance traveled using number of steps
    distance = steps * distanceinonestep / 100;
    distance_x = distance / 1000;
    //update the timer to current time
    timer = millis();
  }
}

//GPS measurment

void displayInfo()
{
  if (millis() - gps_buffer >= 5000)
  {
//Serial.print(F("  Date/Time: "));
 // if (gps.date.isValid())
//  {
 //   Serial.print(gps.date.month());
 //   Serial.print(F("/"));
 //   Serial.print(gps.date.day());
 //   Serial.print(F("/"));
  //  Serial.print(gps.date.year());
 // }
    
  //  Serial.print(F("Location: "));
    if (gps.location.isValid())
    {
      
      lat=gps.location.lat();
     lng=gps.location.lng();
    //   Serial.print(gps.location.lat(), 6);
   //    Serial.print(F(","));
  //     Serial.print(gps.location.lng(), 6);
      map_x = {lat,lng};
    }

    //  Serial.print(F(" Speed(km/h): "));
    if (gps.speed.isValid())
    {
      //   Serial.print(gps.speed.kmph(), 2);
      speed_x = gps.speed.kmph();
      speed_counter += speed_x;
      gps_n++;
      speed_max = max(speed_x, speed_max_determine);
      speed_max_determine = speed_x;
    }

    // Serial.println();
    gps_buffer = millis();
    
  }
}

//SD Card measurment

void sD_Reader_measurment()
{
  // Check if a second has passed since the last measurement
  if (millis() - sd_buffer >= 5000)
  {
    // Open a file for writing and store it in "myFile"
    myFile = SD.open(name, FILE_WRITE);
    // Check if the file was successfully opened
    if (myFile)
    {
      // Write variables to the file
      myFile.print("Time(s): ");
      myFile.print(timers_x);
      myFile.print("\tHR: ");
      myFile.print(hr_x);
      myFile.print("\tSPO2: ");
      myFile.print(spo2_x);
      myFile.print("\tTemp: ");
      myFile.print(temp_x);
      myFile.print("\tSpeed: ");
      myFile.print(speed_x);
      myFile.print("\tSteps: ");
      myFile.print(steps_x);
      myFile.print("\tDistance: ");
      myFile.print(distance_x);
      myFile.print("\tLatitude, Longitude: ");
      myFile.print(lat);
      myFile.print(", ");
      myFile.println(lng);
      // Close the file
      myFile.close();
    }

    // Update the buffer to current time
    sd_buffer = millis();
  }
}






//IOT READ-WRITE

void onOnoffXChange() {

}

void onNameXChange() {

}

void onSexXChange() {

}

void onAgeXChange() {

}

void onHeightXChange() {

}

void onWeightXChange() {

}

void onBmiXChange() {

}

void onBmrXChange() {

}

void onMhrXChange() {

}

void onProfileXChange() {

}

void onChoiceXChange() {

}

void onSdXChange() {

}
