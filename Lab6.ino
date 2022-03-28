//**************************//
// Skeleton code of LAB 6   //
//**************************//
// Task 1
// Description:
//  - Using software serial (UART) library to communicate with GY-25Z
//  - Get 3-Axis acceleration data and Euler Angles
// Connection:
//  (Arduino) D10 - (GY-25Z) TX
//  (Arduino) D11 - (GY-25Z) RX
//===============================================
// Task 2
// Description:
// (a) Apply the filter to remove the short-term fluctuations
//  - Low-pass filter (LPF) with weight value
//  - Shake the breadboard and observe the results by Serial Plotter
// (b) Apply the moving average filter
//  - Create an array to hold 8 acceleration data
//  - Replace the oldest data by the latest data
//  - Calculate the average value of the whole buffer
//  - Compare the result of original ACC_Z, LPF result (weight = 0.1) and moving average result.
//===============================================
// Task 3
// Description:
//  - Use GY-25Z as motion gesture control of a car
//  - Place GY-25Z at the center of the breadboard
//  - Reset the Euler angles to zero by calibration command in setup()
//  - Use the acceleration data of Z-axis (after moving average filter) to determine the acceleration of a car:
//    - the sensitivity of the acceleration control is defined as threshold
//    - when ACC_Z_AVG > (+ve) threshold, moving forward
//    - when ACC_Z_AVG < (-ve) threshold, moving backward
//    - else the car is stopped
//  - Use the Yaw angles to determine the steering of a car, the max steering angle is 90 degree
//    (i) when Z-axis acceleration > 0, GY-25Z face upward
//        - turn left if 0 < yaw_angle < 90
//        - turn right if -90 < yaw_angle < 0
//    (ii) when Z-axis acceleration < 0, GY-25Z face downward
//        - turn left if 90 < yaw_angle < 180
//        - turn right if -180 < yaw_angle < -90
//===============================================

#include <SoftwareSerial.h>
SoftwareSerial mySerial(10, 11);    // PIN10 as RX of Arduino; PIN11 as TX of Arduino;
int ACC[3];                         // Acceleration of X-, Y- and Z-Axis
int YPR[3];                         // Euler Angles
float ACC_X, ACC_Y, ACC_Z;
float ACC_Z_LPF = 0.0;              // Low-pass filter result
float ACC_Z_array[8];               // Array of 8 Z-Axis acceleration data
float ACC_Z_AVG;                    // Moving average result

unsigned char receive_buf[30];      // receive buffer from GY-25Z
unsigned char counter = 0;          // counter of received buffer
boolean packet_rdy = false;

//-----------------------------------------------------------
void setup()
{
  Serial.begin(115200);     // initial serial monitor at baud rate 115200
  mySerial.begin(115200);   // initial serial communication with GY-25Z at baud rate 115200
  mySerial.listen();        // listen to mySerial
  delay(3000);              // wait 3 sec before send command to GY-25Z

  mySerial.write(0XA5);         // Used in task 3
  mySerial.write(0X57);         // Calibrate the Euler angles (0xA5, 0x57, 0x02)
  mySerial.write(0X02);         // Euler angles reset to zero
  mySerial.write(0XFE);
  delay(100);
  
  mySerial.write(0XA5); 
  mySerial.write(0X55);    
  mySerial.write(0x11);         // According to spec, set GY-25Z output Acceleration data and Euler Angles
  mySerial.write(0x0B);         // checksum of this command (0xA5, 0x55, 0x11)
  delay(100);
 
  mySerial.write(0XA5); 
  mySerial.write(0X56);
  mySerial.write(0X02);         // Set GY-25Z as output data automatically
  mySerial.write(0XFD);
  delay(100);
}
//-------------------------------------------------------------
void loop(){
  while(mySerial.available())  // when data recevied from mySerial
  {   
    receive_buf[counter] = (unsigned char)mySerial.read();    // put the received data into buffer
    if(counter == 0 && receive_buf[0] != 0x5A)                // check the 1st byte of packet if it is 0x5A
      return;        
    counter++;
    if(counter == 17)               // total bytes of data packet should be 17
    {    
       counter = 0;                 // reset the counter for next packet
       packet_rdy = true;
    }       
  }

  // Received packet of Euler Angles from GY-25Z:
  //  _________________________________________________________________________________________________________________________________________________________________
  //  |  Header | type | bytes| acceleration of x-axis | acceleration of y-axis | acceleration of z-axis |       Roll     |      Pitch     |       YAW      | Checksum |
  //  |0x5A|0x5A| 0x11 | 0x12 |ACC_X[15:8] | ACC_X[7:0]|ACC_Y[15:8] | ACC_Y[7:0]|ACC_Z[15:8] | ACC_Z[7:0]|R[15:8] | R[7:0]|P[15:8] | P[7:0]|Y[15:8] | Y[7:0]| CHECKSUM |
  //  |____|____|______|______|____________|___________|____________|___________|____________|___________|________|_______|________|_______|________|_______|__________|
  //  | [0]| [1]|  [2] |  [3] |     [4]    |    [5]    |     [6]    |    [7]    |     [8]    |    [9]    |  [10]  |  [11] |  [12]  |  [13] |  [14]  |  [15] |   [16]   | 
  //  -----------------------------------------------------------------------------------------------------------------------------------------------------------------
  if(packet_rdy)
  { 
    if(receive_buf[0] == 0x5A && receive_buf[1] == 0x5A)        // check the header: 0x5A, 0x5A
    {
      //==========================================================================
      // Task 1: Get the 3-axis acceleration data and Euler angles
      //==========================================================================
      if (checksum(receive_buf, 16) == receive_buf[16]){        // check the checksum to eliminate the communication error
        // combine the 16 bits acceleration data received from the packet
        ACC[0] = ;        // acceleration of X-Axis
        ACC[1] = ;        // acceleration of Y-Axis
        ACC[2] = ;        // acceleration of Z-Axis

        // calculate the acceleration data in g = ACC / (32767/2) = ACC / 16383.5
        ACC_X = ;
        ACC_Y = ;
        ACC_Z = ;
       
        // divide the data by 100 as the data included 2 digits after decimal point
        YPR[0] = ;      // Roll
        YPR[1] = ;      // Pitch
        YPR[2] = ;      // Yaw
      
        // print out the acceleration in g of x-, y- and z-axis
        // Hints for Serial Plotter: print the data of each sampling in the same line and separate by '\t'
        // comment the print out statements after finished task 1
        Serial.print(ACC_X); Serial.print("\t");          // print acceleration at X-axis
        Serial.print(ACC_Y); Serial.print("\t");          // print acceleration at Y-axis
        Serial.println(ACC_Z);                            // print acceleration at Z-axis

        //==========================================================================
        // Task 2(a): Apply low-pass filter to Z-axis acceleration data
        //            by equation = (1.0 - weight) * previous_lpf_data + weight * current_data
        //            - try weight = 0.9  (90% of current value)
        //            - by Serial Plotter, compare the results before and after low-pass filter
        //            - try weight = 0.1  (10% 0f current value)
        //            - by Serial Plotter, compare the results before and after low-pass filter
        //==========================================================================
//        float weight = ;                               // how much your values are 'smoothed'

//        ACC_Z_LPF = ;   // from the equation

        // comment the print out statements after finished task 2(a)
//        Serial.print(ACC_Z); Serial.print("\t");          // print acceleration at Z-axis
//        Serial.println(ACC_Z_LPF);                        // print acceleration at Z-axis after LPF
        
        //==========================================================================
        // Task 2(b): Apply moving average filter to Z-axis acceleration data
        //            - create a global variable array to store 8 acceleration data
        //            - on each pass of loop, shift the array to vacate the oldest data
        //            - insert the latest data into array
        //            - calculate the average value of the array buffer
        //            - by Serial Plotter, compare the original data, LPF (in task 2(a)) and moving average result
        //==========================================================================
        // assume the oldest data in index 0 of the array
        // shift the array to left: array[i] = array[i+1]
//        for(int i = 0; i < 7; i++){
//          ACC_Z_array[i] = ;
//        }
        // update the latest data to index 7 of the array
        

        // calculate the average value of the array


        // comment the print out statements after finished task 2(b)
//        Serial.print(ACC_Z); Serial.print("\t");          // print acceleration at Z-axis
//        Serial.print(ACC_Z_LPF); Serial.print("\t");      // print acceleration at Z-axis after LPF (weight = 0.1)
//        Serial.println(ACC_Z_AVG);                        // print acceleration at Z-axis after moving average filter

        //==========================================================================
        // Task 3: Motion gesture control
        //==========================================================================
        // Place GY-25Z at the center of the breadboard
        // Use the result of moving average filter on Z-axis acceleration data (ACC_Z_AVG)
        // - determine the acceleration of the car
        // Use the Yaw angle to determine the steering direction
        // - the Yaw angle is calibrated to 0 when power on
        float acc_threshold = 0.3;              // set the sensitivity of acceleration data as threshold = 0.3
//        if (){         // if Z-axis acceleration is larger than (+)acc_threshold, car moves forward
//          Serial.println("Move Forward~");
//          forward_steering(YPR[2]);             // check the steering direction of moving forward, modify forward_steeing() for task 3
//        }else if (){  // if Z-axis acceleration is smaller than (-)acc_threshold, car moves backward
//          Serial.println("Move Backward~");
//          // when acceleration of z-axis is negative (i.e. the sensor is facing downward)
//          // the reference yaw angle become 180 degree inversed
//          backward_steering(YPR[2]);            // check the steering direction of moving backward
//        }else{                                  // if Z-axis acceleration is within (+)acc_threshold and (-)acc_threshold, car stops
//          Serial.println("STOPPED!");
//        }
      }
    }
    packet_rdy = false;
  }
}

//---------------------------------------------
// check sum function:
//  input:  buf       - buffer array
//          num_bytes - total numbers of bytes to do addition
//---------------------------------------------
unsigned char checksum(unsigned char *buf, int num_bytes)
{
  unsigned char sum = 0;
  for(int i=0; i<num_bytes; i++)
  {
    // sum of the buffer
    sum += buf[i];    
  }
  return sum;
}

//------------------------------------------------------------------------------------
// [Complete this function for task 3]
// If the sensor is facing upward, the yaw angle for steering is from -90 to 90 degree
//------------------------------------------------------------------------------------
void forward_steering(int yaw_angle){
  float yaw_threshold = 10;                     // set the sensitivity of steering as 10 degree
  // the max steering angle should be +-90 degree (i.e. -90 to 90), other values are invalid
//  if (){
//    Serial.println("Steering too much !!!");
//  }else{
//    if (){            // if yaw_angle < (-ve) threshold
//      Serial.println("Steering RIGHT!");
//    }else if (){      // if yaw_angle > (+ve) threshold
//      Serial.println("Steering LEFT!");
//    }
//  }
}

//------------------------------------------------------------------------------------------------------------------
// If the sensor is facing downward, the yaw angle for steering is from -90 to -180 degree and from 90 to 180 degree
//------------------------------------------------------------------------------------------------------------------
void backward_steering(int yaw_angle){
  float yaw_threshold = 10;                     // set the sensitivity of steering as 10 degree
  // as the sensor is flipped, the reference yaw angle becomes 180 degree inversed
  // the max steering angle should be +-90 degree (i.e. 90 to 180 or -90 to -180), other values are invalid
  if (abs(yaw_angle) < 90){                     // if -90 < yaw_angle < 90, out of the steering range 
    Serial.println("Steering too much !!!");
  }else{
    if (YPR[2] < 0){
      if ((YPR[2] - yaw_threshold) > -180) {    // if -180 < yaw angle < -90, steering right
        Serial.println("Steering RIGHT!");
      }
    }else{
      if ((YPR[2] + yaw_threshold) < 180){      // if 90 < yaw angle < 180, steering left
        Serial.println("Steering LEFT!");
      }
    }
  }
}
