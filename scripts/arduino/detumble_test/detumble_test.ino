float xy_wheel_vel, yz_wheel_vel, xz_wheel_vel;
float xy_wheel_velmem = 0, yz_wheel_velmem = 0 , xz_wheel_velmem = 0 ;
float imu_ang_vel_xy, imu_ang_vel_yz, imu_ang_vel_xz;// angular velocity
float prev_imu_ang_vel_xy = 0 , prev_imu_ang_vel_yz = 0 , prev_imu_ang_vel_xz = 0;//angular velocities memory
float ang_acc_xy, ang_acc_yz, ang_acc_xz;// accelration
float imu_orientation_xy, imu_orientation_yz, imu_orientation_xz;//angles
unsigned long prev_time , curr_time; float dt ;//time
float kp = 5, kd= 2.3;//5 and 0.9 gave good results

void setup() {
  Serial.begin(115200);
  // Serial.println("program begun");
  prev_time = micros();
}

void loop() {
  String receivedData = "0,0,0,0,0,0,0";
  if (Serial.available() > 0) {
    receivedData = Serial.readStringUntil('#');

    // Clear remaining buffer to prevent corrupted data
    while (Serial.available()) Serial.read();

    // Parse and validate data
    if (parseData(receivedData)) {

      // Serial.print(imu_ang_vel_xy); Serial.print(",");
      // Serial.print(imu_ang_vel_yz); Serial.print(",");
      // Serial.print(imu_ang_vel_xz); Serial.print(",");
      // Serial.print(imu_orientation_xy); Serial.print(",");
      // Serial.print(imu_orientation_yz); Serial.print(",");
      // Serial.println(imu_orientation_xz);
      curr_time = micros();
      dt = (curr_time - prev_time) / 1000000.0 ;//delta time in seconds
      if (dt <= 0) dt = 0.01;  //prevents dividing by zero errors
      // Serial.print("time: ");Serial.println(curr_time);Serial.println(prev_time);
      // Serial.print("dt: ");Serial.println(dt);
      

      ang_acc_xy = (imu_ang_vel_xy-prev_imu_ang_vel_xy)/dt;//numerical integration
      ang_acc_yz = (imu_ang_vel_yz-prev_imu_ang_vel_yz)/dt;
      ang_acc_xz = (imu_ang_vel_xz-prev_imu_ang_vel_xz)/dt;

    prev_imu_ang_vel_xy = imu_ang_vel_xy;
    prev_imu_ang_vel_yz = imu_ang_vel_yz;
    prev_imu_ang_vel_xz = imu_ang_vel_xz;
    prev_time = curr_time;

    xy_wheel_vel = - (kp * imu_ang_vel_xy + kd * ang_acc_xy);
    yz_wheel_vel = - (kp * imu_ang_vel_yz + kd * ang_acc_yz);
    xz_wheel_vel = - (kp * imu_ang_vel_xz + kd * ang_acc_xz);
    
    if (isinf(xy_wheel_vel) || isnan(xy_wheel_vel) ||
    isinf(yz_wheel_vel) || isnan(yz_wheel_vel) ||
    isinf(xz_wheel_vel) || isnan(xz_wheel_vel)) {
    xy_wheel_vel = xy_wheel_velmem; 
    yz_wheel_vel = yz_wheel_velmem; 
    xz_wheel_vel = xz_wheel_velmem;  
    }
    else{
      xy_wheel_velmem = xy_wheel_vel;
      yz_wheel_velmem = yz_wheel_vel;
      xz_wheel_velmem = xz_wheel_vel;
    }
    

    Serial.print(xy_wheel_vel); Serial.print(",");
    Serial.print(yz_wheel_vel); Serial.print(",");
    Serial.println(xz_wheel_vel);
    
    } else {
      // Serial.print("Error:");Serial.print(receivedData);
      // Serial.println("  Received incorrect data format.");
    }
  }

  // xy_wheel_vel = MapFloat(analogRead(A0), 0, 1013, -20.0, 20.0);
  // yz_wheel_vel = MapFloat(analogRead(A1), 0, 1013, -20.0, 20.0);
  // xz_wheel_vel = MapFloat(analogRead(A2), 0, 1013, -20.0, 20.0);//for manual control

  
  // delay(10);
}

bool parseData(String data) {
  int index = 0;
  float values[7];
  char buf[60];  // Increased buffer size for safety

  data.toCharArray(buf, 60);
  char *ptr = strtok(buf, ",");

  while (ptr != NULL && index < 7) {
    values[index] = atof(ptr);
    ptr = strtok(NULL, ",");
    index++;
  }

  // Check if exactly 6 values were received
  if (index != 7) return false;

  // Validate data (ensure no insane values)
  for (int i = 1; i < 7; i++) {
    if (values[i] < -100 || values[i] > 100.0) {  // Adjust limits if needed
      // Serial.println("Warning: Ignoring out-of-range values.");
      return false;
    }
  }

  // Store values in global variables
  imu_ang_vel_xy = values[1];
  imu_ang_vel_yz = values[2];
  imu_ang_vel_xz = values[3];
  imu_orientation_xy = values[4];
  imu_orientation_yz = values[5];
  imu_orientation_xz = values[6];

  return true;
}

float MapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
