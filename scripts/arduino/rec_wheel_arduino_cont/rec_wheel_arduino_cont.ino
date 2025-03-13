float xy_wheel_vel;
float yz_wheel_vel;
float xz_wheel_vel;
float imu_ang_vel_xy, imu_ang_vel_yz, imu_ang_vel_xz;
float prev_imu_ang_vel_xy, prev_imu_ang_vel_yz, prev_imu_ang_vel_xz;
float ang_acc_xy, ang_acc_yz, ang_acc_xz;
float imu_orientation_xy, imu_orientation_yz, imu_orientation_xz;
unsigned long prev_t , curr_t;

void setup() {
  Serial.begin(115200);
}

void loop() {
  String receivedData = "0,0,0,0,0,0";
  if (Serial.available() > 0) {
    receivedData = Serial.readStringUntil('#');

    // Clear remaining buffer to prevent corrupted data
    while (Serial.available()) Serial.read();

    // Parse and validate data
    if (parseData(receivedData)) {
      Serial.print(imu_ang_vel_xy); Serial.print(",");
      Serial.print(imu_ang_vel_yz); Serial.print(",");
      Serial.print(imu_ang_vel_xz); Serial.print(",");
      Serial.print(imu_orientation_xy); Serial.print(",");
      Serial.print(imu_orientation_yz); Serial.print(",");
      Serial.println(imu_orientation_xz);
      ang_acc_xy = (imu_ang_vel_xy-prev_imu_ang_vel_xy)/(curr_t - prev_t);

    prev_imu_ang_vel_xy = imu_ang_vel_xy;
    prev_imu_ang_vel_yz = imu_ang_vel_yz;
    prev_imu_ang_vel_xz = imu_ang_vel_xz;
    
    
    
    
    } else {
      // Serial.println("Error: Received incorrect data format.");
    }
  }

  xy_wheel_vel = MapFloat(analogRead(A0), 0, 1013, -20.0, 20.0);
  yz_wheel_vel = MapFloat(analogRead(A1), 0, 1013, -20.0, 20.0);
  xz_wheel_vel = MapFloat(analogRead(A2), 0, 1013, -20.0, 20.0);

  delay(10);
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
    if (values[i] < -10.0 || values[i] > 10.0) {  // Adjust limits if needed
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
