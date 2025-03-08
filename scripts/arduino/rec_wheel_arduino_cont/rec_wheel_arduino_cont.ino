float xy_wheel_vel ;
float yz_wheel_vel ;
float xz_wheel_vel;
int count =0;
int increment = 1;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

}


void loop() {
  String receivedData ="0,0,0,0,0,0";
  if (Serial.available() > 0) {
        receivedData = Serial.readStringUntil('#');
            while (Serial.available()) Serial.read();   // Clear remaining buffer
            }  // Read line from ROS 2
 
  xy_wheel_vel = MapFloat(analogRead(A0),0,1013,-20.0,20.0);
  yz_wheel_vel = MapFloat(analogRead(A1),0,1013,-20.0,20.0);
  xz_wheel_vel = MapFloat(analogRead(A2),0,1013,-20.0,20.0);
  // put your main code here, to run repeatedly:
  Serial.print(xy_wheel_vel);
  Serial.print(",");
  Serial.print(yz_wheel_vel);
  Serial.print(",");
  Serial.print(xz_wheel_vel);
  Serial.print(",");
  Serial.print(receivedData);
  Serial.println();

  delay(200);

}

float MapFloat(float x, float in_min, float in_max, float out_min, float out_max)//same as built in map but floats
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
