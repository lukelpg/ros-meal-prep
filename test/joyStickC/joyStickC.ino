#define joyX A0
#define joyY A1

 
void setup() {
  Serial.begin(9600);
}
 
void loop() {
  // put your main code here, to run repeatedly:
  float xValue = analogRead(joyX);
  float yValue = analogRead(joyY);
 
  //print the values with to plot or view
  Serial.print(xValue);
  Serial.print("\t");
  Serial.println(yValue);
}
