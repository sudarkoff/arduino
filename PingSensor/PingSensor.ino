/**
 * Reading distance (cm) from PING))) sensor
 */

const int pingPin = 9;

void setup() {
  // initialize serial communication:
  Serial.begin(9600);
}

void loop()
{
  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin, LOW);

  // The same pin is used to read the signal from the PING))): a HIGH
  // pulse whose duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(pingPin, INPUT);
  long duration = pulseIn(pingPin, HIGH);

  // convert duration to distance
  long cm = usec2cm(duration);

  Serial.print("Distance (cm): "); Serial.println(cm);

  delay(1000);
}

long usec2cm(long usec)
{
  // Speed of sound is 340 m/s or 29 usec/cm.
  // The sound travels out and back, so devide the distance by 2.
  return usec / 29 / 2;
}

