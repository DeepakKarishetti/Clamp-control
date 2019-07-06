/*  Pin layout
 *
 *  Relay switch - Arduino ---> Gear switch
 *  7 ---> COM_1
 *  5 ---> NC_1
 *  3 ---> NO_1
 *  4 ---> COM_2
 *  6 ---> NC_2
 *  8 ---> NO_2
 *  
 */

int gear;

// Pins
const int PIN_3 = 3; 
const int PIN_8 = 4; 

void setup() 
{
  Serial.begin(115200);
  
  pinMode(PIN_3, OUTPUT);
  pinMode(PIN_8, OUTPUT);
}

void loop() 
{
  // forward
  if (gear == 1)
  {
    digitalWrite(PIN_3, HIGH);
    digitalWrite(PIN_8, LOW);
  }
  
  // neutral
  else if (gear  == 0)
  {
    digitalWrite(PIN_3, HIGH);
    digitalWrite(PIN_8, HIGH);
  }
  
  // reverse
  else if (gear == -1)
  {
    digitalWrite(PIN_3, LOW);
    digitalWrite(PIN_8, HIGH);
  }  
}
