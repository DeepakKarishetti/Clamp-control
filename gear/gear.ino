const byte outpin[9] = {4,5,6,7,8,9,10,11,12};
const byte inpin[2] = {2,3};
boolean gearone[9] = {1,1,0,0,0,0,1,1,0};
boolean geartwo[9] = {0,1,1,0,1,1,0,1,1};
boolean gearthree[9] = {0,0,1,0,0,1,1,1,1};
boolean gearfour[9] = {1,0,1,1,0,0,1,1,0};

byte gear=1;
byte beforegear=1;

int upshift=1;
int downshift=1;

void setup()
{
  for (int i=0; i<9; i++)
  {
  pinMode(outpin[i],OUTPUT);
  }
  pinMode(inpin[0],INPUT);
  pinMode(inpin[1],INPUT);
  // Put in gear one
  gear=1;
  beforegear=1;
  for (int i=0;i<9;i++)
  {
    digitalWrite(outpin[i],gearone[i]);
  } 
}

void loop()
{
  
  readswitch();
  while((upshift==1) && (downshift==1))
  {
    readswitch();
  }
  if ((upshift==0) && (gear==1))
  { 
    selecttwo();
    gear=2;
    beforegear=1;
  }
  if ((upshift==0) && (gear==2))
  { 
    selectthree();
    gear=3;
    beforegear=2;
  }
  if ((upshift==0) && (gear==3))
  { 
    selectfour();
    gear=4;
    beforegear=3;
  }
  if ((downshift==0) && (gear==4))
  { 
    selectthree();
    gear=3;
    beforegear=4;
  }
  if ((downshift==0) && (gear==3))
  { 
    selecttwo();
    gear=2;
    beforegear=3;
  }
  if ((downshift==0) && (gear==2))
  { 
    selectone();
    gear=1;
    beforegear=2;
  } 
}

void selectone()
{
   for (int i=0;i<9;i++)
   {
     digitalWrite(outpin[i],gearone[i]);
   }  
}

void selecttwo()
{
   for (int i=0;i<9;i++)
   {
     digitalWrite(outpin[i],geartwo[i]);
   }  
}

void selectthree()
{
   for (int i=0;i<9;i++)
   {
     digitalWrite(outpin[i],gearthree[i]);
   }  
}

void selectfour()
{
   for (int i=0;i<9;i++)
   {
     digitalWrite(outpin[i],gearfour[i]);
   }  
}

void readswitch()
{
  upshift=digitalRead(inpin[0]);
  delay(100);
  downshift=digitalRead(inpin[1]);
  delay(100);
}  
