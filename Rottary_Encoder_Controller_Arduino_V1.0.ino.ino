///////Program for Incrimental rottary encoder controller//////
///////by Suresh Dharmaratna//////////////////////
///////////on 25th February 2022///////////////////
/////////version 1.0//////////////////////////////


// Rotary Encoder Inputs2
#define CLK 2
#define DT 3
#define SW 4//Reset switch


//Varialbe setting of rottary encoder
const int RotPulsePerRev = 20; //pulses per single revalution of the selected rottary encoder
const int Puls4Speed = 5; //recalculate the speed after this number of pulses recived from the encoder
float SpeedCalValue = 0; //Puls4Speed/RotPulsePerRev; //use in speed calculation

int counter = 0;
int tempCounter=0;
int currentStateCLK;
int lastStateCLK;
String currentDir ="";
unsigned long lastButtonPress = 0;
int SerialIn = 5; // for incoming serial data
float RotSpeed=0.0; //rottary encoder spped (pulse per minute)
float RecordTime=0.0;
float RefTime=millis();
//unsigned long RefTime=millis();

void setup() {
  
  // Set encoder pins as inputs
  pinMode(CLK,INPUT);
  pinMode(DT,INPUT);
  pinMode(SW, INPUT_PULLUP);

  // Setup Serial Monitor
  Serial.begin(9600);

  // Read the initial state of CLK
  lastStateCLK = digitalRead(CLK);

  //initializing calculation of speed calibration value
  SpeedCalValue = float(Puls4Speed)/float(RotPulsePerRev);
  //Serial.println(SpeedCalValue);
  //Serial.println(Puls4Speed);
  //Serial.println(RotPulsePerRev);
}

//////////////////////////////////////////////////////////////////
///////start of main function (Loop function)///////////////////
/////////////////////////////////////////////////////////////////////
void loop() {

//Serial interface incoming
   if (Serial.available() > 0) 
   {
     // read the incoming byte:
       SerialIn = Serial.parseInt();
      // say what you got:
      //Serial.print("I received: ");
      //Serial.println( SerialIn);
      serial_flush_buffer();
      if( SerialIn==3)//Reset the counter
      {
        counter=0;
        Serial.println(counter);
      }
      if(SerialIn==2)//Read the speed
      {
        Serial.println(RotSpeed,4);
      }
      if(SerialIn==1)//Read the counter
      {
        Serial.println(counter);
      }      
    }

//Speed calculation 
  if(counter % Puls4Speed ==0 && counter!=0 && tempCounter!=counter)
  {
    //Serial.println(counter);
    tempCounter = counter;
    RecordTime =(millis()-RefTime)/1000; //time difference for 20 pulses in seconds
    //Serial.println(RefTime);
    //Serial.println(millis());
    //Serial.println(RecordTime);
   // RotSpeed = (5/RecordTime);  //rotation speed as number of pulses (of the rottary encoder) per second
    RotSpeed = (SpeedCalValue/RecordTime); //rotation speed as reverlutions per secod
    RefTime = millis();
    //Serial.println(RotSpeed);
  }

   
  // Read the current state of CLK
  currentStateCLK = digitalRead(CLK);

  // If last and current state of CLK are different, then pulse occurred
  // React to only 1 state change to avoid double count
  if (currentStateCLK != lastStateCLK  && currentStateCLK == 1){

    // If the DT state is different than the CLK state then
    // the encoder is rotating CCW so decrement
    if (digitalRead(DT) != currentStateCLK) {
      counter ++;
      currentDir ="CCW";
    } else {
      // Encoder is rotating CW so increment
      counter ++;
      currentDir ="CW";
    }

   
    //Serial.println(counter); //test incremental counter is working
  }

  // Remember last CLK state
  lastStateCLK = currentStateCLK;

}
/////////////////////////////////////////////////////////////////
/////end of main function (void loop function)////////////////
/////////////////////////////////////////////////////////////////



/////////////user defined functions/////////////////////
void serial_flush_buffer()
{
  while (Serial.read() >= 0)
   ; // do nothing
}



//////////////////////////////////////////////////////////////////////
////////////////connect to serial com via msdos prompt////////////////
///1.check comport with following command///////////
//[System.IO.Ports.SerialPort]::getportnames()
//output should be like COM1, COM2, COM3,,...///////////////
////2. Set the $port object with following command. change parameters accordigly COM# and Baudrate
//$port= new-Object System.IO.Ports.SerialPort COM#,Baudrate,None,8,one
//3. now start the serial communication by openning the port and send (write) and recive (Read) as following acommands
//$port.open()
//$port.WriteLine("some string")
//$port.ReadLine()
//$port.Close()
///////////////////////////////////////////////////////////////////////////
