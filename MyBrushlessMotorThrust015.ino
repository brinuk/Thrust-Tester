/* For use with the brushless motor thrust tester 
***********Board details**************
Modify #define as required
Plug in 3S battery then plug in Arduino cable to computer
SD card reader holds an 8gb card
Enters manual mode on startup if pot at zero
Button nearest SD card reader is manual mode and variable speed controller is active
Button nearest to the motor is auto mode motor will increase in steps  to  set max and each step is saved on SD card
Outputs to SD card %ESC drive, volts, currents, Props/min 

speed pot on A0
battery voltage on A1
battery current on A2
Laser enable pin 3
Laser detector pin 7
manual mode button pin 6
auto mode button pin 8
sounder pin 5 
SD card reader attached to SPI bus as follows:
 ** MOSI - pin 11
 ** MISO - pin 12
 ** CLK - pin 13
 ** CS - pin 4

*/

/*-----( Import needed libraries )-----*/
#include <Servo.h>
#include <SPI.h>
#include <SD.h>

/*-----( Declare Constants and Pin Numbers )----
***********************
MODIFY THESE LINES AS REQUIRED
**********************
-*/
#define BATTSCALE 0.0158215 //multiplier for battery voltage display for voltage divider, set dependant on resistance value chosen for voltage divider
#define POT_VALUE_SCALE 1.163 //corrects for errors in pot reading
#define MAXCOUNT 6 //Battery/current voltage averaging count
#define LASER_PIN 3
#define DETECTOR_PIN 7
#define CHIPSELECT 4 //data logger select pin
#define MANUAL_BUTTON_PIN 6
#define AUTO_BUTTON_PIN 8
#define SOUNDER_PIN 5
#define BUTTON_PRESSED LOW
#define BUTTON_NOT_PRESSED HIGH
#define DATA_LOG_FILE "DATALOG.txt"
#define PROP_TIME_SCAN 1000 //scan time (mS) for counting number of props passing
#define SPEED_POT_PIN A0 //analog pin to read pot value
#define BATT_VOLTS_PIN A1 //analog pin to read battery voltage
#define BATT_CURRENT_PIN A2 //analog pin to read battery current
#define SENSOR_SUPPLY_VOLTAGE 5.0 //supply voltage for hall effect current sensor makeVoltageZero
#define SENSITIVITY 40.0 //mV per amp for ACS758LCB-050B
#define QUIESCENT_OUTPUT_VOLTAGE 0.5// for ACS758LCB-050B
#define MIN_BATT_VOLTAGE 10000.0 //used in averaging battery voltage reading
#define MIN_BATT_CURRENT 10000.0 //used in averaging battery current reading
//************************************
#define MAKE_VOLTAGE_ZERO -0.0 //adjust this value to make output volts zero when no current flowing in current sensor ACS758LCB-050B
//************************************
//*********************************
#define NUMBER_OF_PROPS 1 //set to number of props for rpm otherwise props per minute. If set to 1 then gives number of prop passages per PROP_SCAN_TIME
//**********************************
#define AUTO_MODE_TIME_STEP 6000//time mS to run per step in auto mode
#define AUTO_MODE_SPEED_STEP 9//must be multiple of 180 eg 9 = 5% steps
//***********************************
#define AUTO_MODE_PERCENT_MAX_DRIVE 80//set % of max ESC drive value ie 50 will drive ESC at 50%
//***********************************



/*-----( Declare objects )-----*/
  Servo ESC;     // create servo object to control the ESC
  File dataFile;

/*-----( Declare Variables )-----*/
  int potValue = 0;  // value from the analog pin
  float battVoltage = 0.0; //battery voltage used for averaging
  float maxBattVolts = 0.0;//used for averaging
  float minBattVolts = MIN_BATT_VOLTAGE;//used for averaging
  float totalBattVoltage = 0.0; //value for battery woltage A1
  float battCurrent = 0.0; //battery current
  float maxBattCurrent = 0.0;//used for averaging
  float minBattCurrent = MIN_BATT_CURRENT;//used for averaging
  float totalBattCurrent = 0.0; //value for battery current A2
  float battCurrentZeroOffset = 0.0; //offset to zero hall sensor at zero current
  //float battVoltsResult = 0.0; //result of battery voltage averaging
  //float battCurrentResult = 0.0; //result of battery current averaging
  //int battVoltageInspectionCount = 0;
  String dataString = "test,1,2,3,4,5,6";
  bool inManualMode = false;//flag for modes
  bool safeForManualMode = false;//unsafe to enter manual pot not zero
  bool alreadyStarted = false; //remebers that we have already started in manual or auto mode
  long int autoModeTimeOfLastStep = 0;//saves the time of the last auto mode step increase
  int autoModeStepCount = 0;//counts auto mode ESC driver step count
  bool recordDataNow = false;//use for auto mode data diaplay
  bool recordedData = false;//used to prevent duplicate date in auto mode
  int logCounter = 0; //counts logged data entries sent to SD card
  bool logFileToDelete = true;//flag for deleting log file
  


void setup()   /****** SETUP: RUNS ONCE ******/
{
  Serial.begin(115200);
  setupModesAndSounder();  
  setupSDCard();
  //logToSDCard(dataString);
  setupLaser();
  ESC.attach(9,1000,2000); // Attach the ESC on pin 9(pin, min pulse width, max pulse width in microseconds)
  NBDelay(5000);//wait of ESC
  soundSounderShortBursts(3);
  while(!manualModeSafeCheck())//wait until safe for manual mode ie pot at zero
    {
      soundSounderShortBursts(1);
    }
  safeForManualMode = true;
  battCurrentZeroOffset = getZeroCurrentOffset(MAXCOUNT + 20);//average zero current offset to apply to Hall sensor
  //Serial.println(battVoltage,2);
  //Serial.println(battCurrentZeroOffset,2);
 
} 
//----end setup----

//*****loop********
void loop()   /****** LOOP: RUNS CONSTANTLY ******/
  {  
    checkButtons();//check if manual or auto button is pressed
    if(alreadyStarted & inManualMode)//run in manual mode
      {
        displayInformationManualMode();//get laser prop counter ready and battery voltage monitor
        driveESCManualMode();
        logFileToDelete = true;//enable deletion
      }
    if(alreadyStarted & !inManualMode)//run in auto mode
      {
        if(logFileToDelete)
          {
            deleteLogFile(DATA_LOG_FILE);
            logFileToDelete = false;//prevent deletion
            battCurrentZeroOffset = getZeroCurrentOffset(MAXCOUNT + 20);//average zero current offset to apply to Hall sensor
          }
        driveESCAutoMode();
        logInformationAutoMode();//get laser prop counter ready and battery voltage monitor
        
      }
  
  }  

//--Loop Ends---



//*********Functions************************

//**********get Hall sensor zero offset****************

unsigned int getZeroCurrentOffset(int numberOfScans)//returns average raw offset value for current and sets 
  {
    float thisVoltage;
    float thisCurrent;
    float battCurrentOffset;
    int battVoltageInspectionCount = 0;
    while(battVoltageInspectionCount < numberOfScans)//get average battery volts and current
      {
        thisVoltage = BATTSCALE * analogRead(BATT_VOLTS_PIN);//read volts 
        thisCurrent = analogRead(BATT_CURRENT_PIN);// Read the voltage from current sensor
        totalBattVoltage = totalBattVoltage + thisVoltage; //calculates sum of voltage readings
        totalBattCurrent = totalBattCurrent + thisCurrent; //calculate sum of current readings
        if(thisVoltage > maxBattVolts)//save largest voltage value
          {
            maxBattVolts = thisVoltage;
          }
        if(thisCurrent > maxBattCurrent)//save largest current value
          {
            maxBattCurrent = thisCurrent;
          }
        if(thisVoltage < minBattVolts)//save min battery voltage
          {
            minBattVolts = thisVoltage; 
          }
        if(thisCurrent < minBattCurrent)//save min battery current
          {
            minBattCurrent = thisCurrent; 
          }
        battVoltageInspectionCount++;
        NBDelay(10);
      }
      battVoltage = (totalBattVoltage - minBattVolts - maxBattVolts)/(battVoltageInspectionCount - 2); //returns average voltage
      battCurrentOffset = (totalBattCurrent - minBattCurrent - maxBattCurrent)/(battVoltageInspectionCount - 2); //returns average cutrrent
      totalBattVoltage = 0;//reset everything
      totalBattCurrent = 0;//reset everything
      battVoltageInspectionCount = 0;
      maxBattVolts = 0.0;
      maxBattCurrent = 0.0;
      minBattVolts = MIN_BATT_VOLTAGE;
      minBattCurrent = MIN_BATT_CURRENT;
      return battCurrentOffset;
  }
//**********get Hall sensor zero offset ends****************

//***********delete log file*************
void deleteLogFile(String fileName)
  {
  if (SD.exists(fileName))
    {
      SD.remove(fileName);
      Serial.print(fileName);
      Serial.println(" deleted");
    }
  else
    {
      Serial.print(fileName);
      Serial.println(" does not exist");
    }
  }
//**delete log file ends****************

//********* log and display auto mode information on arduino GUI**********
void logInformationAutoMode()
  {
    if(recordDataNow)//on next to last run at a given speed record info
       { 
          String thisData = "";
          long propsPassingCount = 0;
          recordDataNow = false;//reset to prevent repeat recording
          recordedData = true;//reset flag
          getBattVoltageAndCurrent(); //get the average battery voltage
          logCounter++;//increment log counter
          propsPassingCount = PropsPassingLaserCount(PROP_TIME_SCAN);
          //multiply by logCounter to output %drive to ESC
          thisData = String(logCounter * AUTO_MODE_SPEED_STEP * 100/180) + "," +String(battVoltage) + "," + String(battCurrent) + "," + String(propsPassingCount);//form string for storing on SD card
          logToSDCard(thisData);//log data to SD card
       }
  }

//********log and display information ends*************

//***********read buttons***********
 void checkButtons()//checks for buttons pressed and sets flags inManual mode and already Started
  {
    //check manual modebutton
    if(manualButtonPressed() & !inManualMode)//check manual button and pot at zero and in automode
    {
      if(manualModeSafeCheck())
        {
          soundSounder(100);
          inManualMode = true;//indicate mode state
          driveESCManualMode(); //get ESC going
          //displayInformationManualMode();
          alreadyStarted = true;
          logCounter = 0; //reset log counter for new auto start
         }
       else
        {
          soundSounder(750);
        }
    }
  //check auto mode button
  if(autoButtonPressed()& inManualMode)//check auto button pressed in manual mode
    {
      soundSounder(2000);
      inManualMode = false;//indicate mode state
      autoModeStepCount = 0;//set step counter for start of run
      alreadyStarted = true;
    }
  }
//********end read buttons***************

//***************Drive ESC auto mode*******************

void driveESCAutoMode()
  {
    int autoModeESCDriveValue = 0;
    if(autoModeStepCount == 0)//if just starting
      {
        autoModeTimeOfLastStep = millis();//record time
        autoModeStepCount++;//inc step count
        soundSounder(100);
      }
    if(!recordedData & !recordDataNow & millis() > (autoModeTimeOfLastStep + (0.5*AUTO_MODE_TIME_STEP)))//nearly at next speed so record data ensure only one lot of data recorded
      {
        recordDataNow = true;//set flag
        soundSounder(100);
        
      }
    if(millis() > (autoModeTimeOfLastStep + AUTO_MODE_TIME_STEP))//its alreadey run for AUTO_MODE_TIME_STEP mS go to next speed step
      {
        autoModeStepCount++;//inc step count
        autoModeTimeOfLastStep = millis();//record time
        recordDataNow = false;//reset flag
        recordedData = false;//reset flag
      }
    autoModeESCDriveValue = autoModeStepCount*AUTO_MODE_SPEED_STEP;//get drive value
    if(autoModeESCDriveValue <= ((AUTO_MODE_PERCENT_MAX_DRIVE *1.8) + AUTO_MODE_SPEED_STEP - 1))//change % value to actual ie 100% is max 180 and one more step needed 
      {
        ESC.write(autoModeESCDriveValue);    // Send the signal to the ESC
      }
    else//all done return to manual mode
      {
        inManualMode = true;//stop motor
        logCounter = 0;//rest log counter for next time
      }
  }

//************drive ESC auto mode ends ************************

//*********display manual mode information on arduino GUI**********
void displayInformationManualMode()
  {
    if(potValue >7)// only do this if motor is spinning
      {
        getBattVoltageAndCurrent(); //get the average battery voltage and current
        Serial.print(potValue*100/180);//% value of ESC drive max 180
        Serial.print("% ESC drive. ");
        Serial.print(battVoltage,2);
        Serial.print(" volts  ");
        Serial.print(battCurrent,2);
        Serial.print(" amps  ");
        Serial.print(PropsPassingLaserCount(PROP_TIME_SCAN));//read prop passages in PROP_TIME_SCANdivided by NUMBER_OF_PROPS
        Serial.println(" Props/minute");
      }
  }

//********display information ends*************

//*********check manual mode safe**********
bool manualModeSafeCheck()
  {
    bool safe = false;
    potValue = analogRead(SPEED_POT_PIN);   // reads the value of the potentiometer (value between 0 and 1023)
    potValue = map(potValue, 0, 1023, 0, 180);   // scale it to use it with the servo library (value between 0 and 180)
    if(potValue < 5)
      {
        safe = true;
      }
    return safe;
  }
//***********check manual mode safe ends**************

//*****read auto button*************

bool autoButtonPressed()//returns true if pressed
  {
    bool state = false;
    //while(digitalRead(AUTO_BUTTON_PIN) == BUTTON_PRESSED){}//wait of button released
    //NBDelay(10);//wait for contact bounce
    if(digitalRead(AUTO_BUTTON_PIN) == BUTTON_PRESSED)//wait of button pressed
      {
        NBDelay(10);//wait for contact bounce
        if(digitalRead(AUTO_BUTTON_PIN) == BUTTON_PRESSED)//still pressed
          {
            state = true;
          }
      }
    return state;
  }
//************read auto button ends **********

//*****read manual button*************

bool manualButtonPressed()//returns true if pressed
  {
    bool state = false;
    //while(digitalRead(MANUAL_BUTTON_PIN) == BUTTON_PRESSED){}//wait of button released
    //NBDelay(10);//wait for contact bounce
    if(digitalRead(MANUAL_BUTTON_PIN) == BUTTON_PRESSED)//wait of button pressed
      {
        NBDelay(10);//wait for contact bounce
        if(digitalRead(MANUAL_BUTTON_PIN) == BUTTON_PRESSED)//still pressed
          {
            state = true;
          }
      }
    return state;
  }
//************read manual button ends **********

//***sound sounder short bursta***************

void soundSounderShortBursts(int number)// gives number of short bursts of sounder
  {
    for(int n=0; n < number; n++)
      {
        soundSounder(100);
        NBDelay(100);
      }
  }

//***sound sounder short burst ends ***********
//**********sound sounder***************

void soundSounder(int duration) //sound sounder duration in mS
  {
    digitalWrite(SOUNDER_PIN, HIGH);
    NBDelay(duration);
    digitalWrite(SOUNDER_PIN, LOW);
  }

//*************sound sounder ends **************

//*******setup mode buttons and sounder**********

void setupModesAndSounder()
  {
    pinMode(SOUNDER_PIN, OUTPUT);
    digitalWrite(SOUNDER_PIN, LOW);
    pinMode(MANUAL_BUTTON_PIN, INPUT_PULLUP);
    pinMode(AUTO_BUTTON_PIN, INPUT_PULLUP);
  }
//***********setup Laser***************

void setupLaser()
  {
    pinMode(LASER_PIN, OUTPUT);
    digitalWrite(LASER_PIN, LOW);
    pinMode(DETECTOR_PIN, INPUT);
  }

//************setup Laser ends***********

//************setup SD card*************

void setupSDCard()
  {
    Serial.print("Initializing SD card...");
    if (!SD.begin(CHIPSELECT))
      {
        Serial.println(" SD initialization failed!");
        while (1);
      }
    Serial.println("SD initialization done.");
  }

//**8end SD card setup*******************

//***********testlog data to  SD card***************
void logToSDCard(String data)
  {
    // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  dataFile = SD.open(DATA_LOG_FILE, FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile)
    {
      dataFile.println(data);
      dataFile.close();
      // print to the serial port too:
      Serial.println(data);
    }
  // if the file isn't open, pop up an error:
  else
    {
      Serial.println("error opening datalog.txt");
    }
  }
  
///*******log data to SD card ends**************

//******read laser***********

unsigned int PropsPassingLaserCount(long runTime)// counts number of props passing in runTime if NUMBER_OF_PROPS set then returns revs/runtime
{
  unsigned long startTime;
  unsigned int totalProppass =0;
  unsigned long propCount = 0;
  bool propsTurning = true; //flag props turning
  runTime = runTime * 1000;//convert mS to microS
  digitalWrite(LASER_PIN, HIGH);
  startTime = micros();
  while(micros() - startTime < runTime)
  {
    propsTurning = true;
    while(digitalRead(DETECTOR_PIN) < 1)//wait for prop coming, break if prop not turning
    {
      if(micros() - startTime > 2*runTime)//props not turning
        {
          propsTurning = false;
          break;
        }
    }
    while(digitalRead(DETECTOR_PIN) > 0)//wait for prop going, break if prop not turning
    {
      if(micros() - startTime > 2*runTime)//props not turning
        {
          propsTurning = false;
          break;
        }
    }
    if(propsTurning)
      {
        propCount++;
      }
  }
  digitalWrite(LASER_PIN, LOW);
  return ((propCount)*60)/(NUMBER_OF_PROPS*runTime/1000000);//returns revs per runtime per minute
  
  
}

//***************Drive ESC manual mode*******************

void driveESCManualMode()
  {
    potValue = analogRead(SPEED_POT_PIN);   // reads the value of the potentiometer (value between 0 and 1023)
    potValue = potValue * POT_VALUE_SCALE;//adjust max pot value to 1023
    potValue = map(potValue, 0, 1023, 0, 180);   // scale it to use it with the servo library (value between 0 and 180)
    ESC.write(potValue);    // Send the signal to the ESC
  }

//************drive ESC manual mode ends ************************

//**********get average battery voltage****************

void getBattVoltageAndCurrent()//returns average battery voltage and current. Discards biggest and smallest
  {
    float thisVoltage;
    float voltageRaw;
    float voltage;
    float thisCurrent;
     int battVoltageInspectionCount = 0;
    while(battVoltageInspectionCount < MAXCOUNT + 50)//get average battery volts and current
      {
        thisVoltage = BATTSCALE * analogRead(BATT_VOLTS_PIN);//read volts 
        voltageRaw = (5.0 / 1023.0)* (analogRead(BATT_CURRENT_PIN) - battCurrentZeroOffset);// Read the voltage from current sensor
        //Serial.print("voltage ");
        //Serial.println(voltageRaw,1);
        voltage =  voltageRaw  + MAKE_VOLTAGE_ZERO;//adjust value for current sensor parameters
        //Serial.println(voltage,1);
        thisCurrent = voltage/SENSITIVITY*1000;//calulate current flowing in amps
        totalBattVoltage = totalBattVoltage + thisVoltage; //calculates sum of voltage readings
        totalBattCurrent = totalBattCurrent + thisCurrent; //calculate sum of current readings
        if(thisVoltage > maxBattVolts)//save largest voltage value
          {
            maxBattVolts = thisVoltage;
          }
        if(thisCurrent > maxBattCurrent)//save largest current value
          {
            maxBattCurrent = thisCurrent;
          }
        if(thisVoltage < minBattVolts)//save min battery voltage
          {
            minBattVolts = thisVoltage; 
          }
        if(thisCurrent < minBattCurrent)//save min battery current
          {
            minBattCurrent = thisCurrent; 
          }
        battVoltageInspectionCount++;
        NBDelay(10);
      }
      battVoltage = (totalBattVoltage - minBattVolts - maxBattVolts)/(battVoltageInspectionCount - 2); //returns average voltage
      battCurrent = (totalBattCurrent - minBattCurrent - maxBattCurrent)/(battVoltageInspectionCount - 2); //returns average cutrrent
      totalBattVoltage = 0;//reset everything
      totalBattCurrent = 0;//reset everything
      battVoltageInspectionCount = 0;
      maxBattVolts = 0.0;
      maxBattCurrent = 0.0;
      minBattVolts = MIN_BATT_VOLTAGE;
      minBattCurrent = MIN_BATT_CURRENT;
      
  }
//**************Non Blocking delay***********
void NBDelay(int delayTime) //delayTime in mSecs
    {
      long int startTime;
      startTime = millis();
      while(millis() - startTime <= delayTime)
        {
          yield();
        }
    }

//****end non blocking delay***********

//*********( THE END )***********
