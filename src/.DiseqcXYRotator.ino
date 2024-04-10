//programmator Arduino ISP
#include <math.h>
#include <EEPROM.h>
#include <NeoSWSerial.h>

const double RTD = 180.0 / M_PI;  //constant for conversion radians to degrees
const double DTR = M_PI / 180.0;  //constant for conversion degrees to radians

const char Version[]="Diseqc based XY Rotator by OK9UWU & MBSat & theverygaming :)"; // axis X controller

//I/O pins definitions
const int X_HALL_SENS     =  2;
const int X_RELE_POS      =  5;
const int X_RELE_NEG      =  6;
const int X_REF_SW        =  4;
const int Y_HALL_SENS     =  3;
const int Y_RELE_POS      =  9;
const int Y_RELE_NEG      =  10;
const int Y_REF_SW        =  8;
const int rxPin           =  0;
const int txPin           =  1;

const float ppd =  36.5; //36.7111pulses per degree, resolution of hall motor rev counter

byte X_RefSwVal = 0;
byte Y_RefSwVal = 0;

float MaxRange; //max angle for the axis, number fom 1 to 90 degree

float oldX, oldY;
float stepd = 1.0;

volatile float x_curr_pos_float = 0.0;  
volatile float x_target_pos_float = 0.0;

volatile int x_curr_pos_puls = 0;
volatile int x_target_pos_puls = 0;

volatile float y_curr_pos_float = 0.0;  
volatile float y_target_pos_float = 0.0;

volatile int y_curr_pos_puls = 0;
volatile int y_target_pos_puls = 0;

float x_ref_offset_degrees= 0;             //Offset from reference switch, degrees, used by axis referencing 
int x_ref_offset_pulses = 0;  //Offset from reference switch, in pulses, used by axis referencing 

float y_ref_offset_degrees= 0;             //Offset from reference switch, degrees, used by axis referencing 
int y_ref_offset_pulses = 0;  //Offset from reference switch, in pulses, used by axis referencing 

bool x_referenced = true;
bool y_referenced = true;

volatile bool x_move_direction = true;
volatile bool x_moving = false;

volatile bool y_move_direction = true;
volatile bool y_moving = false;

char tempbuf[80];  // keeps the command temporary until CRLF
String buffer;

float Az=0.0, El=0.0, X, Y;

NeoSWSerial SWSer(rxPin, txPin);

void setup() {

  pinMode(rxPin, INPUT);
  pinMode(txPin, OUTPUT);
  // initialize serial communication at 9600 bits per second:
  SWSer.begin(9600);
  
  pinMode(X_REF_SW, INPUT_PULLUP);
  pinMode(Y_REF_SW, INPUT_PULLUP);
  
  pinMode(X_RELE_POS, OUTPUT); 
  pinMode(X_RELE_NEG, OUTPUT);

  pinMode(Y_RELE_POS, OUTPUT);
  pinMode(Y_RELE_NEG, OUTPUT);
 
  digitalWrite(X_RELE_POS, LOW);
  digitalWrite(X_RELE_NEG, LOW);

  digitalWrite(Y_RELE_POS, LOW);
  digitalWrite(Y_RELE_NEG, LOW);

  x_ref_offset_pulses = EEPROMReadInt(0); // 1degree  or -1.5 at 36
  x_ref_offset_degrees = pulses2angle(x_ref_offset_pulses);

  y_ref_offset_pulses = EEPROMReadInt(32); //as of Feb 17th 2023 -1.50degrees, -55 pulses
  y_ref_offset_degrees = pulses2angle(y_ref_offset_pulses);

  //EEPROM.get(64, ppd);

  x_go_ref();
  y_go_ref();

  MaxRange = 90.0;
  
  attachInterrupt(digitalPinToInterrupt(Y_HALL_SENS), y_hall_sens_isr, RISING);
  attachInterrupt(digitalPinToInterrupt(X_HALL_SENS), x_hall_sens_isr, RISING);

  SWSer.println(Version);  
  
  SWSer.print("X offset=");
  SWSer.print(x_ref_offset_degrees);
  SWSer.print("degrees, ");
  SWSer.print(x_ref_offset_pulses);
  SWSer.println("hall pulses ");
  SWSer.print("Y offset=");
  SWSer.print(y_ref_offset_degrees);
  SWSer.print("degrees, ");
  SWSer.print(y_ref_offset_pulses);
  SWSer.println("hall pulses ");
}

void loop(){

  while (SWSer.available() > 0)
  {
    char st[20];
    char rx = SWSer.read();  // read a single charecter
    //float Az, El, X, Y;

    buffer += rx;  //add character to the string buffer
    //SWSer.print(rx);
    if ((rx == '\n') || (rx == '\r'))
    {
    buffer.toCharArray(tempbuf, 40);       
       if (buffer.startsWith("posx"))
            {
                sscanf(tempbuf,"posx%s",&st); // extract attenuation as floating point number
                float posx = strtod(st,NULL);
                SWSer.print("pos_x ");
                x_goto_posf(posx);
            } else
       if (buffer.startsWith("posy"))
            {
                sscanf(tempbuf,"posy%s",&st); // extract attenuation as floating point number
                float posy = strtod(st,NULL);
                y_goto_posf(posy);
            } else
      if (buffer.startsWith("dgetposx"))
            {
                SWSer.print("dpos_x=");
                SWSer.println(pulses2angle(x_curr_pos_puls));
            } else
      if (buffer.startsWith("dgetposy"))
            {
                SWSer.print("dpos_y=");
                SWSer.println(pulses2angle(y_curr_pos_puls));
            } else                    
      if (buffer.startsWith("stop")) // 
            {
               SWSer.println("StopX");
               x_stop_moving();
               y_stop_moving();
            } else
      if (buffer.startsWith("pgetposx")) // 
            {
               SWSer.print("ppos_x=");
               SWSer.println(x_curr_pos_puls);
            } else
      if (buffer.startsWith("pgetposy")) // 
            {
              SWSer.print("ppos_y=");
              SWSer.println(y_curr_pos_puls);
            } else
      if (buffer.startsWith("sox"))
            {
                sscanf(tempbuf,"sox%s",&st); // extract attenuation as floating point number
                x_ref_offset_degrees = strtod(st,NULL);
                x_ref_offset_pulses = angle2pulses(x_ref_offset_degrees);
                EEPROMWriteInt(0, x_ref_offset_pulses);
                
                SWSer.print("X offset=");
                SWSer.print(x_ref_offset_degrees);
                SWSer.print("degrees, ");
                SWSer.print(x_ref_offset_pulses);
                SWSer.println("hall pulses ");
            } else 
      if (buffer.startsWith("soy"))
            {
                sscanf(tempbuf,"soy%s",&st); // extract attenuation as floating point number
                y_ref_offset_degrees = strtod(st,NULL);
                y_ref_offset_pulses = angle2pulses(y_ref_offset_degrees);
                EEPROMWriteInt(32, y_ref_offset_pulses);

                SWSer.print("Y offset=");
                SWSer.print(y_ref_offset_degrees);
                SWSer.print("degrees, ");
                SWSer.print(y_ref_offset_pulses);
                SWSer.println("hall pulses ");
            } else
      /*if (buffer.startsWith("ppd"))
            {
                sscanf(tempbuf,"ppd%s",&st); // extract attenuation as floating point number
                ppd = strtod(st,NULL);
                //x_ref_offset_pulses = angle2pulses(x_ref_offset_degrees);
                EEPROM.put(64, ppd);
                
                SWSer.print("X offset=");
                SWSer.print(x_ref_offset_degrees);
                SWSer.print("degrees, ");
                SWSer.print(x_ref_offset_pulses);
                SWSer.println("hall pulses ");
            } else */
      if (buffer.startsWith("getoffx")) // send back status of reference switch
            {
                //ref_offset_degrees = pulses2angle(ref_offset_pulses);
                SWSer.print("X offset=");
                SWSer.print(x_ref_offset_degrees);
                SWSer.print("degrees, ");
                SWSer.print(x_ref_offset_pulses);
                SWSer.println ("hall pulses ");  
            } else                       
      if (buffer.startsWith("getoffy")) // send back status of reference switch
            {
                SWSer.print("Y offset=");
                SWSer.print(y_ref_offset_degrees);
                SWSer.print("degrees, ");
                SWSer.print(y_ref_offset_pulses);
                SWSer.println ("hall pulses ");
            } else
      if (buffer.startsWith("getyeeprom")) // 
            {
              SWSer.println(EEPROMReadInt(32));
            } else
     /* if (buffer.startsWith("getppd")) // 
            {
              SWSer.println(EEPROMReadInt(64));
            } else */              
      if (buffer.startsWith("gotoref")) // 
            {
               x_go_ref();
               y_go_ref();
               SWSer.println("refxDone");
               
            } else
      if (buffer.startsWith("getrswx")) // send back status of reference switch
            {
               X_RefSwVal = digitalRead(X_REF_SW);
               SWSer.print("RSW status=");
               SWSer.println(X_RefSwVal);  
            } else  
      if (buffer.startsWith("getrswy")) // send back status of reference switch
            {
               Y_RefSwVal = digitalRead(Y_REF_SW);
               SWSer.print("RSW status=");
               SWSer.println(Y_RefSwVal);
            } else                
      if ((buffer.startsWith("AZ")) && (buffer.length() < 26)) {
            char strAz[10];
            char strEl[10];
            if (sscanf(buffer.c_str(), "AZ%s EL%s", strAz, strEl) == 2) {
                //float az, el;
                Az = strtod(strAz, NULL);
                El = strtod(strEl, NULL);
                MBSat_AzEltoXY(Az, El, &X, &Y);
                /*
                SWSer.print(X);
                SWSer.print(" ");
                SWSer.print(Y);
                SWSer.println(" ");
                */
                x_goto_posf(X);
                y_goto_posf(Y);
            } else {
                //float az, el;
                //MBSat_XYtoAzEl(pulses2angle(x_curr_pos_puls), pulses2angle(y_curr_pos_puls), &az, &el);
                char str_az[6];
                char str_el[6];
                dtostrf(Az, 5, 1, str_az);
                dtostrf(El, 5, 1, str_el);
                SWSer.print("AZ");
                SWSer.print(str_az);
                SWSer.print(" EL");
                SWSer.println(str_el);
            }} else              
      if (buffer.startsWith("version"))
            {
              SWSer.println(Version);
            } else
      if (buffer.startsWith("help"))
            {
               
                SWSer.println("Commands for XY antenna rotor controller");
                SWSer.println(" * gotoref                  --move both axes to reference"); 
                SWSer.println(" * setlimx90.0, setlimy90.0 --set software limit for given axis");
                SWSer.println(" * posx90.0, posy90.0       --move given axis to position");
                SWSer.println(" * dgetposx, dgetposy       --return axes positions in degrees");
                SWSer.println(" * pgetposx, pgetposy       --return axes positions in pulses");
                SWSer.println(" * sox1.5, soy1.5           --set axis offsets in degrees");
                SWSer.println(" * stop                     --stop both axes moving");
                SWSer.println(" * AZ360 EL90               --position command from rotctld as EasyCommII, angles are Az El");
                SWSer.println(" * version                  --prints current version details");

            } 
            buffer = "";  //erase buffer for next command                     
    }
  
  }//end while (SWSer.available() > 0)
    /*if(x_move_direction == true)//jedeme +
  {
    if(x_curr_pos_puls >= x_target_pos_puls)
    {
      x_stop_moving();
    } 
  }else//jedeme - 
  if(x_curr_pos_puls <= x_target_pos_puls)
    {
      x_stop_moving();
    }*/
}

/*void x_hall_sens_isr()
{
  if(x_move_direction)
  {
    x_curr_pos_puls++;
  }
  else
  {
    x_curr_pos_puls--;
  }
  if(x_move_direction == true)//jedeme +
  {
    if(x_curr_pos_puls >= x_target_pos_puls)
    {
      x_stop_moving();
    } 
  }else//jedeme - 
  if(x_curr_pos_puls <= x_target_pos_puls)
    {
      x_stop_moving();
    } 

}*/

/*void x_hall_sens_isr() 
{
  noInterrupts();  // Disable interrupts while servicing Hall Sensor ISR
  if (x_move_direction) 
  {
    x_curr_pos_puls++;
    if (x_curr_pos_puls >= x_target_pos_puls) 
    {
      x_stop_moving();
    }
  } else 
  {
    x_curr_pos_puls--;
    if (x_curr_pos_puls <= x_target_pos_puls)
     {
      x_stop_moving();
    }
  }
  interrupts();  // Re-enable interrupts on exit. This will cause all pending interrupts (if there are any) to fire here instead of earlier in this ISR.
}*/

void y_hall_sens_isr()
{
  if(y_move_direction)
  {
    y_curr_pos_puls++;
  }
  else
  {
    y_curr_pos_puls--;
  }
  if(y_move_direction == true)//jedeme +
  {
    if(y_curr_pos_puls >= y_target_pos_puls)
    {
      y_stop_moving();
    } 
  }else//jedeme - 
  if(y_curr_pos_puls <= y_target_pos_puls)
    {
      y_stop_moving();
    } 

}

void x_hall_sens_isr()
{
  if(x_move_direction)
  {
    x_curr_pos_puls++;
  }
  else
  {
    x_curr_pos_puls--;
  }
  if(x_move_direction == true)//jedeme +
  {
    if(x_curr_pos_puls >= x_target_pos_puls)
    {
      x_stop_moving();
    } 
  }else//jedeme - 
  if(x_curr_pos_puls <= x_target_pos_puls)
    {
      x_stop_moving();
    } 

}

/*void y_hall_sens_isr() 
{
  noInterrupts();  // Disable interrupts while servicing Hall Sensor ISR
  if (y_move_direction) 
  {
    y_curr_pos_puls++;
    if (y_curr_pos_puls >= y_target_pos_puls) 
    {
      y_stop_moving();
    }
  } else {
    y_curr_pos_puls--;
    if (y_curr_pos_puls <= y_target_pos_puls) 
    {
      y_stop_moving();
    }
  }
  interrupts();  // Re-enable interrupts on exit. This will cause all pending interrupts (if there are any) to fire here instead of earlier in this ISR.
}*/


//This function will write a 2 byte integer to the eeprom at the specified address and address + 1
void EEPROMWriteInt(int p_address, int p_value)
     {
     byte lowByte = ((p_value >> 0) & 0xFF);
     byte highByte = ((p_value >> 8) & 0xFF);

     EEPROM.write(p_address, lowByte);
     EEPROM.write(p_address + 1, highByte);
     }

//This function will read a 2 byte integer from the eeprom at the specified address and address + 1
unsigned int EEPROMReadInt(int p_address)
     {
     byte lowByte = EEPROM.read(p_address);
     byte highByte = EEPROM.read(p_address + 1);

     return ((lowByte << 0) & 0xFF) + ((highByte << 8) & 0xFF00);
     }

/*void substring(char s[], char sub[], int p, int l) {
   int c = 0;
   
   while (c < l) {
      sub[c] = s[p+c-1];
      c++;
   }
   sub[c] = '\0';
}

double to_radians(double degrees) {
    return degrees * ( M_PI/180.0);
}

double to_degrees(double radians) {
    return radians * (180.0 / M_PI);
}*/

void MBSat_AzEltoXY(float az, float el, float *x, float *y)
{
    const float accuracy = 0.01; // rotor accuracy in degrees

    if (el < 0.1) { //because fun stuff happens when you try to pass EL0.0 to it :)
        el = 0.1;
    }
    if(el <= accuracy)
        *x = 90.0;
    else if(el >= (90.0 - accuracy))
        *x = 0.0;
    else
        *x = -atan(-cos(az * DTR) / tan(el * DTR)) * RTD;

        *y = -asin(sin(az * DTR) * cos(el * DTR)) * RTD;
}

void MBSat_XYtoAzEl(float X, float Y, float *az, float *el) { // do not ask how many stupid hours it took to get this to work
    // Motor Description:
    // X -> lower motor -> North-South
    // Y -> upper motor -> East-West
    // X+ -> North | X- -> South
    // Y+ -> West  | Y- -> East

    float AZIM = acos(sqrt((pow(cos(Y * DTR), 2) * pow(tan(X * DTR), 2)) / (pow(sin(Y * DTR), 2) + pow(tan(X * DTR), 2)))) * RTD;

    // determine quadrant from X and Y angle, then determine correct azimuth and elevation
    if (X < 0) {
        if (Y < 0) {
            *az = 180 - AZIM;
        } else if (Y > 0) {
            *az = AZIM + 180;
        } else { // X movement but no Y movement case
            *az = 180;
            *el = fabs(X + 90);
            return;
        }
    } else if (X > 0) {
        if (Y < 0) {
            *az = AZIM;
        } else if (Y > 0) {
            *az = (360 - AZIM);
        } else { // X movement but no Y movement case
            *az = 0;
            *el = fabs(X - 90);
            return;
        }
    } else { // X=0 case
        if (Y > 0) {
            *az = 270;
            *el = fabs(Y - 90);
        } else if (Y < 0) {
            *az = 90;
            *el = fabs(Y + 90);
        } else { // Both motors are in 0 position -> could Azimuth=0 confuse software?
            *az = 0;
            *el = 90;
        }
        return;
    }

    *el = atan(cos(*az * DTR) / tan(X * DTR)) * RTD; // since we now have azimuth we can solve elevation
}


void x_goto_posf(float x_target_float)
{
   x_target_pos_float = x_target_float;
   
   if (x_target_float <-MaxRange) 
   {
      x_target_pos_float=-MaxRange;
      SWSer.print("Target limited to ");
      SWSer.println(x_target_pos_float);
   }else
   
   if (x_target_float > MaxRange) 
   {
    x_target_pos_float= MaxRange;
    SWSer.print("Target limited to ");
    SWSer.println(x_target_pos_float);
   }
   
   x_target_pos_puls = angle2pulses(x_target_pos_float); 

   SWSer.print("TargetPosPulses=");
   SWSer.println(x_target_pos_puls);
   
   if(x_moving==false)//stojime
   { 
      if(x_referenced)
      {
         if(x_curr_pos_puls < x_target_pos_puls)
         {
            x_move_direction = true;
            x_move_pos();
         }
         else
         {
            x_move_direction = false;
            x_move_neg();
         }
       }else
       {
         SWSer.println("mes::Unable to move, do referencing first!!!");
         x_go_ref();
       }
   }
   else //jedeme
   {
     if(x_move_direction == true)//jedeme +
     {
        if(x_curr_pos_puls > x_target_pos_puls)//mame jet -
        {
          x_stop_moving();
          delay(150);
          //move_direction = false;
          x_move_neg();//jed -
        }
     }
     else  //jedeme -
     {
        if(x_curr_pos_puls < x_target_pos_puls)//mame jet +
        {
          x_stop_moving();
          delay(150);
          //move_direction = true;
          x_move_pos();//jed +
        }
     }
   } 
}

void y_goto_posf(float y_target_float)
{
   y_target_pos_float = y_target_float;
   
   if (y_target_float <-MaxRange) 
   {
      y_target_pos_float=-MaxRange;
      SWSer.print("Target limited to ");
      SWSer.println(y_target_pos_float);
   }else
   
   if (y_target_float > MaxRange) 
   {
    y_target_pos_float= MaxRange;
    SWSer.print("Target limited to ");
    SWSer.println(y_target_pos_float);
   }
   
   y_target_pos_puls = angle2pulses(y_target_pos_float); 

   SWSer.print("TargetPosPulses=");
   SWSer.println(y_target_pos_puls);
   
   if(y_moving==false)//stojime
   { 
      if(y_referenced)
      {
         if(y_curr_pos_puls < y_target_pos_puls)
         {
            y_move_direction = true;
            y_move_pos();
         }
         else
         {
            y_move_direction = false;
            y_move_neg();
         }
       }else
       {
         SWSer.println("mes::Unable to move, do referencing first!!!");
         y_go_ref();
       }
   }
   else //jedeme
   {
     if(y_move_direction == true)//jedeme +
     {
        if(y_curr_pos_puls > y_target_pos_puls)//mame jet -
        {
          y_stop_moving();
          delay(150);
          //move_direction = false;
          y_move_neg();//jed -
        }
     }
     else  //jedeme -
     {
        if(y_curr_pos_puls < y_target_pos_puls)//mame jet +
        {
          y_stop_moving();
          delay(150);
          //move_direction = true;
          y_move_pos();//jed +
        }
     }
   } 
}

int angle2pulses(float ang)
{
  int angle_pulses = ang * ppd;
  return angle_pulses;
}

float pulses2angle(int pulses)
{
  return pulses/float(ppd);
}

void x_go_ref()
{
    detachInterrupt(digitalPinToInterrupt(X_HALL_SENS));

    if (digitalRead(X_REF_SW) == 0) {
        x_move_pos();
        while (digitalRead(X_REF_SW) == 0) {}
        x_stop_moving();
    } else {
        x_move_neg();
        while (digitalRead(X_REF_SW) == (!0)) {}
        x_stop_moving();
    }

    delay(250);

    x_curr_pos_puls = x_ref_offset_pulses;
    X_RefSwVal = digitalRead(X_REF_SW);

    attachInterrupt(digitalPinToInterrupt(X_HALL_SENS), x_hall_sens_isr, RISING); 
}

void y_go_ref()
{
    detachInterrupt(digitalPinToInterrupt(Y_HALL_SENS));

    if (digitalRead(Y_REF_SW) == 0) {
        y_move_pos();
        while (digitalRead(Y_REF_SW) == 0) {}
        y_stop_moving();
    } else {
        y_move_neg();
        while (digitalRead(Y_REF_SW) == (!0)) {}
        y_stop_moving();
    }

    delay(250);

    y_curr_pos_puls = y_ref_offset_pulses;
    Y_RefSwVal = digitalRead(Y_REF_SW);

    attachInterrupt(digitalPinToInterrupt(Y_HALL_SENS), y_hall_sens_isr, RISING); 
}

void x_move_pos()
{
  x_move_direction = true;
  digitalWrite(X_RELE_NEG, HIGH);
  digitalWrite(X_RELE_POS, LOW);
  x_moving = true;
}

void x_move_neg()
{ 
  x_move_direction = false;
  digitalWrite(X_RELE_POS, HIGH);
  digitalWrite(X_RELE_NEG, LOW);
  x_moving = true;
}

void x_stop_moving()
{
  digitalWrite(X_RELE_NEG, LOW);
  digitalWrite(X_RELE_POS, LOW);
  x_moving = false;
}

void y_move_pos()
{
  y_move_direction = true;
  digitalWrite(Y_RELE_NEG, LOW);
  digitalWrite(Y_RELE_POS, HIGH);
  y_moving = true;
}

void y_move_neg()
{ 
  y_move_direction = false;
  digitalWrite(Y_RELE_POS, LOW);
  digitalWrite(Y_RELE_NEG, HIGH);
  y_moving = true;
}

void y_stop_moving()
{
  digitalWrite(Y_RELE_NEG, LOW);
  digitalWrite(Y_RELE_POS, LOW);
  y_moving = false;
}
