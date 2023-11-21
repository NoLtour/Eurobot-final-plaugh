#include <Arduino.h>
#include <Wire.h>                                             // Calls for I2C bus library

#include <HCSR04.h> 

/*
Error lookup table

1  - MD25 Communication error
2  - Use of deprecated command (spontanious movement) or finished
3  - Slice grab command issued but the plaugh is full at the index specified
4  - Slice grab command failed 
5  - Error check on buffer state in movement manager failed
6  - Something related to zeroing
7  - Fetch movement command run when none exist
8  - Slice grab command, but no slices exist in the region specified
9  - Grabbing slice command error
10 - Behaviour command issued but failed at some stage
11 - Straight movement command has missed it's target and is not able to reach it
12 - Static object buffer is full
13 - 
14 - Reset called on non deleteable tracking object
15 - 
20 - Robot is in a dead state after a kill command was issued

*/

#define GREEN_START (true) // true means green start, false means blue start

#define INIT_X_POS (22.5)
#define INIT_Y_POS (GREEN_START?(200-22.5):22.5)
#define INIT_ANGLE (0.0001)

#define MD25ADDRESS         0x58                              // Address of the MD25
#define SPEED1              0x00                              // Byte to send speed to both motors for forward and backwards motion if operated in MODE 2 or 3 and Motor 1 Speed if in MODE 0 or 1
#define SPEED2              0x01                              // Byte to send speed for turn speed if operated in MODE 2 or 3 and Motor 2 Speed if in MODE 0 or 1
#define ENCODER_RIGHT          0x06                              // Byte to read motor encoder 1
#define ENCODER_LEFT          0x02                              // Byte to read motor encoder 2
#define ACCELERATION        0xE                               // Byte to define motor acceleration
#define CMD                 0x10                              // Byte to reset encoder values
#define MODE_SELECTOR       0xF                               // Byte to change between control MODES
#define MODE                ((byte) 0)

#define SCREEN_PIN 26
#define RGB_PIN    9 
#define ACT_PIN    A8

#define WHEEL_LEFT  SPEED2
#define WHEEL_RIGHT SPEED1
#define FLIP_FACTOR -1

#define MAX_WHEEL_RPS 1
#define WHEEL_DIAMETER   (9.96 )
#define WHEEL_RADIUS     (WHEEL_DIAMETER/2.0)
#define WHEEL_SEPERATION (35.6/2)         // (35.6/2) wheel seperation from the middle! (cm)
#define MOVEMENT_PER_DEG (WHEEL_DIAMETER*PI/360.0)
#define MAX_SPEED (MAX_WHEEL_RPS*WHEEL_DIAMETER)

#define FAST_SPEED_MUL 1.5
#define DEFAULT_SPEED 0.35
#define ABS_MAX_SPEED 0.6
#define SPEED_WH_MAX  (1*DEFAULT_SPEED)
#define SPEED_WH_M_N (-0.65*DEFAULT_SPEED)
#define SPEED_GRAD   (SPEED_WH_MAX-SPEED_WH_M_N)
#define START_SLOW_TIME 400.0
#define ABS_MIN_W_SPEED 0.05
#define MIN_ANGLE_D_FOR_TINY_MOV      (14*DEG_TO_RAD)
#define SLOW_F_SEP_OFFSET      5
#define SLOW_F_SEP_GRAD        35
#define SLOW_F_MIN             0.22
#define MAX_R_MULT             4
#define ANG_F_NO_R_MULT        (15*DEG_TO_RAD)
#define ANG_F_MAX_R_MULT       (70*DEG_TO_RAD)

#define R_MULT_GRAD  ((MAX_R_MULT-1)/(ANG_F_MAX_R_MULT-ANG_F_NO_R_MULT))
#define R_MULT_C     (1-(R_MULT_GRAD*ANG_F_NO_R_MULT))

#define FORWARD_SPEED  DEFAULT_SPEED
#define FWD_R_SPEED    (DEFAULT_SPEED*0.3)
#define BWD_R_SPEED   -(DEFAULT_SPEED*0.3)

#define QD_A_1        (4*(FWD_R_SPEED-BWD_R_SPEED))
#define QD_B_1        (-(1.5*QD_A_1 + 4*(FORWARD_SPEED-BWD_R_SPEED-QD_A_1/8)))
#define QD_C_1        (-(3*QD_A_1/4 + QD_B_1))
#define QD_D_1        (BWD_R_SPEED)

#define QD_A_2        (-QD_A_1)
#define QD_B_2        (-(1.5*QD_A_2 + 4*(FORWARD_SPEED-FWD_R_SPEED-QD_A_2/8)))
#define QD_C_2        (-(3*QD_A_2/4 + QD_B_2))
#define QD_D_2        (FWD_R_SPEED)
 

#define TARGET_MARGIN_BEG 3.0
#define MAX_ANGLE_D_FOR_ADJUSTMENT    (0.3*DEG_TO_RAD)  // MAX angle required before the straight movement attempts some form of angle adjustment
#define MAX_ANGLE_D_FOR_AMPLIFICATION (14*DEG_TO_RAD)   // MAX angle upto which it amplifies the angle deviation for stronger movement 
#define SPEED_RATIO_AMPLIFICATION_FACTOR 2              // Amplification factor for speed ratio, increase to make it do more point rotation
#define DEFAULT_ANGLE_MARGIN (3*DEG_TO_RAD)             // The target angle margin for cases where it is not specified
#define POINT_POINT_PURE_MODE_DISTANCE 8                   // distance from target where point to point mode will exclusively rotate on point instead of dynamic movement
#define POINT_POINT_SMAX_A_DEVIATION (4 *DEG_TO_RAD)         //  
#define POINT_POINT_EMAX_A_DEVIATION (22*DEG_TO_RAD)         //  

#define TARGET_W_OFFSET_MARGIN_AMPLIFICATION 0.5    // The increase in margin when doing a point move with target offset
   

#define WALL_BOUNDARY_BOX (25+ROBOT_PHYSICAL_RADIUS)  // The boundary box around the walls before corrective action becomes considered
#define AVOIDANCE_EXPONENT    1                       // The exponent applied to make tighter movement arround obsitcles
#define AVOIDANCE_ADJ_MUL     (1.0/8.0)              // the linear decay assosiated with object avoidance, lower means stronger avoidance
#define AVOIDANCE_IGNORE_SEP  28                      // The seperation where it completely ignores obsitcles
#define AVOIDANCE_MARGIN      2.6                     // The extra margin added to objects, to make their "size" bigger
#define ROBOT_PHYSICAL_RADIUS (35.0/2)                // The centre to edge maximum that describes the robot, for object avoidance
#define ROBOT_ZEROING_DISTANCE 17.6                   // The physical seperation between the frontmost point and the pivot line on the robot
#define REPULSION_DECREASE_FACT 0.25                  // Should be between 0 and 0.5

#define ROB_TARG_AVOID_IGNORE_SEP 18.0                // The seperation from target where it just ignores the effect of obsticles

#define ZERO_REQ_TRAVEL 350

#define SOLENOID_PLAUGH_PINS 46            // 46 plaugh 22 grabber
#define SOLENOID2_PINS       22            // 
#define MISC_INFO_PIN 13                   // 
#define MISC_INFO_PIN2 29                   // 

#define SLOW_POS_MARGIN 5
#define LOW_POS_MARGIN 2.8
#define MID_POS_MARGIN 1.25
#define HIGH_POS_MARGIN 0.75
#define SLICE_APPROACH_DISTANCE (ROBOT_PHYSICAL_RADIUS-2)         // Seperation from the slice before initiating ram movement
#define SLICE_RAM_DISTANCE      7                                  // The length of the ramming distance
#define SLICE_REVERSE_DISTANCE  (3.5)                                // The reverse distance done after grabbing a slice
#define SLICE_DROPPING_OFFSET   24                                 // The reversing distance after dumping a slice

#define OBJECT_HIT_SELECTION_MARGIN      9   // The cm offset from the radius of a target for it to consider it hit in a ping
#define OBJECT_CREATION_DEPTH_OFFSET     5   // The cm depth added when creating a new tracked object from a ping
#define TRACKING_ADDITION_PER_SUCCESS    (2.0)
#define EXISTANCE_RATING_MAX             (TRACKING_ADDITION_PER_SUCCESS*8)
#define TRACKING_DECAY_PER_SECOND        (EXISTANCE_RATING_MAX/60.0)
#define TRACKING_DECAY_PER_MILLISECOND   (TRACKING_DECAY_PER_SECOND/1000.0)
#define TRACKING_ADDITION_INIT           (TRACKING_ADDITION_PER_SUCCESS*3)
#define TRACKING_LOSS_PER_FAIL           (TRACKING_ADDITION_PER_SUCCESS*0.7)
#define DEFUALT_CHECKING_PERIOD          500 // in ms

enum HitType{ X_WALL, Y_WALL, OTHER };
#define HIT_MARGIN 2

#define MAX_PING_LENGTH 120
#define MIN_PING_LENGTH 8

#define MAX_RC_ANGLE_DEVIATION (DEG_TO_RAD*25)
#define MAX_RC_POS_DEVIATION   12

#define RC_UPD_ANGLE_DEVIATION (DEG_TO_RAD*5)
#define RC_UPD_POS_DEVIATION   3

#define X_WALLS_COUNT 6
#define Y_WALLS_COUNT 2
#define REQ_PING_ANGLE 8
#define WALL_PING_MARGIN 30 
//                                      P  St En
double yWalls [Y_WALLS_COUNT][3] =  { { 0, 0, 200 }, { 300, 0, 200 } };
double xWalls [X_WALLS_COUNT][3] =  { { 0, 0, 300 }, { 200, 0, 300 }, { 100, 0, 30 }, { 100, 270, 300 }, { 30, 135, 165 }, { 170, 135, 165 } };

#define GENERIC_ARR_SIZE 8

#define MAIN_DIST_SENSOR_COUNT 2
#define SEQ_PULSE_COUNT 2
#define PREDICTED_CM_UNCERT 3

#define FREE_TRACKING_BUFFER 3
#define OBJECT_BUFFER_MAX (12 + FREE_TRACKING_BUFFER)
#define FREE_TRACKING_POINTER (OBJECT_BUFFER_MAX - FREE_TRACKING_BUFFER)

void timerCheckCommand();
void closePlaugh();
void openPlaugh();

double plaughSlotPositions[3][2] = { { 18.2, 45.6*DEG_TO_RAD }, { 17.5, 0.001 }, { 18.2, -45.6*DEG_TO_RAD } };
double getSliceDroppingAngle( double yDropPoint ){
  if ( abs(yDropPoint - 100) < 50 ){
    return 0;
  }
  return (yDropPoint<100)?(-PI/2):(PI/2);
}

double putRadInRange( double input ){
  while (input >  PI){ input -= 2.0*PI; }
  while (input < -PI){ input += 2.0*PI; }

  return input;
}

void addScore( int input ){
  for (int i=0; i<input; i++){
    digitalWrite( SCREEN_PIN, HIGH );
    delay( 75 );
    digitalWrite( SCREEN_PIN, LOW );
    delay( 75 );
  }
}

// Flashes error LED, pulsing to communicate
// the error code
void errorFlash( int pulseCount ){
  closePlaugh();
  while(true){
    // Pulse representing error code
    for ( int i=0; i<pulseCount; i++ ){
      analogWrite( MISC_INFO_PIN, 255 );
      delay( 300 );
      analogWrite( MISC_INFO_PIN, 0 );
      delay( 150 );
    }
    // Extended delay before repeating
    analogWrite( MISC_INFO_PIN, 0 );
    delay( 1000 );
  }
}

// returns the closest distance between a ping and some object squared, accounting for line direction
double pingClosestInterceptSQRD( double pinOriginX, double pinOriginY, double pingAngle, double objectX, double objectY ){
  double m = tan( pingAngle );
  double c = pinOriginY - m * pinOriginX;

  double min_x = (objectX + m * ( objectY - c ))/(1 + sq(m));

  if ( (min_x - pinOriginX)/cos(pingAngle) < 0 ){
    return sq(objectX - pinOriginX) + sq(objectY - pinOriginY);
  }

  double min_y = m * min_x + c;

  return sq(objectX - min_x) + sq(objectY - min_y);
}


void stopMotors(){                                           // Function to stop motors
  Wire.beginTransmission(MD25ADDRESS);                      // Sets the acceleration to register 10 (0.65s)
  Wire.write(ACCELERATION);
  Wire.write(10);
  Wire.endTransmission();

  Wire.beginTransmission(MD25ADDRESS);                      // Stops motors motor 1 if operated in MODE 0 or 1 and Stops both motors if operated in MODE 2 or 3
  Wire.write(SPEED1);
  Wire.write(128);
  Wire.endTransmission();

  Wire.beginTransmission(MD25ADDRESS);                      // Stops motors motor 2 when operated in MODE 0 or 1 and Stops both motors while in turning sequence if operated in MODE 2 or 3
  Wire.write(SPEED2);
  Wire.write(128);
  Wire.endTransmission();
  delay(150);
}  

void resetEncoders(){                                         // This function resets the encoder values to 0
  Wire.beginTransmission(MD25ADDRESS);
  Wire.write(CMD);
  Wire.write(0x20);                                         
  Wire.endTransmission(); 
  delay(50);
}

// returns in rad
int RME_Rec = 0;
double readMotorEncoder( byte encoderAdress ){
  RME_Rec++;
  Wire.beginTransmission(MD25ADDRESS);                      // Send byte to get a reading from encoder 1
  Wire.write(encoderAdress);
  Wire.endTransmission();

  Wire.requestFrom(MD25ADDRESS, 4);                         // Request 4 bytes from MD25
  //Serial.println("waiting");
  unsigned long startT = millis();
  while(Wire.available() < 4){ if ( millis()-startT > 1000 ){ 
    if ( RME_Rec > 5 ){
      errorFlash( 1 );
    }
    double ret = readMotorEncoder( encoderAdress );
    RME_Rec--;
    return ret;
  } };    // Wait for 4 bytes to arrive
  Wire.clearWriteError();
  //Serial.println("got respons");
  long poss1 = Wire.read();                                 // First byte for encoder 1, HH.
  poss1 <<= 8;
  poss1 += Wire.read();                                     // Second byte for encoder 1, HL
  poss1 <<= 8;
  poss1 += Wire.read();                                     // Third byte for encoder 1, LH
  poss1 <<= 8;
  poss1  +=Wire.read();                                     // Fourth byte for encoder 1, LLalue
  delay(5);                                                 // Wait for everything to make sure everything is sent
  RME_Rec--;
  return(poss1*DEG_TO_RAD*FLIP_FACTOR);                                       // Convert encoder value to rad
}

double readMotorLeftEncoder(){
  return readMotorEncoder( ENCODER_LEFT );
}

double readMotorRightEncoder(){
  return readMotorEncoder( ENCODER_RIGHT );
}

// speed between -1 and 1
void setWheelSpeed( byte wheelAddress, double speed ){
  if ( speed>ABS_MAX_SPEED ){
    speed = (speed<0?-ABS_MAX_SPEED:ABS_MAX_SPEED);
  }


  Wire.beginTransmission(MD25ADDRESS);                     // Sets the acceleration to register 1 (6.375s)
    Wire.write(ACCELERATION);
    Wire.write(1);
    Wire.endTransmission();

    int rSpeed = max(min((FLIP_FACTOR*speed*128)+128, 255), 0);

    Wire.beginTransmission(MD25ADDRESS);                     // Sets a combined motor speed value
    Wire.write(wheelAddress);
    Wire.write(rSpeed);
    Wire.endTransmission();
}

double cLeftSpeed = 0; 
// in 
void setWheelLeftSpeed( double speed ){
  setWheelSpeed( WHEEL_LEFT, speed );
  cLeftSpeed = speed;
}

double cRightSpeed = 0;
// in 
void setWheelRightSpeed( double speed ){
  setWheelSpeed( WHEEL_RIGHT, speed );
  cRightSpeed = speed;
}

double averageAngles( double angle1, double angle2 ){
  return atan2( sin(angle1)+sin(angle2), cos(angle1)+cos(angle2) );
}

enum PingFailureType{ TOO_CLOSE, NO_RESPONSE, TOO_FAR, OUT_OF_BOUNDS };

class Sensor{
public:
  double lastAbsX;
  double lastAbsY;

  double muX;
  double muY;
  double muL;
  double muAlpha;
  double lambda;
  int pin;
  
  bool badPing;
  PingFailureType failType;

  double rawPingLength;
  HitType objectHit;
  double hitData;

  double predXHit;
  double predYHit;

  Sensor( double nMuX, double nMuY, double nMuLambda, int nPin ){
    muX = nMuX;
    muY = nMuY;
    muAlpha = atan2( muY, muX );
    muL = sqrt( sq( muX ) + sq( muY ) );
    lambda = nMuLambda;

    pin = nPin;
  }
  
  void estimatePosition( int inpX, int inpY, double inpTheta ){
    lastAbsX = getXCentOffset( inpTheta ) + inpX;
    lastAbsY = getYCentOffset( inpTheta ) + inpY;

    predXHit =  lastAbsX + rawPingLength*cos(inpTheta + lambda);
    predYHit =  lastAbsY + rawPingLength*sin(inpTheta + lambda);

    if ( predXHit > 300 || predXHit < 0 ){
      badPing = true;
      failType = OUT_OF_BOUNDS;
    }else if ( predYHit > 200 || predYHit < 0 ){
      badPing = true;
      failType = OUT_OF_BOUNDS;
    }
  }

  void validateAngleInRange( int inpX, int inpY, double inpTheta ){
    double hitAngle = 0;

    switch ( objectHit )
    {
    case Y_WALL:{
      if ( inpX < hitData ){
        hitAngle = (lambda + inpTheta);
      }
    }
      break;
    
    default:
      break;
    }
  }

  void updateObjectHit( int inpX, int inpY, double inpTheta ){
    estimatePosition( inpX, inpY, inpTheta );
    
    for ( int i=0; i<Y_WALLS_COUNT; i++ ){  
      if ( abs(yWalls[i][0] - predXHit) < WALL_PING_MARGIN &&  yWalls[i][1] < predYHit && yWalls[i][2] > predYHit ){
        objectHit = Y_WALL;
        hitData = yWalls[i][0];
        return;
      }
    }
    
    for ( int i=0; i<X_WALLS_COUNT; i++ ){  
      if ( abs(xWalls[i][0] - predYHit) < WALL_PING_MARGIN &&  xWalls[i][1] < predXHit && xWalls[i][2] > predXHit ){
        objectHit = X_WALL;
        hitData = xWalls[i][0];
        return;
      }
    }

    objectHit = OTHER;
  }

  double calculateXPosition( double inpTheta, double xPos, double yPos ){
    return hitData + xPos - (predXHit);
    /*if ( predXHit < xPos ){
      return (predXHit-hitData);
    }else{
      return  (hitData-predXHit);
    }*/
  }

  double calculateYPosition( double inpTheta, double xPos, double yPos ){
    return hitData + yPos - (predYHit);
    /*if ( predYHit < yPos ){
      return  (predYHit-hitData);
    }else{
      return  (hitData-predYHit);
    }*/
  }

  double getXCentOffset( double theta ){
    return muL * cos( theta + muAlpha );
  }

  double getYCentOffset( double theta ){
    return muL * sin( theta + muAlpha );
  }
  
  void printUsefulData( double inpTheta, double xPos, double yPos ){
    Serial.print("  P");
    Serial.print(pin);

    if ( badPing ){
      Serial.print(":ERR");
      return;
    }

    switch (objectHit)
    {
    case X_WALL:{
    //Serial.print("-y:");
    //Serial.print( calculateYPosition( inpTheta, xPos, yPos ) );
    }break;

    case Y_WALL:{
    //Serial.print("-x:");
    //Serial.print( calculateXPosition( inpTheta, xPos, yPos ) );
    }break;
    
    default:{
    Serial.print("-U:");
    //Serial.print( 0 );
    }break;
    }
    Serial.print(" x:");
    Serial.print(predXHit);
    Serial.print(" y:");
    Serial.print(predYHit);
    //Serial.print(" r:");
    //Serial.print(rawPingLength);
  }
};  

byte triggerPinMain = 2;
byte echoCount = MAIN_DIST_SENSOR_COUNT;

byte triggerPinSecond = 14;


Sensor * main_Sensors = new Sensor [MAIN_DIST_SENSOR_COUNT] { 
   Sensor( 10.3, -8.2 ,0.0001, 7  ),
   Sensor( 10.3,  8.2 ,0.0001, 6  )
   //Sensor( -5.5 ,9.5,(180-5)*DEG_TO_RAD ,9 ),
   //Sensor( 6.5, 13 ,-(-5)*DEG_TO_RAD ,8 )
};
byte* echoPins = new byte[echoCount];

class PosSensorManager{
public: 
  double predX;
  double predY;
  double predTheta;

  double allPulseDistances [MAIN_DIST_SENSOR_COUNT][SEQ_PULSE_COUNT];
  double avDistances [MAIN_DIST_SENSOR_COUNT];

  
  double angleArray [MAIN_DIST_SENSOR_COUNT*MAIN_DIST_SENSOR_COUNT/2];

  PosSensorManager(){

  }

  double calculateRobotAngle( Sensor * sensor1, Sensor * sensor2, double angleAdjustment, double inpTheta ){

    // this compensates for the sensors not being at centre
    double d = sensor1->lambda - sensor2->lambda;
    double H = sqrt( sq( sensor1->muX - sensor2->muX ) + sq( sensor1->muY - sensor2->muY ) );
    double beta = atan2( sensor1->muY - sensor2->muY, sensor1->muX - sensor2->muX );

    double adjLength1 = abs(H*sin(beta - sensor2->lambda)/sin(d) + sensor1->rawPingLength);
    double adjLength2 = abs(H*sin(beta - sensor1->lambda)/sin(d) + sensor2->rawPingLength);

    Serial.print("  a1:");
    Serial.print(adjLength1 );
    Serial.print("  a2:");
    Serial.print(adjLength2  );/* */

    // actual angle calc

    double adjAngle1 = sensor1->lambda - angleAdjustment + PI/2;
    double adjAngle2 = sensor2->lambda - angleAdjustment + PI/2;

    double alpha = abs( adjAngle1 - adjAngle2 );
    double L = sqrt( sq(adjLength1) + sq(adjLength2) - adjLength1*adjLength2*cos(alpha) );
           beta = asin( adjLength2 * sin( alpha )/ L );
    
    double estTheta = (beta<inpTheta+adjAngle1+angleAdjustment)?( PI-beta-adjAngle1 ):( beta-adjAngle1 );

    Serial.print("  g1:");
    Serial.print(adjAngle1*RAD_TO_DEG);
    Serial.print("  g2:");
    Serial.print(adjAngle2*RAD_TO_DEG);
    Serial.print("  L:");
    Serial.print(L);
    Serial.print("  b:");
    Serial.print(beta);
    // adjusts estimate to thingy
    estTheta = putRadInRange(estTheta + angleAdjustment - PI/2);

    return estTheta;
  }

  void estimatePosition( double inpX, double inpY, double inpTheta ){
    
    // this gets all the angles from calculations and puts it into "estAngle"
    int fAngles = 0;
    double angleAggrigate = 0;

    /*for ( int I=0; I<MAIN_DIST_SENSOR_COUNT; I++ ){
      Sensor * sensor1 = &(main_Sensors[ I ]);
      if ( !sensor1->badPing ){
      for ( int i=I+1; i<MAIN_DIST_SENSOR_COUNT; i++ ){
        Sensor * sensor2 = &(main_Sensors[ i ]);
        if ( !sensor2->badPing ){
        switch (sensor1->objectHit){
        case X_WALL:
        case Y_WALL:
          if ( sensor1->objectHit == sensor2->objectHit && sensor1->hitData == sensor2->hitData ){
            double angleAdjustment;

            if ( sensor1->objectHit == X_WALL ){
              if ( inpY > sensor1->hitData ){
                angleAdjustment = 0;
              }else{
                angleAdjustment = 0;
              }
            }else{
              if ( inpX > sensor1->hitData ){
                angleAdjustment = 0;
              }else{
                angleAdjustment = PI/2;
              }
            }

            double estAngle = calculateRobotAngle( sensor1, sensor2, angleAdjustment, inpTheta );

            if ( estAngle != 0
              && abs(estAngle-inpTheta) < MAX_RC_ANGLE_DEVIATION
             ){
              angleAggrigate += estAngle;
              fAngles++;
            }

            Serial.print("  O:");
            Serial.print(estAngle*RAD_TO_DEG);


          }
          break;
        
        default:break;
        }
        } 
      }}
    }*/

    int fXpos = 0;
    double xPosAggrigate = 0;

    int fYpos = 0;
    double yPosAggrigate = 0;

    for ( int I=0; I<MAIN_DIST_SENSOR_COUNT; I++ ){
      Sensor * cSensor = &(main_Sensors[ I ]);
      if ( !cSensor->badPing ){
        switch (cSensor->objectHit)
        {
        case Y_WALL:{
          double estX = cSensor->calculateXPosition( inpTheta, inpX, inpY );
          if ( abs(estX-inpX) < MAX_RC_POS_DEVIATION ){
            fXpos ++;
            xPosAggrigate += estX;
          }
          }break;

        case X_WALL:{
          double estY = cSensor->calculateYPosition( inpTheta, inpX, inpY );
          if ( abs(estY-inpY) < MAX_RC_POS_DEVIATION ){
            fYpos ++;
            yPosAggrigate += estY;
          }
          }break;
        
        default:
          break;
        }
        
      }
    }
    
    angleAggrigate /= fAngles; fAngles = 0;
    xPosAggrigate  /= fXpos;
    yPosAggrigate  /= fYpos;

    predX = (fXpos != 0 && abs(inpX-xPosAggrigate) > RC_UPD_POS_DEVIATION)?
                    ( (inpX + xPosAggrigate)/2 )  :  inpX;

    predY = (fYpos != 0 && abs(inpY-yPosAggrigate) > RC_UPD_POS_DEVIATION)?
                    ( (inpY + yPosAggrigate)/2 )  :  inpY;

    predTheta = (fAngles != 0 && abs(inpTheta-angleAggrigate) > RC_UPD_ANGLE_DEVIATION )?
                    ( (inpTheta + angleAggrigate)/2 )  :  inpTheta;

    Serial.print("");
  }

  void printPingLens(){
    // sends out pulse, and does basic averaging to get ping lengths
    sendPulse();

    for ( int i=0; i<MAIN_DIST_SENSOR_COUNT; i++ ){ 
      Serial.print("   S");
      Serial.print( main_Sensors[i].pin ); // Sensor pin
      Serial.print(": ");
      Serial.print( main_Sensors[i].rawPingLength ); // Raw length assosiated with ping
      Serial.print(",");  
      if ( main_Sensors[i].badPing ){
      switch(main_Sensors[i].failType){
        case TOO_CLOSE:
          Serial.print("TC"); // Too close error
          break;
        case NO_RESPONSE:
          Serial.print("NR"); // No response error
          break;
        case TOO_FAR:
          Serial.print("TF"); // Too far error
          break;
        case OUT_OF_BOUNDS:
          Serial.print("OO"); // Out of bounds error
          break;
      }
      }else{
          Serial.print("Fn"); // No error
      }
    }
    Serial.println("");
  } 

  bool checkSomethingExists(){
    sendPulse();

    for ( int i=0; i<MAIN_DIST_SENSOR_COUNT; i++ ){ 
      if ( !main_Sensors[i].badPing && main_Sensors[i].rawPingLength < 30 ){
        return true;
      }
    }
    
    return false;
  } 

  void printWallData( double inpTheta, double xPos, double yPos ){   
    for ( int i=0; i<MAIN_DIST_SENSOR_COUNT; i++ ){
      main_Sensors[i].printUsefulData( inpTheta, xPos, yPos );
    }
  }

  void preformBasicSensorCalcs( double inpTheta, double xPos, double yPos ){   
    for ( int i=0; i<MAIN_DIST_SENSOR_COUNT; i++ ){
      main_Sensors[i].estimatePosition( xPos, yPos, inpTheta );
      main_Sensors[i].updateObjectHit(  xPos, yPos, inpTheta);
    }
  }

  void sendPulse(){  
    for (int i=0; i<MAIN_DIST_SENSOR_COUNT; i++){
      avDistances[i] = 0; 

      for (int n=0; n<SEQ_PULSE_COUNT; n++){
        long startT = millis();
        double* distances = HCSR04.measureDistanceCm(  );
        startT = millis() - startT;

        for (int i = 0; i < echoCount; i++) {
          /*if (i > 0) Serial.print(" | ");
          Serial.print(i + 1);
          Serial.print(": ");
          Serial.print(distances[i]);
          Serial.print(" cm | took ");*/
          avDistances[i] += distances[i];
          allPulseDistances[i][n] = distances[i];
        }
        delay(10);
      }

      for (int i=0; i<MAIN_DIST_SENSOR_COUNT; i++){
        avDistances[i] /= SEQ_PULSE_COUNT;

        double closestSeperation = 300;
        double closestValue = 0;
        for ( int n=0; n<SEQ_PULSE_COUNT; n++ ){
          int fSep = abs( avDistances[i] - allPulseDistances[i][n] );
          if ( fSep < closestSeperation ){
            closestSeperation = fSep;
            closestValue = allPulseDistances[i][n];
          }
        }
        /*Serial.print( i );

        Serial.print( " |inAv: " );
        Serial.print(avDistances[i]);*/

        avDistances[i] = 0;
        int avCount = 0;
        for ( int n=0; n<SEQ_PULSE_COUNT; n++ ){
          if ( abs( closestValue - allPulseDistances[i][n] ) < PREDICTED_CM_UNCERT ){
            avCount++;
            avDistances[i] += allPulseDistances[i][n];
          }
        }
        avDistances[i] /= avCount; 

        main_Sensors[i].rawPingLength = avDistances[i];
        if ( avDistances[i] > MAX_PING_LENGTH || avCount <= 1 ){
          main_Sensors[i].badPing = true;
          main_Sensors[i].failType = TOO_FAR;
        }else if( avDistances[i] < MIN_PING_LENGTH ){
          main_Sensors[i].badPing = true;
          main_Sensors[i].failType = avDistances[i]<=0?NO_RESPONSE:TOO_CLOSE;
        }else{ 
          main_Sensors[i].badPing = false;
        }
      }
    }
  }
};

PosSensorManager * posSensorManager = new PosSensorManager();

enum DetectedObjectType{ WALL, UNKNOWN, SLICE, ROBOT, STACKED_SLICES };

double repulsionFactor( DetectedObjectType inpObject ){
  switch (inpObject){
  case SLICE: return   16.0/2 + AVOIDANCE_MARGIN  + ROBOT_PHYSICAL_RADIUS - 3.5;
  case WALL:  return   4.0    + AVOIDANCE_MARGIN  + ROBOT_PHYSICAL_RADIUS;
  case UNKNOWN:
  case STACKED_SLICES: 
  case ROBOT: return   35.0/2 + AVOIDANCE_MARGIN + ROBOT_PHYSICAL_RADIUS;
  }

  return 0;
}
 
enum BotErrorType { POINT_INTERCEPTS_OBJECT, NONE };

class TrackedObject{
private:
  unsigned long lastCalcUpdate = 0;
  double existanceRating = 0;

public:
  bool nonDeleteable = false;
  bool deleted;
  bool ignore;

  bool forDynamicAllocation = false;

  int indexID = -2;

  double repulsionRadius; 
  double repulsionRadiusSQ; 
  double checkProxSQ; 
  DetectedObjectType type;

  double xPos;
  double yPos;

  double selectionRadiusSQ;
  double seperationBufferSQ; // variable used for a very specific function to save processing time

  bool suggestSlow;

  double returnArray [3];

  unsigned long lastUpdate;
  int miscData; // for a wall this is the corresponding ID

  bool hadError;

  TrackedObject(){
    reset( true );
  }

  // recalculates the existance rating and returns it
  double getExistanceRating(){
    if ( nonDeleteable ){
      return EXISTANCE_RATING_MAX;
    }

    if ( !(indexID<FREE_TRACKING_POINTER) ){
      existanceRating = max( existanceRating - (millis() - lastCalcUpdate)*(TRACKING_DECAY_PER_MILLISECOND), 0);
    }

    lastCalcUpdate = millis();

    return existanceRating;
  }

  void setMaxExistance(){
    getExistanceRating();
    existanceRating = EXISTANCE_RATING_MAX;
  }

  void addExistanceHit(){
    existanceRating = min( EXISTANCE_RATING_MAX, getExistanceRating() + TRACKING_ADDITION_PER_SUCCESS );
  } 
  void addExistanceHit( double amount ){
    existanceRating = min( EXISTANCE_RATING_MAX, getExistanceRating() + amount );
  } 

  void removeExistanceHit(){
    existanceRating = max( 0, getExistanceRating() - (indexID<FREE_TRACKING_POINTER?TRACKING_LOSS_PER_FAIL/222:TRACKING_LOSS_PER_FAIL)  );
  }

  void setObjectType( DetectedObjectType inpType, double x, double y ){
    reset( false );
    deleted = false;
    nonDeleteable = inpType==STACKED_SLICES;

    xPos = x;
    yPos = y;

    type = inpType;
    repulsionRadius = repulsionFactor( inpType );
    repulsionRadiusSQ = sq( repulsionRadius );
    checkProxSQ       = sq( repulsionRadius + AVOIDANCE_IGNORE_SEP );
    selectionRadiusSQ = sq( repulsionRadius + OBJECT_HIT_SELECTION_MARGIN - (ROBOT_PHYSICAL_RADIUS + AVOIDANCE_MARGIN) );
    ignore = false;

    lastUpdate = millis();
    lastCalcUpdate = lastUpdate;
  }

  void addPositionData( double x, double y ){
    xPos = (xPos + x)/2;
    yPos = (yPos + y)/2;

    lastUpdate = millis();
  }

  void overwritePositionData( double x, double y ){
    xPos = x;
    yPos = y;

    lastUpdate = millis();
  }
  
  void reset( bool nonDeleteOveride ){
    if ( nonDeleteable && !nonDeleteOveride ){
      errorFlash( 14 );
    }

    deleted = true;
    ignore = true;
    lastUpdate = -1;

    xPos = -1;
    yPos = -1; 

    if ( indexID < FREE_TRACKING_POINTER ){
      existanceRating = EXISTANCE_RATING_MAX;
    }else{
      existanceRating = 0;
    }

    lastUpdate = millis();
    lastCalcUpdate = lastUpdate;
  }

  bool checkIntercepting( double inpX, double inpY ){
    return repulsionRadiusSQ > getSeperationSQ( inpX, inpY );
  } 

  bool checkRayIntercepting( double inpX, double inpY, double angle ){ 
    return pingClosestInterceptSQRD( inpX, inpY, angle, xPos, yPos ) < repulsionRadiusSQ;
  }

  double getSeperationSQ( double inpX, double inpY ){
    return ( sq( inpX - xPos ) + sq( inpY - yPos ) );
  }

  double getSignificanceMultip( double inpX, double inpY ){
    if ( ignore ){
      return 0;
    }

    double sepSQ = getSeperationSQ( inpX, inpY );

    if ( sepSQ > checkProxSQ ){
      return 0;
    }

    double sep = sqrt(sepSQ) - repulsionRadius;
 
    suggestSlow = sep<8;

    return pow(1.0/(max(0.01, sep) * AVOIDANCE_ADJ_MUL), AVOIDANCE_EXPONENT);
  }

  bool avoidanceMemoryDir = false;
  unsigned long avoidanceMemoryTime = 0;
  double getRepulsionAngle( double inpX, double inpY, double inpAngle ){

    //return atan2( inpY-yPos, inpX-xPos );

    bool clockwise;

    /*if ( millis()-avoidanceMemoryTime > 400 ){
      clockwise = (inpY>100) != ( ((-PI/2) < inpAngle) && ((PI/2) > inpAngle) );
      avoidanceMemoryDir = clockwise;
    }else{
      clockwise = avoidanceMemoryDir;
    }

    avoidanceMemoryTime = millis();

    return atan2( inpY-yPos, inpX-xPos ) + (clockwise?(-PI*0.3):(PI*0.3));*/

    double oppositeAngle = atan2( inpY-yPos, inpX-xPos );

    double a1 = oppositeAngle + PI*REPULSION_DECREASE_FACT;
    double a2 = oppositeAngle - PI*REPULSION_DECREASE_FACT;

    if ( abs(putRadInRange( a1-inpAngle )) < abs(putRadInRange( a2-inpAngle )) ){
      return a1;
    }else{
      return a2;
    }
  }

  double * getRepulsionAngleComponents( double inpX, double inpY, double inpAngle ){
    returnArray[2] = -1;
    if ( ignore || getExistanceRating()<(TRACKING_ADDITION_PER_SUCCESS) ){
      return returnArray;
    }

    double avoidMult = getSignificanceMultip( inpX, inpY );

    // check if it actually needs to account for anything
    if ( avoidMult != 0 ){
      double avoidAngle = getRepulsionAngle( inpX, inpY, inpAngle );

      returnArray[0] = avoidMult*cos(avoidAngle);
      returnArray[1] = avoidMult*sin(avoidAngle);
      
      returnArray[2] = 1;
    }

    return returnArray;
  }
};

TrackedObject allTrackedObjects [OBJECT_BUFFER_MAX];

#define STATIC_OBSITCAL_COUNT 2
TrackedObject allStaticObsticals [STATIC_OBSITCAL_COUNT];
TrackedObject wallSurrigate;

enum Tracking_Buffer_Access_Type { FREE_ONLY, COMPLETE, STATIC_ONLY };

// returns a tracking object to put data in, if the buffer is full it will overwrite the oldest detected one
TrackedObject * newTrackedObject( Tracking_Buffer_Access_Type fullBufferAccess ){
  int endingPointer   = (fullBufferAccess==STATIC_ONLY?FREE_TRACKING_POINTER:OBJECT_BUFFER_MAX);
  int startingPointer = (fullBufferAccess==FREE_ONLY?FREE_TRACKING_POINTER:0);

  TrackedObject * shitestObject = &(allTrackedObjects[startingPointer]);
  
  for ( int i=startingPointer; i<endingPointer; i++ ){
    TrackedObject * cTObject = &(allTrackedObjects[i]); 

    cTObject->indexID = i;

    if ( cTObject->deleted ){
      Serial.print(" Ov");
      Serial.print(cTObject->indexID);
      Serial.print("? ");
      return cTObject;
    }

    if ( cTObject->getExistanceRating() < shitestObject->getExistanceRating() && cTObject->type != STACKED_SLICES ){
      shitestObject = cTObject;
    }
  }

  Serial.print(" Ov");
  Serial.print(shitestObject->deleted);
  Serial.print(":");
  Serial.print(shitestObject->indexID);
  Serial.print("! ");

  shitestObject->reset( false );

  if ( fullBufferAccess == STATIC_ONLY ){
    errorFlash( 12 );
  }

  return shitestObject;
}

TrackedObject * newTrackedObject( DetectedObjectType type, double xPos, double yPos, Tracking_Buffer_Access_Type fullBufferAccess ){
  TrackedObject * trackedObject = newTrackedObject( fullBufferAccess );
  trackedObject->setObjectType( type, xPos, yPos );

  return trackedObject;
}

#define MOVEMENT_BUFFER_MAX 8
#define MOVEMENT_COMMAND_SUCCESS_BUFFER 6

// returns index if hit an object, and -1 if none hit. This method makes use of a vector raycast
int checkHitsAnyObjects( double xOrigin, double yOrigin, double angle ){
  for ( int i=0; i<OBJECT_BUFFER_MAX; i++ ){
    if ( allTrackedObjects[i].checkRayIntercepting( xOrigin, yOrigin, angle ) && allTrackedObjects[i].type!=SLICE ){
      return i;
    }
  }

  return -1;
}

// returns the object nearest to the ping position, while EXCLUDING objects outside selection radius
int findNearestObjectHit( double xInput, double yInput ){
  double mHSep = 999999;
  int nOIndex = -1; 
  
  for (int i=0; i<OBJECT_BUFFER_MAX; i++){

    if ( !(allTrackedObjects[i]).ignore ){
      double nSep = sq((allTrackedObjects[i]).xPos - xInput) + sq((allTrackedObjects[i]).yPos - yInput);

      if ( nSep < mHSep && (allTrackedObjects[i]).selectionRadiusSQ > nSep ){
        mHSep = nSep;
        nOIndex = i; 
      }
    }
  } 

  Serial.print( mHSep );
  Serial.print( "-" );

  return nOIndex;
}

enum MovementType{ POINT_ROTATION, STRAIGHT, POINT_TO_POINT, LINE_ARC, SPONTANIOUS, COMPLEX };

int nextMovementCommandID = 0;

class MovementCommand{
public:
  MovementType commandType;
  int commandID;

  double x = 0;
  double y = 0;
  double theta = 0;

  double margin = 0;
  double angleMargin = 0;

  bool ignoreCollisions;
  bool canReverse;

  MovementCommand(){
    reset();
  }

  void reset(){
    margin           = 1;  
    angleMargin      = 999;
    ignoreCollisions = false;
    commandID = nextMovementCommandID;
    nextMovementCommandID++;
    canReverse = false;
  }
};



void forgetSlicesInRegion( double x1, double y1, double x2, double y2 ){
  if ( y1 > y2 ){
    double tmp = y1;
    y1 = y2;
    y2 = y1;
  }
  if ( x1 > x2 ){
    double tmp = x1;
    x1 = x2;
    x2 = x1;
  }

  for ( int i=0; i<OBJECT_BUFFER_MAX; i++ ){
    TrackedObject * cObject = &(allTrackedObjects[i]);
    if ( cObject->type == SLICE && ( cObject->xPos > x1 && cObject->xPos < x2 ) && ( cObject->yPos > y1 && cObject->yPos < y2 ) ){
      cObject->deleted = true;
      cObject->ignore  = true;
    }
  }
}

void forgetAllSlices(){
  forgetSlicesInRegion( 0, 0, 300, 200 );
}

class MovementManager{
public:
  bool disableCommandLegalCheck = false;

  double yDistanceCounter = 0;
  double xDistanceCounter = 0;

  unsigned long lastPosUpdate = 0;
  double lastLEncVal = 0;
  double lastREncVal = 0;

  double targSeperation = 0;

  double xPos  =  INIT_X_POS;
  double yPos  =   INIT_Y_POS;
  double theta =  INIT_ANGLE;

  bool   hasTarget   = false;  
  double targetTheta = 0;
  double target_grad = 0; 
  double target_c    = 0;
  unsigned long lastCommandTime = 0; 

  double prevSeperation = 0;
  int    cComMode       = 0;

  bool canBeFast = false;
  bool isNearObject = false;

  MovementCommand movementBuffer [MOVEMENT_BUFFER_MAX]; 
  int movementCommandSuccessBuffer [MOVEMENT_COMMAND_SUCCESS_BUFFER];
  int CSBPointer = 0;
  MovementCommand * cMovCommand;
  int    commandsLeft = 0;
  int    bufferStartP = 0;

  bool ignoreObjectCollision = false;
  bool ignoreWallCollision   = false;
  
  BotErrorType lastError = NONE;

  MovementManager(){
    for (int i=0; i<MOVEMENT_COMMAND_SUCCESS_BUFFER; i++){
      movementCommandSuccessBuffer[i] = -1;
    }
  }

  bool checkCommandSuccess( int commandID ){
      Serial.print("c");
      Serial.print(commandID);
      Serial.print("! ");
    for (int i=0; i<MOVEMENT_COMMAND_SUCCESS_BUFFER; i++){
      Serial.print(i);
      Serial.print("-");
      Serial.print(movementCommandSuccessBuffer[i]);
      Serial.print(" ");
      if ( movementCommandSuccessBuffer[i] == commandID ){
        return true;
      }
    }
    return false;
  }
  
  int getMostRecentCommandID(){ 
    return getMostRecentCommand()->commandID;
  }
  
  MovementCommand * getMostRecentCommand(){
    if ( commandsLeft==0 ){
      errorFlash( 7 ); 
    }
    int lastCommandIndex = (bufferStartP + commandsLeft - 1)%MOVEMENT_BUFFER_MAX; 

    return &(movementBuffer[lastCommandIndex]);
  }

  double adjustTargetAngleByCollideables( double inpAngle ){
    isNearObject = false;

    if ( cMovCommand->ignoreCollisions ){
      return inpAngle;
    }

    if ( prevSeperation < ROB_TARG_AVOID_IGNORE_SEP ){
      return inpAngle;
    }

    double angXCompAdditions = cos(inpAngle);
    double angYCompAdditions = sin(inpAngle);


    if ( !ignoreObjectCollision ){
      for ( int i=0; i<OBJECT_BUFFER_MAX; i++ ){
        TrackedObject * cObject = &(allTrackedObjects[i]);

        double * returnedComponents = cObject->getRepulsionAngleComponents( xPos, yPos, theta );

        if ( returnedComponents[2] != -1 ){
          angXCompAdditions += returnedComponents[0];
          angYCompAdditions += returnedComponents[1];

          canBeFast = false;
          isNearObject = isNearObject || cObject->suggestSlow; 
        }
      }
    }

    if ( !ignoreWallCollision ){
      for (int i=0; i<STATIC_OBSITCAL_COUNT; i++){
        TrackedObject * cObject = &(allStaticObsticals[i]);

        double * returnedComponents = cObject->getRepulsionAngleComponents( xPos, yPos, theta );

        if ( returnedComponents[2] != -1 ){
          angXCompAdditions += returnedComponents[0];
          angYCompAdditions += returnedComponents[1];

          canBeFast = false;
        }
      }

      if ( xPos > 300-WALL_BOUNDARY_BOX ){
        wallSurrigate.overwritePositionData( 300, yPos );
        double * returnedComponents = wallSurrigate.getRepulsionAngleComponents( xPos, yPos, theta );

        if ( returnedComponents[2] != -1 ){
          angXCompAdditions += returnedComponents[0];
          angYCompAdditions += returnedComponents[1];

          canBeFast = false;
        }
      }else if ( xPos < WALL_BOUNDARY_BOX ){
        wallSurrigate.overwritePositionData( 0, yPos );
        double * returnedComponents = wallSurrigate.getRepulsionAngleComponents( xPos, yPos, theta );

        if ( returnedComponents[2] != -1 ){
          angXCompAdditions += returnedComponents[0];
          angYCompAdditions += returnedComponents[1];

          canBeFast = false;
        }
      }

      if ( yPos > 200-WALL_BOUNDARY_BOX ){
        wallSurrigate.overwritePositionData( xPos, 200 );
        double * returnedComponents = wallSurrigate.getRepulsionAngleComponents( xPos, yPos, theta );
      

        if ( returnedComponents[2] != -1 ){
          angXCompAdditions += returnedComponents[0];
          angYCompAdditions += returnedComponents[1];

          canBeFast = false;
        }
      }else if ( yPos < WALL_BOUNDARY_BOX ){
        wallSurrigate.overwritePositionData( xPos, 0 );
        double * returnedComponents = wallSurrigate.getRepulsionAngleComponents( xPos, yPos, theta );

        if ( returnedComponents[2] != -1 ){
          angXCompAdditions += returnedComponents[0];
          angYCompAdditions += returnedComponents[1];

          canBeFast = false;
        }
      }
    }

    digitalWrite( MISC_INFO_PIN2, isNearObject?HIGH:LOW );

    inpAngle = atan2( angYCompAdditions, angXCompAdditions );
 
    return inpAngle;
  }

  void printState(  ){
    printState( -1 );
  }

  void printState( int numbInp ){
    if ( numbInp != -1 ){ 
    Serial.println("");
    Serial.print("\ti: ");
    Serial.print(numbInp);
    }
    Serial.print("\txPos: ");
    Serial.print(xPos);
    Serial.print("\tyPos: ");
    Serial.print(yPos);
    Serial.print("\ttheta: ");
    Serial.print(theta/DEG_TO_RAD);
    Serial.print("\tC: ");
    Serial.print(bufferStartP);
    Serial.print("-");
    Serial.print(commandsLeft); 
    Serial.print("\tTx: ");
    Serial.print(cMovCommand->x);
    Serial.print("\tTy: ");
    Serial.print(cMovCommand->y);
    Serial.print("\tSe: ");
    Serial.print(max( abs( cMovCommand->x - xPos ), abs( cMovCommand->y - yPos ) ));
    Serial.print("\tTT: ");
    Serial.print(targetTheta/DEG_TO_RAD);
    Serial.print("\tM: ");
    Serial.print(cMovCommand->margin);
    Serial.print("-");
    Serial.print(cMovCommand->angleMargin);
  }

  bool moveStraight( double distance, double margin ){
    return moveStraight( distance, margin, false );
  }

  bool moveStraight( double distance, double margin, bool ignoreCollisions ){ 
    double xTarget = xPos + distance*cos(theta);
    double yTarget = yPos + distance*sin(theta);

    if ( !ignoreCollisions && !checkTargetLegal( xTarget, yTarget ) ){
      return false;
    } 

    
    MovementCommand * command = pushCommand();

    command->commandType = STRAIGHT;
    command->margin = margin;
    command->x = xTarget;
    command->y = yTarget;
    command->canReverse = distance<0;

    manageRemainingCommands();

    return true;
  }

  void rotate( double targetAngle, double margin ){
    
  }

  bool checkTargetLegal( double x, double y ){
    if ( disableCommandLegalCheck ){
      return true;
    }

    for (int i=0; i<OBJECT_BUFFER_MAX; i++){
      if ( !allTrackedObjects[i].ignore && allTrackedObjects[i].checkIntercepting( x, y ) ){
        return false;
      }
    }

    return true;
  }

  bool pointToPointMove( double x, double y, double angle, double margin ){
    return pointToPointMove( x, y, angle, margin, false );
  }
  
  void clearAngleMarginOnRecentCommand(){
    editAngleMarginOnRecentCommand( PI );
  }

  bool pointToPointMove( double x, double y, double angle, double margin, bool ignoreCollisions ){

    if ( ignoreCollisions || checkTargetLegal(x,y) ){
      MovementCommand * command = pushCommand();

      command->angleMargin = DEFAULT_ANGLE_MARGIN;
      command->x = x;
      command->y = y;
      command->margin = margin;
      command->theta = angle;
      command->commandType = POINT_TO_POINT;
      command->ignoreCollisions = ignoreCollisions;

      manageRemainingCommands();

      return true;
    }
    
    lastError = POINT_INTERCEPTS_OBJECT; 

    return false;
  }
  
  MovementCommand * pushCommand(){
    int targetIndex = (bufferStartP + commandsLeft)%MOVEMENT_BUFFER_MAX;
    commandsLeft++;
    errorCheck();
    return &(movementBuffer[ targetIndex ]);
  } 

  double angleProcessing( double thetaOffset ){
    bool flip = thetaOffset<0;
    thetaOffset = (flip?-1:1) * thetaOffset;

    if ( abs(thetaOffset) < MAX_ANGLE_D_FOR_ADJUSTMENT ){
      thetaOffset = 0;
    }else if( thetaOffset < MAX_ANGLE_D_FOR_AMPLIFICATION ){
      thetaOffset = (MAX_ANGLE_D_FOR_AMPLIFICATION + thetaOffset)/2;
    }

    return (flip?-1:1) * thetaOffset;
  } 

  void splitVelForRotation( double thetaOffset ){
    splitVelForRotation( thetaOffset, false );
  }

  double calcSlowFactor( bool nearObject, double objectSperation, double thetaSeperation  ){
    
    double angSF = max(1, min(abs(thetaSeperation*R_MULT_GRAD)+R_MULT_C, MAX_R_MULT));
    double sepSF = abs(thetaSeperation)>(DEG_TO_RAD*40)?1:max( min(1, (objectSperation-SLOW_F_SEP_OFFSET)/SLOW_F_SEP_GRAD), SLOW_F_MIN);

    return angSF * sepSF * (nearObject?0.4:1);
  }

  void splitVelForRotation( double thetaOffset, bool reverse ){  
    thetaOffset = angleProcessing( putRadInRange( thetaOffset ) );   

    double speedRatio = thetaOffset>=0?(  ( 1 + min(1, thetaOffset*SPEED_RATIO_AMPLIFICATION_FACTOR)  )/2  ):(  ( 1 + max(-1, thetaOffset*SPEED_RATIO_AMPLIFICATION_FACTOR)  )/2  ); 

    double slowf  = calcSlowFactor( isNearObject, targSeperation, thetaOffset );
    //Serial.print(" sF: ");
    //Serial.print(slowf);

    double sr2 = sq(speedRatio);
    double sr3 = sr2*speedRatio;

    double rightSpeed = slowf * ( QD_A_1*sr3 + QD_B_1*sr2 + QD_C_1*speedRatio + QD_D_1 ) * (canBeFast?FAST_SPEED_MUL:1); 

    double leftSpeed  = slowf * ( QD_A_2*sr3 + QD_B_2*sr2 + QD_C_2*speedRatio + QD_D_2 ) * (canBeFast?FAST_SPEED_MUL:1); 
     
    double absL = abs( leftSpeed );
    double absR = abs( rightSpeed );
    /*if ( absL < ABS_MIN_W_SPEED || absR < ABS_MIN_W_SPEED ){
      if ( abs( thetaOffset ) < MIN_ANGLE_D_FOR_TINY_MOV ){ 
        rightSpeed = ABS_MIN_W_SPEED;
        leftSpeed = ABS_MIN_W_SPEED;
      }else if( rightSpeed < leftSpeed ){
        rightSpeed = -ABS_MIN_W_SPEED;
        leftSpeed = ABS_MIN_W_SPEED;
      }else{ 
        rightSpeed = ABS_MIN_W_SPEED;
        leftSpeed = -ABS_MIN_W_SPEED;
      }
    }*/

    /*double fwdSp  = WHEEL_SEPERATION*( (2*leftSpeed/(leftSpeed-rightSpeed)) - 1 );
    double mulFac = max(min(1/fwdSp, 2), 1);
    rightSpeed *= mulFac;
    leftSpeed  *= mulFac;*/
    


    double absPD = abs( absL - absR )/(( absL + absR )*2);
    if ( ((absL < ABS_MIN_W_SPEED || absR < ABS_MIN_W_SPEED)) && absPD < 0.5 ){
      double mulFac = min( 4, max( ABS_MIN_W_SPEED/absL, ABS_MIN_W_SPEED/absR ) );
      rightSpeed *= mulFac;
      leftSpeed  *= mulFac;
    }

    if ( reverse ){
      setWheelLeftSpeed(  -rightSpeed );
      setWheelRightSpeed( -leftSpeed );
    }else{
      setWheelLeftSpeed( leftSpeed );
      setWheelRightSpeed( rightSpeed );
    }  

    /*Serial.print(" W:");
    Serial.print(slowf);
    Serial.print(",");
    Serial.print(rightSpeed);
    Serial.print(",");
    Serial.print(leftSpeed); */
  } 

  void errorCheck(){
    if ( commandsLeft > MOVEMENT_BUFFER_MAX ){
      errorFlash( 5 );
    }
  }

  bool addTargetWithOffset( double x, double y, double angle, double margin, double offset, bool ignoreCollisions ){
    return pointToPointMove( x+offset*cos(angle+PI), y+offset*sin(angle+PI), angle, margin, ignoreCollisions );
  }
  
  bool addTargetWithOffset( double x, double y, double angle, double margin, double offset ){
    return addTargetWithOffset(x, y, angle, margin, offset, false );
  }

  bool addTargetWithInitOffset( double x, double y, double angle, double margin, double offset ){
    return addTargetWithInitOffset( x, y, angle, margin, offset, 0 );
  }

  // collisionMode = 0-Full collisions, 1-No collision on final node, 2-No collisions
  bool addTargetWithInitOffset( double x, double y, double angle, double margin, double offset, int collisionMode ){
    switch (collisionMode)
    {
    default:
      return addTargetWithOffset( x, y, angle, margin*TARGET_W_OFFSET_MARGIN_AMPLIFICATION, offset ) || pointToPointMove( x, y, angle, margin );
      break;
    
    case 1:
      return addTargetWithOffset( x, y, angle, margin*TARGET_W_OFFSET_MARGIN_AMPLIFICATION, offset ) || pointToPointMove( x, y, angle, margin, true );
      break;
    
    case 2:
      return addTargetWithOffset( x, y, angle, margin*TARGET_W_OFFSET_MARGIN_AMPLIFICATION, offset, true ) || pointToPointMove( x, y, angle, margin, true );
      break;
    }
    //addTargetWithOffset( x, y, angle, margin*TARGET_W_OFFSET_MARGIN_AMPLIFICATION, offset );
    //pointToPointMove( x, y, angle, margin );
  }

  void manageRemainingCommands(){
    
    if ( !hasTarget && commandsLeft != 0 ){
      cMovCommand = &(movementBuffer[ bufferStartP ]);
  
      targetTheta = cMovCommand->theta ; 

      target_grad = tan(targetTheta);
      hasTarget = true;
      target_c = cMovCommand->y - target_grad * cMovCommand->x;
      lastCommandTime = millis();

      cComMode       = 0;
      prevSeperation = max( abs( xPos - cMovCommand->x ), abs( yPos - cMovCommand->y ) ); 
      
      Serial.println("");
      Serial.println("-- N COMMAND! --");
      
      Serial.print("cL:");
      Serial.print(commandsLeft);
      Serial.print("  cT:");
      Serial.print(cMovCommand->commandType);
      Serial.println("");
      Serial.println("-- N COMMAND! --");
    } 
  }

  void nextCommand(){
    commandsLeft--;
    bufferStartP = (bufferStartP+1)%MOVEMENT_BUFFER_MAX;
    hasTarget = false;

    if ( commandsLeft==0 ){
      stop();
    }else{
      manageRemainingCommands();
    }
  }

  void clearCommands(){
    stop();
  }

  void editAngleMarginOnRecentCommand( double newAngleMargin ){
    getMostRecentCommand()->angleMargin = newAngleMargin;
  }

  void markCurrentCommandSuccess(){
    movementCommandSuccessBuffer[CSBPointer] = cMovCommand->commandID;
    CSBPointer = (CSBPointer+1)%MOVEMENT_COMMAND_SUCCESS_BUFFER;
  }

  void actionLoop( bool printData ){ 
    manageRemainingCommands(); 
    if ( printData ){
      printState( 0 ); 
    }
    recalculatePosition();

    analogWrite( RGB_PIN, (int) 255*abs(sin( (millis())/600.0 )) ); 

    canBeFast = true;

    if ( hasTarget ){

      targSeperation    = max( abs( xPos - cMovCommand->x ), abs( yPos - cMovCommand->y ) );   

      if (  (targSeperation < cMovCommand->margin || ( targSeperation<TARGET_MARGIN_BEG && targSeperation > prevSeperation )) && abs(putRadInRange(theta-cMovCommand->theta)) < cMovCommand->angleMargin ){
        markCurrentCommandSuccess();
        
        nextCommand(); 
      }else {
        hasTarget = true; 

        switch ( cMovCommand->commandType ){
        case SPONTANIOUS: 
          updateSpontToTargetMovement();
          break;
        
        case POINT_TO_POINT: 
          updatePPToTargetMovement( );
          break;

        case STRAIGHT: 
          updateMoveStraight( );
          break;

        default:

          break;
        } 
      }

      prevSeperation = targSeperation;
    }
  }

  void updateMoveStraight( ){
    if ( prevSeperation-targSeperation < -0.4  ){
      markCurrentCommandSuccess();
      nextCommand(); 

      //errorFlash( 11 ); // issues lots FIX!

      return;
    }

    double cSlowFactor = calcSlowFactor( isNearObject, targSeperation, 0 );

    if ( cMovCommand->canReverse ){
      setWheelRightSpeed( -DEFAULT_SPEED*cSlowFactor );
      setWheelLeftSpeed( -DEFAULT_SPEED*cSlowFactor );
    }else{
      setWheelRightSpeed( DEFAULT_SPEED*cSlowFactor );
      setWheelLeftSpeed( DEFAULT_SPEED*cSlowFactor );
    }
    
    //setWheelRightSpeed( FORWARD_SPEED*cSlowFactor );
    //setWheelLeftSpeed( FORWARD_SPEED*cSlowFactor );
  }

  void stop(){
    commandsLeft = 0;
    stopMotors();
    hasTarget = false;

    for ( int i=0; i<MOVEMENT_BUFFER_MAX; i++ ){
      movementBuffer[i].reset();
    }
  }

  bool wideRangeMode = false;
  
  void updatePPToTargetMovement( ){
    if ( targSeperation < cMovCommand->margin || cComMode == 1 ){
      cComMode = 1;
      cMovCommand->margin = 999;
      dirMCRotateTo( targetTheta );
    }else{
      double pathTheta = atan2( cMovCommand->y-yPos, cMovCommand->x-xPos );
      double thetaDeviation = putRadInRange(pathTheta-theta);

      if ( targSeperation < POINT_POINT_PURE_MODE_DISTANCE ){ //&& abs(thetaDeviation) > POINT_POINT_MAX_A_DEVIATION
        goStraightToTarget( cMovCommand->x, cMovCommand->y );
      }else{
        goStraightToTarget( cMovCommand->x, cMovCommand->y );
      }
    }
  }

  void dirMCRotateTo( double alpha ){
      double angleOff = putRadInRange(alpha-theta);
      double rotMul   = (angleOff<0?-1.0:1.0) * max( (angleOff/1.55) + 0.58, 1 );

      setWheelLeftSpeed ( -rotMul * FWD_R_SPEED );
      setWheelRightSpeed(  rotMul * FWD_R_SPEED );
  }

  void updateSpontToTargetMovement(){
    //double xMinSep = (xPos+target_grad*(targetY-target_c))/(1+sq(target_grad));
    if ( max( abs(xPos-cMovCommand->x), abs(yPos-cMovCommand->y) ) < 15 ){
      double xMinSep = (xPos+target_grad*( yPos - target_c ))/( 1 + sq( target_grad ) ); 

      double xFakeTarget = xMinSep + (( cMovCommand->x - xMinSep )/2)  ; 
      double yFakeTarget = target_grad*xFakeTarget + target_c ; 
      
      goStraightToTarget( xFakeTarget, yFakeTarget );
    }else{
      double xMinSep = (xPos+target_grad*( yPos - target_c ))/( 1 + sq( target_grad ) );
      double yMinSep = target_grad*xMinSep + target_c;

      double xFakeTarget = xMinSep + (( cMovCommand->x - xMinSep )<0?-1:1)*4 + (xMinSep - xPos)*2;
      //double xFakeTarget = xMinSep + ( targetX - xMinSep )/5;
      //double yFakeTarget = target_grad*xFakeTarget + target_c;
      double yFakeTarget = target_grad*xFakeTarget + target_c + (yMinSep - yPos)*2;
       
      
      goStraightToTarget( xFakeTarget, yFakeTarget );
    }
    
    //goStraightToTarget( targetX, targetY );
  }
  
  void goStraightToTarget( double X, double Y ){
    double alpha = adjustTargetAngleByCollideables(atan2( Y-yPos, X-xPos )) - theta; 

    /*Serial.print("  alpha: ");
    Serial.print(alpha/DEG_TO_RAD);
    Serial.print("   ");

    Serial.print("  rX: ");
    Serial.print(X);
    Serial.print("   ");

    Serial.print("  rY: ");
    Serial.print(Y);
    Serial.print("   ");*/

    splitVelForRotation( alpha  );
  }

  void recalculatePosition(){  
    double cLEncVal = readMotorLeftEncoder();
    double cREncVal = readMotorRightEncoder(); 
    double dr_Dist = (cLEncVal - lastLEncVal) * WHEEL_RADIUS;
    double dl_Dist = (cREncVal - lastREncVal) * WHEEL_RADIUS;
    //Serial.print("dr_Dist: ");
    //Serial.print(dr_Dist);
    //Serial.print("  dl_Dist: ");
    //Serial.print(dl_Dist);
    double prevX = xPos;
    double prevY = yPos;

    if ( abs(dr_Dist - dl_Dist)<0.0001 ){ // straight movement
      double forward_move = dl_Dist;
      xPos +=  forward_move*cos( theta );
      yPos +=  forward_move*sin( theta );
      
    }else{ // arc movement
      
      /*double movementRadius = max( abs(WHEEL_SEPERATION * ( 2 * dl_Dist/(dl_Dist - dr_Dist) - 1)), 0.1);

      double left_move; 
      double turning_circ_angle;
      double forward_move;
      double angle_move;

      // forward movement
      if ( dr_Dist + dl_Dist >= 0 ){
        // right movement (negative y)
        if ( dl_Dist > dr_Dist ){
          //Serial.print("F RIGHT!!");
          turning_circ_angle = -dl_Dist/movementRadius; 
          left_move = -movementRadius*(1-cos( turning_circ_angle ) );
        }else{// left movement (positive y)
          //Serial.print("F LEFT!!"); 
          turning_circ_angle = dr_Dist/movementRadius;
          left_move = movementRadius*(1-cos( turning_circ_angle ) );
        }
        forward_move = abs(movementRadius*sin(turning_circ_angle));
        angle_move = atan2( left_move, forward_move );
      }else{
        // left rotation (negative y)
        if ( dl_Dist < dr_Dist ){
          //Serial.print("B LEFT!!");
          turning_circ_angle = -dl_Dist/movementRadius; 
          left_move = -movementRadius*(1-cos( turning_circ_angle ) );
          forward_move = -(movementRadius*sin(turning_circ_angle));
          angle_move = atan( left_move/forward_move );
        }else{// right rotation (positive y)
          //Serial.print("B RIGHT!!"); 
          turning_circ_angle = -dr_Dist/movementRadius;
          left_move = movementRadius*(1-cos( turning_circ_angle ) );
          forward_move = -(movementRadius*sin(turning_circ_angle));
          angle_move = atan( left_move/forward_move );
        }
      }
      
      //Serial.print("  mRad: ");
      //Serial.print(movementRadius);

      //Serial.print("   angleMov: ");
      //Serial.print(angle_move);
      //Serial.print("   left_move: ");
      //Serial.print(left_move);
      //Serial.print("   forw_move: ");
      //Serial.print(forward_move);
      //Serial.print("   tCirc_angle: ");
      //Serial.print(turning_circ_angle);

      if ( isnan( forward_move ) )*/

      if ( abs( dl_Dist ) < 0.001 ){
        dl_Dist = 0.001;
      }
      if ( abs( dr_Dist ) < 0.001 ){
        dr_Dist = 0.001;
      }

      double turningCirc = WHEEL_SEPERATION*( (2*dl_Dist/(dl_Dist-dr_Dist)) - 1 );
      double turningAng  = dr_Dist/( turningCirc - WHEEL_SEPERATION );

      double forwardMove = turningCirc*sin( turningAng );
      double rightMove   = turningCirc*( 1 - cos( turningAng ) );

      xPos +=  forwardMove*cos( theta ) + rightMove*sin( -turningAng );
      yPos +=  forwardMove*sin( theta ) - rightMove*cos( -turningAng );

      theta = putRadInRange(theta - turningAng);
      
      //Serial.println("");
    }

    lastPosUpdate = millis();
    lastLEncVal = cLEncVal;
    lastREncVal = cREncVal;

    xDistanceCounter +=  abs(xPos - prevX);
    yDistanceCounter += abs( yPos - prevY );
  }
};

MovementManager * movementManager = new MovementManager();

enum BehaviourMode{ RETURN_HOME, SETUP_BEHAVIOUR, IDLE, STEALING, DEAD  };

enum SecondaryMode{ GENERIC_MODE, X_ZERO, Y_ZERO, NONE_OVERWRITE, GRABBING_SLICE, DROPPING_SLICE };

#define ZEROING_POSITION_COUNT 16
double zeroPositions [ZEROING_POSITION_COUNT][3] = {
  { 0, 22.5, PI },
  { 0, 67  , PI },
  { 67, 0  , -PI/2 },
  { 115, 0  , -PI/2 },

  { 0, 200-22.5, PI },
  { 0, 200-67  , PI },
  { 67, 200-0  , PI/2 },
  { 115, 200-0  , PI/2 },

  { 300-0, 22.5, 0 },
  { 300-0, 67  , 0 },
  { 300-67, 0  , -PI/2 },
  { 300-115, 0  , -PI/2 },

  { 300-0, 200-22.5, 0 },
  { 300-0, 200-67  , 0 },
  { 300-67, 200-0  , PI/2 },
  { 300-115, 200-0  , PI/2 },
}; 

#define ZERO_EDGE_MARGIN  (30.0)
#define Y_ZERO_POS_COUNT (4*6)
#define X_ZERO_POS_COUNT (4*8)
#define ZERO_WALL_INIT_GAP (15.0 + ROBOT_PHYSICAL_RADIUS)

double yZeroPositions [Y_ZERO_POS_COUNT][3];
double xZeroPositions [X_ZERO_POS_COUNT][3];

void defineZeroPositions(){
  int yPerCorn = Y_ZERO_POS_COUNT/4;

  for (int i=0; i<yPerCorn; i++){
    double addInc = (100.0-ZERO_EDGE_MARGIN*2)* (double(i)/yPerCorn);

    yZeroPositions[i][0] = 0;
    yZeroPositions[i][1] = ZERO_EDGE_MARGIN + addInc;
    yZeroPositions[i][2] = -PI;
    
    yZeroPositions[i+yPerCorn][0] = 0;
    yZeroPositions[i+yPerCorn][1] = 200 - (ZERO_EDGE_MARGIN + addInc);
    yZeroPositions[i+yPerCorn][2] = -PI;

    yZeroPositions[i+yPerCorn*2][0] = 300;
    yZeroPositions[i+yPerCorn*2][1] = 200 - (ZERO_EDGE_MARGIN + addInc);
    yZeroPositions[i+yPerCorn*2][2] = 0;

    yZeroPositions[i+yPerCorn*3][0] = 300;
    yZeroPositions[i+yPerCorn*3][1] = ZERO_EDGE_MARGIN + addInc;
    yZeroPositions[i+yPerCorn*3][2] = 0;
  }

  int xPerCorn = X_ZERO_POS_COUNT/4;

  for (int i=0; i<xPerCorn; i++){
    double addInc = (150.0-ZERO_EDGE_MARGIN*2)* (double(i)/xPerCorn);

    xZeroPositions[i][0] = addInc;
    xZeroPositions[i][1] = 0;
    xZeroPositions[i][2] = -PI/2;
    
    xZeroPositions[i+xPerCorn][0] = 300 - (addInc);
    xZeroPositions[i+xPerCorn][1] = 0;
    xZeroPositions[i+xPerCorn][2] = -PI/2;

    xZeroPositions[i+xPerCorn*2][0] = 300 - (addInc);
    xZeroPositions[i+xPerCorn*2][1] = 200;
    xZeroPositions[i+xPerCorn*2][2] = PI/2;

    xZeroPositions[i+xPerCorn*3][0] = addInc;
    xZeroPositions[i+xPerCorn*3][1] = 200;
    xZeroPositions[i+xPerCorn*3][2] = PI/2;
  }
}
 

class PositioningSetting{
public:

};

void stopIfObject(){
  posSensorManager->printPingLens();
}

class BoardManager{
public: 
  BehaviourMode currentMode          = SETUP_BEHAVIOUR;
  SecondaryMode currentSecondaryMode = GENERIC_MODE;

  int setupCommandIndex = 0;

  double zeroAngle;
  double zeroX;
  double zeroY;

  double objectCheckingPeriod = DEFUALT_CHECKING_PERIOD;
  unsigned long nextCheckingTime = DEFUALT_CHECKING_PERIOD;

  bool enabledAutoZero = false;

  int movementSuccessIDSec = -2;
  int commandIndexSec = 0;

  TrackedObject * sliceBeingGrabbed;

  bool holdingSlice[3] = { false, false, false };
  
  BoardManager(){
    
  } 

  // this is actually for zeroing X
  bool enterYZeroMode(){ 
    double nDist  = 99999999;
    int    nIndx = -1;

    for ( int i=0; i<Y_ZERO_POS_COUNT; i++ ){
      double cDist = sq(movementManager->xPos - yZeroPositions[i][0]) + sq(movementManager->yPos - yZeroPositions[i][1]);
      if ( cDist < nDist ){
        double angle = yZeroPositions[i][2];

        bool hitsSomething = -1 != checkHitsAnyObjects( yZeroPositions[i][0] + ZERO_WALL_INIT_GAP*cos(angle+PI), yZeroPositions[i][1] + ZERO_WALL_INIT_GAP*sin(angle+PI), angle );

        if ( !hitsSomething ){
          nDist = cDist;
          nIndx = i;
        }
      }
    }

    if ( nIndx == -1 ){
      errorFlash( 5 );
    }
    
    
    zeroX  = yZeroPositions[nIndx][0];
    zeroY  = yZeroPositions[nIndx][1];
    zeroAngle = yZeroPositions[nIndx][2];

    if (movementManager->addTargetWithOffset( zeroX, zeroY, zeroAngle, 1, ZERO_WALL_INIT_GAP )){
      currentSecondaryMode = Y_ZERO;
      movementManager->clearCommands();
      movementManager->addTargetWithOffset( zeroX, zeroY, zeroAngle, 1, ZERO_WALL_INIT_GAP );
    }
    return false;
  }

  // this is actually for zeroing Y
  bool enterXZeroMode(){
    double nDist  = 99999999;
    int    nIndx = -1;

    for ( int i=0; i<X_ZERO_POS_COUNT; i++ ){
      double cDist = sq(movementManager->xPos - xZeroPositions[i][0]) + sq(movementManager->yPos - xZeroPositions[i][1]);
      if ( cDist < nDist ){
        double angle = xZeroPositions[i][2];

        bool hitsSomething = -1 != checkHitsAnyObjects( xZeroPositions[i][0] + ZERO_WALL_INIT_GAP*cos(angle+PI), xZeroPositions[i][1] + ZERO_WALL_INIT_GAP*sin(angle+PI), angle );

        if ( !hitsSomething ){
          nDist = cDist;
          nIndx = i;
        }
      }
    }

    if ( nIndx == -1 ){
      errorFlash( 6 );
    }
    
    zeroX  = xZeroPositions[nIndx][0];
    zeroY  = xZeroPositions[nIndx][1];
    zeroAngle = xZeroPositions[nIndx][2];
 
    if (movementManager->addTargetWithOffset( zeroX, zeroY, zeroAngle, 1, ZERO_WALL_INIT_GAP ) || true ){ // FIX THIS!!!!!
      currentSecondaryMode = X_ZERO;
      movementManager->clearCommands();
      movementManager->addTargetWithOffset( zeroX, zeroY, zeroAngle, 1, ZERO_WALL_INIT_GAP, true );
    }
    
    return false;
  }

  void printState(){
    Serial.print("\tsM: ");
    Serial.print(currentSecondaryMode);
  }

  void killRobot(){
    currentMode = DEAD;
    stopMotors();
    closePlaugh();

    errorFlash( 20 );
    for (;;){}
  }
  
  void mainBehaviourLoop( bool shouldPrint ){
    if ( shouldPrint ){
      Serial.println("");
      printState();
    }

    //posSensorManager->sendPulse();

    //movementManager->recalculatePosition();

    //posSensorManager->preformBasicSensorCalcs( movementManager->theta, movementManager->xPos, movementManager->yPos );
    
    //posSensorManager->printWallData( movementManager->theta, movementManager->xPos, movementManager->yPos );

    //posSensorManager->estimatePosition( movementManager->theta, movementManager->xPos, movementManager->yPos );

    movementManager->actionLoop( shouldPrint );
    
    // SECONDARY MODE BEHAVIOUR

    switch ( currentSecondaryMode )
    {
    case X_ZERO:
    case Y_ZERO:
      if ( !movementManager->hasTarget ){
        if ( X_ZERO == currentSecondaryMode ){
          //zeroY = movementManager->yPos;
        }else{
          //zeroX = movementManager->xPos;
        }


        setWheelLeftSpeed( 0.15 );
        setWheelRightSpeed( 0.15 );
        delay(2000);
        stopMotors();

        movementManager->recalculatePosition();
        movementManager->xPos = zeroX + ROBOT_ZEROING_DISTANCE*cos( zeroAngle + PI );
        movementManager->yPos = zeroY + ROBOT_ZEROING_DISTANCE*sin( zeroAngle + PI );
        movementManager->theta = zeroAngle;

        
        setWheelLeftSpeed( -0.15 );
        setWheelRightSpeed( -0.15 );
        delay(1000);
        stopMotors();
        movementManager->recalculatePosition();

        if ( X_ZERO == currentSecondaryMode ){
          movementManager->yDistanceCounter = 0; 
        }else{
          movementManager->xDistanceCounter = 0; 
        }
        currentSecondaryMode = GENERIC_MODE; 
      }
      break;
    
    case GRABBING_SLICE:
      if ( !movementManager->hasTarget ){
        if ( !movementManager->checkCommandSuccess( movementSuccessIDSec ) ){
          currentSecondaryMode = GENERIC_MODE;

          errorFlash( 9 );
        }else{
          switch (commandIndexSec){
          case 0:
            Serial.println("!!!!!!case 0"); 

            movementManager->clearCommands();

            openPlaugh();

            movementManager->moveStraight( SLICE_APPROACH_DISTANCE + SLICE_RAM_DISTANCE, MID_POS_MARGIN, true );
            movementSuccessIDSec = movementManager->getMostRecentCommandID();

            break;

          case 1:
            Serial.println("!!!!!!case 1");
            movementManager->clearCommands();
            sliceBeingGrabbed->reset( true );
            closePlaugh();
            movementManager->moveStraight( -SLICE_REVERSE_DISTANCE, MID_POS_MARGIN, true );
            movementSuccessIDSec = movementManager->getMostRecentCommandID();
 
          
            break; 

          case 2:   
            Serial.println("!!!!!!case 2");
            currentSecondaryMode = GENERIC_MODE;
          
            break; 
          }

          commandIndexSec++;
        } 
      }

      break;

    case DROPPING_SLICE:
      if ( !movementManager->hasTarget ){
        if ( !movementManager->checkCommandSuccess( movementSuccessIDSec ) ){
          currentSecondaryMode = GENERIC_MODE;

          errorFlash( 10 );
        }else{
          switch (commandIndexSec){
          case 0:
            openPlaugh();
            
            newTrackedObject( STACKED_SLICES, movementManager->xPos, movementManager->yPos, STATIC_ONLY );

            movementManager->moveStraight( -SLICE_DROPPING_OFFSET, LOW_POS_MARGIN, true );
            movementSuccessIDSec = movementManager->getMostRecentCommandID();

            break; 

          case 1:
            closePlaugh();
            currentSecondaryMode = GENERIC_MODE;

            addScore( 9 );

            break;
          } 

          commandIndexSec++;
        }
      }
      break;
    
    default:
      performPrimaryModeBehaviour();
      break;
    }
     
  
    // ADDITIONAL STUFF
    //checkWantsToZero();
    checkNeedsToCheck();
  }

  void performPrimaryModeBehaviour(){
    switch ( currentMode )
      {
      case SETUP_BEHAVIOUR: 
        if ( !movementManager->hasTarget ){
         /* if ( movementManager->xPos > 150 ){
            movementManager->pointToPointMove( 50, 50, PI/2, 1 );
          }else{
            movementManager->pointToPointMove( 250, 150, PI/2, 1 );
          }*/

          switch (setupCommandIndex)
          {
          case 0: 
            movementManager->disableCommandLegalCheck = true;
            //enterXZeroMode();
            break;
          case 1:
            //enterYZeroMode();
            break;
          case 2: 
            grabNearestSlice( 1 ); 
            break;
          case 3:
            grabNearestSlice( GREEN_START?0:2 ); 
            break;
          case 4: 
            //movementManager->pointToPointMove( 60, 200-60, 0, SLOW_POS_MARGIN );
            //movementManager->clearAngleMarginOnRecentCommand(); 
            break;
          case 5: 
            grabNearestSlice( GREEN_START?2:0 );  
            break;
          case 6:
            Serial.println("RUNNING: 6666666666666");
            movementManager->pointToPointMove( 300-93.5,(GREEN_START?(200-46):(46)) , (GREEN_START?(-DEG_TO_RAD*45):(DEG_TO_RAD*45)), LOW_POS_MARGIN ); 
            movementManager->editAngleMarginOnRecentCommand( DEG_TO_RAD*25 );
            break;
          case 7:
            if ( GREEN_START ){
              dropSlices( 300-24.5, 200-73.5, -PI/2 );
            }else{
              dropSlices( 300-24.5, 73.5, PI/2 );
            }
            
            break;
          case 8:
            if ( GREEN_START ){
              movementManager->pointToPointMove( 300-23.5, 200-40  ,  DEG_TO_RAD*90 , MID_POS_MARGIN ); 
              movementManager->pointToPointMove( 300-22.5, 200-23.5, -DEG_TO_RAD*180, HIGH_POS_MARGIN ); 
            }else{
              movementManager->pointToPointMove( 300-23.5, 40  , -DEG_TO_RAD*90 , MID_POS_MARGIN ); 
              movementManager->pointToPointMove( 300-22.5, 23.5, DEG_TO_RAD*180, HIGH_POS_MARGIN ); 
            }
            break;

          case 9:
            grabNearestSlice( 1 ); 
            break;

          case 10:
            grabNearestSlice( GREEN_START?2:0 ); 
            break;

          case 11:
            //grabNearestSlice( GREEN_START?0:2 ); 
            break;

          case 12:
            if ( GREEN_START ){
              dropSlices( 150+37.5, 200-23, (DEG_TO_RAD*90) );
            }else{
              dropSlices( 150+37.5, 23, -(DEG_TO_RAD*90) );
            }
            break;
          
          case 13:
            setModeReturnHome();
            break;

          default:
          
            break;
          }
          setupCommandIndex++;
        }

        break;

      case RETURN_HOME:
        if ( !movementManager->hasTarget ){
          closePlaugh();
          stopMotors();
          addScore( 16 );
          killRobot();
        }
        break;
      
      default:
        break;
      }
  }

  void setModeReturnHome(){
    movementManager->clearCommands();

    currentMode = RETURN_HOME;
    currentSecondaryMode = NONE_OVERWRITE;
    
    forgetAllSlices();
    closePlaugh();
    
    if ( GREEN_START ){
      movementManager->pointToPointMove( 26, 200-26, 0, MID_POS_MARGIN );
    }else{
      movementManager->pointToPointMove( 26, 26, 0, MID_POS_MARGIN );
    }
    
    movementManager->clearAngleMarginOnRecentCommand();
  }

  void checkNeedsToCheck(){
    int success_count = 0;
    
    if ( millis() > nextCheckingTime ){
      nextCheckingTime = millis() + objectCheckingPeriod;
      movementManager->actionLoop( false );

      //checkForObjects();
      while ( posSensorManager->checkSomethingExists() ){
        stopMotors();
        timerCheckCommand(); 
        while ( posSensorManager->checkSomethingExists() ){
          stopMotors();
          timerCheckCommand(); 
          while ( posSensorManager->checkSomethingExists() ){
            stopMotors();
            timerCheckCommand(); 
          }
        }
      }
    }
  }

  int findNearestSlice( double targetX, double targetY, double maxSep ){
    int nEstIndx = -1;
    double nSepSQ = 999999;

    for ( int i=0; i<OBJECT_BUFFER_MAX; i++ ){
      TrackedObject * cObject = &(allTrackedObjects[i]);

      if ( cObject->type == SLICE && !cObject->deleted ){
        double sepSQ = sq(cObject->xPos - targetX) + sq(cObject->yPos - targetY);

        if ( sepSQ < nSepSQ && sepSQ < sq(maxSep) ){
          nSepSQ = sepSQ;
          nEstIndx = i;
        }
      }
    }

    return nEstIndx;
  }

  void checkWantsToZero(){
    if ( enabledAutoZero && currentSecondaryMode == GENERIC_MODE ){
      if ( movementManager->yDistanceCounter > ZERO_REQ_TRAVEL ){
        enterXZeroMode();
      }else if ( movementManager->xDistanceCounter > ZERO_REQ_TRAVEL ){
        enterYZeroMode();
      }
    }
  }

  bool dropSlices( double xInp, double yInp ){
    return dropSlices( xInp, yInp, 999 );
  }

  bool dropSlices( double xInp, double yInp, double prefAngle ){
    
    //movementManager->addTargetWithInitOffset( xInp, yInp, getSliceDroppingAngle( yInp ), LOW_POS_MARGIN, SLICE_DROPPING_OFFSET, 1 );
    movementManager->pointToPointMove( xInp, yInp, (prefAngle>99)?getSliceDroppingAngle( yInp ):prefAngle, LOW_POS_MARGIN, 0 ); 

    movementSuccessIDSec = movementManager->getMostRecentCommandID();

    currentSecondaryMode = DROPPING_SLICE;
    commandIndexSec = 0;

    return true;
  }

  bool grabSlice( TrackedObject * sliceObject, double approachAngle, int plaughIndex){
    if ( holdingSlice[plaughIndex] ){
      errorFlash(3);
      return false;
    } 

    double offsetLength = plaughSlotPositions[plaughIndex][0];
    double offsetAngle  = plaughSlotPositions[plaughIndex][1];

    //offsetLength = 16;
    //offsetAngle = PI/4;

    double plaughXOffset = - offsetLength * cos( offsetAngle + approachAngle );
    double plaughYOffset = - offsetLength * sin( offsetAngle + approachAngle );

    Serial.print("GRABBING!!!  :");
    Serial.print(plaughIndex);
    Serial.print(",l-");
    Serial.print(plaughSlotPositions[plaughIndex][0]);
    Serial.print(",a-");
    Serial.print(plaughSlotPositions[plaughIndex][1]);
    Serial.print("-");
    Serial.print(approachAngle); 
    Serial.println(":   GRABBING!!!");

    movementManager->clearCommands();

    bool success = movementManager->addTargetWithOffset( plaughXOffset + sliceObject->xPos, plaughYOffset + sliceObject->yPos, approachAngle, MID_POS_MARGIN, SLICE_APPROACH_DISTANCE );
    
    //movementManager->pointToPointMove( plaughXOffset + sliceObject->xPos - 10, plaughYOffset + sliceObject->yPos, approachAngle, MID_POS_MARGIN );


    movementManager->editAngleMarginOnRecentCommand( 1*DEG_TO_RAD );
    // TODO FIX
    //movementManager->pointToPointMove( sliceObject->xPos, sliceObject->yPos, 1.0, 1, true );
    Serial.println("zzzzzz!!!");

    if ( !success ){
      errorFlash(4);
      return false;
    }

    movementSuccessIDSec = movementManager->getMostRecentCommandID();
    commandIndexSec = 0;
    sliceBeingGrabbed = sliceObject;

    currentSecondaryMode = GRABBING_SLICE;

    closePlaugh();

    return true;
  }

  bool grabNearestSlice( int plaughIndex ){
    return grabNearestSlice( movementManager->xPos, movementManager->yPos, 999999, plaughIndex );
  }

  bool grabNearestSlice( double xInp, double yInp, double maxSep, int plaughIndex ){
    int fIndx = findNearestSlice( xInp, yInp, maxSep );

    if ( fIndx == -1 ){
      errorFlash(8);
      return false;
    }

    TrackedObject * tSlice = &(allTrackedObjects[ fIndx ]);

    return grabSlice( tSlice, atan2( tSlice->yPos-movementManager->yPos, tSlice->xPos-movementManager->xPos ), plaughIndex );
  }

  void checkForObjects(){
    movementManager->actionLoop( false );
    posSensorManager->sendPulse();
    double loggedXpos  = movementManager->xPos;
    double loggedYpos  = movementManager->yPos;
    double loggedTheta = movementManager->theta;
    movementManager->actionLoop( false );
    posSensorManager->preformBasicSensorCalcs( loggedTheta, loggedXpos, loggedYpos );
    movementManager->actionLoop( false );

    //posSensorManager->printPingLens();
    //Serial.println(""); 

    for ( int i=0; i<MAIN_DIST_SENSOR_COUNT; i++ ){
      Sensor * cSensor = &(main_Sensors[i]);
      double absSensorAngle = cSensor->muAlpha + movementManager->theta;
      /*Serial.print(" S"); 
      Serial.print(cSensor->pin); 
      Serial.print("-"); */

      // if the sensor doesn't have a bad ping and the object hit is not indentified it continues with the processing
      if ( !cSensor->badPing ){
        if ( cSensor->objectHit == OTHER ){
          
          int hitIndex = findNearestObjectHit( cSensor->predXHit, cSensor->predYHit ); 
          TrackedObject * hitishObject = &(allTrackedObjects[ hitIndex ]);

          /*Serial.print("G");  
          Serial.print(hitIndex); 
          Serial.print("="); 
          Serial.print(hitIndex==-1?-1:hitishObject->indexID); 
          Serial.print("-"); */

          if ( hitIndex == -1 || hitishObject->type==SLICE ){
            //Serial.print("C");
            // if it didn't hit something, then it'll add a new object to track 
            TrackedObject * trackedThingy = newTrackedObject( UNKNOWN, cSensor->predXHit+OBJECT_CREATION_DEPTH_OFFSET*cos( absSensorAngle ), cSensor->predYHit+OBJECT_CREATION_DEPTH_OFFSET*sin( absSensorAngle ), FREE_ONLY );
           
            trackedThingy->addExistanceHit( TRACKING_ADDITION_INIT );  
          }else{
            //Serial.print("A"); 
            // else it reaffirms that the thing that was ping'd does in fact exist, and if it is a non static object modifies positional data
            if ( hitishObject->forDynamicAllocation ){
              hitishObject->addExistanceHit();
              hitishObject->addPositionData( cSensor->predXHit+OBJECT_CREATION_DEPTH_OFFSET*cos( absSensorAngle ), cSensor->predYHit+OBJECT_CREATION_DEPTH_OFFSET*sin( absSensorAngle ) );
              Serial.print(hitishObject->getExistanceRating());
            } 
          }
        }else{// If there isn't any new objects being brought into existance, then it checks if the sensor SHOULD of hit something 
          updateEmptyRaycast( cSensor->lastAbsX, cSensor->lastAbsY, absSensorAngle );
        }
      }else if ( cSensor->failType == TOO_FAR || cSensor->failType == NO_RESPONSE ){
        updateEmptyRaycast( cSensor->lastAbsX, cSensor->lastAbsY, absSensorAngle );
      }

      movementManager->actionLoop( false );
    }
    
  }

  void updateEmptyRaycast( double xOrigin, double yOrigin, double angle ){
    int raycastHitID = checkHitsAnyObjects( xOrigin, yOrigin, angle );

    if ( raycastHitID != -1 ){
      TrackedObject * notFoundObject = &(allTrackedObjects[raycastHitID]);

      notFoundObject->removeExistanceHit();
      //Serial.print("D"); 
      //Serial.print(raycastHitID); 

      if ( notFoundObject->getExistanceRating() == 0 ){
        notFoundObject->reset( false );
      }
    }else{ 
      //Serial.print("N");  
    }
  }

  //void grabNearestSlice( int plaughIndex ){
  //  grabNearestSlice( movementManager->xPos, movementManager->yPos, -1, 100, plaughIndex );
  //}

};

BoardManager * boardManager = new BoardManager(); 

void boardObjectInitiation(){ 
  // this loads all slices onto the board
  for (int I=-1; I<2; I+=2){
    for (int i=-1; i<2; i+=2){ 
      newTrackedObject( SLICE, 150 + i*(150-57.5), 100 + I*(100-(45/2)), STATIC_ONLY );
      newTrackedObject( SLICE, 150 + i*(150-77.5), 100 + I*(100-(45/2)), STATIC_ONLY );

      newTrackedObject( SLICE, 150 + i*(150-112.5), 100 + I*(100-72.5), STATIC_ONLY );
    }
  }

  // this adds the small walls as objects to avoid
  for (int i=0; i<2; i++){
    allStaticObsticals[i].setObjectType( WALL, (xWalls[i+2][1]+xWalls[i+2][2])/2, xWalls[i+2][0] );
  }

  wallSurrigate.setObjectType( WALL, 0, 0 );
}

#define SAMP_PER_PIX 10

#define X_PIXELS (300/SAMP_PER_PIX)
#define Y_PIXELS (200/SAMP_PER_PIX)

char renderedImage [X_PIXELS][Y_PIXELS+1];

void resetScreenString(){
  for ( int pX=0; pX<X_PIXELS; pX++ ){
    for ( int pY=0; pY<Y_PIXELS; pY++ ){
      renderedImage[pX][pY] = '.';
    } 
    renderedImage[pX][Y_PIXELS] = '\n'; 
  }
} 

void setRendPix( double xPos, double yPos, char val ){
  if ( xPos > 300 || xPos < 0 || yPos > 200 || yPos < 0 ){
    return;
  } 

  renderedImage[ ( int(xPos/SAMP_PER_PIX) ) ][ ( int(yPos/SAMP_PER_PIX) ) ] = val;
}

void printMap( bool kill ){
  resetScreenString();

  for (int i=0; i<OBJECT_BUFFER_MAX; i++){
    TrackedObject * cTObj = &(allTrackedObjects[i]);
    
    if (!cTObj->ignore){
      int trackingStrength = 9.99*(cTObj->getExistanceRating()/EXISTANCE_RATING_MAX);
      char cVal = 'U';

      switch (trackingStrength){
        case 0: { cVal = '0'; } break;
        case 1: { cVal = '1'; } break;
        case 2: { cVal = '2'; } break;
        case 3: { cVal = '3'; } break;
        case 4: { cVal = '4'; } break;
        case 5: { cVal = '5'; } break;
        case 6: { cVal = '6'; } break;
        case 7: { cVal = '7'; } break;
        case 8: { cVal = '8'; } break;
        case 9: { cVal = '9'; } break;
      }

      setRendPix( cTObj->xPos, cTObj->yPos, cVal );
    }
  }
  if ( movementManager->hasTarget ){
    setRendPix( (movementManager->cMovCommand)->x, (movementManager->cMovCommand)->y, 'T' );
  }
  
  setRendPix( movementManager->xPos, movementManager->yPos, 'R' );
  if ( kill ){
    delay(50);
    stopMotors();
    delay(50);
  }

  Serial.println( "------------------" );
  Serial.println( renderedImage[0] );
  for ( int pX=0; pX<X_PIXELS; pX++ ){
    //Serial.print( '|' );
    //Serial.println( '|' );
  }
  Serial.println( "------------------" );
}

void cornerMovementProtocol(){
  movementManager->xPos = 150;
  movementManager->yPos = 100;

  movementManager->pointToPointMove( 22.5, 45, 0, 1);
  movementManager->pointToPointMove( 22.5, 200-45, 0, 1);

  movementManager->pointToPointMove( 300-22.5, 200-45, 0, 1);
  movementManager->pointToPointMove( 300-22.5, 45, 0, 1);
  
  movementManager->pointToPointMove( 22.5, 200-45, 0, 1);

  movementManager->pointToPointMove( 150, 100, 0, 1);
}

void singleObsticalTestFunc(){
  movementManager->xPos = 0;
  movementManager->yPos = 0;

  newTrackedObject( SLICE, 150/4, 0, STATIC_ONLY );
  //movementManager->pointToPointMove( 300, 0, 0, 1 );
  //movementManager->pointToPointMove( 0, 0, 0, 1 );

  boardManager->currentMode = IDLE;
}

void sliceGrabTest(){
  movementManager->xPos  = 150;
  movementManager->yPos  = 100;
  movementManager->theta = 90 * DEG_TO_RAD;

  newTrackedObject( SLICE, movementManager->xPos+100, movementManager->yPos+40, STATIC_ONLY );
  //newTrackedObject( SLICE, 200, 0, STATIC_ONLY );
  //newTrackedObject( SLICE, 300, 0, STATIC_ONLY );

  //newTrackedObject( SLICE, 150, 0 );
  //boardManager->grabNearestSlice( 0, 0, 2000, 0 );

  boardManager->grabNearestSlice( 1 );
  //movementManager->pointToPointMove( 300, 0, 0, 1 );
  //movementManager->pointToPointMove( 0, 0, 0, 1 );

  boardManager->currentMode = IDLE;

  //movementManager->moveStraight( 120, 1 );
}

void basicMovementTest(){
  for(;;){
    setWheelLeftSpeed( 0.2 );
    setWheelRightSpeed( -0.2 );
    delay( 2000 );
    setWheelLeftSpeed( 0.2 );
    setWheelRightSpeed( 0.2 );
    delay( 1000 );
    stopMotors();
    delay( 1000 );
  }
}

void squareMovementTest(){
  movementManager->xPos = 150;
  movementManager->yPos = 100;
  movementManager->theta = 0;

  movementManager->ignoreObjectCollision = true;
  movementManager->ignoreWallCollision = false;

  boardManager->currentMode = IDLE;

  movementManager->clearCommands(); 
  movementManager->pointToPointMove( 35, 35, 0, 0.5 );
  movementManager->pointToPointMove( 300-35, 35, 0, 1 );
  movementManager->pointToPointMove( 300-35, 200-35, 0, 1 );
  movementManager->pointToPointMove( 300-35, 200-35, 0, 1 ); 
  movementManager->pointToPointMove( 150, 100, 0, 0.5 );
  movementManager->editAngleMarginOnRecentCommand( 1*DEG_TO_RAD ); 
  
  movementManager->clearCommands(); 
  movementManager->pointToPointMove( 35, 35, 0, 0.5 );
  movementManager->pointToPointMove( 300-35, 35, 0, 1 );
  movementManager->pointToPointMove( 300-35, 200-35, 0, 1 );
  movementManager->pointToPointMove( 300-35, 200-35, 0, 1 ); 
  movementManager->pointToPointMove( 150, 100, 0, 0.5 );
  movementManager->editAngleMarginOnRecentCommand( 1*DEG_TO_RAD ); 
  
  movementManager->clearCommands(); 
  movementManager->pointToPointMove( 35, 35, 0, 0.5 );
  movementManager->pointToPointMove( 300-35, 35, 0, 1 );
  movementManager->pointToPointMove( 300-35, 200-35, 0, 1 );
  movementManager->pointToPointMove( 300-35, 200-35, 0, 1 ); 
  movementManager->pointToPointMove( 150, 100, 0, 0.5 );
  movementManager->editAngleMarginOnRecentCommand( 1*DEG_TO_RAD ); 
}
void squareMovementObsticalTest(){
  movementManager->xPos = 30;
  movementManager->yPos = 30;
  movementManager->theta = 0;

  //movementManager->ignoreObjectCollision = true;
  movementManager->ignoreWallCollision = true;

  boardManager->currentMode = IDLE;

  movementManager->clearCommands(); 
  movementManager->pointToPointMove( 30, 200-30, 0, 1 ); 
  movementManager->pointToPointMove( 300-30, 200-30, 0, 1 );
  movementManager->pointToPointMove( 300-30, 30, 0, 1 );
  movementManager->pointToPointMove( 30, 30, 0, 0.5 ); 
  //movementManager->clearCommands(); 
  movementManager->pointToPointMove( 30, 200-30, 0, 1 ); 
  movementManager->pointToPointMove( 300-30, 200-30, 0, 1 );
  movementManager->pointToPointMove( 300-30, 30, 0, 1 );
  movementManager->pointToPointMove( 30, 30, 0, 0.5 ); 
  //movementManager->clearCommands(); 
  movementManager->pointToPointMove( 30, 200-30, 0, 1 ); 
  movementManager->pointToPointMove( 300-30, 200-30, 0, 1 );
  movementManager->pointToPointMove( 300-30, 30, 0, 1 );
  movementManager->pointToPointMove( 30, 30, 0, 0.5 );  
  movementManager->editAngleMarginOnRecentCommand( 1*DEG_TO_RAD ); 

  newTrackedObject( STACKED_SLICES, 150, 10, STATIC_ONLY );
  newTrackedObject( STACKED_SLICES, 150, 200-10, STATIC_ONLY );
  newTrackedObject( STACKED_SLICES, 10, 100, STATIC_ONLY );
  newTrackedObject( STACKED_SLICES, 300-10, 100, STATIC_ONLY );
}

void lineMovementTest(){
  movementManager->xPos = 0;
  movementManager->yPos = 0;
  movementManager->theta = 0;

  //movementManager->ignoreObjectCollision = true;
  movementManager->ignoreWallCollision = true;

  boardManager->currentMode = IDLE;

  movementManager->clearCommands(); 
  movementManager->pointToPointMove( 200, 0, 0, 1 );
  movementManager->pointToPointMove( 200, 0, -PI/4, 1 ); 
  movementManager->pointToPointMove( 0, 0, PI/2, 1 );
  movementManager->pointToPointMove( 0, 0, PI/4, 1 );
  movementManager->pointToPointMove( 0, 0, 0, 0.5 );
  movementManager->editAngleMarginOnRecentCommand( 1*DEG_TO_RAD ); 
}

void raycastForHitToLED(){
  bool somethingThere = -1 != checkHitsAnyObjects( movementManager->xPos, movementManager->yPos, movementManager->theta );

  digitalWrite( MISC_INFO_PIN, somethingThere?HIGH:LOW );
}

void openPlaugh(){
  stopMotors();
  delay(60);
  movementManager->recalculatePosition();
  digitalWrite( SOLENOID_PLAUGH_PINS, HIGH );
  delay(60);
}

void closePlaugh(){
  stopMotors();
  delay(60);
  movementManager->recalculatePosition();
  digitalWrite( SOLENOID_PLAUGH_PINS, LOW ); 
  delay(60);
}


unsigned long ACTIVATION_TIME = 0;
void timerCheckCommand(){
  unsigned long timeProgress = millis()-ACTIVATION_TIME;
  if ( timeProgress > (100*1000) ){
    stopMotors();
    closePlaugh();
    addScore(1);
    errorFlash(2);
  }else if ( timeProgress > ((100-15)*1000) ){
    if ( boardManager->currentMode != RETURN_HOME ){
      boardManager->setModeReturnHome();
    }
  }
}
double analogInitStr = 0;

void setup() { 
  for ( int i=0; i<OBJECT_BUFFER_MAX; i++ ){
    allTrackedObjects[i].indexID = i;
    allTrackedObjects[i].reset( true ); 
  }
  for ( int i=0; i<FREE_TRACKING_BUFFER; i++ ){ 
    allTrackedObjects[i+FREE_TRACKING_POINTER].forDynamicAllocation = true; 
  }

  for ( int i=0; i<MAIN_DIST_SENSOR_COUNT; i++ ){
    echoPins[i] = (byte) main_Sensors[i].pin;
  }
  HCSR04.begin(triggerPinMain, echoPins, echoCount  );

  Wire.begin();                                               // Begin I2C bus
  Serial.begin(9600);                                         // Begin serial
  delay(100);                                                 // Wait for everything to power up

  Wire.beginTransmission(MD25ADDRESS);                        // Set MD25 operation MODE
  Wire.write(MODE_SELECTOR);
  Wire.write(MODE);                                           
  Wire.endTransmission();
  delay(50);      

  resetEncoders();    
  delay(10);                                                 // Cals a function that resets the encoder values to 0 
  stopMotors();
   
  pinMode( 12, OUTPUT );
  pinMode( 52, OUTPUT );
  pinMode( 50, OUTPUT );
  pinMode( SCREEN_PIN, OUTPUT );
  pinMode( 48, OUTPUT );
  pinMode( RGB_PIN, OUTPUT );


  pinMode( ACT_PIN, INPUT );

  pinMode( SOLENOID_PLAUGH_PINS, OUTPUT ); 
  pinMode( SOLENOID2_PINS, OUTPUT );
  pinMode( MISC_INFO_PIN, OUTPUT );
  pinMode( MISC_INFO_PIN2, OUTPUT );
  Serial.println( "4" );


  boardObjectInitiation(); 
  Serial.println( "4.5" );
  defineZeroPositions();
  Serial.println( "4.7" );
  movementManager->disableCommandLegalCheck = true;

  if ( GREEN_START ){
    forgetSlicesInRegion( 0, 0, 300, 100 );
  }else{
    forgetSlicesInRegion( 0, 100, 300, 200 );
  }
  Serial.println( "3" );
  
  //sliceGrabTest();

  //singleObsticalTestFunc();
  //cornerMovementProtocol();
  
  //boardManager->enterYZeroMode();
  //

  //basicMovementTest();
  //squareMovementObsticalTest();
  //squareMovementTest();
  //lineMovementTest();

  //movementManager->xPos = 150;
  //movementManager->yPos = 100;  
  
  Serial.println( "2" );
  closePlaugh();
  Serial.println( "2.3" );

  delay( 50 );
} 

int tRad = 20;

void testFunc(){
  setWheelLeftSpeed(-0.1);
  setWheelRightSpeed(0.1);
  delay(10000);
  stopMotors();

  delay(300000);


}

int printIndex = 0;
bool toggle = false;

bool startED = false;

void waitingForStart(){ 
  if ( startED ){
    return;
  }else{
    delay( 400 );
    analogInitStr = analogRead( ACT_PIN );
  }

  while ( (abs(analogRead( ACT_PIN ) - analogInitStr)/analogInitStr) < 0.1  && !startED ){//  
    digitalWrite( 12, (millis()%3000<1000)?LOW:HIGH  );
    Serial.println( (abs(analogRead( ACT_PIN ) - analogInitStr)/analogInitStr) );
    delay(100);
  }
  ACTIVATION_TIME = millis();
  startED = true;
}

void loop() { 
  waitingForStart();
  timerCheckCommand();
   
  analogWrite( 12, (int) 255*abs(sin( (millis())/1000.0 )) );
  //analogWrite( 52, (int) 255*abs(sin( (millis())/3600.0 )) );
  //analogWrite( 50, (int) 255*abs(sin( (millis())/6700.0 )) );
  //analogWrite( 48, (int) 255*abs(sin( (millis())/1200.0 )) );  

  boardManager->mainBehaviourLoop( false ); 
  //posSensorManager->printPingLens();

  //posSensorManager->printPingLens();

  if ( long(printIndex)*2000 < millis()  ){
    printIndex++;
    //printMap( true );
    /*printIndex++;
    analogWrite( 3, toggle?((int) 0):((int) 255) );
    analogWrite( 4, toggle?((int) 0):((int) 255) );
    analogWrite( 5, toggle?((int) 0):((int) 255) );*/

    if (toggle  ){
      //openPlaugh();
      //digitalWrite( SOLENOID2_PINS, LOW );
    }else{
      //closePlaugh();
      //digitalWrite( SOLENOID2_PINS, HIGH );
    }

    toggle = toggle != true;
    
    //raycastForHitToLED();
  } 

}
