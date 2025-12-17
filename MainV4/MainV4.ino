/*
    This version includes maze mapping, floodfill solving.
    created on: 16/08/2024
*/

#include "Init.h"

Compass compass(4, 3);
BinMaze binMaze(1);
BinMaze imaginedMaze(0);


void setup() {

  Serial.begin(9600);

  //Set Compass
  compass.setOrient('B');
  compass.setCoordinate(0, 0);

  //TOF X-Shut
  pinMode(26, OUTPUT);
  pinMode(27, OUTPUT);
  pinMode(28, OUTPUT);
  pinMode(29, OUTPUT);
  pinMode(30, OUTPUT);
  pinMode(31, OUTPUT);

  //Motor
  pinMode(12, OUTPUT);        //PWMA
  pinMode(13, OUTPUT);        //PWMB
  pinMode(14, OUTPUT);        //M2A
  pinMode(15, OUTPUT);        //M2B
  pinMode(16, OUTPUT);        //M1A
  pinMode(17, OUTPUT);        //M1B
  pinMode(66, INPUT_PULLUP);  //M2 C1
  pinMode(67, INPUT_PULLUP);  //M2 C2
  pinMode(68, INPUT_PULLUP);  //M1 C1
  pinMode(69, INPUT_PULLUP);  //M1 C2

  //Switch S3
  pinMode(11, INPUT);//Physical Pulldown

  //Setup
  //attachPCINT(digitalPinToPCINT(11), halt, RISING);
  attachPCINT(digitalPinToPCINT(68), encoder1, CHANGE);
  attachPCINT(digitalPinToPCINT(69), encoder1, CHANGE);
  attachPCINT(digitalPinToPCINT(67), encoder2, CHANGE);
  attachPCINT(digitalPinToPCINT(66), encoder2, CHANGE);
  initState1 = digitalRead(68);
  initState2 = digitalRead(66);

  digitalWrite(26, LOW);
  digitalWrite(27, LOW);
  digitalWrite(28, LOW);
  digitalWrite(29, LOW);
  digitalWrite(30, LOW);
  digitalWrite(31, LOW);

  Wire.begin();

  digitalWrite(28, HIGH);
  delay(100);
  sensor1.init(true);
  Serial.println("01");
  delay(100);
  sensor1.setAddress((uint8_t)01);
  digitalWrite(29, HIGH);
  delay(100);
  sensor2.init(true);
  Serial.println("02");
  delay(100);
  sensor2.setAddress((uint8_t)02);
  digitalWrite(30, HIGH);
  delay(100);
  sensor3.init(true);
  Serial.println("03");
  delay(100);
  sensor3.setAddress((uint8_t)03);
  digitalWrite(31, HIGH);
  delay(100);
  sensor4.init(true);
  Serial.println("04");
  delay(100);
  sensor4.setAddress((uint8_t)04);
  digitalWrite(26, HIGH);
  delay(100);
  sensor5.init(true);
  Serial.println("05");
  delay(100);
  sensor5.setAddress((uint8_t)05);
  digitalWrite(27, HIGH);
  delay(100);
  sensor6.init(true);
  Serial.println("06");
  delay(100);
  sensor6.setAddress((uint8_t)06);

  sensor1.startContinuous();
  sensor2.startContinuous();
  sensor3.startContinuous();
  sensor4.startContinuous();
  sensor5.startContinuous();
  sensor6.startContinuous();

  refreshTof();
  /*findThisState();
  binMaze.updateMaze(compass.getOrient(), compass.coordinate[0], compass.coordinate[1], thisCellFront, thisCellRight, thisCellBack, thisCellLeft);
  imaginedMaze.updateMaze(compass.getOrient(), compass.coordinate[0], compass.coordinate[1], thisCellFront, thisCellRight, thisCellBack, thisCellLeft);
  compass.show();*/
}

void loop() {
  // -----------------------Required every loop ------------------------------


  //**************************************************************************

  m1(0);
  m2(0);
  //delay(20);

  if(compass.coordinate[0] == end[0][0] && compass.coordinate[1] == end[0][1] && scanCount % 2 == 0){

    Serial.println("Scan Start to Center ENDS");
    scanCount++;
    Serial.println("Main Maze");
    binMaze.display();
    delay(1000);
    /*binMaze.maze2PathMatrix();
    binMaze.display();
    FloodFill solveMaze(binMaze);
    solveMaze.compute();
    while(1){
    //wait till button press
      while(1){
        if(digitalRead(11) == HIGH){
          delay(100);
          break;
        }
      }
      delay(1000);

      //reset coordinaes
      compass.setCoordinate(start[0], start[1]);
      compass.setOrient('B');

      //run shortest path
      tracePath(solveMaze);
      binMaze.showPath();
    }*/
  }
  else if(compass.coordinate[0] == start[0] && compass.coordinate[1] == start[1] && scanCount % 2 != 0){
    Serial.println("Scan Center to Start ENDS");
    scanCount++;
    delay(1000);
  }

  if(compass.coordinate[0] == start[0] && compass.coordinate[1] == start[1] && scanCount == 2){

    Serial.println("One Scan Completed");
    binMaze.display();
    FloodFill solveMaze(binMaze);
    solveMaze.compute();
    while(1){
    //wait till button press
      while(1){
        if(digitalRead(11) == HIGH){
          delay(100);
          break;
        }
      }
      delay(1000);

      //reset coordinates
      compass.setCoordinate(start[0], start[1]);
      compass.setOrient('B');

      //run shortest path
      tracePath(solveMaze);
      binMaze.showPath();
    }
      
    }

  FloodFill scanMaze(imaginedMaze);
  if(scanCount % 2 == 0)
    scanMaze.computeStart2End(compass.coordinate[0], compass.coordinate[1]);
  else
    scanMaze.computeEnd2Start(compass.coordinate[0], compass.coordinate[1]);
  
  tracePathToScan(scanMaze);
  
}

void moveToNextCell() {

  int xdeg = 445;                        //380
  float part_before_deacclerate = 0.80;  //0.80
  float deaccleration = 0.3;             //0.09
  float max_part_of_pid = 0.5;

  //part_before_deacclerate += (part_before_deacclerate - part_of_pid);

  if (frontFlag) {
    wheel1_initial_pos = enc1;
    wheel2_initial_pos = enc2;
    compass.updateCoord();
    frontFlag = false;
  }
  while (min(abs(enc2 - wheel2_initial_pos), abs(enc1 - wheel1_initial_pos)) < abs(int(xdeg * COUNT_PER_DEGREE))) {

    refreshTof();

    // force stop
    if ((tofa + tofd) / 2 <= FRONT_TERMINATE_DIST) {
      m1(0);
      m2(0);
      break;
    }

    // deaccleration
    if (abs(enc1 - wheel1_initial_pos) > int(part_before_deacclerate * abs(int(xdeg * COUNT_PER_DEGREE))) && abs(enc2 - wheel2_initial_pos) > int(part_before_deacclerate * abs(int(xdeg * COUNT_PER_DEGREE)))) {

      motor_speed = motor_speed - deaccleration * abs(enc1 - wheel1_initial_pos);
      if (motor_speed < 100) { motor_speed = 100; }
    }

    //use PID
    // follow left wall
    if (checkLeft() && checkNextLeft()) {
      pid_left();
    }
    // follow right wall
    else if (checkRight() && checkNextRight()) {
      pid_right();
    }
    //runnn....
    else {
      long enc_count_without_pid = abs(xdeg * COUNT_PER_DEGREE) - ((enc1 - wheel1_initial_pos) + (enc2 - wheel2_initial_pos)) / 2;

      move_forward_with_encoder(xdeg * COUNT_PER_DEGREE, enc_count_without_pid, deaccleration, part_before_deacclerate);
    }
    delay(5);
  }
  m1(0);
  m2(0);
  //buzzer
  //reset speed
  motor_speed = MOTOR_SPEED;
  //reset encoder pid
  previous_fwd = 0;
  integral_fwd = 0;
  //reset encoder
  lastEnc1 = 0;
  lastEnc2 = 0;
  initState1 = 0;
  initState2 = 0;
  enc1 = 0;
  enc2 = 0;
  frontFlag = true;
}

void turn_90(char dir) {
  int deg = 90;
  float coff = 0.9 ;//0.9275                               //small= 1.3 big =1.36
  int steps = deg * 2.34 * COUNT_PER_DEGREE * coff;  //2.34 = dist betw wheel / dia of wheel
  float feed_forward = ff_turn * steps;

  if (turnFlag) {
    wheel1_initial_pos = enc1;
    wheel2_initial_pos = enc2;
    turnFlag = false;
  }
  if (dir == 'r' || dir == 'R') {
      compass.updateOrient('R');
  } else {
      compass.updateOrient('L');
    }

  // Loop until both motors have reached the target position
  while (min(abs(enc2 - wheel2_initial_pos), abs(enc1 - wheel1_initial_pos)) < abs(int(steps))) {

    double now = millis();
    dt = (now - last_time) / 1000.00;
    last_time = now;

    //Encoder pid
    if (intm_turn == 0) {
      integral_turn = 0;
      intm_turn = INT_MEMORY;
    }

    double error = steps - (abs(enc1 - wheel1_initial_pos) + abs(enc2 - wheel2_initial_pos)) / 2;
    double proportional = kp_turn * (error);
    double derivative = kd_turn * ((error) - (previous_turn)) / dt;
    double outTemp = proportional + derivative + feed_forward;
    integral_turn += ki_turn * error * dt;
    integral_turn = constrain(integral_turn, -50, 50);          // Integral Windup
    double iMax = constrain(255 - outTemp, 0, MAX_LIMIT_TURN);  // Maximum allowed integral term before saturating output
    double iMin = constrain(0 - outTemp, MIN_LIMIT_TURN, 0);
    integral_turn = constrain(integral_turn, iMin, iMax);

    previous_turn = error;
    double output_turn = proportional + integral_turn + derivative + feed_forward;
    output_turn = constrain(output_turn, MIN_LIMIT_TURN, MAX_LIMIT_TURN);
    intm_turn--;

    delay(5);
    /*Serial.print(abs(enc1 - wheel1_initial_pos));
    Serial.print(", ");
    Serial.print(abs(enc2 - wheel2_initial_pos));
    Serial.print(", ");
    Serial.println(output_turn);*/

    if (dir == 'R' || dir == 'r') {
      m1(output_turn);
      m2(output_turn * (-1));
    } else {
      m1(output_turn * (-1));
      m2(output_turn);
    }
  }
  m1(0);
  m2(0);
  wheel1_initial_pos = enc1;
  wheel2_initial_pos = enc2;
  delay(100);
  while (abs(enc1 - wheel1_initial_pos) < 65 || abs(enc2 - wheel2_initial_pos) < 65) {
    m1(-180);
    m2(-180);
  }
  // Stop the motors after the turn is completed
  m1(0);
  m2(0);
  lastEnc1 = 0;
  lastEnc2 = 0;
  initState1 = 0;
  initState2 = 0;
  enc1 = 0;
  enc2 = 0;
  turnFlag = true;
}

void tracePath(FloodFill pathtofollow){

  for (char desiredDir : pathtofollow.maze.pathMap) {

    if(desiredDir == 'X'){
      continue;
    }
    if(desiredDir == 'Z'){
      break;
    }
    compass.show();
    
    if(compass.getOrient() != desiredDir){
      
      switch (desiredDir) {

        case 'T':
          switch (compass.getOrient()) {
            case 'R':
              delay(100);
              turn_90('L');
              break;
            case 'B':
              delay(100);
              turn_90('R');
              align_yaw_and_center();
              turn_90('R');
              break;
            case 'L':
              delay(100);
              turn_90('R');
              break;
            default:
              break;
          }
          break;

        case 'R':
          switch (compass.getOrient()) {
            case 'T':
              delay(100);
              turn_90('R');
              break;
            case 'B':
              delay(100);
              turn_90('L');
              break;
            case 'L':
              delay(100);
              turn_90('R');
              align_yaw_and_center();
              turn_90('R');
              break;
            default:
              break;
          }
          break;
        
        case 'B':
          switch (compass.getOrient()) {
            case 'T':
              delay(100);
              turn_90('R');
              align_yaw_and_center();
              turn_90('R');
              break;
            case 'R':
              delay(100);
              turn_90('R');
              break;
            case 'L':
              delay(100);
              turn_90('L');
              break;
            default:
              break;
          }
          break;
        
        case 'L':
          switch (compass.getOrient()) {
            case 'T':
              delay(100);
              turn_90('L');
              break;
            case 'R':
              delay(100);
              turn_90('R');
              align_yaw_and_center();
              turn_90('R');
              break;
            case 'B':
              delay(100);
              turn_90('R');
              break;
            default:
              break;
          }
          break;
        default:
          break;
      }
    }

    align_yaw_and_center();
    moveToNextCell();
    align_yaw_and_center();
  }
  compass.show();
  Serial.println("Finished!");
  return;

}

void tracePathToScan(FloodFill pathtofollow){

  for (char desiredDir : pathtofollow.maze.pathMap) {

    if(desiredDir == 'X'){
      continue;
    }

    if(imaginedMaze.maze[compass.coordinate[0]][compass.coordinate[1]] == 0){

      refreshTof();
      findThisState();
      binMaze.updateMaze(compass.getOrient(), compass.coordinate[0], compass.coordinate[1], thisCellFront, thisCellRight, thisCellBack, thisCellLeft);
      imaginedMaze.updateMaze(compass.getOrient(), compass.coordinate[0], compass.coordinate[1], thisCellFront, thisCellRight, thisCellBack, thisCellLeft);
    
    }
    else{
      refreshTof();
      findThisState();
    }

    if(desiredDir == 'Z'){
      break;
    }

    
    if(compass.getOrient() != desiredDir){
      
      switch (desiredDir) {

        case 'T':
          switch (compass.getOrient()) {
            case 'R':
              if(thisCellLeft == true)
                return;
              delay(100);
              turn_90('L');
              break;
            case 'B':
              if(thisCellBack == true)
                return;
              delay(100);
              turn_90('R');
              align_yaw_and_center();
              turn_90('R');
              break;
            case 'L':
              if(thisCellRight == true)
                return;
              delay(100);
              turn_90('R');
              break;
            default:
              break;
          }
          break;

        case 'R':
          switch (compass.getOrient()) {
            case 'T':
              if(thisCellRight == true)
                return;
              delay(100);
              turn_90('R');
              break;
            case 'B':
              if(thisCellLeft == true)
                return;
              delay(100);
              turn_90('L');
              break;
            case 'L':
              if(thisCellBack == true)
                return;
              delay(100);
              turn_90('R');
              align_yaw_and_center();
              turn_90('R');
              break;
            default:
              break;
          }
          break;
        
        case 'B':
          switch (compass.getOrient()) {
            case 'T':
              if(thisCellBack == true)
                return;
              delay(100);
              turn_90('R');
              align_yaw_and_center();
              turn_90('R');
              break;
            case 'R':
              if(thisCellRight == true)
                return;
              delay(100);
              turn_90('R');
              break;
            case 'L':
              if(thisCellLeft == true)
                return;
              delay(100);
              turn_90('L');
              break;
            default:
              break;
          }
          break;
        
        case 'L':
          switch (compass.getOrient()) {
            case 'T':
              if(thisCellLeft == true)
                return;
              delay(100);
              turn_90('L');
              break;
            case 'R':
              if(thisCellBack == true)
                return;
              delay(100);
              turn_90('R');
              align_yaw_and_center();
              turn_90('R');
              break;
            case 'B':
              if(thisCellRight == true)
                return;
              delay(100);
              turn_90('R');
              break;
            default:
              break;
          }
          break;
        default:
          break;
      }

    }

    align_yaw_and_center();
    moveToNextCell();
    align_yaw_and_center();
    prevFwdOrient = compass.getOrient();
    compass.show();
    
  }
  compass.show();
  Serial.println("Finished!");
  return;

}

void scanNextCell(int targetCount) {
  /* Require Explicit tof refresh*/
  if (abs(enc1 - wheel1_initial_pos) >= int(0.35 * abs(targetCount)) && abs(enc2 - wheel2_initial_pos) >= int(0.35 * abs(targetCount)) && !checkFront()) {
    findThisState();
  }
}

void findThisState() {

  if (tofa <= 10 && tofd <= 10) {  //According to angled tof reading > SIDE_WALL_CHECK_DIST
    thisCellFront = true;
  } else {
    thisCellFront = false;
  }

  if (checkLeft()) {
    thisCellLeft = true;
  } else {
    thisCellLeft = false;
  }

  if (checkRight()) {
    thisCellRight = true;
  } else {
    thisCellRight = false;
  }
  if (prevFwdOrient == compass.getOrient()){
    thisCellBack = false;
  } else  {
    thisCellBack = true;
  }
}

bool checkAlignment() {
  if (offAngle() <= 0.1745 && offAngle() >= -0.1745) {  // 10deg
    return true;
  } else {
    return false;
  }
}

double offAngle() {
  return tan((tofd - tofa) / TOF_A_D_DIST);
}

bool checkFront() {
  if (tofa <= FRONT_DIST && tofd <= FRONT_DIST) {
    return true;
  } else {
    return false;
  }
}

bool checkLeft() {
  if (tofe <= SIDE_WALL_CHECK_DIST) {
    return true;
  } else {
    return false;
  }
}

bool checkRight() {
  if (toff <= SIDE_WALL_CHECK_DIST) {
    return true;
  } else {
    return false;
  }
}

bool checkNextLeft() {
  if (tofb <= NEXT_SIDE_WALL_CHECK_DIST) {
    return true;
  } else {
    return false;
  }
}

bool checkNextRight() {
  if (tofc <= NEXT_SIDE_WALL_CHECK_DIST) {
    return true;
  } else {
    return false;
  }
}


void encoder1() {
  lastEnc1 = digitalRead(69);
  if (lastEnc1 != initState1) {

    if (digitalRead(68) != lastEnc1) {
      enc1++;
    } else {
      enc1--;
    }
  }
  initState1 = lastEnc1;
}
void encoder2() {
  lastEnc2 = digitalRead(67);
  if (lastEnc2 != initState2) {

    if (digitalRead(66) != lastEnc2) {
      enc2++;
    } else {
      enc2--;
    }
  }
  initState2 = lastEnc2;
}

void pid_left() {

  float kp = KP_POSITIVE;
  float ki = KI_POSITIVE;
  float kd = KD_POSITIVE;
  if (tofb > 14) {
    kp = KP_NEGITIVE;
    ki = KI_NEGITIVE;
    kd = KD_NEGITIVE;
  }
  double now = millis();
  dt = (now - last_time) / 1000.00;
  last_time = now;

  if (intm_left == 0) {
    integral_left = 0;
    intm_left = INT_MEMORY;
  }
  double error = SIDE_WALL_DIST - tofb;
  double proportional = kp * (error);
  double derivative = kd * ((error) - (previous_left)) / dt;
  double outTemp = proportional + derivative;
  integral_left += ki * error * dt;
  integral_left = constrain(integral_left, MIN_LIMIT, MAX_LIMIT);  // Integral Windup
  double iMax = constrain(MAX_LIMIT - outTemp, 0, MAX_LIMIT);      // Maximum allowed integral term before saturating output
  double iMin = constrain(0 - outTemp, MIN_LIMIT, 0);
  integral_left = constrain(integral_left, iMin, iMax);

  previous_left = error;
  double output = proportional + integral_left + derivative;
  output = constrain(output, MIN_LIMIT, MAX_LIMIT);
  intm_left--;

  motor_speed = MOTOR_SPEED - abs(output);
  if (output <= 0) {
    m1(motor_speed);
    m2(MOTOR_SPEED);
  } else {
    m1(MOTOR_SPEED);
    m2(motor_speed);
  }
  motor_speed = MOTOR_SPEED;
}

void pid_right() {

  float kp = KP_POSITIVE;
  float ki = KI_POSITIVE;
  float kd = KD_POSITIVE;
  if (tofc > 14) {
    kp = KP_NEGITIVE;
    ki = KI_NEGITIVE;
    kd = KD_NEGITIVE;
  }
  double now = millis();
  dt = (now - last_time) / 1000.00;
  last_time = now;

  if (intm_right == 0) {
    integral_right = 0;
    intm_right = INT_MEMORY;
  }
  double error = SIDE_WALL_DIST - tofc;
  double proportional = kp * (error);
  double derivative = kd * ((error) - (previous_right)) / dt;
  double outTemp = proportional + derivative;
  integral_right += ki * error * dt;
  integral_right = constrain(integral_right, MIN_LIMIT, MAX_LIMIT);  // Integral Windup
  double iMax = constrain(MAX_LIMIT - outTemp, 0, MAX_LIMIT);        // Maximum allowed integral term before saturating output
  double iMin = constrain(0 - outTemp, MIN_LIMIT, 0);
  integral_right = constrain(integral_left, iMin, iMax);

  previous_right = error;
  double output = proportional + integral_right + derivative;
  output = constrain(output, MIN_LIMIT, MAX_LIMIT);
  intm_right--;

  motor_speed = MOTOR_SPEED - abs(output);
  if (output >= 0) {
    m1(motor_speed);
    m2(MOTOR_SPEED);
  } else {
    m1(MOTOR_SPEED);
    m2(motor_speed);
  }
  motor_speed = MOTOR_SPEED;
}

void move_forward_with_encoder(int totalCountForCell, long encCountWithoutPid, double deaccleration, double part_before_deacclerate) {

  long wheel1_initial_position = enc1;
  long wheel2_initial_position = enc2;

  while ((abs(enc1 - wheel1_initial_position) <= encCountWithoutPid) && (abs(enc2 - wheel2_initial_position) <= encCountWithoutPid)) {

    refreshTof();

    if ((tofa + tofd) / 2 <= FRONT_TERMINATE_DIST) {
      m1(0);
      m2(0);
      break;
    }

    //Deaccleration calculation
    if (abs(enc1 - wheel1_initial_position) > int(part_before_deacclerate * abs(encCountWithoutPid)) && abs(enc2 - wheel2_initial_position) > int(part_before_deacclerate * abs(encCountWithoutPid))) {

      motor_speed = motor_speed - deaccleration * abs(enc1 - wheel1_initial_position);
      if (motor_speed < 80) { motor_speed = 80; }
    }

    double now = millis();
    dt = (now - last_time) / 1000.00;
    last_time = now;

    //Encoder pid
    if (intm_fwd == 0) {
      integral_fwd = 0;
      intm_fwd = INT_MEMORY;
    }
    double error = enc1 - enc2;
    double proportional = kp_fwd * (error);
    double derivative = kd_fwd * ((error) - (previous_fwd)) / dt;
    double outTemp = proportional + derivative;
    integral_fwd += ki_fwd * error * dt;
    integral_fwd = constrain(integral_fwd, -10, 10);           // Integral Windup
    double iMax = constrain(100 - outTemp, 0, MAX_LIMIT_FWD);  // Maximum allowed integral term before saturating output
    double iMin = constrain(0 - outTemp, MIN_LIMIT_FWD, 0);
    integral_fwd = constrain(integral_fwd, iMin, iMax);

    previous_fwd = error;
    double output_fwd = proportional + integral_fwd + derivative;
    output_fwd = constrain(output_fwd, MIN_LIMIT_FWD, MAX_LIMIT_FWD);
    //output_fwd    = 0;
    intm_fwd--;

    motor_speed = MOTOR_SPEED - abs(output_fwd);

    if (output_fwd >= 0) {
      m1(motor_speed);
      m2(MOTOR_SPEED);
    } else {
      m1(MOTOR_SPEED);
      m2(motor_speed);
    }
    motor_speed = MOTOR_SPEED;
  }
  m1(0);
  m2(0);
  //reset speed
  motor_speed = MOTOR_SPEED;
}

void align_yaw_and_center() {

  refreshTof();
  if ((tofa + tofd) / 2 <= FRONT_DIST) {
    while (tofa <= 1.5 || tofd <= 1.5) {
      refreshTof();
      if (tofa <= 1.5) {
        m1(-100);
      } else {
        m1(0);
      }
      if (tofd <= 1.5) {
        m2(-100);
      } else {
        m2(0);
      }
      if ((tofa + tofd) / 2 > FRONT_DIST) { break; }
    }
    while (tofa >= 2.5 || tofd >= 2.5) {
      refreshTof();
      if (tofa >= 2.5) {
        m1(100);
      } else {
        m1(0);
      }
      if (tofd >= 2.5) {
        m2(100);
      } else {
        m2(0);
      }
      if ((tofa + tofd) / 2 > FRONT_DIST + 1) { break; }
    }
  }
  m1(0);
  m2(0);
}

void refreshTof() {
  tofa = sensor1.readRangeContinuousMillimeters() / 10.0 - 2.5;
  tofb = sensor2.readRangeContinuousMillimeters() / 10.0 - 6.5;
  tofc = sensor3.readRangeContinuousMillimeters() / 10.0 - 2.3;
  tofd = sensor4.readRangeContinuousMillimeters() / 10.0 - 3.3;
  tofe = sensor5.readRangeContinuousMillimeters() / 10.0 - 5.3;
  toff = sensor6.readRangeContinuousMillimeters() / 10.0 + 0.2;
}

void m1(double sped) {
  if (int(sped) == 0) {
    digitalWrite(17, HIGH);
    digitalWrite(16, HIGH);
    analogWrite(12, int(sped));
  }
  if (int(sped) < 0) {
    digitalWrite(17, HIGH);
    digitalWrite(16, LOW);
    analogWrite(12, int(sped) * (-1));
  }
  if (int(sped) > 0) {
    digitalWrite(17, LOW);
    digitalWrite(16, HIGH);
    analogWrite(12, int(sped));
  }
}

void m2(double sped) {
  if (int(sped) == 0) {
    digitalWrite(14, HIGH);
    digitalWrite(15, HIGH);
    analogWrite(13, int(sped));
  }
  if (int(sped) < 0) {
    digitalWrite(14, HIGH);
    digitalWrite(15, LOW);
    analogWrite(13, int(sped) * (-1));
  }
  if (int(sped) > 0) {
    digitalWrite(14, LOW);
    digitalWrite(15, HIGH);
    analogWrite(13, int(sped));
  }
}

void halt(){

  if(!haltFlag){
    delay(5);
    haltFlag = true;
    while (haltFlag) {}
  }
  haltFlag = false;
}



