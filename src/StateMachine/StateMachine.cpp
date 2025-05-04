#include "Arduino.h"
#include "StateMachine.h"

StateMachine::StateMachine(){
  _state = State::SEEK;
  _input = uint8_t(0);  
}

/*
  getState() - Retrieves current state
*/
State StateMachine::getState(){
  return _state;
}

/*
  getPrevInput() - Retrieves last used input
*/
uint8_t StateMachine::getPrevInput(){
  return _input;
}

/*
  run() - Calculate next state
*/
State StateMachine::run(int xcoord, int laserLeft, int lasterRight){
  _input = _calcInput(xcoord, laserLeft, lasterRight);
  _state = _calcState(_input, _state);
  return _state;
}

/*
  Compress sensor data into binary
*/
uint8_t StateMachine::_calcInput(int xcoord, int laserLeft, int laserRight){
  // Tune these parameters as needed
  int laserInfinity = 55; // farthest distance we can expect inside maze
  unsigned long timeLimit = 180000; // maximum time allowed for putzing around the maze

  uint8_t out = 0b00000000;

  // Flip CAM on if ball is detected
  if(xcoord >= 0){
    out |= 0b00001000;
  }

  // Flip W1 and/or W0 if the laser distance is within "infinity"
  if(laserLeft < laserInfinity){
    out |= 0b00000100;
  }
  if(laserRight < laserInfinity){
    out |= 0b00000010;
  }

  // Flip TIME if out of time
  if(millis() >= timeLimit){
    out |= 0b00000001;
  }

  return out;
}

State StateMachine::_calcState(uint8_t in, State st){
  uint8_t out;
  uint8_t past = static_cast<int>(st);

  // Divide up past state into bits
  bool Q2 = past & 4; 
  bool Q1 = past & 2; 
  bool Q0 = past & 1;
  // Same with the input
  bool CAM  = in & 8; 
  bool W1   = in & 4; 
  bool W0   = in & 2; 
  bool TIME = in & 1;

  // Compute third output bit
  uint8_t out2 = Q2 << 2;

  // Compute second output bit
  uint8_t out1 = (
    (Q1 & Q0 & !W1)
    | (!Q2 & !Q0 & CAM & !TIME)
    | (!Q2 & !Q1 & CAM & !TIME)
    | (Q0  & !W1 & !W0 & !TIME)
  ) << 1;

  // Comput first output bit
  uint8_t out0 = (
    (Q1 & Q0)
    | (Q0  & !W1 & !W0 & !TIME)
    | (!Q2 & !Q1 &  W0 & !CAM)
    | (!Q2 & !Q1 &  W0 & TIME)
    | (!Q2 & !Q1 &  W1 & TIME)
    | (!Q2 & !Q1 &  W1 & !CAM)
  );

  // Combine output bits (also mask out the leading bits
  // just in case Arduino does something weird with them)
  out = (out2 | out1 | out0) & 15;
  
  return static_cast<State>(out);  
}