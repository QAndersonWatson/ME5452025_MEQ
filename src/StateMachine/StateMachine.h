/*
    StateMachine.h - Library for maze bot intelligent strategy switching
    Created by Eric S. Jurado, May 3, 2025
*/
#ifndef StateMachine_h
#define StateMachine_h

#include "Arduino.h"

enum class State
{
    SEEK        = 0b000,    // Seeking wall or ball phase
    WALLFOLLOW  = 0b001,    // Wall-following method
    PICKUP      = 0b010,    // Picking up ball method
    UTURN       = 0b011,    // U-turn at exit
    EXIT        = 0b100     // Exiting strategy
};

class StateMachine
{
    public:
    StateMachine();
    State getState();
    uint8_t getPrevInput();
    State run(int xcoord, int laserLeft, int lasterRight);

    private:
    State _state;
    uint8_t _input;

    uint8_t _calcInput(int xcoord, int laserLeft, int lasterRight);
    State _calcState(uint8_t in, State past);


};

#endif