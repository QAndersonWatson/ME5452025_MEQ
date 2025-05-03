/*
    StateMachine.h - Library for maze bot intelligent strategy switching
    Created by Eric S. Jurado, May 3, 2025
*/
#ifndef StateMachine_h
#define StateMachine_h

#include "Arduino.h"

enum States
{
    SEEK,       // Seeking wall or ball phase
    WALLFOLLOW, // Wall-following method
    PICKUP,     // Picking up ball method
    UTURN,      // U-turn at exit
    EXIT        // Exiting strategy
};

class StateMachine
{
    public:
    stateMachine();
    States getState();
    uint8_t getPrevInput();
    States run(int xcoord, int laserLeft, int lasterRight);

    private:
    States _state;
    uint8_t _input;

    uint8_t _calcInput(int xcoord, int laserLeft, int lasterRight);
    States _calcState(uint8_t in, States past);


};

#endif