#include "Encoder.h"
#include <wiringPi.h>
#include <iostream>

Encoder::Encoder(uint8_t pin_a, uint8_t pin_b)
{
    _pin_a = pin_a;
    _pin_b = pin_b;
    
    if (wiringPiSetup() == -1)
    {
        std::cerr << "Failed to initialize WiringPi." << std::endl;
        exit(0);
    }

    pinMode(_pin_a, INPUT);
    pinMode(_pin_b, INPUT);
    pullUpDnControl(_pin_a, PUD_UP);
    pullUpDnControl(_pin_b, PUD_UP);
}

Encoder::~Encoder() {
    std::cout << "disconnected Encoder" << std::endl;
}

int wiringPiISR(Encoder &obj, int mode, void (*function)(void)) {
    return wiringPiISR(obj._pin_a, mode, function);
};
