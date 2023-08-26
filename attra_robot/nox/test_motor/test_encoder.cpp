#include <iostream>
#include <wiringPi.h>
#include "Encoder.h"

volatile int encoder_L_Count = 0;
volatile int encoder_R_Count = 0;

Encoder encoder_L(0, 3);
Encoder encoder_R(2, 4);

void read_Encoder_L() {
    encoder_L_Count++;
}
void read_Encoder_R() {
    encoder_R_Count++;
}

int main()
{
    if (wiringPiSetup() == -1)
    {
        std::cerr << "Failed to initialize WiringPi." << std::endl;
        return 1;
    }

    wiringPiISR(encoder_L, INT_EDGE_RISING, &read_Encoder_L);
    wiringPiISR(encoder_R, INT_EDGE_RISING, &read_Encoder_R);

    while (true)
    {
        std::cout << "==================" << std::endl;
        std::cout << "pos_l : " << encoder_L_Count << std::endl;
        std::cout << "pos_r : " << encoder_R_Count << std::endl;
        encoder_L_Count = encoder_R_Count = 0;
        delay(100);
    }

    return 0;
}
