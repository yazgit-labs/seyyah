#ifndef QUADRATUREENCODER_H
#define QUADRATUREENCODER_H
#include "Arduino.h"


enum WheelFace {LEFT = -1,RIGHT = 1 };

class Encoder {
public:
        Encoder(WheelFace);
        Encoder(WheelFace fow,int greenCablePin,int yellowCablePin);
        Encoder(void);
        int greenCablePin;
        int yellowCablePin;
        WheelFace faceOfWheel;
        long encoderTicks;

        volatile bool _greenSet;
        volatile bool _yellowSet;
        volatile bool _greenPrev;
        volatile bool _yellowPrev;

        void initialize();
        void attach(int green,int yellow);
        void handleInterruptGreen();
        void handleInterruptYellow();

        inline int parseEncoder();



};


#endif
