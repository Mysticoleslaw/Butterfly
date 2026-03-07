#include "FrskySbus.h"

#define debug_port Serial
#define sbus_port Serial2

FrskySbus frsky_(sbus_port);
bool failsafe_ = false;
bool lost_frame_ = false;
#define NUM_CHANNELS 8
uint16_t servo_channels_[FrskySbus::kNumChannelsMax];

void setup()
{
    debug_port.begin(115200);
    while (!debug_port)
    {
    };
    frsky_.begin();
    debug_port.println("Ready!");
}

void loop()
{
    if (frsky_.checkForNewMessage())
    {
        // frsky_.getNewMessage(servo_channels_, failsafe_, lost_frame_);
        PrintChannels();
    }
}

void PrintChannels()
{
    debug_port.print("Rx: ");
    for (int i = 0; i < NUM_CHANNELS; i++)
    {
        debug_port.print(frsky_.getChannelValue[i]);
        debug_port.print(",");
    }
    debug_port.println();
}