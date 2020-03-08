#include <Arduino.h>
#include <SoftwareSerial.h>
#include "mmw_output.h"


class IWR1443 {
public:
    IWR1443(SoftwareSerial &debug);
    void setupComms();
    int getSamples(double array[], int num_samples);
    void stopComms();
private:
    void waitForMagicWord();
    void loadHeader();
    uint32_t processTLVs();
    MmwDemo_output_message_header header;
    SoftwareSerial &debug;
};
