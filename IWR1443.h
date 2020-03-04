#include <Arduino.h>
#include "mmw_output.h"


class IWR1443 {
public:
    IWR1443();
    void setupComms();
    int getSamples(double array[], int num_samples);
    void stopComms();
private:
    void waitForMagicWord();
    void loadHeader();
    double processTLVs();
    MmwDemo_output_message_header header;

};
