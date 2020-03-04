#include "IWR1443.h"
#include "detected_obj.h"
#include "Arduino.h"
#define BAUD_RX 921600
#define BAUD_TX 115200
constexpr char magicWord[8] = {0x02, 0x01, 0x04, 0x03, 0x06, 0x05, 0x08, 0x07};
const char* setupCommands[] = {
    "sensorStop",
    "flushCfg",
    "dfeDataOutputMode 1",
    "channelCfg 1 1 0",
    "adcCfg 2 1",
    "adcbufCfg 0 1 0 1",
    "profileCfg 0 77 7 7 212.8 0 0  18.32 1 1024 5000 0 0 40",
    "chirpCfg 0 0 0 0 0 0 0 1",
    "frameCfg 0 0 1 0 500 1 0",
    "calibDcRangeSig 0 -5 5 32",
    "guiMonitor 1   1 0  0 0  1",
    "RangeLimitCfg 1 1.0 5.0",
    "sensorStart",
};

void IWR1443::waitForMagicWord() {
  int mw_offset = 0;
  while (mw_offset != sizeof(magicWord)) {
    while (Serial.available() == 0)
      ;
    int next_byte = Serial.read();
    if (next_byte == magicWord[mw_offset]) {
      mw_offset++;
    } else {
      mw_offset = 0;
    }
  }
}

void IWR1443::loadHeader() {
    uint8_t* start = (uint8_t*)&header.version;
    while (start < (((uint8_t*)&header.numTLVs) + sizeof(header.numTLVs))) {
        while (Serial.available() == 0)
          ;
        int next_byte = Serial.read();
        *(start++) = next_byte;
    }
    //Endianness ??
}

IWR1443::IWR1443() {}
void IWR1443::setupComms() {
  Serial.begin(BAUD_TX);
  delay(10);
  Serial.write('\n');
  for (auto &s : setupCommands) {
    delay(10);
    Serial.write(s);
    Serial.write('\n');
  }
  Serial.flush();
  Serial.begin(BAUD_RX);
}

double IWR1443::processTLVs()
{
    for (int tlv_num = 0; tlv_num < header.numTLVs; tlv_num++) {
        MmwDemo_output_message_tl tag_length;
        uint8_t *ptr = (uint8_t*)&tag_length;
        for (int i = 0; i < sizeof(tag_length); ++i) {
            while (Serial.available() == 0)
              ;
            int next_byte = Serial.read();
            *(ptr++) = next_byte;
        }
        printf("Tag %d, Length %d\n", tag_length.type, tag_length.length);
        if (tag_length.type == MMWDEMO_OUTPUT_MSG_DETECTED_POINTS) { // Obj detected.
            /* Get Metadata */
            MmwDemo_output_message_dataObjDescr obj_metadata;
            uint8_t *ptr = (uint8_t*)&obj_metadata;
            for (int i = 0; i < sizeof(obj_metadata); ++i) {
                while (Serial.available() == 0)
                  ;
                int next_byte = Serial.read();
                *(ptr++) = next_byte;
            }
            /* Process all of the associated objects. */
            for (int obj_num = 0; obj_num < obj_metadata.numDetetedObj; obj_num++) {
                MmwDemo_detectedObj detected_obj;
                uint8_t *ptr = (uint8_t*)&detected_obj;
                for (int i = 0; i < sizeof(detected_obj); ++i) {
                    while (Serial.available() == 0)
                      ;
                    int next_byte = Serial.read();
                    *(ptr++) = next_byte;
                }
                //TODO(Ben) Figure out Q format.
                int attempt = detected_obj.rangeIdx + detected_obj.x * 65536;
                double real_range_meters = attempt / 1048576.0;
                double adjusted_range = (real_range_meters - 0.0696) * 1000;
                return adjusted_range;
            }

        } else {
            return -1;
        }
    }
}

int IWR1443::getSamples(double array[], int num_samples) {
  for (int i = 0; i < num_samples; ++i) {
    waitForMagicWord();
    loadHeader();
    double height = processTLVs();
    if (height == -1) {
        return i;
    }
    array[i] = processTLVs();
  }
  return num_samples;
}

void IWR1443::stopComms()
{
    Serial.flush();
    Serial.begin(BAUD_TX);
    Serial.write(setupCommands[0]);
    Serial.flush();
    Serial.end();
}
