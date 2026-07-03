#pragma once

#include <Arduino.h>
#include "driver/twai.h"

// SN65HVD230 baglantisi (harici MCP2515 YOK — dahili TWAI kullanilir):
//   ESP32 GPIO5 (TX) -> SN65HVD230 D (CTX)
//   ESP32 GPIO4 (RX) -> SN65HVD230 R (CRX)
//   3V3 -> VCC, GND -> GND (ortak)
//   CANH/CANL -> motor surucunun CAN hatti
#define CAN_TX_PIN GPIO_NUM_5
#define CAN_RX_PIN GPIO_NUM_4

struct BaudConfig {
    twai_timing_config_t timing;
    const char* label;
};

// Her ID icin istatistik: kac kez gorulduA hangi baytlari degisiyor.
// changeMask, bayt-deger eslestirmesinin (hangi bayt RPM/sicaklik?)
// ilk ipucudur: hic degismeyen baytlar sabit/rezerve alandir.
struct IdStats {
    uint32_t id;
    bool extended;
    uint8_t dlc;
    uint32_t count;
    unsigned long firstSeenMs;
    unsigned long lastSeenMs;
    uint8_t lastData[8];
    uint8_t changeMask;
};

class CanSniffer {
   public:
    CanSniffer();
    void begin();
    void handle();

   private:
    bool initTWAI(const BaudConfig& cfg, bool listenOnly);
    bool scanBaud();
    IdStats* recordFrame(const twai_message_t& msg, uint8_t& diffMask);
    void printFrame(const twai_message_t& msg, uint8_t diffMask);
    void printSummary();
    void handleSerialCommand();

    static const BaudConfig BAUD_LIST[];
    static const size_t BAUD_COUNT;

    IdStats stats[64];
    uint8_t statCount;
    uint16_t markerNo;
    bool csvMode;
    unsigned long lastSummary;
};
