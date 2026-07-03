#include "CanSniffer.h"

// Motor surucu (E32): Cin menseli kontrolculerde 250k yaygin, arac hatti
// (TUFAN-AKS CanManager) 500k kullaniyor — ikisi de oncelikli denenir.
const BaudConfig CanSniffer::BAUD_LIST[] = {
    {TWAI_TIMING_CONFIG_500KBITS(), "500 kbps"},
    {TWAI_TIMING_CONFIG_250KBITS(), "250 kbps"},
    {TWAI_TIMING_CONFIG_125KBITS(), "125 kbps"},
    {TWAI_TIMING_CONFIG_1MBITS(),  "1000 kbps"},
};
const size_t CanSniffer::BAUD_COUNT =
    sizeof(CanSniffer::BAUD_LIST) / sizeof(CanSniffer::BAUD_LIST[0]);

CanSniffer::CanSniffer()
    : stats{}, statCount(0), markerNo(0), csvMode(false), lastSummary(0) {}

void CanSniffer::begin() {
    Serial.println("\n========================================");
    Serial.println("  Motor CAN Sniffer — TWAI + SN65HVD230");
    Serial.println("        [LISTEN-ONLY / pasif mod]");
    Serial.println("========================================");
    Serial.println("Seri komutlar:");
    Serial.println("  c -> CSV modu ac/kapa (analiz kaydi icin)");
    Serial.println("  m -> senaryo isaretleyici bas (orn. 'gaza bastim')");
    Serial.println("  s -> ozeti hemen yazdir");

    if (!scanBaud()) {
        Serial.println("\n[HATA] TWAI baslatilmadi! Kontrol listesi:");
        Serial.println("  1. SN65HVD230 VCC -> 3V3 (5V DEGIL!)");
        Serial.println("  2. GND ortak mi?");
        Serial.println("  3. ESP TX(GPIO5) -> cip D/CTX, ESP RX(GPIO4) -> "
                       "cip R/CRX (RX/TX capraz DEGIL, duz)");
        while (1) {
            delay(1000);
        }
    }

    Serial.println("\n--- Dinleniyor ---");
    Serial.println(
        "TYPE     | ID           | DLC | BYTES (HEX, degisenler [..])");
    Serial.println(
        "---------|--------------|-----|------------------------------");
}

void CanSniffer::handle() {
    handleSerialCommand();

    if (millis() - lastSummary >= 5000) {
        lastSummary = millis();
        printSummary();
    }

    twai_message_t msg;
    if (twai_receive(&msg, pdMS_TO_TICKS(20)) != ESP_OK) {
        return;
    }

    uint8_t diffMask = 0;
    recordFrame(msg, diffMask);
    printFrame(msg, diffMask);
}

// ------------------------------------------------------------------ TWAI ---

bool CanSniffer::initTWAI(const BaudConfig& cfg, bool listenOnly) {
    // LISTEN-ONLY: hatta hicbir bit (ACK dahil) basilmaz. Yanlis baud'da
    // bile motor surucunun hata sayaclari etkilenmez.
    // NOT: Hatta bizden baska ACK'layan bir dugum yoksa (hat sadece
    // surucu + sniffer ise) surucu ACK alamayip yayini kesebilir — o
    // durumda dogru baud kesinlestikten sonra listenOnly=false ile
    // baslatmak gerekir (asagida scanBaud iki asamali bunu yapar).
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        CAN_TX_PIN, CAN_RX_PIN,
        listenOnly ? TWAI_MODE_LISTEN_ONLY : TWAI_MODE_NORMAL);
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &cfg.timing, &f_config) != ESP_OK) {
        return false;
    }
    if (twai_start() != ESP_OK) {
        twai_driver_uninstall();
        return false;
    }
    return true;
}

bool CanSniffer::scanBaud() {
    // Her baud'da 6 sn pasif dinle; frame yakalanan hizda kilitlen.
    while (true) {
        for (size_t i = 0; i < BAUD_COUNT; i++) {
            const BaudConfig& cfg = BAUD_LIST[i];
            Serial.printf("Deneniyor: %s (listen-only, 6 sn) ... ",
                          cfg.label);
            if (!initTWAI(cfg, true)) {
                Serial.println("driver hatasi!");
                return false;
            }

            twai_message_t msg;
            unsigned long t0 = millis();
            bool got = false;
            while (millis() - t0 < 6000) {
                if (twai_receive(&msg, pdMS_TO_TICKS(100)) == ESP_OK) {
                    got = true;
                    break;
                }
            }

            if (got) {
                Serial.println("FRAME YAKALANDI — kilitlendi!");
                return true;
            }
            Serial.println("frame yok.");
            twai_stop();
            twai_driver_uninstall();
        }
        Serial.println("[UYARI] Hicbir hizda frame yok — tarama basa "
                       "donuyor. Kontak acik mi? CAN-H/L dogru mu? "
                       "Terminasyon (~60 ohm) var mi?");
    }
}

// ------------------------------------------------------------ istatistik ---

IdStats* CanSniffer::recordFrame(const twai_message_t& msg,
                                 uint8_t& diffMask) {
    const uint8_t dlc =
        (msg.data_length_code > 8) ? 8 : msg.data_length_code;

    IdStats* s = nullptr;
    for (uint8_t i = 0; i < statCount; i++) {
        if (stats[i].id == msg.identifier) {
            s = &stats[i];
            break;
        }
    }
    if (s == nullptr) {
        if (statCount >= 64) {
            return nullptr;
        }
        s = &stats[statCount++];
        memset(s, 0, sizeof(IdStats));
        s->id = msg.identifier;
        s->extended = msg.extd;
        s->firstSeenMs = millis();
    }

    s->dlc = dlc;
    diffMask = 0;
    if (s->count > 0) {
        for (uint8_t i = 0; i < dlc; i++) {
            if (msg.data[i] != s->lastData[i]) {
                diffMask |= (1 << i);
            }
        }
        s->changeMask |= diffMask;
    }
    memcpy(s->lastData, msg.data, 8);
    s->lastSeenMs = millis();
    s->count++;
    return s;
}

// ----------------------------------------------------------------- cikti ---

void CanSniffer::printFrame(const twai_message_t& msg, uint8_t diffMask) {
    const uint8_t dlc =
        (msg.data_length_code > 8) ? 8 : msg.data_length_code;

    if (csvMode) {
        // ms,id_hex,ext,dlc,b0..b7 — pandas: pd.read_csv(f, comment='#')
        Serial.printf("%lu,%08lX,%d,%d", millis(),
                      (unsigned long)msg.identifier, msg.extd ? 1 : 0, dlc);
        for (uint8_t i = 0; i < 8; i++) {
            if (i < dlc) {
                Serial.printf(",%d", msg.data[i]);
            } else {
                Serial.print(",");
            }
        }
        Serial.println();
        return;
    }

    if (msg.extd) {
        Serial.printf("EXT      | 0x%08lX   |  %d  |",
                      (unsigned long)msg.identifier, dlc);
    } else {
        Serial.printf("STD      | 0x%03lX        |  %d  |",
                      (unsigned long)msg.identifier, dlc);
    }
    // Bir onceki frame'e gore degisen baytlar [..] ile vurgulanir —
    // canli alanlari (RPM/akim adaylari) gozle ayirmayi kolaylastirir.
    for (uint8_t i = 0; i < dlc; i++) {
        if (diffMask & (1 << i)) {
            Serial.printf("[%02X]", msg.data[i]);
        } else {
            Serial.printf(" %02X ", msg.data[i]);
        }
    }
    Serial.println();
}

void CanSniffer::printSummary() {
    if (statCount == 0) {
        Serial.println("\n# [OZET] Hic frame alinamadi — kontak acik mi? "
                       "CAN-H/L dogru mu? Terminasyon var mi?\n");
        return;
    }

    // '#' onekli satirlar CSV analizinde yorum olarak atlanir.
    Serial.printf("\n# [OZET] %d farkli ID | id / adet / ort.periyot / "
                  "degisen baytlar\n",
                  statCount);
    for (uint8_t i = 0; i < statCount; i++) {
        const IdStats& s = stats[i];
        unsigned long span = s.lastSeenMs - s.firstSeenMs;
        unsigned long avgP = (s.count > 1) ? span / (s.count - 1) : 0;

        Serial.printf("#  0x%08lX %s %7lu %6lu ms [", (unsigned long)s.id,
                      s.extended ? "EXT" : "STD", (unsigned long)s.count,
                      avgP);
        for (uint8_t b = 0; b < s.dlc; b++) {
            Serial.print((s.changeMask & (1 << b)) ? 'X' : '.');
        }
        Serial.println("]");
    }
    Serial.println();
}

void CanSniffer::handleSerialCommand() {
    if (!Serial.available()) {
        return;
    }
    char c = Serial.read();
    switch (c) {
        case 'c':
            csvMode = !csvMode;
            if (csvMode) {
                Serial.println("ms,id_hex,ext,dlc,b0,b1,b2,b3,b4,b5,b6,b7");
            } else {
                Serial.println("# CSV modu kapandi");
            }
            break;
        case 'm':
            markerNo++;
            Serial.printf("# MARKER %u @ %lu ms\n", markerNo, millis());
            break;
        case 's':
            lastSummary = 0;
            break;
        default:
            break;
    }
}
