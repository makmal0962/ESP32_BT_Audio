#include "AudioTools.h"
#include "BluetoothA2DPSink.h"

#define LED_PIN 2

I2SStream i2s;
BluetoothA2DPSink a2dp_sink(i2s);

bool bt_connected = false;

void bt_connection_changed(esp_a2d_connection_state_t state, void *ptr) {
    bt_connected = (state == ESP_A2D_CONNECTION_STATE_CONNECTED);
    if (bt_connected) digitalWrite(LED_PIN, HIGH); // on when connected
}

void setup() {
    Serial.begin(115200);
    pinMode(LED_PIN, OUTPUT);

    auto cfg = i2s.defaultConfig();
    cfg.pin_ws   = 23; // LCK
    cfg.pin_data = 19; // DIN
    cfg.pin_bck  = 18; // BCK
    i2s.begin(cfg);

    a2dp_sink.set_on_connection_state_changed(bt_connection_changed);
    a2dp_sink.set_auto_reconnect(true);
    a2dp_sink.start("ESP32 BT Audio");
}

void loop() {
    if (!bt_connected) {
        digitalWrite(LED_PIN, HIGH);
        delay(500);
        digitalWrite(LED_PIN, LOW);
        delay(500);
    }
}