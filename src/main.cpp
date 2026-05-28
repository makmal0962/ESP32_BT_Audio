#include "AudioTools.h"
#include "BluetoothA2DPSink.h"
#include <LittleFS.h>

#define LED_PIN 2

I2SStream i2s;
BluetoothA2DPSink a2dp_sink(i2s);

volatile bool bt_connected             = false;
volatile bool connect_chime_pending    = false;
volatile bool disconnect_chime_pending = false;
volatile bool chime_blocking           = false;

QueueHandle_t chime_queue;

void play_chime_direct(const char *path) {
    File f = LittleFS.open(path, "r");
    if (!f) return;
    f.seek(44);
    int16_t mono[256], stereo[512];
    while (f.available()) {
        size_t n = f.read((uint8_t *)mono, sizeof(mono));
        int samples = n / 2;
        for (int i = 0; i < samples; i++) {
            stereo[i*2]   = mono[i];
            stereo[i*2+1] = mono[i];
        }
        i2s.write((uint8_t *)stereo, samples * 4);
    }
    f.close();
}

void chime_task(void *param) {
    const char *path;
    for (;;) {
        if (xQueueReceive(chime_queue, &path, portMAX_DELAY)) {
            chime_blocking = true;
            delay(300);
            play_chime_direct(path);
            chime_blocking = false;
        }
    }
}

void enqueue_chime(const char *path) {
    xQueueOverwrite(chime_queue, &path); // drop old if not yet played
}

void read_data_stream(const uint8_t *data, uint32_t len) {
    if (chime_blocking) return;
    i2s.write(data, len);
}

void bt_connection_changed(esp_a2d_connection_state_t state, void *ptr) {
    bt_connected = (state == ESP_A2D_CONNECTION_STATE_CONNECTED);
    if (bt_connected) {
        digitalWrite(LED_PIN, HIGH);
        connect_chime_pending = true;
    } else {
        disconnect_chime_pending = true;
    }
}

void setup() {
    Serial.begin(115200);
    pinMode(LED_PIN, OUTPUT);
    LittleFS.begin(true);

    chime_queue = xQueueCreate(1, sizeof(const char *));
    xTaskCreatePinnedToCore(chime_task, "chime", 4096, NULL, 1, NULL, 1);

    auto cfg = i2s.defaultConfig();
    cfg.pin_bck  = 18;
    cfg.pin_ws   = 23;
    cfg.pin_data = 19;
    i2s.begin(cfg);

    play_chime_direct("/on.wav"); // blocking is fine before A2DP starts

    a2dp_sink.set_stream_reader(read_data_stream, false);
    a2dp_sink.set_on_connection_state_changed(bt_connection_changed);
    a2dp_sink.set_auto_reconnect(true);
    a2dp_sink.start("ESP32 BT Audio");
}

void loop() {
    if (connect_chime_pending) {
        connect_chime_pending = false;
        enqueue_chime("/connect.wav");
    }

    if (!bt_connected) {
        if (disconnect_chime_pending) {
            disconnect_chime_pending = false;
            enqueue_chime("/disconnect.wav");
        }
        digitalWrite(LED_PIN, HIGH);
        delay(500);
        if (!bt_connected) digitalWrite(LED_PIN, LOW);
        delay(500);
    }
}