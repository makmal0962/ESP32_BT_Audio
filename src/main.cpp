#include "AudioTools.h"
#include "BluetoothA2DPSink.h"
#include <LittleFS.h>
#include <U8g2lib.h>
#include <Wire.h>

#define LED_PIN         2
#define DEVICE_NAME     "ESP32 BT Audio"
#define PEER_SHOW_MS    2000
#define SCROLL_SPEED    30   // px/sec
#define SCROLL_GAP      20   // px between loop

#define BTN_PLAY 26
#define BTN_PREV 16
#define BTN_NEXT 17
#define DEBOUNCE_MS 50

I2SStream i2s;
BluetoothA2DPSink a2dp_sink(i2s);
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

volatile bool bt_connected             = false;
volatile bool connect_chime_pending    = false;
volatile bool disconnect_chime_pending = false;
volatile bool chime_blocking           = false;

QueueHandle_t    chime_queue;
SemaphoreHandle_t track_mutex;
uint32_t last_btn_ms = 0;

struct TrackInfo {
    char     title[128];
    char     artist[128];
    char     album[128];
    char     peer_name[64];
    uint32_t duration_ms;
    uint32_t position_ms;
    uint32_t position_ts;   // millis() at last AVRCP position update
    bool     playing;
    bool     show_peer;
    uint32_t peer_until;
    uint32_t scroll_reset;  // millis() when track changed → resets scroll
};

TrackInfo track = {};

// --- Chime ---
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
    xQueueOverwrite(chime_queue, &path);
}

// --- A2DP stream ---
void read_data_stream(const uint8_t *data, uint32_t len) {
    if (chime_blocking) return;
    i2s.write(data, len);
}

// --- AVRCP callbacks ---
void avrc_metadata_callback(uint8_t id, const uint8_t *text) {
    xSemaphoreTake(track_mutex, portMAX_DELAY);
    switch (id) {
        case ESP_AVRC_MD_ATTR_TITLE:
            strncpy(track.title, (char *)text, 127);
            track.scroll_reset = millis(); // reset scroll on new track
            break;
        case ESP_AVRC_MD_ATTR_ARTIST:  strncpy(track.artist, (char *)text, 127); break;
        case ESP_AVRC_MD_ATTR_ALBUM:   strncpy(track.album,  (char *)text, 127); break;
        case ESP_AVRC_MD_ATTR_PLAYING_TIME:
            track.duration_ms = atoi((char *)text);
            break;
    }
    xSemaphoreGive(track_mutex);
}

void avrc_playstatus_callback(esp_avrc_playback_stat_t playback) {
    xSemaphoreTake(track_mutex, portMAX_DELAY);
    track.playing = (playback == ESP_AVRC_PLAYBACK_PLAYING);
    xSemaphoreGive(track_mutex);
}

void avrc_position_callback(uint32_t pos_ms) {
    xSemaphoreTake(track_mutex, portMAX_DELAY);
    track.position_ms = pos_ms;
    track.position_ts = millis();
    xSemaphoreGive(track_mutex);
}

void bt_connection_changed(esp_a2d_connection_state_t state, void *ptr) {
    bt_connected = (state == ESP_A2D_CONNECTION_STATE_CONNECTED);
    if (bt_connected) {
        digitalWrite(LED_PIN, HIGH);
        connect_chime_pending = true;
    } else {
        xSemaphoreTake(track_mutex, portMAX_DELAY);
        memset(track.title,  0, sizeof(track.title));
        memset(track.artist, 0, sizeof(track.artist));
        memset(track.album,  0, sizeof(track.album));
        track.duration_ms = track.position_ms = 0;
        track.playing     = false;
        xSemaphoreGive(track_mutex);
        disconnect_chime_pending = true;
    }
}

// --- Display helpers ---
void formatTime(uint32_t ms, char *buf) {
    uint32_t s = ms / 1000;
    sprintf(buf, "%u:%02u", s / 60, s % 60);
}

// scrolling UTF8 text — clipped to 128px width
void drawScrollUTF8(const char *text, int y, uint32_t scroll_start) {
    int tw = u8g2.getUTF8Width(text);
    if (tw <= 128) {
        u8g2.drawUTF8(0, y, text);
        return;
    }
    float elapsed  = (millis() - scroll_start) / 1000.0f;
    int   period   = tw + SCROLL_GAP;
    int   offset   = (int)(elapsed * SCROLL_SPEED) % period;
    u8g2.drawUTF8(-offset,          y, text);
    u8g2.drawUTF8(-offset + period, y, text); // looping copy
}

// --- Display task ---
void display_task(void *param) {
    u8g2.begin();

    for (;;) {
        uint32_t now = millis();
        u8g2.clearBuffer();

        if (!bt_connected) {
            // Waiting screen — device name + animated dots
            u8g2.setFont(u8g2_font_unifont_t_chinese3);
            int nw = u8g2.getUTF8Width(DEVICE_NAME);
            u8g2.drawUTF8((128 - nw) / 2, 28, DEVICE_NAME);

            char dots[5] = {};
            int d = (now / 500) % 4;
            for (int i = 0; i < d; i++) dots[i] = '.';
            int dw = u8g2.getStrWidth(dots);
            u8g2.drawStr((128 - dw) / 2, 48, dots);

        } else {
            xSemaphoreTake(track_mutex, portMAX_DELAY);
            TrackInfo t = track;
            xSemaphoreGive(track_mutex);

            if (t.show_peer && now < t.peer_until) {
                // Connected — briefly show peer name
                u8g2.setFont(u8g2_font_unifont_t_chinese3);
                int lw = u8g2.getUTF8Width("Connected to:");
                u8g2.drawUTF8((128 - lw) / 2, 24, "Connected to:");
                int pw = u8g2.getUTF8Width(t.peer_name);
                u8g2.drawUTF8((128 - pw) / 2, 44, t.peer_name);
            } else {
                if (t.show_peer) {
                    xSemaphoreTake(track_mutex, portMAX_DELAY);
                    track.show_peer = false;
                    xSemaphoreGive(track_mutex);
                }

                // Main screen
                // layout: y=14 title | y=30 artist | y=46 album
                //         y=49-56 seekbar | y=63 time
                u8g2.setFont(u8g2_font_unifont_t_chinese3);
                drawScrollUTF8(t.title[0]  ? t.title  : "No Track", 14, t.scroll_reset);
                drawScrollUTF8(t.artist[0] ? t.artist : "",          30, t.scroll_reset);
                drawScrollUTF8(t.album[0]  ? t.album  : "",          46, t.scroll_reset);

                // Interpolate position
                uint32_t pos = t.position_ms;
                if (t.playing && t.position_ts > 0)
                    pos += (now - t.position_ts);
                if (t.duration_ms > 0 && pos > t.duration_ms)
                    pos = t.duration_ms;

                // Seekbar
                u8g2.drawFrame(0, 49, 128, 8);
                if (t.duration_ms > 0) {
                    int fill = (int)(128ULL * pos / t.duration_ms);
                    if (fill > 0) u8g2.drawBox(0, 49, fill, 8);
                }

                // Time
                char pos_str[8], dur_str[8];
                formatTime(pos, pos_str);
                formatTime(t.duration_ms, dur_str);
                u8g2.setFont(u8g2_font_5x7_tf);
                u8g2.drawStr(0, 63, pos_str);
                u8g2.drawStr(128 - u8g2.getStrWidth(dur_str), 63, dur_str);
            }
        }

        u8g2.sendBuffer();
        vTaskDelay(pdMS_TO_TICKS(50)); // ~20fps
    }
}

void handle_buttons() {
    if (millis() - last_btn_ms < DEBOUNCE_MS) return;
    if (!digitalRead(BTN_PLAY)) {
        last_btn_ms = millis();
        track.playing ? a2dp_sink.pause() : a2dp_sink.play();
    } else if (!digitalRead(BTN_PREV)) {
        last_btn_ms = millis();
        a2dp_sink.previous();
    } else if (!digitalRead(BTN_NEXT)) {
        last_btn_ms = millis();
        a2dp_sink.next();
    }
}

// --- Setup ---
void setup() {
    Serial.begin(115200);
    pinMode(LED_PIN, OUTPUT);
    pinMode(BTN_PLAY, INPUT_PULLUP);
    pinMode(BTN_PREV, INPUT_PULLUP);
    pinMode(BTN_NEXT, INPUT_PULLUP);
    LittleFS.begin(true);

    track_mutex = xSemaphoreCreateMutex();
    chime_queue = xQueueCreate(1, sizeof(const char *));

    xTaskCreatePinnedToCore(chime_task,   "chime",   4096, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(display_task, "display", 4096, NULL, 1, NULL, 1);

    auto cfg = i2s.defaultConfig();
    cfg.pin_bck  = 18;
    cfg.pin_ws   = 23;
    cfg.pin_data = 19;
    i2s.begin(cfg);

    play_chime_direct("/on.wav");

    a2dp_sink.set_stream_reader(read_data_stream, false);
    a2dp_sink.set_on_connection_state_changed(bt_connection_changed);
    a2dp_sink.set_avrc_metadata_attribute_mask(
        ESP_AVRC_MD_ATTR_TITLE | ESP_AVRC_MD_ATTR_ARTIST |
        ESP_AVRC_MD_ATTR_ALBUM | ESP_AVRC_MD_ATTR_PLAYING_TIME
    );
    a2dp_sink.set_avrc_metadata_callback(avrc_metadata_callback);
    a2dp_sink.set_avrc_rn_playstatus_callback(avrc_playstatus_callback);
    a2dp_sink.set_avrc_rn_play_pos_callback(avrc_position_callback, 1); // 1s interval
    a2dp_sink.set_auto_reconnect(true);
    a2dp_sink.start(DEVICE_NAME);
}

void loop() {
    handle_buttons();
    if (connect_chime_pending) {
        connect_chime_pending = false;
        enqueue_chime("/connect.wav");

        delay(500); // let AVRC settle
        const char *pname = a2dp_sink.get_peer_name();
        xSemaphoreTake(track_mutex, portMAX_DELAY);
        strncpy(track.peer_name, (pname && pname[0]) ? pname : "Unknown Device", 63);
        track.show_peer  = true;
        track.peer_until = millis() + PEER_SHOW_MS;
        xSemaphoreGive(track_mutex);
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