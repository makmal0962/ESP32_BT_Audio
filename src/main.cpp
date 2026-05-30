#include "AudioTools.h"
#include "BluetoothA2DPSink.h"
#include <LittleFS.h>
#include <U8g2lib.h>
#include <Wire.h>
#include "AudioTools/AudioCodecs/CodecMP3Helix.h"
#include "arduinoFFT.h"

#define LED_PIN         2
#define DEVICE_NAME     "ESP32 BT Audio"
#define PEER_SHOW_MS    2000
#define SCROLL_SPEED    19   // px/sec
#define SCROLL_GAP      64   // px between loop

#define BTN_PLAY 26
#define BTN_PREV 16
#define BTN_NEXT 17
#define BTN_MODE 25
#define DEBOUNCE_MS 100

I2SStream i2s;
BluetoothA2DPSink a2dp_sink(i2s);
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

volatile bool bt_connected             = false;
volatile bool connect_chime_pending    = false;
volatile bool disconnect_chime_pending = false;
volatile bool chime_blocking           = false;
volatile bool bt_ready = false;

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

struct GifAnim {
    File     file;
    uint8_t  frame_buf[768]; // 128*48/8
    uint32_t frame_count;
    uint32_t frame_ms;        // delay per frame
    uint32_t last_frame_ts;
    uint32_t current_frame;
};
GifAnim gif = {};

#define FFT_SAMPLES   1024
#define AUDIO_BUF_LEN FFT_SAMPLES * 2

enum ScreenMode { SCREEN_MAIN, SCREEN_FFT, SCREEN_WAVE };
volatile ScreenMode screen_mode = SCREEN_MAIN;

int16_t  audio_ring[AUDIO_BUF_LEN];
uint32_t audio_ring_pos = 0;
SemaphoreHandle_t audio_mutex;

float fft_real[FFT_SAMPLES];
float fft_imag[FFT_SAMPLES];
ArduinoFFT<float> FFT(fft_real, fft_imag, FFT_SAMPLES, 44100.0);

const int NYQUIST_BINS = FFT_SAMPLES / 2;
int binToBar[NYQUIST_BINS];          // bar index for each FFT bin (1..NYQUIST_BINS-1)
const float SAMPLE_RATE = 44100.0;
const int NUM_BARS = 16;

float fft_peak = 1.0;
int barHeight[NUM_BARS]     = {0}; // smoothed bar heights
int barPeakHold[NUM_BARS]   = {0}; // peak hold per bar
uint32_t barPeakMs[NUM_BARS]  = {0};    // timestamp of peak hold
float waveform_peak = 1.0;

volatile bool audio_started = false;

// --- GIF animation ---
void gif_open(const char *path, uint32_t frame_count, uint32_t fps) {
    if (gif.file) gif.file.close();
    gif.file          = LittleFS.open(path, "r");
    gif.frame_count   = frame_count;
    gif.frame_ms      = 1000 / fps;
    gif.last_frame_ts = 0;
    gif.current_frame = 0;
}

bool gif_next_frame() {
    if (!gif.file) return false;
    uint32_t now = millis();
    if (now - gif.last_frame_ts < gif.frame_ms) return false;
    gif.last_frame_ts = now;
    size_t n = gif.file.read(gif.frame_buf, 768); // 128*48/8
    if (n < 768) {
        gif.file.seek(0);
        gif.file.read(gif.frame_buf, 768);
    }
    return true;
}


// --- Chime ---
void play_chime_mp3(const char *path) {
    File f = LittleFS.open(path, "r");
    if (!f) return;
    MP3DecoderHelix mp3decoder;
    EncodedAudioStream decoder(&i2s, &mp3decoder);
    StreamCopy copier(decoder, f);
    decoder.begin();
    while (f.available()) copier.copy();
    decoder.end();
    f.close();
}

void chime_task(void *param) {
    const char *path;
    for (;;) {
        if (xQueueReceive(chime_queue, &path, portMAX_DELAY)) {
            while (!bt_ready) vTaskDelay(10);
            chime_blocking = true;
            // delay(300);
            play_chime_mp3(path);
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
    audio_started = true;
    // mix stereo to mono into ring buffer
    int16_t *samples = (int16_t *)data;
    int count = len / 4; // stereo 16-bit = 4 bytes/frame
    if (xSemaphoreTake(audio_mutex, 0)) { // non-blocking, skip if busy
        for (int i = 0; i < count; i++) {
            audio_ring[audio_ring_pos % AUDIO_BUF_LEN] =
                (samples[i*2] / 2) + (samples[i*2+1] / 2); // L+R mix
            audio_ring_pos++;
        }
        xSemaphoreGive(audio_mutex);
    }
}

// --- AVRCP callbacks ---
void avrc_metadata_callback(uint8_t id, const uint8_t *text) {
    xSemaphoreTake(track_mutex, portMAX_DELAY);
    switch (id) {
        case ESP_AVRC_MD_ATTR_TITLE:
            strncpy(track.title, (char *)text, 127);
            strncpy(track.artist, "", 127);
            strncpy(track.album,  "", 127);
            track.scroll_reset = millis();
            break;
        case ESP_AVRC_MD_ATTR_ARTIST:
            strncpy(track.artist, (char *)text, 127);
            break;
        case ESP_AVRC_MD_ATTR_ALBUM:
            strncpy(track.album, (char *)text, 127);
            break;
        case ESP_AVRC_MD_ATTR_PLAYING_TIME:
            track.duration_ms = atoi((char *)text);
            break;
    }
    xSemaphoreGive(track_mutex);
}

void avrc_playstatus_callback(esp_avrc_playback_stat_t playback) {
    Serial.printf("Playback state: %d\n", playback);
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
void drawScrollUTF8(const char *text, int y, uint32_t scroll_start, int tw) {
    if (tw <= 128) {
        u8g2.drawUTF8(0, y, text);
        return;
    }
    float elapsed = (millis() - scroll_start) / 1000.0f;
    int   period  = tw + SCROLL_GAP;
    int   offset  = (int)(elapsed * SCROLL_SPEED) % period;
    u8g2.drawUTF8(-offset,          y, text);
    u8g2.drawUTF8(-offset + period, y, text);
}

// --- FFT Display ---
void setupFrequencyMapping() {
    float f_min = (float)SAMPLE_RATE / FFT_SAMPLES;   // 86 Hz – lowest possible
    float f_max = SAMPLE_RATE / 2.0;                  // 22050 Hz
    float logMin = log10(f_min);
    float logMax = log10(f_max);
    
    // For each bin (k=1..NYQUIST_BINS-1), compute its frequency and bar index
    for (int k = 1; k < NYQUIST_BINS; k++) {
        float freq = (float)k * SAMPLE_RATE / FFT_SAMPLES;
        // Logarithmic mapping: bar = (log10(freq) - logMin) / (logMax - logMin) * NUM_BARS
        float t = (log10(freq) - logMin) / (logMax - logMin);
        int bar = constrain((int)(t * NUM_BARS), 0, NUM_BARS - 1);
        binToBar[k] = bar;
    }
}

void draw_fft() {
    if (!audio_started) return;

    // snapshot
    float snapshot[FFT_SAMPLES];
    xSemaphoreTake(audio_mutex, portMAX_DELAY);
    uint32_t start = audio_ring_pos;
    for (int i = 0; i < FFT_SAMPLES; i++)
        snapshot[i] = audio_ring[(start + i) % AUDIO_BUF_LEN];
    xSemaphoreGive(audio_mutex);

    for (int i = 0; i < FFT_SAMPLES; i++) {
        fft_real[i] = snapshot[i];
        fft_imag[i] = 0;
    }

    FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
    FFT.compute(FFTDirection::Forward);
    FFT.complexToMagnitude();

    int   barBinCount[NUM_BARS] = {};
    float barMag[NUM_BARS]      = {};

    for (int k = 1; k < NYQUIST_BINS; k++) {
        int b = binToBar[k];
        barMag[b]      += fft_real[k];
        barBinCount[b] += 1;
    }

    const float DB_FLOOR = -40.0f;
    const float DB_CEIL  =  0.0f;

    for (int b = 0; b < NUM_BARS; b++) {
        if (barBinCount[b] > 0) {
            barMag[b] /= barBinCount[b];
            barMag[b] = barMag[b] > 0 ? 10.0f * log10f(barMag[b]) : DB_FLOOR;
            barMag[b] = barMag[b] < DB_FLOOR ? DB_FLOOR : barMag[b];
        } else {
            barMag[b] = DB_FLOOR;
        }
    }

    // peak detection with decay
    float current_peak = DB_FLOOR;
    for (int b = 0; b < NUM_BARS; b++)
    if (barMag[b] > current_peak) current_peak = barMag[b];

    // Decay factor – adjust to taste (0.995 = fairly slow release, 0.98 = faster release)
    const float PEAK_DECAY = 0.998f;

    // Update the running peak
    // fft_peak = max(current_peak, fft_peak * PEAK_DECAY);
    fft_peak = current_peak > (fft_peak * PEAK_DECAY) ? current_peak : (fft_peak * PEAK_DECAY);
    if (fft_peak < DB_FLOOR) fft_peak = DB_FLOOR;

    uint32_t now_ms = millis();

    const int BAR_DECAY           = 3; // bar fall
    const uint32_t PEAK_BAR_HOLD_MS    = 800;
    const int PEAK_BAR_DECAY      = 1; // peak dot fall after hold

    const int BAR_AREA  = 54;
    const int BAR_WIDTH = 128 / NUM_BARS;
    const int START_X   = (128 - NUM_BARS * BAR_WIDTH) / 2;

    // for (int b = 0; b < NUM_BARS; b++) {
    //     float normalized = (barMag[b] + DB_FLOOR) / (DB_CEIL - DB_FLOOR); // 0..1
    //     int h = constrain((int)(normalized * BAR_AREA), 0, BAR_AREA);
    //     if (h > 0) u8g2.drawBox(START_X + b * BAR_WIDTH, 63 - h, BAR_WIDTH - 1, h);
    // }

    for (int b = 0; b < NUM_BARS; b++) {
        float normalized = (barMag[b] + DB_FLOOR) / (fft_peak + DB_FLOOR);
        int target = constrain((int)(normalized * BAR_AREA), 0, BAR_AREA);

        // rise instantly, decay time-based
        if (target > barHeight[b])
            barHeight[b] = target;
        else
            barHeight[b] -= BAR_DECAY;
        if (barHeight[b] < 0) barHeight[b] = 0;

        // peak hold dot
        if (target >= barPeakHold[b]) {
            barPeakHold[b] = target;
            barPeakMs[b]   = now_ms;
        } else if (now_ms - barPeakMs[b] > PEAK_BAR_HOLD_MS) {
            barPeakHold[b] -= PEAK_BAR_DECAY;
            if (barPeakHold[b] < 0) barPeakHold[b] = 0;
        }

        int h    = constrain((int)barHeight[b],   0, BAR_AREA);
        // int h = constrain((int)(normalized * BAR_AREA), 0, BAR_AREA);
        int peak = constrain((int)barPeakHold[b], 0, BAR_AREA);

        if (h > 0)    u8g2.drawBox(START_X + b * BAR_WIDTH, 63 - h, BAR_WIDTH - 1, h);
        if (peak > h) u8g2.drawHLine(START_X + b * BAR_WIDTH, 63 - peak, BAR_WIDTH - 1);
    }
}

void draw_waveform() {
    xSemaphoreTake(audio_mutex, portMAX_DELAY);
    uint32_t start = audio_ring_pos;
    int16_t buf[128];
    int step = AUDIO_BUF_LEN / 128;
    for (int i = 0; i < 128; i++)
        buf[i] = audio_ring[(start + i * step) % AUDIO_BUF_LEN];
    xSemaphoreGive(audio_mutex);

    const int TOP    = 9;  // below "WAVE" label
    const int BOTTOM = 63;
    const int MID    = (TOP + BOTTOM) / 2; // ~36
    const int HALF   = (BOTTOM - TOP) / 2; // ~27

    // find peak for auto-scale
    float current_peak = 0;
    for (int i = 0; i < 128; i++) {
        float abs_val = abs(buf[i]);
        if (abs_val > current_peak) current_peak = abs_val;
    }

    // Decay factor – adjust to taste (0.95 = faster, 0.98 = slower)
    const float PEAK_DECAY = 0.998f;
    const float MIN_PEAK = 1.0f;  // prevent division by zero / too sensitive

    // Update the running peak with decay
    waveform_peak = max(current_peak, waveform_peak * PEAK_DECAY);
    if (waveform_peak < MIN_PEAK) waveform_peak = MIN_PEAK;

    // Serial.printf("Wave peak: %.2f\n", waveform_peak);
    
    // u8g2.drawHLine(0, MID, 128); // center line
    for (int x = 0; x < 128; x++) {
        int y = MID - (buf[x] * HALF / (int)waveform_peak);
        y = constrain(y, TOP, BOTTOM);
        u8g2.drawPixel(x, y);
    }
}

// --- Display task ---
void display_task(void *param) {
    u8g2.begin();
    Wire.setClock(800000UL);

    for (;;) {
        uint32_t now = millis();

        if (!bt_connected) {
            gif_next_frame();
            uint8_t *buf = u8g2.getBufferPtr();
            memcpy(buf, gif.frame_buf, 768);   // pages 0-5: gif (128x48)
            memset(buf + 768, 0, 256);         // pages 6-7: clear for text row

            // device name overlay at bottom
            u8g2.setFont(u8g2_font_5x7_tf);
            u8g2.drawStr(0, 63, "Waiting for Connection...");

        } else if (screen_mode == SCREEN_FFT) {
            u8g2.clearBuffer();
            u8g2.setFont(u8g2_font_5x7_tf);
            u8g2.drawStr(0, 7, "FFT");
            draw_fft();
        } else if (screen_mode == SCREEN_WAVE) {
            u8g2.clearBuffer();
            u8g2.setFont(u8g2_font_5x7_tf);
            u8g2.drawStr(0, 7, "WAVE");
            draw_waveform();
        } else {
            u8g2.clearBuffer();
            xSemaphoreTake(track_mutex, portMAX_DELAY);
            TrackInfo t = track;
            xSemaphoreGive(track_mutex);

            if (t.show_peer && now < t.peer_until) {
                // Connected — briefly show peer name
                u8g2.setFont(u8g2_font_unifont_t_gb2312);
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
                u8g2.setFont(u8g2_font_unifont_t_gb2312);
                drawScrollUTF8(t.title[0]  ? t.title  : "No Track", 14, t.scroll_reset, u8g2.getUTF8Width(t.title[0]  ? t.title  : "No Track"));
                drawScrollUTF8(t.artist[0] ? t.artist : "",         30, t.scroll_reset, u8g2.getUTF8Width(t.artist[0] ? t.artist : ""));
                drawScrollUTF8(t.album[0]  ? t.album  : "",         46, t.scroll_reset, u8g2.getUTF8Width(t.album[0]  ? t.album  : ""));
                
                // Interpolate position
                uint32_t pos = t.position_ms;
                if (t.playing && t.position_ts > 0)
                    pos += (now - t.position_ts);
                if (t.duration_ms > 0 && pos > t.duration_ms)
                    pos = t.duration_ms;

                // Seekbar
                u8g2.drawFrame(0, 49, 128, 7);
                if (t.duration_ms > 0) {
                    int fill = (int)(128ULL * pos / t.duration_ms);
                    if (fill > 0) u8g2.drawBox(0, 49, fill, 7);
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
        static uint32_t fps_last = 0;
        static uint32_t fps_count = 0;
        fps_count++;
        if (millis() - fps_last >= 1000) {
            Serial.printf("FPS: %lu\n", fps_count);
            fps_count = 0;
            fps_last = millis();
        }
        vTaskDelay(1);
    }
}

void handle_buttons() {
    static uint32_t play_press_ms  = 0;
    static bool     play_held      = false;
    static bool     prev_last      = HIGH;
    static bool     next_last      = HIGH;
    static bool     mode_last      = HIGH;
    uint32_t now = millis();

    bool play_btn = !digitalRead(BTN_PLAY);
    if (play_btn && !play_held) {
        play_press_ms = now;
        play_held     = true;
    } else if (play_btn && play_held) {
        if (now - play_press_ms >= 3000) {
            play_held = false;
            a2dp_sink.disconnect();
        }
    } else if (!play_btn && play_held) {
        play_held = false;
        uint32_t held = now - play_press_ms;
        if (held >= DEBOUNCE_MS) {
            xSemaphoreTake(track_mutex, portMAX_DELAY);
            bool is_playing = track.playing;
            xSemaphoreGive(track_mutex);
            is_playing ? a2dp_sink.pause() : a2dp_sink.play();
        }
    }

    bool prev_btn = !digitalRead(BTN_PREV);
    bool next_btn = !digitalRead(BTN_NEXT);
    bool mode_btn = !digitalRead(BTN_MODE);

    if (now - last_btn_ms > DEBOUNCE_MS) {
        if (prev_btn && !prev_last) { last_btn_ms = now; a2dp_sink.previous(); }
        if (next_btn && !next_last) { last_btn_ms = now; a2dp_sink.next(); }
        if (mode_btn && !mode_last) { last_btn_ms = now; screen_mode = (ScreenMode)((screen_mode + 1) % 3); }
    }

    prev_last = prev_btn;
    next_last = next_btn;
    mode_last = mode_btn;
}

// --- Setup ---
void setup() {
    Serial.begin(115200);
    pinMode(LED_PIN, OUTPUT);
    pinMode(BTN_PLAY, INPUT_PULLUP);
    pinMode(BTN_PREV, INPUT_PULLUP);
    pinMode(BTN_NEXT, INPUT_PULLUP);
    pinMode(BTN_MODE, INPUT_PULLUP);
    LittleFS.begin(true);

    track_mutex = xSemaphoreCreateMutex();
    audio_mutex = xSemaphoreCreateMutex();
    chime_queue = xQueueCreate(1, sizeof(const char *));

    xTaskCreatePinnedToCore(chime_task,   "chime",   4096, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(display_task, "display", 8192, NULL, 2, NULL, 1);

    auto cfg = i2s.defaultConfig();
    cfg.pin_bck  = 18;
    cfg.pin_ws   = 23;
    cfg.pin_data = 19;
    i2s.begin(cfg);

    gif_open("/connecting.raw", 11, 8);
    setupFrequencyMapping();

    enqueue_chime("/on.mp3");

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
    i2s_config_t cfg_i2s = {};
    a2dp_sink.set_bits_per_sample(16);
    a2dp_sink.start(DEVICE_NAME);
    bt_ready = true;
    
}

void loop() {
    handle_buttons();
    if (connect_chime_pending) {
        connect_chime_pending = false;
        enqueue_chime("/connect.mp3");

        // show peer screen immediately before delay
        xSemaphoreTake(track_mutex, portMAX_DELAY);
        strncpy(track.peer_name, "...", 63);
        track.show_peer  = true;
        track.peer_until = millis() + PEER_SHOW_MS + 500; // extend by delay duration
        xSemaphoreGive(track_mutex);

        delay(500);

        const char *pname = a2dp_sink.get_peer_name();
        xSemaphoreTake(track_mutex, portMAX_DELAY);
        strncpy(track.peer_name, (pname && pname[0]) ? pname : "Unknown Device", 63);
        xSemaphoreGive(track_mutex);
    }

    if (!bt_connected) {
        if (disconnect_chime_pending) {
            disconnect_chime_pending = false;
            enqueue_chime("/disconnect.mp3");
        }
        digitalWrite(LED_PIN, HIGH);
        delay(500);
        if (!bt_connected) digitalWrite(LED_PIN, LOW);
        delay(500);
    }
}