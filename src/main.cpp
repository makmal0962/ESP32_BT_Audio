#include "AudioTools.h"
#include "BluetoothA2DPSink.h"
#include <LittleFS.h>
#include <U8g2lib.h>
#include <Wire.h>
#include "AudioTools/AudioCodecs/CodecMP3Helix.h"
#include "arduinoFFT.h"
#include "bintobar_generated.h"
#include <Preferences.h>
#include <driver/adc.h>
#include <driver/i2s.h>

#define PIN_LED         2
#define DEVICE_NAME     "ESP32 BT Audio"
#define PEER_SHOW_MS    2000
#define SCROLL_SPEED    20   // px/sec
#define SCROLL_GAP      64   // px between loop

#define BTN_PLAY 26
#define BTN_PREV 16
#define BTN_NEXT 17
#define BTN_MODE 25
#define DEBOUNCE_MS 100

#define PIN_LINE_MODE  27
#define ADC_PIN         35
#define ADC_CHANNEL     ADC1_CHANNEL_7 // GPIO35
#define ADC_I2S_PORT I2S_NUM_0 

#define FFT_SAMPLES   1024
#define AUDIO_BUF_LEN FFT_SAMPLES
#define WAVE_ZOOM 0.5 // <1.0 = zoom in (more detail), >1.0 = zoom out

const int NYQUIST_BINS = FFT_SAMPLES / 2;
// int binToBar[NYQUIST_BINS];          // bar index for each FFT bin (1..NYQUIST_BINS-1)
const float SAMPLE_RATE = 44100.0;
const int NUM_BARS = 16;

float fft_real[FFT_SAMPLES];
float fft_imag[FFT_SAMPLES];
int16_t audio_ring[AUDIO_BUF_LEN];

struct TrackInfo {
    char     title[128];
    char     artist[128];
    char     album[128];
    char     peer_name[64];
    uint32_t duration_ms;
    uint32_t position_ms;
    uint32_t position_ts;
    bool     playing;
    bool     show_peer;
    uint32_t peer_until;
    uint32_t scroll_reset;
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

int barHeight[NUM_BARS]     = {0}; // smoothed bar heights
int barPeakHold[NUM_BARS]   = {0}; // peak hold per bar
uint32_t barPeakMs[NUM_BARS]  = {0};    // timestamp of peak hold

ArduinoFFT<float> FFT(fft_real, fft_imag, FFT_SAMPLES, 44100.0);

QueueHandle_t     chime_queue;
SemaphoreHandle_t track_mutex;
SemaphoreHandle_t audio_mutex;

uint32_t audio_ring_pos  = 0;
int32_t adc_dc_avg      = 0;
char bottom_text[64] = "";

volatile bool bt_connected              = false;
volatile bool chime_blocking            = false;
volatile bool connect_event             = false;
volatile bool disconnect_event          = false;
volatile bool bt_ready                  = false;
volatile bool bt_shutdown_pending       = false;
volatile bool start_adc_pending         = false;
volatile bool stop_adc_pending          = false;
volatile bool adc_stopped               = true;
volatile bool auto_reconnect_enabled    = true;
volatile bool line_audio_active         = false;
volatile bool line_audio_muted          = false;
bool          mode_switching            = true;

enum InputMode { MODE_BT, MODE_LINE };
volatile InputMode input_mode = MODE_BT;

enum ScreenMode { SCREEN_MAIN, SCREEN_FFT, SCREEN_WAVE };
volatile ScreenMode screen_mode = SCREEN_MAIN;

Preferences prefs;
I2SStream i2s;
BluetoothA2DPSink a2dp_sink(i2s);
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

uint16_t adc_buf[256];
int16_t  lp_prev = 0;
int reconnect_count      = 0;
esp_bd_addr_t saved_peer = {};
bool peer_loaded         = false;
int blink_count          = 0;

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
    MP3DecoderHelix *dec = new MP3DecoderHelix();
    if (!dec) { f.close(); return; }
    EncodedAudioStream decoder(&i2s, dec);
    StreamCopy copier(decoder, f);
    decoder.begin();
    while (f.available()) copier.copy();
    decoder.end();
    f.close();
    delete dec;
}

void chime_task(void *param) {
    const char *path;
    for (;;) {
        if (xQueueReceive(chime_queue, &path, portMAX_DELAY)) {
            Serial.printf("Chime: %s, MaxAlloc: %u\n", path, ESP.getMaxAllocHeap());
            if (ESP.getMaxAllocHeap() < 18000) Serial.println("[W] Chime: Heap too low, waiting");
            uint32_t timeout = millis() + 5000;
            while (ESP.getMaxAllocHeap() < 18000 && millis() < timeout) vTaskDelay(100);
            if (ESP.getMaxAllocHeap() >= 18000) {
                chime_blocking = true;
                play_chime_mp3(path);
            }
            else {
                Serial.println("[E] Chime: Skipped, heap too low/too much fragmentation!");
            }
            chime_blocking = false;
        }
        vTaskDelay(10);
    }
}

void enqueue_chime(const char *path) {
    xQueueOverwrite(chime_queue, &path);
}

// --- A2DP stream ---
void read_data_stream(const uint8_t *data, uint32_t len) {
    if (chime_blocking) return;
    i2s.write(data, len);
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

void bt_save_peer (const esp_bd_addr_t addr) {
    prefs.begin("bt", false);
    prefs.putBytes("peer", addr, sizeof(esp_bd_addr_t));
    prefs.end();
}

bool bt_load_peer (esp_bd_addr_t out_addr) {
    prefs.begin("bt", true);
    size_t len = prefs.getBytes("peer", out_addr, sizeof(esp_bd_addr_t));
    prefs.end();
    return len == sizeof(esp_bd_addr_t);
}

void bt_connection_changed(esp_a2d_connection_state_t state, void *ptr) {
    bool was_connected = bt_connected;
    bt_connected = (state == ESP_A2D_CONNECTION_STATE_CONNECTED);
    if (bt_connected) {
        digitalWrite(PIN_LED, HIGH);
        if (xSemaphoreTake(track_mutex, pdMS_TO_TICKS(100))) {
            track.show_peer  = true;
            track.peer_until = millis() + PEER_SHOW_MS + 2000;
            xSemaphoreGive(track_mutex);
        }
        connect_event = true;
    } else if (was_connected) {
        if (xSemaphoreTake(track_mutex, pdMS_TO_TICKS(100))) {
            memset(track.title,     0, sizeof(track.title));
            memset(track.artist,    0, sizeof(track.artist));
            memset(track.album,     0, sizeof(track.album));
            memset(track.peer_name, 0, sizeof(track.peer_name));
            track.duration_ms = track.position_ms = 0;
            track.playing     = false;
            xSemaphoreGive(track_mutex);
        }
        disconnect_event = true;
    }
}

void core_0_loop(void *param) {
    for (;;) {
        // --- BT shutdown ---
        if (bt_shutdown_pending) {
            auto_reconnect_enabled = true;
            peer_loaded     = false;
            reconnect_count = 0;
            blink_count     = 0;
            bt_shutdown_pending = false;
            // if (bt_connected) enqueue_chime("/disconnect.mp3");
            Serial.printf("[SHUTDOWN] before end | Heap: %u\n", ESP.getFreeHeap());
            a2dp_sink.end(false);
            bt_ready = false;
            Serial.printf("[SHUTDOWN] after end | Heap: %u\n", ESP.getFreeHeap());
        }

        // --- Start ADC ---
        if (start_adc_pending) {
            start_adc_pending = false;

            // start line ADC
            if (chime_blocking) Serial.println("Start ADC blocked by chime busy. waiting");
            while(chime_blocking) vTaskDelay(10);
            Serial.println("Stopping I2S");
            i2s.end();

            gpio_reset_pin((gpio_num_t)18);
            gpio_reset_pin((gpio_num_t)23);
            gpio_reset_pin((gpio_num_t)19);

            Serial.println("Starting ADC");
            i2s_config_t i2s_cfg = {
                .mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN),
                .sample_rate          = 44100,
                .bits_per_sample      = I2S_BITS_PER_SAMPLE_16BIT,
                .channel_format       = I2S_CHANNEL_FMT_ONLY_LEFT,
                .communication_format = I2S_COMM_FORMAT_STAND_I2S,
                .intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1,
                .dma_buf_count        = 4,
                .dma_buf_len          = 256,
                .use_apll             = false,
            };
            i2s_driver_install(ADC_I2S_PORT, &i2s_cfg, 0, NULL);
            i2s_set_adc_mode(ADC_UNIT_1, ADC_CHANNEL);
            adc1_config_width(ADC_WIDTH_BIT_12);
            adc1_config_channel_atten(ADC_CHANNEL, ADC_ATTEN_DB_12);
            i2s_adc_enable(ADC_I2S_PORT);

            xSemaphoreTake(audio_mutex, portMAX_DELAY);
            memset(audio_ring, 0, sizeof(audio_ring));
            audio_ring_pos = 0;
            adc_dc_avg = 0;
            xSemaphoreGive(audio_mutex);
            line_audio_active = true;
        }

        // --- Line ADC read ---
        if (line_audio_active) {
            if (stop_adc_pending) {
                stop_adc_pending = false;
                line_audio_active = false;
                i2s_adc_disable(ADC_I2S_PORT);
                i2s_driver_uninstall(ADC_I2S_PORT);
                xSemaphoreTake(audio_mutex, portMAX_DELAY);
                memset(audio_ring, 0, sizeof(audio_ring));
                audio_ring_pos = 0;
                xSemaphoreGive(audio_mutex);
                adc_stopped = true; // signal loop() it's safe to proceed
                continue;
            }
            // if (!line_audio_muted) {
                size_t bytes_read = 0;
                i2s_read(ADC_I2S_PORT, adc_buf, sizeof(adc_buf), &bytes_read, pdMS_TO_TICKS(10));
                if (xSemaphoreTake(audio_mutex, 0)) {
                    for (int i = 0; i < (int)(bytes_read / 2); i++) {
                        int16_t raw = ((int16_t)(adc_buf[i] & 0x0FFF) - 2048) << 4;
                        adc_dc_avg += (raw - adc_dc_avg) >> 7;
                        int16_t dc_removed = raw - (int16_t)adc_dc_avg;

                        // low-pass anti-alias
                        int16_t filtered = (int16_t)(0.4f * dc_removed + 0.6f * lp_prev);
                        lp_prev = filtered;

                        int32_t val = (int32_t)filtered * 3 / 2; // gain boost
                        val = val > INT16_MAX ? INT16_MAX : (val < INT16_MIN ? INT16_MIN : val);
                        audio_ring[audio_ring_pos % AUDIO_BUF_LEN] = (int16_t)val;
                        audio_ring_pos++;
                    }
                    xSemaphoreGive(audio_mutex);
                }
            // }
            continue; // skip BT event handling while in line mode
        }

        // --- Disconnect event ---
        if (disconnect_event) {
            disconnect_event = false;
            peer_loaded     = false;
            auto_reconnect_enabled = false;
            reconnect_count = 0;
            blink_count     = 0;
            enqueue_chime("/disconnect.mp3");
        }

        // --- LED and auto reconnect ---
        if (bt_ready && !bt_connected) {
            // load peer once
            if (!peer_loaded) {
                peer_loaded = bt_load_peer(saved_peer);
                reconnect_count = 0;
                blink_count = 0;
            }
            // attempt reconnect once per 3 blink cycles (every ~3s)
            if (peer_loaded && auto_reconnect_enabled && reconnect_count < 5 && !bt_connected) {
                if (blink_count % 4 == 0) {
                    Serial.printf("[RECONNECT] Attempt: %d\n", ++reconnect_count);
                    a2dp_sink.connect_to(saved_peer);
                }
                blink_count++;
            }

            digitalWrite(PIN_LED, HIGH);
            vTaskDelay(pdMS_TO_TICKS(500));
            if (!bt_connected) digitalWrite(PIN_LED, LOW);
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        // --- BT ready wait ---
        if (!bt_ready) { vTaskDelay(10); continue; }

        // --- Connect event ---
        if (connect_event) {
            connect_event = false;
            enqueue_chime("/connect.mp3");
            vTaskDelay(pdMS_TO_TICKS(500));
            const char *pname = a2dp_sink.get_peer_name();
            xSemaphoreTake(track_mutex, portMAX_DELAY);
            strncpy(track.peer_name, (pname && pname[0]) ? pname : "Unknown Device", 63);
            track.peer_until = millis() + PEER_SHOW_MS;
            xSemaphoreGive(track_mutex);
            esp_bd_addr_t *addr = a2dp_sink.get_current_peer_address();
            if (addr) bt_save_peer(*addr);
        }

        vTaskDelay(2);
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

void draw_fft() {
    xSemaphoreTake(audio_mutex, portMAX_DELAY);
    uint32_t start = audio_ring_pos;
    for (int i = 0; i < FFT_SAMPLES; i++) {
        fft_real[i] = (float)audio_ring[(start + i) % AUDIO_BUF_LEN];
        fft_imag[i] = 0;
    }
    xSemaphoreGive(audio_mutex);

    FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
    FFT.compute(FFTDirection::Forward);
    FFT.complexToMagnitude();

    int   barBinCount[NUM_BARS] = {};
    float barMag[NUM_BARS]      = {};

    for (int k = 1; k < NYQUIST_BINS; k++) {
        int b = (int8_t)pgm_read_byte(&binToBar[k - 1]);
        barMag[b]      += fft_real[k];
        barBinCount[b] += 1;
    }

    const float DB_FLOOR   = (input_mode == MODE_LINE) ? -55.0f : -60.0f;
    const float FFT_REF    = 32768.0f * FFT_SAMPLES / 4.0f;
    const float NOISE_GATE = (input_mode == MODE_LINE) ? 200.0f : 0.0f;

    for (int b = 0; b < NUM_BARS; b++) {
        if (barBinCount[b] > 0) {
            barMag[b] /= barBinCount[b];
            if (barMag[b] < NOISE_GATE) barMag[b] = 0;
            barMag[b] = barMag[b] > 0 ? 20.0f * log10f(barMag[b] / FFT_REF) : DB_FLOOR;
            barMag[b] = barMag[b] < DB_FLOOR ? DB_FLOOR : barMag[b];
        } else {
            barMag[b] = DB_FLOOR;
        }
    }

    // --- debug log ---
    // static uint32_t last_log = 0;
    // if (input_mode == MODE_LINE && millis() - last_log > 1000) {
    //     last_log = millis();
    //     for (int b = 0; b < NUM_BARS; b++)
    //         Serial.printf("bar[%02d] mag=%6.1f bins=%d\n", b, barMag[b], barBinCount[b]);
    //     Serial.println("---");
    // }
    // if (input_mode == MODE_LINE && millis() - last_log > 100) {
    //     last_log = millis();
    //     Serial.println(barMag[0]);
    // }

    // gate bar 0 because too noisy (hardware limitation)
    barMag[0] = (input_mode == MODE_LINE && barMag[0] < -46.0f) ? DB_FLOOR : barMag[0]; 


    const int      BAR_AREA         = 54;
    const int      BAR_WIDTH        = 128 / NUM_BARS;
    const int      START_X          = (128 - NUM_BARS * BAR_WIDTH) / 2;
    const int      BAR_DECAY        = 2;
    const int      PEAK_BAR_DECAY   = 1;
    const uint32_t PEAK_BAR_HOLD_MS = 750;
    uint32_t now_ms = millis();

    for (int b = 0; b < NUM_BARS; b++) {
        float normalized = (barMag[b] - DB_FLOOR) / (0.0f - DB_FLOOR);
        normalized = normalized < 0.0f ? 0.0f : (normalized > 1.0f ? 1.0f : normalized);
        int target = (int)(normalized * BAR_AREA);

        if (target >= barHeight[b]) barHeight[b] = target;
        else                        barHeight[b] -= BAR_DECAY;
        if (barHeight[b] < 0) barHeight[b] = 0;

        if (target >= barPeakHold[b]) {
            barPeakHold[b] = target;
            barPeakMs[b]   = now_ms;
        } else if (now_ms - barPeakMs[b] > PEAK_BAR_HOLD_MS) {
            barPeakHold[b] -= PEAK_BAR_DECAY;
            if (barPeakHold[b] < 0) barPeakHold[b] = 0;
        }

        int h    = constrain(barHeight[b],   0, BAR_AREA);
        int peak = constrain(barPeakHold[b], 0, BAR_AREA);
        if (h > 0)    u8g2.drawBox(START_X + b * BAR_WIDTH, 63 - h, BAR_WIDTH - 1, h);
        if (peak > h) u8g2.drawHLine(START_X + b * BAR_WIDTH, 63 - peak, BAR_WIDTH - 1);
    }
}

// --- Waveform Display ---
void draw_waveform() {
    float snapshot[128];
    xSemaphoreTake(audio_mutex, portMAX_DELAY);
    uint32_t start = audio_ring_pos;
    int step = (int)((AUDIO_BUF_LEN / 128) * WAVE_ZOOM);
    if (step < 1) step = 1;
    for (int i = 0; i < 128; i++)
        snapshot[i] = audio_ring[(start + i * step) % AUDIO_BUF_LEN];
    xSemaphoreGive(audio_mutex);

    const int TOP    = 9;
    const int BOTTOM = 63;
    const int MID    = (TOP + BOTTOM) / 2;
    const int HALF   = (BOTTOM - TOP) / 2;
    const float REF  = 32768.0f; // fixed full-scale reference

    for (int x = 0; x < 128; x++) {
        int y = MID - (int)(snapshot[x] / REF * HALF);
        y = constrain(y, TOP, BOTTOM);
        u8g2.drawPixel(x, y);
    }
}

// --- Display task ---
void display_task(void *param) {
    Serial.printf("Before u8g2: MaxAlloc: %u\n", ESP.getMaxAllocHeap());
    u8g2.setBusClock(800000);
    u8g2.begin();

    for (;;) {
        uint32_t now = millis();

        switch (screen_mode) {
            case SCREEN_MAIN: {
                if (!bt_connected || input_mode == MODE_LINE) {
                    // animation screen
                    if (input_mode == MODE_BT || digitalRead(PIN_LINE_MODE)) gif_next_frame();
                    uint8_t *buf = u8g2.getBufferPtr();
                    memcpy(buf, gif.frame_buf, 768);   // pages 0-5: gif (128x48)
                    memset(buf + 768, 0, 256);         // pages 6-7: clear for text row

                    // device name overlay at bottom
                    u8g2.setFont(u8g2_font_tallpixelextended_tr);
                    u8g2.drawUTF8((128 - u8g2.getUTF8Width(bottom_text)) / 2, 60, bottom_text);
                } else {
                    // bluetooth main screen
                    u8g2.clearBuffer();
                    xSemaphoreTake(track_mutex, portMAX_DELAY);
                    TrackInfo t = track;
                    xSemaphoreGive(track_mutex);

                    if (t.show_peer && now < t.peer_until) {
                        // Connected — briefly show peer name
                        u8g2.setFont(u8g2_font_tallpixelextended_tr);
                        int lw = u8g2.getUTF8Width("Connected to:");
                        u8g2.drawUTF8((128 - lw) / 2, 24, "Connected to:");
                        int pw = u8g2.getUTF8Width(t.peer_name);
                        u8g2.drawUTF8((128 - pw) / 2, 40, t.peer_name);
                    } else {
                        if (t.show_peer) {
                            xSemaphoreTake(track_mutex, portMAX_DELAY);
                            track.show_peer = false;
                            xSemaphoreGive(track_mutex);
                        }

                        // Main screen
                        // layout: y=14 title | y=30 artist | y=46 album
                        //         y=49-56 seekbar | y=63 time
                        u8g2.setFont(u8g2_font_unifont_t_japanese3);
                        drawScrollUTF8(t.title[0]  ? t.title  : "No Information", 14, t.scroll_reset, u8g2.getUTF8Width(t.title[0]  ? t.title  : "No Information"));
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

                        // peer name
                        u8g2.setFont(u8g2_font_squeezed_r6_tr);
                        u8g2.drawStr((128 - u8g2.getUTF8Width(t.peer_name)) / 2, 63, t.peer_name);
                    }
                }
                break;
            }
            case SCREEN_FFT: {
                u8g2.clearBuffer();
                u8g2.setFont(u8g2_font_5x7_tf);
                u8g2.drawStr(0, 7, "FFT");
                draw_fft();
                break;
            }
            case SCREEN_WAVE: {
                u8g2.clearBuffer();
                u8g2.setFont(u8g2_font_5x7_tf);
                u8g2.drawStr(0, 7, "WAVE");
                draw_waveform();
                break;
            }
            default:
                break;
        } 

        u8g2.sendBuffer();
        // static uint32_t fps_last = 0;
        // static uint32_t fps_count = 0;
        // fps_count++;
        // if (millis() - fps_last >= 1000) {
        //     Serial.printf("FPS: %u\n", fps_count);
        //     fps_count = 0;
        //     fps_last = millis();
        // }
        // static uint32_t heap_last = 0;
        // if (millis() - heap_last >= 1000) {
        //     Serial.printf("[loop] Heap: %u\n", ESP.getFreeHeap());
        //     Serial.printf("[loop] Max alloc Heap: %u\n", ESP.getMaxAllocHeap());
        //     heap_last = millis();
        // };

        vTaskDelay(1);
    }
}


// --- Save Helper ---
void save_state() {
    prefs.begin("settings", false);
    InputMode save_input_mode = input_mode == MODE_BT ? MODE_LINE : MODE_BT;
    prefs.putInt("input_mode", (int)save_input_mode);
    prefs.end();
}

// --- Button handling ---
void handle_buttons() {
    static uint32_t play_press_ms  = 0;
    static uint32_t mode_press_ms  = 0;
    static uint32_t prev_press_ms  = 0;
    static uint32_t next_press_ms  = 0;
    static bool     play_held      = false;
    static bool     mode_held      = false;
    static bool     prev_held      = false;
    static bool     next_held      = false;
    static bool play_long_fired = false;
    static bool mode_long_fired = false;
    static bool prev_long_fired = false;
    static bool next_long_fired = false;
    uint32_t now = millis();

    bool play_btn = !digitalRead(BTN_PLAY);
    if (play_btn && !play_held) {
        play_press_ms   = now;
        play_held       = true;
        play_long_fired = false;
    } else if (play_btn && play_held) {
        if (!play_long_fired && now - play_press_ms >= 3000) {
            play_long_fired = true;
            if (input_mode == MODE_BT){
                screen_mode = SCREEN_MAIN;
                strcpy(bottom_text, "Disconnecting . . .");
                a2dp_sink.disconnect();
                strcpy(bottom_text, "Waiting for Connection...");
            }
        }
    } else if (!play_btn && play_held) {
        play_held = false;
        if (!play_long_fired && now - play_press_ms >= DEBOUNCE_MS)  {
            if (input_mode == MODE_BT) {
                xSemaphoreTake(track_mutex, portMAX_DELAY);
                bool is_playing = track.playing;
                xSemaphoreGive(track_mutex);
                is_playing ? a2dp_sink.pause() : a2dp_sink.play();
            }
            else if (input_mode == MODE_LINE) {
                bool val = digitalRead(PIN_LINE_MODE);
                digitalWrite(PIN_LINE_MODE, !val);
                strcpy(bottom_text, val ? "Muted" : "Line Input Mode");
                line_audio_muted = val;
            }
        }
        play_long_fired = false;
    }

    bool mode_btn = !digitalRead(BTN_MODE);
    if (mode_btn && !mode_held) {
        mode_press_ms   = now;
        mode_held       = true;
        mode_long_fired = false;
    } else if (mode_btn && mode_held) {
        if (!mode_long_fired && now - mode_press_ms >= 1000) {
            mode_long_fired = true;
            strcpy(bottom_text, "Loading . . .");
            screen_mode = SCREEN_MAIN;
            digitalWrite(PIN_LED, LOW);
            save_state();
            if (input_mode == MODE_BT) {
                enqueue_chime("/disconnect.mp3");
                bt_shutdown_pending = true;
                while (bt_ready) vTaskDelay(10);
                input_mode = MODE_LINE;
            } else if (input_mode == MODE_LINE) {
                input_mode = MODE_BT;
            }
            mode_switching = true;
        }
    } else if (!mode_btn && mode_held) {
        mode_held = false;
        if (!mode_long_fired && now - mode_press_ms >= DEBOUNCE_MS)
            screen_mode = static_cast<ScreenMode>((screen_mode + 1) % 3);
        mode_long_fired = false;
    }

    bool prev_btn = !digitalRead(BTN_PREV);
    if (prev_btn && !prev_held) {
        prev_press_ms   = now;
        prev_held       = true;
        prev_long_fired = false;
    } else if (prev_btn && prev_held) {
        if (!prev_long_fired && now - prev_press_ms >= 1000) {
            prev_long_fired = true;
            // reserved for future use
        }
    } else if (!prev_btn && prev_held) {
        prev_held = false;
        if (!prev_long_fired && now - prev_press_ms >= DEBOUNCE_MS)
            if (input_mode == MODE_BT) a2dp_sink.previous();
        prev_long_fired = false;
    }

    bool next_btn = !digitalRead(BTN_NEXT);
    if (next_btn && !next_held) {
        next_press_ms   = now;
        next_held       = true;
        next_long_fired = false;
    } else if (next_btn && next_held) {
        if (!next_long_fired && now - next_press_ms >= 1000) {
            next_long_fired = true;
            // reserved for future use
        }
    } else if (!next_btn && next_held) {
        next_held = false;
        if (!next_long_fired && now - next_press_ms >= DEBOUNCE_MS)
            if (input_mode == MODE_BT) a2dp_sink.next();
        next_long_fired = false;
    }
}

void handle_mode_switching() {
    if (input_mode == MODE_BT && mode_switching) {
        auto_reconnect_enabled = true;
        stop_adc_pending = true;
        Serial.println("Starting BT Mode");
        while (!adc_stopped) vTaskDelay(10); // wait for core 0 to cleanly stop ADC
        adc_stopped = false;
        digitalWrite(PIN_LINE_MODE, LOW);

        gif_open("/connecting.raw", 11, 8);
        Serial.printf("[BT] before i2s.begin | Heap: %u MaxAlloc: %u\n", ESP.getFreeHeap(), ESP.getMaxAllocHeap());
        vTaskDelay(pdMS_TO_TICKS(50));
        auto cfg = i2s.defaultConfig();
        cfg.pin_bck  = 18;
        cfg.pin_ws   = 23;
        cfg.pin_data = 19;
        i2s.begin(cfg);
        
        Serial.printf("[BT] before a2dp start | Heap: %u MaxAlloc: %u\n", ESP.getFreeHeap(), ESP.getMaxAllocHeap());
        a2dp_sink.set_task_priority(configMAX_PRIORITIES - 5);
        a2dp_sink.set_task_core(0);
        a2dp_sink.set_stream_reader(read_data_stream, false);
        a2dp_sink.set_on_connection_state_changed(bt_connection_changed);
        a2dp_sink.set_avrc_metadata_attribute_mask(
            ESP_AVRC_MD_ATTR_TITLE | ESP_AVRC_MD_ATTR_ARTIST |
            ESP_AVRC_MD_ATTR_ALBUM | ESP_AVRC_MD_ATTR_PLAYING_TIME
        );
        a2dp_sink.set_avrc_metadata_callback(avrc_metadata_callback);
        a2dp_sink.set_avrc_rn_playstatus_callback(avrc_playstatus_callback);
        a2dp_sink.set_avrc_rn_play_pos_callback(avrc_position_callback, 1); // 1s interval
        // a2dp_sink.set_auto_reconnect(true);
        a2dp_sink.start(DEVICE_NAME);
        enqueue_chime("/on.mp3");
        strcpy(bottom_text, "Waiting for Connection...");
        Serial.printf("[BT] after a2dp start | Heap: %u MaxAlloc: %u\n", ESP.getFreeHeap(), ESP.getMaxAllocHeap());
        vTaskDelay(pdMS_TO_TICKS(1000));
        bt_ready = true;
        Serial.printf("[BT] after bt_ready | Heap: %u MaxAlloc: %u\n", ESP.getFreeHeap(), ESP.getMaxAllocHeap());
        // dump_heap();
        // dump_heap_blocks();
    }

    else if (input_mode == MODE_LINE && mode_switching) {
        start_adc_pending = true;
        stop_adc_pending = false;
        line_audio_muted = false;
        Serial.println("Starting Line Mode");
        gif_open("/line_in.raw", 2, 2);
        digitalWrite(PIN_LINE_MODE, HIGH);
        strcpy(bottom_text, "Line Input Mode");
    }
    mode_switching = false;
}

void dump_heap_blocks() {
    multi_heap_info_t info;
    heap_caps_get_info(&info, MALLOC_CAP_8BIT);
    Serial.printf("Total free: %u\n",      info.total_free_bytes);
    Serial.printf("Total allocated: %u\n", info.total_allocated_bytes);
    Serial.printf("Largest block: %u\n",   info.largest_free_block);
    Serial.printf("Min ever free: %u\n",   info.minimum_free_bytes);
    Serial.printf("Alloc blocks: %u\n",    info.allocated_blocks);
    Serial.printf("Free blocks: %u\n",     info.free_blocks);
}

void dump_heap() {
    Serial.printf("=== HEAP DUMP ===\n");
    Serial.printf("Free: %u  MaxAlloc: %u\n", ESP.getFreeHeap(), ESP.getMaxAllocHeap());
    
    // print all free blocks
    heap_caps_print_heap_info(MALLOC_CAP_8BIT);
    
    Serial.printf("=================\n");
}

// --- Setup ---
void setup() {
    Serial.begin(115200);
    esp_bt_controller_mem_release(ESP_BT_MODE_BLE);
    Serial.printf("Starting ESP32. MaxAlloc: %u\n", ESP.getMaxAllocHeap());
    // dump_heap();
    // dump_heap_blocks();
    pinMode(PIN_LED, OUTPUT);
    pinMode(PIN_LINE_MODE, OUTPUT);
    pinMode(BTN_PLAY, INPUT_PULLUP);
    pinMode(BTN_PREV, INPUT_PULLUP);
    pinMode(BTN_NEXT, INPUT_PULLUP);
    pinMode(BTN_MODE, INPUT_PULLUP);
    LittleFS.begin(true);

    track_mutex = xSemaphoreCreateMutex();
    audio_mutex = xSemaphoreCreateMutex();
    chime_queue = xQueueCreate(1, sizeof(const char *));

    xTaskCreatePinnedToCore(chime_task,   "chime",   3072, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(display_task, "display", 10240, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(core_0_loop, "core_0_loop", 4096, NULL, 2, NULL, 0);

    gif_open("/connecting.raw", 11, 8);
    strcpy(bottom_text, "Loading . . .");
    // setupFrequencyMapping();


    prefs.begin("settings", true);
    input_mode  = (InputMode)prefs.getInt("input_mode",  MODE_BT);
    prefs.end();
}

void loop() {
    handle_buttons();
    handle_mode_switching();
    vTaskDelay(10);
}