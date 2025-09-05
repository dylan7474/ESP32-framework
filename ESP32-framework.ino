#include <SimpleRotary.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <Wire.h>
#include <driver/i2s.h>
#include <math.h>

#include "config.h"

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

Adafruit_SH1106G display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
SimpleRotary VolumeSelector(33, 4, 23);
SimpleRotary ChannelSelector(25, 32, 2);

volatile byte VolumeChange = 0, VolumePush = 0;
volatile byte ChannelChange = 0, ChannelPush = 0;

int beepVolume = 10;
int channel = 0;

void encoderTask(void *pvParameters) {
  for (;;) {
    byte vol_rotate_event = VolumeSelector.rotate();
    if (vol_rotate_event != 0) VolumeChange = vol_rotate_event;
    byte chan_rotate_event = ChannelSelector.rotate();
    if (chan_rotate_event != 0) ChannelChange = chan_rotate_event;
    byte vol_push_event = VolumeSelector.pushType(200);
    if (vol_push_event != 0) VolumePush = vol_push_event;
    byte chan_push_event = ChannelSelector.pushType(200);
    if (chan_push_event != 0) ChannelPush = chan_push_event;
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

void setup() {
  Serial.begin(115200);

  display.begin(0x3C, true);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0, 0);
  display.println("ESP32 Framework");
  display.display();

  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = 44100,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = 0,
    .dma_buf_count = 8,
    .dma_buf_len = 64,
    .use_apll = false,
    .tx_desc_auto_clear = true,
    .fixed_mclk = 0
  };
  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_BCLK_PIN,
    .ws_io_num = I2S_LRCLK_PIN,
    .data_out_num = I2S_DOUT_PIN,
    .data_in_num = I2S_PIN_NO_CHANGE
  };
  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);

  xTaskCreatePinnedToCore(encoderTask, "Encoder", 2048, NULL, 3, NULL, 1);
}

void loop() {
  byte localVolChange = VolumeChange;
  byte localVolPush = VolumePush;
  byte localChanChange = ChannelChange;
  byte localChanPush = ChannelPush;

  if (localVolChange != 0) VolumeChange = 0;
  if (localVolPush != 0) VolumePush = 0;
  if (localChanChange != 0) ChannelChange = 0;
  if (localChanPush != 0) ChannelPush = 0;

  if (localVolChange != 0) {
    beepVolume += (localVolChange == 1) ? 1 : -1;
    beepVolume = constrain(beepVolume, 0, 20);
  }
  if (localChanChange != 0) {
    channel += (localChanChange == 1) ? 1 : -1;
  }
  if (localVolPush == 1) {
    playBeep(1000, 100);
  }
  if (localChanPush == 1) {
    playBeep(600, 100);
  }

  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("Volume: ");
  display.println(beepVolume);
  display.print("Channel: ");
  display.println(channel);
  display.display();

  delay(50);
}

void playBeep(int freq, int duration_ms) {
  long amplitude = map(beepVolume, 0, 20, 0, 25000);
  const int sampleRate = 44100;
  int samples = sampleRate * duration_ms / 1000;
  size_t bytes_written;
  for (int i = 0; i < samples; i++) {
    int16_t sample = (int16_t)(sin(2 * PI * freq * i / sampleRate) * amplitude);
    i2s_write(I2S_NUM_0, &sample, sizeof(sample), &bytes_written, portMAX_DELAY);
  }
}
