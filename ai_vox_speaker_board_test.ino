#include <Adafruit_ST7789.h>  // Hardware-specific library for ST7789
#include <FS.h>
#include <SD.h>
#include <SPI.h>
#include <WiFi.h>
#include <driver/gpio.h>
#include <driver/i2c_master.h>
#include <driver/i2s_pdm.h>
#include <driver/i2s_std.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <chrono>
#include <string>
#include <thread>
#include <vector>

#include "button_gpio.h"
#include "clogger.h"
#include "es8311.h"
#include "esp_check.h"
#include "iot_button.h"
#include "led_strip.h"

#define EXAMPLE_SAMPLE_RATE (16000)
#define EXAMPLE_MCLK_MULTIPLE (384)  // If not using 24-bit data width, 256 should be enough
#define EXAMPLE_MCLK_FREQ_HZ (EXAMPLE_SAMPLE_RATE * EXAMPLE_MCLK_MULTIPLE)

namespace {
#include "pcm.h"

constexpr char kVersion[] = "1.0.2";
constexpr char kWifiSsid[] = "emekefun";      // Change this to your WiFi SSID
constexpr char kWifiPassword[] = "501416wf";  // Change this to your WiFi password

constexpr auto kKeyA = GPIO_NUM_46;
constexpr auto kKeyB = GPIO_NUM_45;
constexpr auto kKeyBoot = GPIO_NUM_0;

constexpr gpio_num_t kEs8311I2sMclk = GPIO_NUM_11;
constexpr gpio_num_t kEs8311I2sSclk = GPIO_NUM_10;
constexpr gpio_num_t kEs8311I2sWs = GPIO_NUM_8;
constexpr gpio_num_t kEs8311I2sDsdin = GPIO_NUM_7;
constexpr gpio_num_t kEs8311I2sAsdout = GPIO_NUM_9;

constexpr gpio_num_t kEs8311I2cScl = GPIO_NUM_12;
constexpr gpio_num_t kEs8311I2cSda = GPIO_NUM_13;

constexpr gpio_num_t kDisplayCs = GPIO_NUM_15;
// constexpr gpio_num_t kDisplayRst = GPIO_NUM_21;
constexpr gpio_num_t kDisplayDc = GPIO_NUM_14;
constexpr gpio_num_t kDisplayMosi = GPIO_NUM_21;
constexpr gpio_num_t kDisplaySclk = GPIO_NUM_17;
constexpr gpio_num_t kDisplayBlk = GPIO_NUM_16;

constexpr gpio_num_t kBatteryLevelValue = GPIO_NUM_18;

constexpr gpio_num_t kSDCardMosi = GPIO_NUM_39;
constexpr gpio_num_t kSDCardMiso = GPIO_NUM_40;
constexpr gpio_num_t kSDCardClk = GPIO_NUM_38;

i2c_master_bus_handle_t g_i2c_master_bus = nullptr;

i2s_chan_handle_t g_es8311_tx_handle = 0;
i2s_chan_handle_t g_es8311_rx_handle = 0;
i2s_chan_handle_t g_pdm_rx_handle = 0;

button_handle_t g_button_a_handle = nullptr;
button_handle_t g_button_b_handle = nullptr;
button_handle_t g_button_boot_handle = nullptr;
volatile uint32_t g_battery_level = 0;
volatile bool g_echo_enable = false;

SPIClass g_fspi(FSPI);
SPIClass g_hspi(HSPI);
Adafruit_ST7789 g_display(&g_fspi, kDisplayCs, kDisplayDc, -1);
sdcard_type_t g_sdcard_type = CARD_NONE;

led_strip_handle_t g_led_strip;

void InitEs8311I2s() {
  CLOGI();

  i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_MASTER);
  chan_cfg.auto_clear = true;  // Auto clear the legacy data in the DMA buffer
  ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &g_es8311_tx_handle, &g_es8311_rx_handle));
  // ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &g_es8311_tx_handle, nullptr));
  i2s_std_config_t std_cfg = {
      .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(EXAMPLE_SAMPLE_RATE),
      .slot_cfg =
          {
              .data_bit_width = I2S_DATA_BIT_WIDTH_16BIT,
              .slot_bit_width = I2S_SLOT_BIT_WIDTH_AUTO,
              .slot_mode = I2S_SLOT_MODE_MONO,
              .slot_mask = I2S_STD_SLOT_LEFT,
              .ws_width = I2S_DATA_BIT_WIDTH_16BIT,
              .ws_pol = false,
              .bit_shift = true,
#ifdef I2S_HW_VERSION_2
              .left_align = true,
              .big_endian = false,
              .bit_order_lsb = false,
#endif
          },
      .gpio_cfg =
          {
              .mclk = kEs8311I2sMclk,
              .bclk = kEs8311I2sSclk,
              .ws = kEs8311I2sWs,
              .dout = kEs8311I2sDsdin,
              .din = kEs8311I2sAsdout,
              // .din = GPIO_NUM_NC,
              .invert_flags =
                  {
                      .mclk_inv = false,
                      .bclk_inv = false,
                      .ws_inv = false,
                  },
          },
  };

  std_cfg.clk_cfg.mclk_multiple = static_cast<i2s_mclk_multiple_t>(EXAMPLE_MCLK_MULTIPLE);

  ESP_ERROR_CHECK(i2s_channel_init_std_mode(g_es8311_tx_handle, &std_cfg));
  ESP_ERROR_CHECK(i2s_channel_init_std_mode(g_es8311_rx_handle, &std_cfg));
  ESP_ERROR_CHECK(i2s_channel_enable(g_es8311_tx_handle));
  ESP_ERROR_CHECK(i2s_channel_enable(g_es8311_rx_handle));
}

void InitEs8311() {
  CLOGI();
  const i2c_master_bus_config_t i2c_master_bus_config = {
      .i2c_port = I2C_NUM_1,
      .sda_io_num = kEs8311I2cSda,
      .scl_io_num = kEs8311I2cScl,
      .clk_source = I2C_CLK_SRC_DEFAULT,
      .glitch_ignore_cnt = 7,
      .intr_priority = 0,
      .trans_queue_depth = 0,
      .flags =
          {
              .enable_internal_pullup = 1,
              .allow_pd = 0,
          },
  };

  ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_master_bus_config, &g_i2c_master_bus));
  CLOGI("g_i2c_master_bus:%p", g_i2c_master_bus);

  es8311_handle_t es_handle = es8311_create(g_i2c_master_bus, ES8311_ADDRRES_0);
  const es8311_clock_config_t es_clk = {.mclk_inverted = false,
                                        .sclk_inverted = false,
                                        .mclk_from_mclk_pin = true,
                                        .mclk_frequency = EXAMPLE_MCLK_FREQ_HZ,
                                        .sample_frequency = EXAMPLE_SAMPLE_RATE};
  ESP_ERROR_CHECK(es8311_init(es_handle, &es_clk, ES8311_RESOLUTION_16, ES8311_RESOLUTION_16));
  ESP_ERROR_CHECK(es8311_sample_frequency_config(es_handle, EXAMPLE_SAMPLE_RATE * EXAMPLE_MCLK_MULTIPLE, EXAMPLE_SAMPLE_RATE));
  ESP_ERROR_CHECK(es8311_voice_volume_set(es_handle, 80, nullptr));
  ESP_ERROR_CHECK(es8311_microphone_config(es_handle, false));
  ESP_ERROR_CHECK(es8311_microphone_gain_set(es_handle, ES8311_MIC_GAIN_18DB));
}

void InitButtons() {
  // Button Boot
  {
    button_config_t btn_cfg = {
        .long_press_time = 1000,
        .short_press_time = 50,
    };

    button_gpio_config_t gpio_cfg = {
        .gpio_num = kKeyBoot,
        .active_level = 0,
        .enable_power_save = false,
        .disable_pull = false,
    };

    ESP_ERROR_CHECK(iot_button_new_gpio_device(&btn_cfg, &gpio_cfg, &g_button_boot_handle));
    ESP_ERROR_CHECK(iot_button_register_cb(
        g_button_boot_handle,
        BUTTON_SINGLE_CLICK,
        nullptr,
        [](void *button_handle, void *usr_data) {
          CLOGI("button BOOT single click");
          g_echo_enable = !g_echo_enable;
        },
        nullptr));
    button_event_args_t event_args = {
        .long_press =
            {
                .press_time = 1000,
            },
    };

    ESP_ERROR_CHECK(iot_button_register_cb(
        g_button_boot_handle,
        BUTTON_LONG_PRESS_START,
        &event_args,
        [](void *button_handle, void *usr_data) {
          CLOGI("button BOOT long press start");
          WiFi.begin(kWifiSsid, kWifiPassword);
        },
        nullptr));
  }

  // Button A
  {
    button_config_t btn_cfg = {
        .long_press_time = 1000,
        .short_press_time = 50,
    };

    button_gpio_config_t gpio_cfg = {
        .gpio_num = kKeyA,
        .active_level = 1,
        .enable_power_save = false,
        .disable_pull = false,
    };

    ESP_ERROR_CHECK(iot_button_new_gpio_device(&btn_cfg, &gpio_cfg, &g_button_a_handle));
    ESP_ERROR_CHECK(iot_button_register_cb(
        g_button_a_handle,
        BUTTON_SINGLE_CLICK,
        nullptr,
        [](void *button_handle, void *usr_data) {
          CLOGI("button A single click");
          SD.end();
          g_sdcard_type = CARD_NONE;

          if (!SD.begin(-1, g_hspi)) {
            CLOGE("Card Mount Failed");
          }

          g_sdcard_type = SD.cardType();

          if (g_sdcard_type == CARD_NONE) {
            CLOGE("No SD card attached");
            return;
          }

          if (g_sdcard_type == CARD_MMC) {
            CLOGI("sdcard type: MMC");
          } else if (g_sdcard_type == CARD_SD) {
            CLOGI("sdcard type: SDSC");
          } else if (g_sdcard_type == CARD_SDHC) {
            CLOGI("sdcard type: SDHC");
          } else {
            CLOGI("sdcard type: UNKNOWN");
          }
        },
        nullptr));
  }

  // Button B
  {
    button_config_t btn_cfg = {
        .long_press_time = 1000,
        .short_press_time = 50,
    };

    button_gpio_config_t gpio_cfg = {
        .gpio_num = kKeyB,
        .active_level = 1,
        .enable_power_save = false,
        .disable_pull = false,
    };

    ESP_ERROR_CHECK(iot_button_new_gpio_device(&btn_cfg, &gpio_cfg, &g_button_b_handle));
    ESP_ERROR_CHECK(iot_button_register_cb(
        g_button_b_handle, BUTTON_SINGLE_CLICK, nullptr, [](void *button_handle, void *usr_data) { CLOGI("button B single click"); }, nullptr));
  }
}

void InitDisplay() {
  pinMode(kDisplayBlk, OUTPUT);
  digitalWrite(kDisplayBlk, 1);

  g_display.init(240, 240);
  g_display.setRotation(2);
  g_display.fillScreen(ST77XX_BLACK);
  g_display.setCursor(0, 0);
  g_display.setTextColor(ST77XX_BLUE);
}

void DisplayLoop(void *) {
  bool printed_titile = false;
  constexpr uint32_t kTextSize = 2;
  constexpr uint32_t kTextHeight = kTextSize * 8;
  constexpr uint32_t kTextWidth = kTextSize * 6;
  const std::string kTitileKeyBoot("Key Boot: ");
  const std::string kTitileKeyA("Key A: ");
  const std::string kTitileKeyB("Key B: ");
  const std::string kTitleBatteryLevel("Battery Level: ");
  const std::string kTitleBatteryStatus("Wifi: ");
  const std::string kIP("IP: ");
  const std::string kEchoStatus("Echo Status: ");
  const std::string kSdcardType("SDCard Type: ");
  IPAddress last_ip;
  auto last_wifi_status = WL_STOPPED;
  uint8_t last_echo_status = 0;
  auto last_sdcard_type = CARD_NONE;

loop_start:
  uint32_t line = 0;
  uint32_t y = 5;

  auto now = std::chrono::system_clock::now();
  auto time_sec = std::chrono::system_clock::to_time_t(now);
  auto time_since_epoch = now.time_since_epoch();
  auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(time_since_epoch % std::chrono::seconds(1)).count();
  std::tm tm;
  localtime_r(&time_sec, &tm);

  g_display.setTextWrap(false);
  g_display.setTextSize(kTextSize);

  // time:
  constexpr uint32_t kTextLength = 13;
  std::vector<char> time_str(kTextLength + 1);
  snprintf(time_str.data(), time_str.size(), "%02d:%02d:%02d.%03lld", tm.tm_hour, tm.tm_min, tm.tm_sec, ms);
  g_display.setCursor(0, y);
  g_display.setTextColor(ST77XX_BLUE);
  g_display.fillRect(0, y, 12 * kTextLength - 1, 16, ST77XX_BLACK);
  g_display.println(time_str.data());

  // version:
  y += 24;
  if (!printed_titile) {
    g_display.setCursor(0, y);
    std::string version_text("Version: ");
    version_text += kVersion;
    g_display.setTextColor(ST77XX_WHITE);
    g_display.fillRect(0, y, 12 * version_text.length(), 16, ST77XX_BLACK);
    g_display.println(version_text.c_str());
  }

  // key boot:
  y += 24;
  if (!printed_titile) {
    g_display.setCursor(0, y);
    g_display.setTextColor(ST77XX_WHITE);
    g_display.print(kTitileKeyBoot.c_str());
  }
  g_display.fillRect(kTitileKeyBoot.length() * kTextWidth, y, kTextWidth * 1, kTextHeight, ST77XX_BLACK);
  g_display.setCursor(kTitileKeyBoot.length() * kTextWidth, y);
  if (iot_button_get_key_level(g_button_boot_handle) == 0) {
    g_display.setTextColor(ST77XX_RED);
    g_display.print(0);
  } else {
    g_display.setTextColor(ST77XX_GREEN);
    g_display.print(1);
  }

  // key A:
  y += 24;
  if (!printed_titile) {
    g_display.setCursor(0, y);
    g_display.setTextColor(ST77XX_WHITE);
    g_display.print(kTitileKeyA.c_str());
  }
  g_display.fillRect(kTitileKeyA.length() * kTextWidth, y, kTextWidth * 1, kTextHeight, ST77XX_BLACK);
  g_display.setCursor(kTitileKeyA.length() * kTextWidth, y);
  if (iot_button_get_key_level(g_button_a_handle) == 0) {
    g_display.setTextColor(ST77XX_RED);
    g_display.print(0);
  } else {
    g_display.setTextColor(ST77XX_GREEN);
    g_display.print(1);
  }
  // key B:
  y += 24;
  if (!printed_titile) {
    g_display.setCursor(0, y);
    g_display.setTextColor(ST77XX_WHITE);
    g_display.print(kTitileKeyB.c_str());
  }
  g_display.fillRect(kTitileKeyB.length() * kTextWidth, y, kTextWidth * 1, kTextHeight, ST77XX_BLACK);
  g_display.setCursor(kTitileKeyB.length() * kTextWidth, y);
  if (iot_button_get_key_level(g_button_b_handle) == 0) {
    g_display.setTextColor(ST77XX_RED);
    g_display.print(0);
  } else {
    g_display.setTextColor(ST77XX_GREEN);
    g_display.print(1);
  }

  // battery level
  y += 24;
  if (!printed_titile) {
    g_display.setCursor(0, y);
    g_display.setTextColor(ST77XX_WHITE);
    g_display.print(kTitleBatteryLevel.c_str());
  }

  g_display.fillRect(kTitleBatteryLevel.length() * kTextWidth, y, kTextWidth * 4, kTextHeight, ST77XX_BLACK);
  g_display.setCursor(kTitleBatteryLevel.length() * kTextWidth, y);
  g_display.setTextColor(ST77XX_BLUE);
  g_display.print(g_battery_level);

  // wifi status
  y += 24;
  if (!printed_titile) {
    g_display.setCursor(0, y);
    g_display.setTextColor(ST77XX_WHITE);
    g_display.print(kTitleBatteryStatus.c_str());
    g_display.setTextColor(ST77XX_RED);
    g_display.print("stopped");
  }

  const auto wifi_status = WiFi.status();
  if (wifi_status != last_wifi_status) {
    g_display.fillRect(kTitleBatteryStatus.length() * kTextWidth, y, kTextWidth * 16, kTextHeight, ST77XX_BLACK);
    g_display.setCursor(kTitleBatteryStatus.length() * kTextWidth, y);
    g_display.setTextColor(ST77XX_BLUE);
    switch (wifi_status) {
      case WL_NO_SHIELD:
        g_display.print("no shield");
        break;
      case WL_STOPPED:
        g_display.setTextColor(ST77XX_RED);
        g_display.print("stopped");
        break;
      case WL_IDLE_STATUS:
        g_display.print("idle");
        break;
      case WL_NO_SSID_AVAIL:
        g_display.print("no ssid");
        break;
      case WL_SCAN_COMPLETED:
        g_display.print("scan completed");
        break;
      case WL_CONNECTED:
        g_display.setTextColor(ST77XX_GREEN);
        g_display.print("connected");
        break;
      case WL_CONNECT_FAILED:
        g_display.print("connect failed");
        break;
      case WL_CONNECTION_LOST:
        g_display.print("connection lost");
        break;
      case WL_DISCONNECTED:
        g_display.print("disconnected");
        break;
    }
    last_wifi_status = wifi_status;
  }

  // ip
  y += 24;
  if (!printed_titile) {
    g_display.setCursor(0, y);
    g_display.setTextColor(ST77XX_WHITE);
    g_display.print(kIP.c_str());
    g_display.setTextColor(ST77XX_RED);
    g_display.print(last_ip);
  }

  auto ip(WiFi.localIP());
  if (last_ip != ip) {
    g_display.fillRect(kIP.length() * kTextWidth, y, kTextWidth * (last_ip.toString().length()), kTextHeight, ST77XX_BLACK);
    g_display.setCursor(kIP.length() * kTextWidth, y);
    if (ip == IPAddress()) {
      g_display.setTextColor(ST77XX_RED);
    } else {
      g_display.setTextColor(ST77XX_GREEN);
    }
    g_display.print(ip);
    last_ip = ip;
  }

  // echo status
  y += 24;
  if (!printed_titile) {
    g_display.setCursor(0, y);
    g_display.setTextColor(ST77XX_WHITE);
    g_display.print(kEchoStatus.c_str());
    g_display.setTextColor(ST77XX_RED);
    g_display.print(last_echo_status);
  }

  const auto echo_status = g_echo_enable;
  if (echo_status != last_echo_status) {
    g_display.fillRect(kEchoStatus.length() * kTextWidth, y, kTextWidth, kTextHeight, ST77XX_BLACK);
    g_display.setCursor(kEchoStatus.length() * kTextWidth, y);
    if (echo_status) {
      g_display.setTextColor(ST77XX_GREEN);
    } else {
      g_display.setTextColor(ST77XX_RED);
    }
    g_display.println(echo_status);
    last_echo_status = echo_status;
  }

  // sdcard type
  y += 24;
  if (!printed_titile) {
    g_display.setCursor(0, y);
    g_display.setTextColor(ST77XX_WHITE);
    g_display.print(kSdcardType.c_str());
    g_display.setTextColor(ST77XX_RED);
    g_display.print("NONE");
  }

  const auto sdcard_type = g_sdcard_type;
  if (sdcard_type != last_sdcard_type) {
    g_display.fillRect(kSdcardType.length() * kTextWidth, y, kTextWidth * 4, kTextHeight, ST77XX_BLACK);
    g_display.setCursor(kSdcardType.length() * kTextWidth, y);
    switch (sdcard_type) {
      case CARD_NONE:
        g_display.setTextColor(ST77XX_RED);
        g_display.print("NONE");
        break;
      case CARD_SD:
        g_display.setTextColor(ST77XX_GREEN);
        g_display.print("SD");
        break;
      case CARD_SDHC:
        g_display.setTextColor(ST77XX_GREEN);
        g_display.print("SDHC");
        break;
      default:
        break;
    }
    last_sdcard_type = sdcard_type;
  }

  vTaskDelay(100 / portTICK_PERIOD_MS);
  printed_titile = true;
  goto loop_start;
}

void EchoLoop(void *) {
  constexpr size_t kBufferSize = 2 * 16000 / 1000 * 80;
  uint8_t s_buffer[kBufferSize] = {0};
  size_t bytes_read = 0;

  while (true) {
    if (!g_echo_enable) {
      vTaskDelay(10 / portTICK_PERIOD_MS);
      continue;
    }
    ESP_ERROR_CHECK(i2s_channel_read(g_es8311_rx_handle, s_buffer, kBufferSize, &bytes_read, UINT32_MAX));
    ESP_ERROR_CHECK(i2s_channel_write(g_es8311_tx_handle, s_buffer, bytes_read, nullptr, UINT32_MAX));
  }
}

void InitRGBLed() {
  // LED strip general initialization, according to your led board design
  led_strip_config_t strip_config = {.strip_gpio_num = GPIO_NUM_41,  // The GPIO that connected to the LED strip's data line
                                     .max_leds = 1,                  // The number of LEDs in the strip,
                                     .led_model = LED_MODEL_WS2812,  // LED strip model
                                     .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,  // The color order of the strip: GRB
                                     .flags = {
                                         .invert_out = false,  // don't invert the output signal
                                     }};

  // LED strip backend configuration: RMT
  led_strip_rmt_config_t rmt_config = {.clk_src = RMT_CLK_SRC_DEFAULT,     // different clock source can lead to different power consumption
                                       .resolution_hz = 10 * 1000 * 1000,  // RMT counter clock frequency
                                       .mem_block_symbols = 0,             // the memory block size used by the RMT channel
                                       .flags = {
                                           .with_dma = 0,  // Using DMA can improve performance when driving more LEDs
                                       }};
  ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &g_led_strip));
  ESP_ERROR_CHECK(led_strip_clear(g_led_strip));
}
}  // namespace

void setup() {
  CLOGI();
  g_fspi.begin(kDisplaySclk, -1, kDisplayMosi, -1);
  g_hspi.begin(kSDCardClk, kSDCardMiso, kSDCardMosi, -1);

  InitEs8311I2s();
  InitEs8311();
  InitButtons();
  InitDisplay();
  InitRGBLed();
  pinMode(kBatteryLevelValue, INPUT);

  xTaskCreate(DisplayLoop, "DisplayLoop", 4096 * 4, nullptr, 5, nullptr);

  size_t bytes_written = 0;
  CLOGI("i2s_channel_write");
  ESP_ERROR_CHECK(i2s_channel_write(g_es8311_tx_handle, kPcmData, sizeof(kPcmData), &bytes_written, UINT32_MAX));
  CLOGI("i2s_channel_write bytes_written:%u", bytes_written);

  xTaskCreate(EchoLoop, "EchoLoop", 4096 * 4, nullptr, 5, nullptr);
}

void loop() {
  g_battery_level = analogRead(kBatteryLevelValue);

  static auto s_update_time = 0;
  static uint32_t s_color_index = 0;
  constexpr uint32_t kColors[][3] = {
      {255, 0, 0},
      {0, 255, 0},
      {0, 0, 255},
  };

  if (s_update_time == 0 || millis() - s_update_time > 500) {
    s_update_time = millis();
    const auto &color = kColors[s_color_index++ % (sizeof(kColors) / sizeof(kColors[0]))];
    ESP_ERROR_CHECK(led_strip_set_pixel(g_led_strip, 0, color[0], color[1], color[2]));
    ESP_ERROR_CHECK(led_strip_refresh(g_led_strip));
  }
}