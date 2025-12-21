#include <Arduino.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>

//Preferences
#include <Preferences.h>
Preferences preferences;
uint8_t mac0, mac1, mac2, mac3, mac4, mac5;

#define BIND_PIN 1


//  ESPNow stuff: inpired by Rui Santos & Sara Santos - Random Nerd Tutorials https://RandomNerdTutorials.com/esp-now-auto-pairing-esp32-esp8266/
uint8_t peerAddress[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
uint8_t myMacAddress[6];

typedef struct struct_message {  // Structure to send/receive data
  uint8_t msgType;
  uint8_t macAddr[6];
  short channels[16];
} struct_message;

typedef struct struct_pairing {  // structure for pairing
  uint8_t msgType;
  uint8_t macAddr[6];
  uint8_t channel;
} struct_pairing;

esp_now_peer_info_t peer;

// Create 2 struct_message and one for binding
struct_message myData;  // data to send
struct_message inData;  // data received
struct_pairing pairingData;

enum PairingStatus { NOT_PAIRED,
                     PAIR_REQUEST,
                     PAIR_REQUESTED,
                     PAIR_PAIRED
};
PairingStatus pairingStatus = NOT_PAIRED;

enum MessageType { PAIRING,
                   PAIRED,
                   DATA
};
MessageType messageType;

int channel = 1;  //wifi channel (must be the same on both buddy boxes)

unsigned long currentMillis = millis();
unsigned long previousMillis = 0;  // Stores last time temperature was published
const long interval = 10;          // Interval at which to publish sensor readings
unsigned long start;               // used to measure Pairing time
bool dataToSend = false;
long lastSent;
long dataInterval;

#define DEBUG_ESPNOW
#define DEBUG_PPM
#define DEBUG_SBUS
//#define DEBUG_DATA  //will display data received and sent (comment  for higher speed)

#define RX_PIN 44  //CPMM PIN connected to Rx pin of ESP32-C3
#define TX_PIN 43  //Tx pin of ESP32-C3 connected to S_PORT pin




/* Arduino port of HID USB Host by ESP32beans 
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include "driver/gpio.h"
#include "usb/usb_host.h"

#include "hid_host.h"
#include "hid_usage_keyboard.h"
#include "hid_usage_mouse.h"

/* GPIO Pin number for quit from example logic */
#define APP_QUIT_PIN GPIO_NUM_0

static const char *TAG = "example";
QueueHandle_t hid_host_event_queue;
bool user_shutdown = false;

/**
 * @brief HID Host event
 *
 * This event is used for delivering the HID Host event from callback to a task.
 */
typedef struct {
  hid_host_device_handle_t hid_device_handle;
  hid_host_driver_event_t event;
  void *arg;
} hid_host_event_queue_t;

/**
 * @brief HID Protocol string names
 */
static const char *hid_proto_name_str[] = { "NONE", "KEYBOARD", "MOUSE" };

/**
 * @brief Key event
 */
typedef struct {
  enum key_state { KEY_STATE_PRESSED = 0x00,
                   KEY_STATE_RELEASED = 0x01 } state;
  uint8_t modifier;
  uint8_t key_code;
} key_event_t;

/* Main char symbol for ENTER key */
#define KEYBOARD_ENTER_MAIN_CHAR '\r'
/* When set to 1 pressing ENTER will be extending with LineFeed during serial
 * debug output */
#define KEYBOARD_ENTER_LF_EXTEND 1

/**
 * @brief Scancode to ascii table
 */
const uint8_t keycode2ascii[57][2] = {
  { 0, 0 },                                               /* HID_KEY_NO_PRESS        */
  { 0, 0 },                                               /* HID_KEY_ROLLOVER        */
  { 0, 0 },                                               /* HID_KEY_POST_FAIL       */
  { 0, 0 },                                               /* HID_KEY_ERROR_UNDEFINED */
  { 'a', 'A' },                                           /* HID_KEY_A               */
  { 'b', 'B' },                                           /* HID_KEY_B               */
  { 'c', 'C' },                                           /* HID_KEY_C               */
  { 'd', 'D' },                                           /* HID_KEY_D               */
  { 'e', 'E' },                                           /* HID_KEY_E               */
  { 'f', 'F' },                                           /* HID_KEY_F               */
  { 'g', 'G' },                                           /* HID_KEY_G               */
  { 'h', 'H' },                                           /* HID_KEY_H               */
  { 'i', 'I' },                                           /* HID_KEY_I               */
  { 'j', 'J' },                                           /* HID_KEY_J               */
  { 'k', 'K' },                                           /* HID_KEY_K               */
  { 'l', 'L' },                                           /* HID_KEY_L               */
  { 'm', 'M' },                                           /* HID_KEY_M               */
  { 'n', 'N' },                                           /* HID_KEY_N               */
  { 'o', 'O' },                                           /* HID_KEY_O               */
  { 'p', 'P' },                                           /* HID_KEY_P               */
  { 'q', 'Q' },                                           /* HID_KEY_Q               */
  { 'r', 'R' },                                           /* HID_KEY_R               */
  { 's', 'S' },                                           /* HID_KEY_S               */
  { 't', 'T' },                                           /* HID_KEY_T               */
  { 'u', 'U' },                                           /* HID_KEY_U               */
  { 'v', 'V' },                                           /* HID_KEY_V               */
  { 'w', 'W' },                                           /* HID_KEY_W               */
  { 'x', 'X' },                                           /* HID_KEY_X               */
  { 'y', 'Y' },                                           /* HID_KEY_Y               */
  { 'z', 'Z' },                                           /* HID_KEY_Z               */
  { '1', '!' },                                           /* HID_KEY_1               */
  { '2', '@' },                                           /* HID_KEY_2               */
  { '3', '#' },                                           /* HID_KEY_3               */
  { '4', '$' },                                           /* HID_KEY_4               */
  { '5', '%' },                                           /* HID_KEY_5               */
  { '6', '^' },                                           /* HID_KEY_6               */
  { '7', '&' },                                           /* HID_KEY_7               */
  { '8', '*' },                                           /* HID_KEY_8               */
  { '9', '(' },                                           /* HID_KEY_9               */
  { '0', ')' },                                           /* HID_KEY_0               */
  { KEYBOARD_ENTER_MAIN_CHAR, KEYBOARD_ENTER_MAIN_CHAR }, /* HID_KEY_ENTER */
  { 0, 0 },                                               /* HID_KEY_ESC             */
  { '\b', 0 },                                            /* HID_KEY_DEL             */
  { 0, 0 },                                               /* HID_KEY_TAB             */
  { ' ', ' ' },                                           /* HID_KEY_SPACE           */
  { '-', '_' },                                           /* HID_KEY_MINUS           */
  { '=', '+' },                                           /* HID_KEY_EQUAL           */
  { '[', '{' },                                           /* HID_KEY_OPEN_BRACKET    */
  { ']', '}' },                                           /* HID_KEY_CLOSE_BRACKET   */
  { '\\', '|' },                                          /* HID_KEY_BACK_SLASH      */
  { '\\', '|' },
  /* HID_KEY_SHARP           */  // HOTFIX: for NonUS Keyboards repeat
                                 // HID_KEY_BACK_SLASH
  { ';', ':' },                  /* HID_KEY_COLON           */
  { '\'', '"' },                 /* HID_KEY_QUOTE           */
  { '`', '~' },                  /* HID_KEY_TILDE           */
  { ',', '<' },                  /* HID_KEY_LESS            */
  { '.', '>' },                  /* HID_KEY_GREATER         */
  { '/', '?' }                   /* HID_KEY_SLASH           */
};

/**
 * @brief Makes new line depending on report output protocol type
 *
 * @param[in] proto Current protocol to output
 */
static void hid_print_new_device_report_header(hid_protocol_t proto) {
  static hid_protocol_t prev_proto_output = HID_PROTOCOL_MAX;

  if (prev_proto_output != proto) {
    prev_proto_output = proto;
    printf("\r\n");
    if (proto == HID_PROTOCOL_MOUSE) {
      printf("Mouse\r\n");
    } else if (proto == HID_PROTOCOL_KEYBOARD) {
      printf("Keyboard\r\n");
    } else {
      printf("Generic\r\n");
    }
    fflush(stdout);
  }
}

/**
 * @brief HID Keyboard modifier verification for capitalization application
 * (right or left shift)
 *
 * @param[in] modifier
 * @return true  Modifier was pressed (left or right shift)
 * @return false Modifier was not pressed (left or right shift)
 *
 */
static inline bool hid_keyboard_is_modifier_shift(uint8_t modifier) {
  if (((modifier & HID_LEFT_SHIFT) == HID_LEFT_SHIFT) || ((modifier & HID_RIGHT_SHIFT) == HID_RIGHT_SHIFT)) {
    return true;
  }
  return false;
}

/**
 * @brief HID Keyboard get char symbol from key code
 *
 * @param[in] modifier  Keyboard modifier data
 * @param[in] key_code  Keyboard key code
 * @param[in] key_char  Pointer to key char data
 *
 * @return true  Key scancode converted successfully
 * @return false Key scancode unknown
 */
static inline bool hid_keyboard_get_char(uint8_t modifier, uint8_t key_code,
                                         unsigned char *key_char) {
  uint8_t mod = (hid_keyboard_is_modifier_shift(modifier)) ? 1 : 0;

  if ((key_code >= HID_KEY_A) && (key_code <= HID_KEY_SLASH)) {
    *key_char = keycode2ascii[key_code][mod];
  } else {
    // All other key pressed
    return false;
  }

  return true;
}

/**
 * @brief HID Keyboard print char symbol
 *
 * @param[in] key_char  Keyboard char to stdout
 */
static inline void hid_keyboard_print_char(unsigned int key_char) {
  if (!!key_char) {
    putchar(key_char);
#if (KEYBOARD_ENTER_LF_EXTEND)
    if (KEYBOARD_ENTER_MAIN_CHAR == key_char) {
      putchar('\n');
    }
#endif  // KEYBOARD_ENTER_LF_EXTEND
    fflush(stdout);
  }
}

/**
 * @brief Key Event. Key event with the key code, state and modifier.
 *
 * @param[in] key_event Pointer to Key Event structure
 *
 */
static void key_event_callback(key_event_t *key_event) {
  unsigned char key_char;

  hid_print_new_device_report_header(HID_PROTOCOL_KEYBOARD);

  if (key_event->KEY_STATE_PRESSED == key_event->state) {
    if (hid_keyboard_get_char(key_event->modifier, key_event->key_code,
                              &key_char)) {

      hid_keyboard_print_char(key_char);
    }
  }
}

/**
 * @brief Key buffer scan code search.
 *
 * @param[in] src       Pointer to source buffer where to search
 * @param[in] key       Key scancode to search
 * @param[in] length    Size of the source buffer
 */
static inline bool key_found(const uint8_t *const src, uint8_t key,
                             unsigned int length) {
  for (unsigned int i = 0; i < length; i++) {
    if (src[i] == key) {
      return true;
    }
  }
  return false;
}

/**
 * @brief USB HID Host Keyboard Interface report callback handler
 *
 * @param[in] data    Pointer to input report data buffer
 * @param[in] length  Length of input report data buffer
 */
static void hid_host_keyboard_report_callback(const uint8_t *const data,
                                              const int length) {
  hid_keyboard_input_report_boot_t *kb_report =
    (hid_keyboard_input_report_boot_t *)data;

  if (length < sizeof(hid_keyboard_input_report_boot_t)) {
    return;
  }

  static uint8_t prev_keys[HID_KEYBOARD_KEY_MAX] = { 0 };
  key_event_t key_event;

  for (int i = 0; i < HID_KEYBOARD_KEY_MAX; i++) {

    // key has been released verification
    if (prev_keys[i] > HID_KEY_ERROR_UNDEFINED && !key_found(kb_report->key, prev_keys[i], HID_KEYBOARD_KEY_MAX)) {
      key_event.key_code = prev_keys[i];
      key_event.modifier = 0;
      key_event.state = key_event.KEY_STATE_RELEASED;
      key_event_callback(&key_event);
    }

    // key has been pressed verification
    if (kb_report->key[i] > HID_KEY_ERROR_UNDEFINED && !key_found(prev_keys, kb_report->key[i], HID_KEYBOARD_KEY_MAX)) {
      key_event.key_code = kb_report->key[i];
      key_event.modifier = kb_report->modifier.val;
      key_event.state = key_event.KEY_STATE_PRESSED;
      key_event_callback(&key_event);
    }
  }

  memcpy(prev_keys, &kb_report->key, HID_KEYBOARD_KEY_MAX);
}

/**
 * @brief USB HID Host Mouse Interface report callback handler
 *
 * @param[in] data    Pointer to input report data buffer
 * @param[in] length  Length of input report data buffer
 */
static void hid_host_mouse_report_callback(const uint8_t *const data,
                                           const int length) {
  // Standard HID Boot Protocol Mouse Report.
  typedef struct __attribute__((packed)) {
    uint8_t buttons;
    int8_t x;
    int8_t y;
    int8_t wheel;
    int8_t pan;
  } hid_mouse_report_t;

  hid_mouse_report_t *mouse_report = (hid_mouse_report_t *)data;

  if (length < 3) {
    return;
  }

  hid_print_new_device_report_header(HID_PROTOCOL_MOUSE);

  printf("buttons: %02x x: %5d y: %5d",
         mouse_report->buttons, mouse_report->x, mouse_report->y);
  if (length > 3) {
    printf(" wheel: %5d", mouse_report->wheel);
    if (length > 4) {
      printf(" pan: %5d", mouse_report->pan);
    }
  }
  printf("\r\n");
  fflush(stdout);
}

/**
 * @brief USB HID Host Generic Interface report callback handler
 *
 * 'generic' means anything else than mouse or keyboard
 *
 * @param[in] data    Pointer to input report data buffer
 * @param[in] length  Length of input report data buffer
 */
static void hid_host_generic_report_callback(const uint8_t *const data,
                                             const int length) {
  hid_print_new_device_report_header(HID_PROTOCOL_NONE);
  for (int i = 0; i < min(10, length); i++) {
    printf("%02X", data[i]);
  }
  putchar('|');

  switch (length) {
    // This assumes the HID is a Thrustmaster T16000M flight joystick
    case 9:
      {
        typedef struct __attribute__((packed)) {
          uint16_t buttons;
          uint8_t hat;
          uint16_t x;
          uint16_t y;
          uint8_t z;  // Twist
          uint8_t throttle;
        } t16k_t;

        const t16k_t *const joy = (const t16k_t *const)data;
         printf("buttons: %d hat: %d X: %d Y: %d Z: %d throttle: %d", joy->buttons, joy->hat, joy->x, joy->y, joy->z, joy->throttle);

        myData.channels[0] = map(joy->x, 0, 16383, 192, 1792);      //Aileron to SBUS
        myData.channels[1] = map(joy->y, 0, 16383, 1792, 192);      //Ele to SBUS
        myData.channels[2] = map(joy->throttle, 0, 255, 1792, 192); //Throttle to SBUS
        myData.channels[3] = map(joy->z, 0, 255, 1792, 192);        //Rudder to SBUS
        dataToSend = true;
      }
      break;
    case 7:
      // This assumes the HID is a Logitech Extreme 3D Pro flight joystick
      {
        typedef struct __attribute__((packed)) {
          uint32_t x : 10;
          uint32_t y : 10;
          uint32_t hat : 4;
          uint32_t z : 8;
          uint8_t buttons_a;
          uint8_t throttle;
          uint8_t buttons_b;
        } le3dp_t;

        const le3dp_t *const joy = (const le3dp_t *const)data;
        printf("X: %04x Y: %04x hat: %x Z: %02x buttons_a: %02x throttle: %02x buttons_a: %02x",
               joy->x, joy->y, joy->hat, joy->z, joy->buttons_a, joy->throttle,
               joy->buttons_b);
      }
      break;
    case 64:
      {
        // Assume the HID is a Sony Dual Shock 4 PlayStation 4 game controller
        // 14 Buttons, 6 Axes, 1 D-Pad
        typedef struct __attribute__((packed)) {
          uint8_t ReportID;  // always 0x01
          uint8_t leftXAxis;
          uint8_t leftYAxis;
          uint8_t rightXAxis;
          uint8_t rightYAxis;
          uint8_t dPad : 4;     // dpad[3-0]
          uint8_t button1 : 4;  // Triangle[7], circle[6], cross[5], square[4]
          uint8_t button2;      // R3:7,L3:6,Options:5,share:4,R2:3,L2:2,R1:1,L1:0
          uint8_t button3 : 2;  // tpad click[1], logo[0]
          uint8_t reportCnt : 6;
          uint8_t L2Axis;
          uint8_t R2Axis;
          uint16_t timestamp;  // increment by 188 per report
          uint8_t batteryLvl;
          uint16_t gyroX;
          uint16_t gyroY;
          uint16_t gyroZ;
          int16_t accelX;
          int16_t accelY;
          int16_t accelZ;
          uint8_t filler[39];
        } DS4GamepadReport_t;
        const DS4GamepadReport_t *const ds4 =
          (const DS4GamepadReport_t *const)data;
        printf("Left X,Y: %3u,%3u Right X,Y: %3u,%3u "
               "DPad: %x Button1: 0x%x Button2: 0x%02x Button3: 0x%x "
               "Throttle Left,Right: %3u,%3u",
               ds4->leftXAxis, ds4->leftYAxis, ds4->rightXAxis, ds4->rightYAxis,
               ds4->dPad, ds4->button1, ds4->button2, ds4->button3,
               ds4->L2Axis, ds4->R2Axis);
        break;
      }
    default:
      break;
  }
  printf("\r\n");
  fflush(stdout);
}

/**
 * @brief USB HID Host interface callback
 *
 * @param[in] hid_device_handle  HID Device handle
 * @param[in] event              HID Host interface event
 * @param[in] arg                Pointer to arguments, does not used
 */
void hid_host_interface_callback(hid_host_device_handle_t hid_device_handle,
                                 const hid_host_interface_event_t event,
                                 void *arg) {
  uint8_t data[64] = { 0 };
  size_t data_length = 0;
  hid_host_dev_params_t dev_params;
  ESP_ERROR_CHECK(hid_host_device_get_params(hid_device_handle, &dev_params));

  switch (event) {
    case HID_HOST_INTERFACE_EVENT_INPUT_REPORT:
      ESP_ERROR_CHECK(hid_host_device_get_raw_input_report_data(
        hid_device_handle, data, 64, &data_length));

      if (HID_SUBCLASS_BOOT_INTERFACE == dev_params.sub_class) {
        if (HID_PROTOCOL_KEYBOARD == dev_params.proto) {
          hid_host_keyboard_report_callback(data, data_length);
        } else if (HID_PROTOCOL_MOUSE == dev_params.proto) {
          hid_host_mouse_report_callback(data, data_length);
        }
      } else {
        hid_host_generic_report_callback(data, data_length);
      }

      break;
    case HID_HOST_INTERFACE_EVENT_DISCONNECTED:
      ESP_LOGI(TAG, "HID Device, protocol '%s' DISCONNECTED",
               hid_proto_name_str[dev_params.proto]);
      ESP_ERROR_CHECK(hid_host_device_close(hid_device_handle));
      break;
    case HID_HOST_INTERFACE_EVENT_TRANSFER_ERROR:
      ESP_LOGI(TAG, "HID Device, protocol '%s' TRANSFER_ERROR",
               hid_proto_name_str[dev_params.proto]);
      break;
    default:
      ESP_LOGE(TAG, "HID Device, protocol '%s' Unhandled event",
               hid_proto_name_str[dev_params.proto]);
      break;
  }
}

/**
 * @brief USB HID Host Device event
 *
 * @param[in] hid_device_handle  HID Device handle
 * @param[in] event              HID Host Device event
 * @param[in] arg                Pointer to arguments, does not used
 */
void hid_host_device_event(hid_host_device_handle_t hid_device_handle,
                           const hid_host_driver_event_t event, void *arg) {
  hid_host_dev_params_t dev_params;
  ESP_ERROR_CHECK(hid_host_device_get_params(hid_device_handle, &dev_params));
  const hid_host_device_config_t dev_config = {
    .callback = hid_host_interface_callback, .callback_arg = NULL
  };


  switch (event) {
    case HID_HOST_DRIVER_EVENT_CONNECTED:
      ESP_LOGI(TAG, "HID Device, protocol '%s' CONNECTED",
               hid_proto_name_str[dev_params.proto]);

      ESP_ERROR_CHECK(hid_host_device_open(hid_device_handle, &dev_config));
      if (HID_SUBCLASS_BOOT_INTERFACE == dev_params.sub_class) {
        ESP_ERROR_CHECK(hid_class_request_set_protocol(hid_device_handle,
                                                       HID_REPORT_PROTOCOL_BOOT));
        if (HID_PROTOCOL_KEYBOARD == dev_params.proto) {
          ESP_ERROR_CHECK(hid_class_request_set_idle(hid_device_handle, 0, 0));
        }
      }
      ESP_ERROR_CHECK(hid_host_device_start(hid_device_handle));
      break;
    default:
      break;
  }
}

/**
 * @brief Start USB Host install and handle common USB host library events while
 * app pin not low
 *
 * @param[in] arg  Not used
 */
static void usb_lib_task(void *arg) {
  const gpio_config_t input_pin = {
    .pin_bit_mask = BIT64(APP_QUIT_PIN),
    .mode = GPIO_MODE_INPUT,
    .pull_up_en = GPIO_PULLUP_ENABLE,
  };
  ESP_ERROR_CHECK(gpio_config(&input_pin));

  const usb_host_config_t host_config = {
    .skip_phy_setup = false,
    .intr_flags = ESP_INTR_FLAG_LEVEL1,
  };

  ESP_ERROR_CHECK(usb_host_install(&host_config));
  xTaskNotifyGive((TaskHandle_t)arg);

  while (gpio_get_level(APP_QUIT_PIN) != 0) {
    uint32_t event_flags;
    usb_host_lib_handle_events(portMAX_DELAY, &event_flags);

    // Release devices once all clients has deregistered
    if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
      usb_host_device_free_all();
      ESP_LOGI(TAG, "USB Event flags: NO_CLIENTS");
    }
    // All devices were removed
    if (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE) {
      ESP_LOGI(TAG, "USB Event flags: ALL_FREE");
    }
  }
  // App Button was pressed, trigger the flag
  user_shutdown = true;
  ESP_LOGI(TAG, "USB shutdown");
  // Clean up USB Host
  vTaskDelay(10);  // Short delay to allow clients clean-up
  ESP_ERROR_CHECK(usb_host_uninstall());
  vTaskDelete(NULL);
}

/**
 * @brief HID Host main task
 *
 * Creates queue and get new event from the queue
 *
 * @param[in] pvParameters Not used
 */
void hid_host_task(void *pvParameters) {
  hid_host_event_queue_t evt_queue;
  // Create queue
  hid_host_event_queue = xQueueCreate(10, sizeof(hid_host_event_queue_t));

  // Wait queue
  while (!user_shutdown) {
    if (xQueueReceive(hid_host_event_queue, &evt_queue, pdMS_TO_TICKS(50))) {
      hid_host_device_event(evt_queue.hid_device_handle, evt_queue.event,
                            evt_queue.arg);
    }
  }

  xQueueReset(hid_host_event_queue);
  vQueueDelete(hid_host_event_queue);
  vTaskDelete(NULL);
}

/**
 * @brief HID Host Device callback
 *
 * Puts new HID Device event to the queue
 *
 * @param[in] hid_device_handle HID Device handle
 * @param[in] event             HID Device event
 * @param[in] arg               Not used
 */
void hid_host_device_callback(hid_host_device_handle_t hid_device_handle,
                              const hid_host_driver_event_t event, void *arg) {
  const hid_host_event_queue_t evt_queue = {
    .hid_device_handle = hid_device_handle, .event = event, .arg = arg
  };
  xQueueSend(hid_host_event_queue, &evt_queue, 0);
}

//ESPNow
void readGetMacAddress() {
  uint8_t baseMac[6];
  esp_err_t ret = esp_wifi_get_mac(WIFI_IF_STA, baseMac);
  if (ret == ESP_OK) {
    printMAC(baseMac);
  } else {
    Serial.println("Failed to read MAC address");
  }
  myMacAddress[0] = baseMac[0];
  myMacAddress[1] = baseMac[1];
  myMacAddress[2] = baseMac[2];
  myMacAddress[3] = baseMac[3];
  myMacAddress[4] = baseMac[4];
  myMacAddress[5] = baseMac[5];
}
void addPeer(const uint8_t *mac_addr, uint8_t chan) {  //add peer to ESPNow peers list
#ifdef DEBUG_ESPNOW
  Serial.print("addpeer ");
  printMAC(mac_addr);
#endif
  ESP_ERROR_CHECK(esp_wifi_set_channel(chan, WIFI_SECOND_CHAN_NONE));
  esp_now_del_peer(mac_addr);                     //delete previous peer with same MAC address
  memset(&peer, 0, sizeof(esp_now_peer_info_t));  // erase peer (store 0 in each byte)
  peer.channel = chan;
  peer.encrypt = false;
  memcpy(peer.peer_addr, mac_addr, sizeof(uint8_t[6]));
  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
  memcpy(peerAddress, mac_addr, sizeof(uint8_t[6]));  //copy macaddr to sender address
}

void printMAC(const uint8_t *mac_addr) {  // print MAC address in hexa
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.println(macStr);
}
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  /* Serial.print("\r\nsend Status:\t");
  Serial.print(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  Serial.print(" to ");
  printMAC(mac_addr); */
}

void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {  // triggered when receiving data on ESPNow link
  /*Serial.print("Packet received with ");
  Serial.print("data size = ");
  Serial.print(sizeof(incomingData));*/
  uint8_t type = incomingData[0];
  switch (type) {
    case DATA:                           // we received data from sender
      if (pairingStatus == PAIR_PAIRED)  //only accept these data when paired
      {
        blinkLed();
        memcpy(&inData, incomingData, sizeof(inData));
        if ((inData.macAddr[0] + inData.macAddr[1] + inData.macAddr[2] + inData.macAddr[3] + inData.macAddr[4] + inData.macAddr[5]) != (myMacAddress[0] + myMacAddress[1] + myMacAddress[2] + myMacAddress[3] + myMacAddress[4] + myMacAddress[5])) {
          {
#ifdef DEBUG_DATA
            Serial.print("ESPNow data received,\t");

            //for (int8_t i = 0; i < 16; i++) {  // do something with the SBUS values for each channel
            // Serial.print(inData.channels[i]);
            // Serial.print("\t");
            //}
            //Serial.println("");
#endif

            //for (int8_t i = 0; i < 16; i++) {  // resend to the SBUS S_PORT pin for each channel
            // data.ch[i] = inData.channels[i];
            //}
            //data.lost_frame = false;  //initialize SBUS lost
            //data.failsafe = false;
            dataToSend = true;
          }
        }
      }
      break;

    case PAIRING:  // we received pairing data from server
      //if (pairingStatus != PAIR_PAIRED)  //only accept these data when not paired
      {
        memcpy(&pairingData, incomingData, sizeof(pairingData));
        if ((pairingData.macAddr[0] + pairingData.macAddr[1] + pairingData.macAddr[2] + pairingData.macAddr[3] + pairingData.macAddr[4] + pairingData.macAddr[5]) != (myMacAddress[0] + myMacAddress[1] + myMacAddress[2] + myMacAddress[3] + myMacAddress[4] + myMacAddress[5])) {
#ifdef DEBUG_ESPNOW
          Serial.print("received PAIRING : Pairing done for sender MAC Address: ");
          printMAC(pairingData.macAddr);
          Serial.print(" in ");
          Serial.print(millis() - start);
          Serial.println("ms");
#endif
          start = millis();
          addPeer(pairingData.macAddr, pairingData.channel);  // add the sender  to the peer list

          preferences.putInt("mac0", pairingData.macAddr[0]);  //keep track of the peer address
          preferences.putInt("mac1", pairingData.macAddr[1]);
          preferences.putInt("mac2", pairingData.macAddr[2]);
          preferences.putInt("mac3", pairingData.macAddr[3]);
          preferences.putInt("mac4", pairingData.macAddr[4]);
          preferences.putInt("mac5", pairingData.macAddr[5]);

          pairingData.msgType = PAIRED;                                   //and send back a "PAIRED" message to sender
          memcpy(pairingData.macAddr, myMacAddress, sizeof(uint8_t[6]));  //with my MACaddress
          esp_now_send(peerAddress, (uint8_t *)&pairingData, sizeof(pairingData));
          previousMillis = millis();
          pairingStatus = PAIR_PAIRED;  // set the pairing status
        }
      }
      break;


    case PAIRED:  // we received pairing data from peer
      memcpy(&pairingData, incomingData, sizeof(pairingData));
      //if ((pairingData.macAddr[0] + pairingData.macAddr[1] + pairingData.macAddr[2] + pairingData.macAddr[3] + pairingData.macAddr[4] + pairingData.macAddr[5]) != (myMacAddress[0] + myMacAddress[1] + myMacAddress[2] + myMacAddress[3] + myMacAddress[4] + myMacAddress[5]))
      {
#ifdef DEBUG_ESPNOW
        Serial.println("Paired message received from ");
        printMAC(pairingData.macAddr);
#endif
        addPeer(pairingData.macAddr, pairingData.channel);  // add the sender  to the peer list

        preferences.putInt("mac0", pairingData.macAddr[0]);  //store again the Peer MacAddress
        preferences.putInt("mac1", pairingData.macAddr[1]);
        preferences.putInt("mac2", pairingData.macAddr[2]);
        preferences.putInt("mac3", pairingData.macAddr[3]);
        preferences.putInt("mac4", pairingData.macAddr[4]);
        preferences.putInt("mac5", pairingData.macAddr[5]);
        pairingStatus = PAIR_PAIRED;  // set the pairing status to "PAIRED" and now we can send/receive DATA
      }
      break;
  }
}
void blinkLed(void) {
  if ((millis() - dataInterval) > 50) {
    //digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    dataInterval = millis();
  }
}
void setup(void) {
  BaseType_t task_created;
  ESP_LOGI(TAG, "HID Joystick");

  /*
   * Create usb_lib_task to:
   * - initialize USB Host library
   * - Handle USB Host events while APP pin in in HIGH state
   */
  task_created =
    xTaskCreatePinnedToCore(usb_lib_task, "usb_events", 4096,
                            xTaskGetCurrentTaskHandle(), 2, NULL, 0);
  assert(task_created == pdTRUE);

  // Wait for notification from usb_lib_task to proceed
  ulTaskNotifyTake(false, 1000);

  /*
   * HID host driver configuration
   * - create background task for handling low level event inside the HID driver
   * - provide the device callback to get new HID Device connection event
   */
  const hid_host_driver_config_t hid_host_driver_config = {
    .create_background_task = true,
    .task_priority = 5,
    .stack_size = 4096,
    .core_id = 0,
    .callback = hid_host_device_callback,
    .callback_arg = NULL
  };

  ESP_ERROR_CHECK(hid_host_install(&hid_host_driver_config));

  // Task is working until the devices are gone (while 'user_shutdown' if false)
  user_shutdown = false;

  /*
   * Create HID Host task process for handle events
   * IMPORTANT: Task is necessary here while there is no possibility to interact
   * with USB device from the callback.
   */
  task_created =
    xTaskCreate(&hid_host_task, "hid_task", 4 * 1024, NULL, 2, NULL);
  assert(task_created == pdTRUE);

  Serial.begin(115200);
  Serial.println();
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(BIND_PIN, INPUT_PULLUP);
  //Preferences
  preferences.begin("Trainer", false);

  //preferences.clear();              // Remove all preferences under the opened namespace
  //preferences.remove("counter");    // remove the counter key only

  //ESPNow
  if (digitalRead(BIND_PIN) == false) {  //during boot if "BIND" button is pressed
    preferences.clear();                 // Remove all preferences under the opened namespace
    Serial.println("binding cleared");
  }
  mac0 = preferences.getInt("mac0", 0xFF);  // retreive peer address (or FF:FF:FF:FF:FF:FF if "bind" has been pressed)
  mac1 = preferences.getInt("mac1", 0xFF);
  mac2 = preferences.getInt("mac2", 0xFF);
  mac3 = preferences.getInt("mac3", 0xFF);
  mac4 = preferences.getInt("mac4", 0xFF);
  mac5 = preferences.getInt("mac5", 0xFF);

  Serial.println("read preferences :");
  Serial.print("\tmacPeer ");
  peerAddress[0] = mac0;
  peerAddress[1] = mac1;
  peerAddress[2] = mac2;
  peerAddress[3] = mac3;
  peerAddress[4] = mac4;
  peerAddress[5] = mac5;
  printMAC(peerAddress);
  //preferences.end();  // Close the Preferences

  WiFi.mode(WIFI_STA);  //launch Wifi to get my Mac Address
  WiFi.STA.begin();
  Serial.print("my MAC Address:  ");
  readGetMacAddress();
  WiFi.disconnect();
  delay(1000);
  start = millis();

  // set WiFi channel
  ESP_ERROR_CHECK(esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE));
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
  }

  // set callback routines for ESPNow
  esp_now_register_send_cb(esp_now_send_cb_t(OnDataSent));
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
  // add peer and send request
  addPeer(peerAddress, channel);
  if (mac0 & mac1 & mac2 & mac3 & mac4 & mac5 == 0xFF) pairingStatus = PAIR_REQUEST;  //launch auto pairing if peerAddress is FF:FF:FF:FF:FF:FF
  else pairingStatus = PAIR_PAIRED;                                                   //launch auto pairing done when already paired
  myData.channels[0] = (192 + 1792) / 2;  //Ailerons at mid point
  myData.channels[1] = (192 + 1792) / 2;  //Ele
  myData.channels[2] = 192;               //Throttle
  myData.channels[3] = (192 + 1792) / 2;  //Rudder
}

void loop() {
  //ESPNow
  switch (pairingStatus) {
    case PAIR_REQUEST:
#ifdef DEBUG_ESPNOW
      Serial.print("I request pairing on channel ");
      Serial.println(channel);
#endif

      pairingData.msgType = PAIRING;  // set pairing data to send to the server
      pairingData.channel = channel;

      pairingData.macAddr[0] = myMacAddress[0];
      pairingData.macAddr[1] = myMacAddress[1];
      pairingData.macAddr[2] = myMacAddress[2];
      pairingData.macAddr[3] = myMacAddress[3];
      pairingData.macAddr[4] = myMacAddress[4];
      pairingData.macAddr[5] = myMacAddress[5];

      // add peer and send request
      addPeer(peerAddress, channel);
      esp_now_send(peerAddress, (uint8_t *)&pairingData, sizeof(pairingData));  // send the PAIRING message
      previousMillis = millis();
      pairingStatus = PAIR_REQUESTED;
      break;

    case PAIR_REQUESTED:
      currentMillis = millis();
      if (currentMillis - previousMillis > 4000) {  // time out to allow receiving response from peer
        previousMillis = currentMillis;
        Serial.println(" timeout pairing ");
        pairingStatus = PAIR_REQUEST;  //retry sending PAIRING message
      }
      break;



    case PAIR_PAIRED:  //send data
      if (dataToSend) {
        blinkLed();
        myData.msgType = DATA;
        // for (int8_t i = 0; i < data.NUM_CH; i++) {  // do something with the SBUS values for each channel
        //myData.channels[i] = constrain(data.ch[i], 192, 1792);
        //myData.channels[i] = data.ch[i];
        // }

        memcpy(pairingData.macAddr, myMacAddress, sizeof(uint8_t[6]));                     //with my MACaddress
        esp_err_t result = esp_now_send(peerAddress, (uint8_t *)&myData, sizeof(myData));  //send data to the peer
        dataToSend = false;
        lastSent = millis();
      }
      break;
  }
  if ((millis() - lastSent) > 20) dataToSend = true;
}
