#ifndef Objects_h
#define Objects_h

#define PRODUCT_ID "esp7ina219"
#define FIRMWARE_VERSION "1.3.4"

// Define the server and paths for OTA
#define UPDATEURL "https://raw.githubusercontent.com/buildybee/beegreen-firmware-upgrade/refs/heads/main/esp7ina219.txt"
#define FIRMWAREDOWNLOAD "https://raw.githubusercontent.com/buildybee/beegreen-firmware-upgrade/refs/heads/main/firmware/esp7ina219/"

// mqtt topics
#define HEARBEAT_TOPIC "beegreen/heartbeat"
#define BEEGREEN_STATUS "beegreen/status"

#define PUMP_CONTROL_TOPIC "beegreen/pump_trigger"
#define PUMP_STATUS_TOPIC "beegreen/pump_status"

#define SET_SCHEDULE "beegreen/set_schedule"

#define CURRENT_CONSUMPTION "beegreen/current_consumption"
#define GET_UPDATE_REQUEST "beegreen/firmware_upgrade"
#define REQUEST_ALL_SCHEDULES "beegreen/get_schedules"
#define GET_ALL_SCHEDULES "beegreen/get_schedules_response"
#define NEXT_SCHEDULE "beegreen/next_schedule_due"

#define RESTART "beegreen/restart"

// I2C Pins
#define SDA_PIN 5
#define SCL_PIN 4

#define INA219_I2C_ADDR 0x40
#define MCP7940_I2C_ADDR 0x6F

//INA219_HDWR_CONFIG
#define SHUNT 0.01
#define MAX_CURRENT 3.4

// I/O constants
#define BUTTON_PIN 14
#define MOSFET_PIN 12  // Drives the pump
#define LED_PIN 13     // prod will be 13
#define NUM_LEDS 1

// opertational constants
#define LED_BRIGHTNESS 100
#define POWER_CONSUMPTION_THRESHOLD 50
#define PING_INTERVAL 6000

// Button timing constants
#define BUTTON_DEBOUNCE_TIME 80         // Button debounce time in milliseconds
#define BUTTON_DOUBLE_CLICK_TIME 1500   // Double click window in milliseconds
#define BUTTON_LONG_CLICK_TIME 3000     // Long press time in milliseconds

#define DRD_ADDRESS 0x00  // RTC memory address
#define EEPROM_START_ADDR 0x01 // EEPROM starts after DRD's byte

#define HEARTBEAT_TIMER 30000
#define DRD_TIMEOUT 3.0  // 3 second window for double reset

enum ConnectivityStatus {
  LOCALCONNECTED,
  LOCALNOTCONNECTED,
  SERVERCONNECTED,
  SERVERNOTCONNECTED,
};

typedef struct {
char mqtt_server[60] = "";
uint16_t mqtt_port;
char mqtt_user[32] = "";
char mqtt_password[32] = "";
} MqttCredentials;

// Enum for RGB LED colors
enum LedColor {
    RED = 0xAA4141,         // Red
    GREEN = 0x46FF6e, // Green
    YELLOW = 0xFFFF00, // Yellow
    BLUE = 0x0097ff, // Blue
    MAGENTA = 0x8800FF,
    OFF = 0x000000, // Off
};


struct  State {
  float temp;
  float humidity;
  float currentConsumption = 0; //ref value on for testing 
  bool autoMode = 0;
  ConnectivityStatus radioStatus = ConnectivityStatus::LOCALNOTCONNECTED;
  bool pumpRunning = false;
  bool waterTankEmpty = false;
};

struct Hardconfig {
  uint heartbeat = 60;
  uint motorCutoffThreshold = 200;
  uint aht20ReadInterval= 60;
};

#endif