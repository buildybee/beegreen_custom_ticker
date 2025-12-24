#ifndef MCP7940_SCHEDULER_H
#define MCP7940_SCHEDULER_H

#include <Arduino.h>
#include "MCP7940.h"

#define NTP_SERVER "pool.ntp.org"
#define MAX_SCHEDULES 10

// Bitmask for days of the week for schedule repetition
#define DOW_SUNDAY    (1 << 0)
#define DOW_MONDAY    (1 << 1)
#define DOW_TUESDAY   (1 << 2)
#define DOW_WEDNESDAY (1 << 3)
#define DOW_THURSDAY  (1 << 4)
#define DOW_FRIDAY    (1 << 5)
#define DOW_SATURDAY  (1 << 6)
#define DOW_EVERYDAY  (0b01111111)

// Represents a single watering event
struct ScheduleItem {
    uint8_t hour;           // Watering start hour (0-23)
    uint8_t minute;         // Watering start minute (0-59)
    uint16_t duration_sec;  // Duration of watering in seconds
    uint8_t daysOfWeek;     // Bitmask for repeating days (use DOW_... defines)
    bool enabled;           // True if this schedule is active
};

// A structure to hold all watering schedules, designed to be stored in RTC RAM
struct WateringSchedules {
    ScheduleItem items[MAX_SCHEDULES];
};

// Enumeration for Alarm State
enum ALARM {
    ONTRIGGER,
    OFFTRIGGER
};

class MCP7940Scheduler {
public:
    MCP7940Scheduler();

    // Initialize the RTC and set initial time if available from NTP
    void begin();

    // Set time zone
    void setTimeZone(float timezoneOffsetHours);

    // Get the current time zone
    float getTimeZone();

    // Update time from NTP (called at startup or once a week)
    bool updateTimeFromNTP();

    // Get the current date and time as a string
    String getCurrentTimestamp();

    // Set a watering schedule
    bool setSchedules(const WateringSchedules& schedules);

    // Get a watering schedule
    bool getSchedules(WateringSchedules& schedules);

    // Set next watering schedule
    bool setNextAlarm();

    DateTime getNextDueAlarm() const; // NEW: Getter for the next alarm time

    bool setManualStopTime(uint16_t duration_sec);

    // Check if a specific alarm is triggered
    bool alarmTriggered(ALARM alarm);

    // Get the current alarms (returns both Alarm 0 and Alarm 1)
    void getAlarms(DateTime &alarm0, DateTime &alarm1);

    // Power fail helpers (proxied from underlying RTC)
    bool getPowerFail();
    DateTime getPowerDown();
    DateTime getPowerUp();
    bool clearPowerFail();

    


private:
  MCP7940_Class rtc;
  DateTime _nextDueAlarm; // NEW: Stores the time of the next due alarm
  float timezoneOffset;     // Time zone offset in hours
  uint8_t ALARM_TYPE = 7;
};

#endif // MCP7940_SCHEDULER_H