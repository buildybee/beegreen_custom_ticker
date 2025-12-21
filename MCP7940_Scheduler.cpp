#include "MCP7940_Scheduler.h"
#include <NTPClient.h>
#include <WiFiUdp.h>

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, NTP_SERVER);

MCP7940Scheduler::MCP7940Scheduler() : timezoneOffset(5.5) , _nextDueAlarm(0){}

void MCP7940Scheduler::begin() {
    rtc.begin();
    
    // ENABLE BATTERY BACKUP - Critical for retaining time/data during power failure
    rtc.setBattery(true);
    Serial.println("Battery backup enabled (VBATEN set)");
    Serial.println(rtc.getBattery() ? "Battery backup: ON" : "Battery backup: OFF");
    
    while (!rtc.deviceStatus()) {
        Serial.println(F("Oscillator is off, turning it on."));
        if (!rtc.deviceStart()) {
            Serial.println(F("Oscillator did not start, trying again."));
            delay(1000);
        }
    }
    if (updateTimeFromNTP()) {
        Serial.println("Time updated from NTP.");
    } else {
        Serial.println("Failed to update time from NTP.");
    }
}

void MCP7940Scheduler::setTimeZone(float tzOffset) {
    timezoneOffset = tzOffset;
}

float MCP7940Scheduler::getTimeZone() { return timezoneOffset; }

bool MCP7940Scheduler::updateTimeFromNTP() {
    timeClient.begin();
    if (timeClient.forceUpdate()) {
        time_t ntpTime = timeClient.getEpochTime() + static_cast<uint32_t>(timezoneOffset * 3600);
        rtc.adjust(DateTime(ntpTime));
        return true;
    }
    return false;
}

String MCP7940Scheduler::getCurrentTimestamp() {
    DateTime now = rtc.now();
    char buffer[20];
    snprintf(buffer, sizeof(buffer), "%04d-%02d-%02d %02d:%02d:%02d",
             now.year(), now.month(), now.day(),
             now.hour(), now.minute(), now.second());
    return String(buffer);
}


bool MCP7940Scheduler::setSchedules(const WateringSchedules& schedules) {
    return rtc.writeRAM(0, schedules);
}

bool MCP7940Scheduler::getSchedules(WateringSchedules& schedules) {
    return rtc.readRAM(0, schedules) > 0;
}

bool MCP7940Scheduler::setManualStopTime(uint16_t duration_sec) {
    // This function implements the "Hardware Override" for a manual run.
    // It directly programs the OFFTRIGGER without touching saved schedules.
    
    // CRITICAL FIX: Clear any pre-existing alarm flag before setting a new one.
    // This prevents a race condition where a stale flag from a previous (or empty)
    // schedule could cause an immediate trigger.
    rtc.setAlarmState(ALARM::OFFTRIGGER, false);
    rtc.clearAlarm(ALARM::OFFTRIGGER);
    
    DateTime now = rtc.now();
    DateTime offAlarmTime = now + TimeSpan(duration_sec);

    
    if (!rtc.setAlarm(ALARM::OFFTRIGGER, ALARM_TYPE, offAlarmTime, true)) {
        Serial.println("Failed to set manual OFFTRIGGER alarm.");
        return false;
    }
    Serial.println("Manual stop time set.");
    return true;
}

bool MCP7940Scheduler::setNextAlarm() {
    DateTime now = rtc.now();
    WateringSchedules allSchedules;
    if (!getSchedules(allSchedules)) {
        Serial.println("Could not read schedules from RTC RAM. Initializing blank schedule set.");
        memset(&allSchedules, 0, sizeof(allSchedules));
    }

    DateTime earliestNextAlarm(0);
    uint16_t durationForNextAlarm = 0;

    // Iterate through all possible schedule slots
    for (int i = 0; i < MAX_SCHEDULES; ++i) {
        const auto& schedule = allSchedules.items[i];
        if (!schedule.enabled || schedule.daysOfWeek == 0) {
            continue; // Skip disabled or empty schedules
        }

        // Check the next 7 days to find the earliest future occurrence for this schedule
        for (int dayOffset = 0; dayOffset < 7; ++dayOffset) {
            DateTime checkDate = now + TimeSpan(dayOffset, 0, 0, 0);
            
            // The library returns 1 for Monday, ..., 7 for Sunday.
            // Our bitmask needs 0 for Sunday, 1 for Monday, etc.
            // The `dayOfWeek % 7` trick correctly maps 7 (Sun) -> 0, 1 (Mon) -> 1, etc.
            uint8_t dayOfWeek = checkDate.dayOfTheWeek();
            uint8_t bitIndex = dayOfWeek % 7;

            // Check if the schedule is set to run on this day of the week
            if ((schedule.daysOfWeek >> bitIndex) & 1) {
                DateTime potentialAlarmTime(checkDate.year(), checkDate.month(), checkDate.day(),
                                            schedule.hour, schedule.minute, 0);
                
                // If this potential time is in the future...
                if (potentialAlarmTime.unixtime() > now.unixtime()) {
                    // ...and it's the first one we've found, or it's earlier than the one we already have...
                    if (earliestNextAlarm.unixtime() == 0 || potentialAlarmTime.unixtime() < earliestNextAlarm.unixtime()) {
                        // ...then this is our new best candidate for the next alarm.
                        earliestNextAlarm = potentialAlarmTime;
                        durationForNextAlarm = schedule.duration_sec;
                    }
                    // IMPORTANT: We found the soonest possible time for this schedule,
                    // so we can stop checking future days for this specific schedule and move to the next.
                    break; 
                }
            }
        }
    }

    // Clear any previous hardware alarms before setting new ones
    rtc.clearAlarm(ALARM::ONTRIGGER);
    rtc.clearAlarm(ALARM::OFFTRIGGER);
    rtc.setAlarmState(ALARM::ONTRIGGER, false);
    rtc.setAlarmState(ALARM::OFFTRIGGER, false);

    _nextDueAlarm = earliestNextAlarm; // Store the final result

    if (earliestNextAlarm.unixtime() > 0) {
        Serial.printf("Next alarm set for: %04d-%02d-%02d %02d:%02d:%02d\n",
                      earliestNextAlarm.year(), earliestNextAlarm.month(), earliestNextAlarm.day(),
                      earliestNextAlarm.hour(), earliestNextAlarm.minute(), earliestNextAlarm.second());

        // Set the hardware alarm to turn the pump ON
        if (!rtc.setAlarm(ALARM::ONTRIGGER, ALARM_TYPE, earliestNextAlarm, true)) {
            Serial.println("Failed to set ONTRIGGER alarm.");
            return false;
        }
        
        // Calculate the stop time and set the hardware alarm to turn the pump OFF
        DateTime offAlarmTime = earliestNextAlarm + TimeSpan(durationForNextAlarm);
        if (!rtc.setAlarm(ALARM::OFFTRIGGER, ALARM_TYPE, offAlarmTime, true)) {
            Serial.println("Failed to set OFFTRIGGER alarm.");
            rtc.setAlarmState(ALARM::ONTRIGGER, false); // Rollback the ON alarm
            return false;
        }
        // Defensive: clear any stale latched flags after programming new alarms
        // This ensures past-due IF bits don't linger across power events.
        rtc.clearAlarm(ALARM::ONTRIGGER);
        rtc.clearAlarm(ALARM::OFFTRIGGER);
        return true;
    } 
    
    Serial.println("No future alarms to set.");
    return false;
}

bool MCP7940Scheduler::alarmTriggered(ALARM alarm) {
    return rtc.isAlarm(static_cast<uint8_t>(alarm));
}

void MCP7940Scheduler::getAlarms(DateTime &onAlarm, DateTime &offAlarm) {
    uint8_t alarmType;
    onAlarm = rtc.getAlarm(0, alarmType);
    offAlarm = rtc.getAlarm(1, alarmType);
}

DateTime MCP7940Scheduler::getNextDueAlarm() const {
    return _nextDueAlarm;
}

bool MCP7940Scheduler::getPowerFail() {
    return rtc.getPowerFail();
}

DateTime MCP7940Scheduler::getPowerDown() {
    return rtc.getPowerDown();
}

DateTime MCP7940Scheduler::getPowerUp() {
    return rtc.getPowerUp();
}

bool MCP7940Scheduler::clearPowerFail() {
    return rtc.clearPowerFail();
}