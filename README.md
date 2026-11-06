The College of Computing and Information Sciences (CoCIS) at Makerere University requires reliable security for valuable equipment and faces high electricity costs caused by lights left on in unoccupied rooms. Following a management meeting in early October 2026, a pilot automated system was commissioned for two lecture rooms and the main building entrance.
This report presents a practical, implementable design and simulation of a Smart Control and Monitoring System that addresses access control, occupancy tracking, intelligent lighting, data logging, configurable schedules and multi-device coordination using ESP32 microcontrollers on the Wokwi platform with ESP-IDF.
1.2 System Architecture Overview
The system is distributed across three logical control terminals, each potentially running on its own ESP32:
•	Entrance Terminal (Master): Handles RFID entry/exit scanning, door actuators, global occupancy count, access schedule enforcement, SD-card logging, serial management interface and acts as the central coordinator.
•	Lecture Room 1 Terminal: Detects room occupancy (PIR / IR sensors) and controls the room lighting relay. Reports status to the master.
•	Lecture Room 2 Terminal: Identical functionality to Lecture Room 1.
Communication between terminals uses ESP-NOW (lightweight, connectionless protocol ideal for ESP32-to-ESP32 messaging) for low-latency status updates and commands. The Entrance terminal maintains an authoritative state (building occupancy, schedule, logs). Room terminals periodically send occupancy and light status; the master can push configuration or force light commands if needed.
For Wokwi practicality, the core logic can be demonstrated on a single ESP32 with simulated sensors and multiple RFID tags, while the multi-device design is fully described and can be expanded to separate Wokwi projects linked by ESP-NOW simulation notes.
1.3 Hardware Components (Practical Set)
Component	Purpose	Notes / Wokwi Equivalent
ESP32 DevKit (×3)	Main controllers for each terminal	Wokwi ESP32 board
RC522 RFID Module (×2)	Entry and Exit readers at main entrance	RFID / RC522 element + tags
PIR Motion Sensors (×2)	Room occupancy detection	PIR sensor or button/switch simulation
Relay Modules (×2)	Switch lecture-room lights	LED + resistor as light indicator
Servo or Solenoid (×2)	Door lock / barrier actuation	Servo motor element
MicroSD Card Module	Persistent storage of access logs & config	SD card element (or NVS + serial dump)
RTC (DS3231) optional	Accurate time for schedule enforcement	Software RTC or NTP simulation
Buzzer + LEDs	Audio/visual feedback (granted/denied)	Buzzer & LED elements
16×2 or OLED LCD (optional)	Local status display at entrance	LCD or SSD1306 element
1.4 Software Design (ESP-IDF)
The firmware is developed with ESP-IDF (FreeRTOS-based). Key software modules:
•	RFID Task: Polls both RC522 readers, looks up UID against authorised list (stored in NVS or SD), updates occupancy counter, triggers door servo, writes log entry.
•	Occupancy & Lighting Tasks (per room): Read PIR; maintain a soft timer. If motion detected → lights ON and timer reset. If timer expires → lights OFF. Report stated via ESP-NOW.
•	Schedule Enforcement Task: Compares current time (RTC/NTP) against configurable open/close windows. Outside hours, RFID grants are denied (except emergency override) and exit is also restricted while people remain inside (as required).
•	Serial Menu Task: Interactive CLI over UART. Commands for viewing occupancy, recent logs, light status, changing schedule, adding/removing RFID UIDs. Protected by a simple password (hashed or plain for demo).
•	SD Logging Module: Appends CSV lines: timestamp, UID, direction (IN/OUT), result (GRANTED/DENIED), current occupancy.
•	Synchronisation: FreeRTOS mutex protects the shared occupancy counter and configuration structure. ESP-NOW messages are acknowledged; critical updates are retransmitted. Room nodes treat the master as source of truth for schedule.
1.5 Core Features Implemented
1.	RFID-based Access Control: Separate entry and exit readers. Valid UID → servo unlocks for a short time + green LED + log. Invalid → red LED + buzzer + log.
2.	Building Occupancy Tracking: Running count of people inside derived from successful IN/OUT events. Queryable via serial menu.
3.	Automated Lighting: Each lecture room turns lights on when occupancy is detected and off after a configurable idle timeout (e.g. 5 minutes).
4.	Configurable Access Schedule: Start and end times stored in NVS. Outside the window, access is blocked. Schedule can be changed only after successful authentication in the serial menu.
5.	SD Card Logging: All access events and important system events written to /sdcard/access_log.csv.
6.	Serial Management Interface: Text menus offering: View occupancy, view recent logs, Light status, change schedule, Manage authorised cards, System status.
7.	Resource Synchronisation: Mutex-protected shared variables + ESP-NOW heartbeat and state exchange prevent race conditions and keep terminals consistent.
