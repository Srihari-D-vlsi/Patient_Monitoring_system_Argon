// Particle Argon Phone/Device Tracker with MPU6050 Fall Detection + Department Detection + Orientation + Temperature
// Detects any Bluetooth device (phone, smartwatch, etc.) + fall detection + Particle Argon beacons + body position + temperature

#include "Particle.h"
#include "Wire.h"

// For logging
SerialLogHandler logHandler(115200, LOG_LEVEL_ERROR, {
    { "app", LOG_LEVEL_TRACE }, // enable all app messages
});

// MPU6050 I2C address
#define MPU6050_ADDR 0x68

// MPU6050 Registers
#define MPU6050_PWR_MGMT_1   0x6B
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_ACCEL_YOUT_H 0x3D
#define MPU6050_ACCEL_ZOUT_H 0x3F
#define MPU6050_TEMP_OUT_H   0x41

// Fall detection thresholds
#define FALL_THRESHOLD 0.5        // G-force threshold (less than 0.5g indicates free fall)
#define FALL_DURATION_US 300000   // Microseconds (300ms - reduced false positives)

// Orientation thresholds (in g's)
#define STANDING_Z_MIN 0.7        // When standing, Z-axis should be > 0.7g
#define LYING_Z_MAX 0.4           // When lying down, Z-axis should be < 0.4g

// Timing constants
#define DEVICE_RE_CHECK_MS 7500
#define DEVICE_NOT_HERE_MS 30000
#define PUBLISH_INTERVAL_MS 1100     // Rate limiting for Particle.publish
#define STATUS_UPDATE_INTERVAL_MS 300000  // 5 minutes (300,000 ms)

// EEPROM address for storing device address
#define DEVICE_EEPROM_ADDRESS 0xa

// Department tracking - Particle Argon BLE addresses
// IMPORTANT: Replace these with actual MAC addresses of your Argon devices
BleAddress arg1Address("AA:BB:CC:DD:EE:01"); // ARG1 - Pediatric Department
BleAddress arg2Address("AA:BB:CC:DD:EE:02"); // ARG2 - Cardiac Department

// Status variable - set this to true to send location
bool statuss = false;

// Global variables
BleAddress searchAddress;
system_tick_t lastSeen = 0;
system_tick_t lastPublish = 0;
system_tick_t lastStatusUpdate = 0;
int lastRSSI = 0;
String status;
String deviceName = "";

// Department tracking
String currentDepartment = "";
String lastPublishedDept = "";
system_tick_t lastDeptSeen = 0;

// MPU6050 variables
bool mpuInitialized = false;
unsigned long fallStartTime = 0;
bool isFalling = false;

// Orientation tracking
String currentOrientation = "lying down";  // Default state
String lastOrientation = "lying down";

// Temperature tracking
float currentTemperature = 0.0;

// Presence enum
typedef enum {
    PresenceUnknown,
    Here,
    NotHere
} DevicePresenceType;

// Default status
DevicePresenceType present = PresenceUnknown;

// Status messages
const char * messages[] {
    "unknown",
    "here",
    "not here"
};

// Your location coordinates (update these with actual values)
double latitude = 10.0266;  // Example: Kanayannur, Kerala
double longitude = 76.3119;

// Function prototypes
void scanResultCallback(const BleScanResult *scanResult, void *context);
bool checkDeviceStateChanged(DevicePresenceType *presence);
void eventHandler(system_event_t event, int duration, void*);
bool isLearningModeOn();
void setLearningModeOn();
void setLearningModeOff();
void sendLocationUpdate();
int setStatusFunction(const char* command);
bool initMPU6050();
void readMPU6050(int16_t &ax, int16_t &ay, int16_t &az);
float readTemperature();
float calculateTotalAcceleration(int16_t ax, int16_t ay, int16_t az);
void checkFallDetection();
void checkOrientation();
void publishFallAlert();
void publishDepartment(String department, int rssi);
void publishPeriodicStatus();
bool canPublish();

void setup() {
    // Initialize I2C for MPU6050
    Wire.begin();
    
    // Set LED pin for learning mode
    pinMode(D7, OUTPUT);
    
    // Set up button handler
    System.on(button_click, eventHandler);
    
    // Register Particle function to control statuss
    Particle.function("setStatus", setStatusFunction);
    
    // Set scan timeout to 5 seconds
    BLE.setScanTimeout(500);
    
    // Initialize MPU6050
    mpuInitialized = initMPU6050();
    if(mpuInitialized) {
        Log.info("✓ MPU6050 initialized successfully!");
    } else {
        Log.error("✗ MPU6050 initialization failed!");
        Log.warn("Check wiring: SDA->D0, SCL->D1, VCC->3.3V, GND->GND");
    }
    
    // Load saved device address from EEPROM
    EEPROM.get(DEVICE_EEPROM_ADDRESS, searchAddress);
    
    // Warning about address
    if(searchAddress == BleAddress("ff:ff:ff:ff:ff:ff")) {
        Log.warn("=== SETUP REQUIRED ===");
        Log.warn("1. Press MODE button (blue LED turns ON)");
        Log.warn("2. Keep your phone/device nearby");
        Log.warn("3. Wait for blue LED to turn OFF");
        Log.warn("======================");
    } else {
        Log.info("Device tracker started!");
        Log.info("Searching for device...");
    }
    
    Log.info("🏥 Department tracking enabled");
    Log.info("ARG1: Pediatric Department");
    Log.info("ARG2: Cardiac Department");
    Log.info("📊 Status updates every 5 minutes");
    
    // Initialize last status update time
    lastStatusUpdate = millis();
}

void loop() {
    // Check orientation and temperature if MPU6050 is initialized
    if(mpuInitialized) {
        checkFallDetection();
        checkOrientation();
        currentTemperature = readTemperature();
    }
    
    // Scan for devices at regular intervals
    if((millis() > lastSeen + DEVICE_RE_CHECK_MS) || (millis() > lastDeptSeen + DEVICE_RE_CHECK_MS)) {
        BLE.scan(scanResultCallback, NULL);
    }
    
    // Publish periodic status update every 5 minutes
    if(millis() - lastStatusUpdate >= STATUS_UPDATE_INTERVAL_MS) {
        publishPeriodicStatus();
        lastStatusUpdate = millis();
    }
    
    // Check if device state has changed
    if(checkDeviceStateChanged(&present)) {
        if(!canPublish()) {
            delay(PUBLISH_INTERVAL_MS - (millis() - lastPublish));
        }
        
        // Get the address string
        char address[18];
        searchAddress.toString().toCharArray(address, sizeof(address));
        
        // Create Google Maps link
        String googleMapsLink = String::format("https://www.google.com/maps?q=%f,%f", latitude, longitude);
        
        // Create payload with status AND location link
        if(deviceName.length() > 0) {
            status = String::format("{\"name\":\"%s\",\"address\":\"%s\",\"lastSeen\":%d,\"lastRSSI\":%i,\"status\":\"%s\",\"location\":\"%s\",\"department\":\"%s\",\"orientation\":\"%s\",\"temperature\":%.2f}",
                deviceName.c_str(), address, lastSeen, lastRSSI, messages[present], googleMapsLink.c_str(), currentDepartment.c_str(), currentOrientation.c_str(), currentTemperature);
        } else {
            status = String::format("{\"address\":\"%s\",\"lastSeen\":%d,\"lastRSSI\":%i,\"status\":\"%s\",\"location\":\"%s\",\"department\":\"%s\",\"orientation\":\"%s\",\"temperature\":%.2f}",
                address, lastSeen, lastRSSI, messages[present], googleMapsLink.c_str(), currentDepartment.c_str(), currentOrientation.c_str(), currentTemperature);
        }
        
        // Publish the status with location
        Particle.publish("status", status, PRIVATE, WITH_ACK);
        lastPublish = millis();
        Particle.process();
        
        // If statuss is true and device is detected, also send separate location event
        if(statuss && present == Here) {
            delay(PUBLISH_INTERVAL_MS);
            sendLocationUpdate();
        }
    }
}

// Check if we can publish (rate limiting)
bool canPublish() {
    return (millis() - lastPublish >= PUBLISH_INTERVAL_MS);
}

// Publish periodic status update (every 5 minutes)
void publishPeriodicStatus() {
    if(!canPublish()) {
        delay(PUBLISH_INTERVAL_MS - (millis() - lastPublish));
    }
    
    // Create periodic status payload
    String periodicPayload = String::format(
        "{\"orientation\":\"%s\",\"department\":\"%s\",\"temperature\":%.2f,\"timestamp\":%lu}",
        currentOrientation.c_str(), currentDepartment.c_str(), currentTemperature, millis()
    );
    
    // Publish periodic status
    Particle.publish("periodic_status", periodicPayload, PRIVATE, WITH_ACK);
    lastPublish = millis();
    
    Log.info("📊 Periodic status: %s | %s | %.2f°C", 
             currentOrientation.c_str(), 
             currentDepartment.c_str(), 
             currentTemperature);
    
    Particle.process();
}

// Check orientation (lying down vs standing)
void checkOrientation() {
    int16_t ax, ay, az;
    readMPU6050(ax, ay, az);
    
    // Convert to g's
    float az_g = az / 16384.0;
    
    // Determine orientation based on Z-axis
    // When standing upright, Z-axis points up (~1g)
    // When lying down, Z-axis is horizontal (~0g)
    if(az_g > STANDING_Z_MIN) {
        currentOrientation = "standing";
    } else if(fabs(az_g) < LYING_Z_MAX) {
        currentOrientation = "lying down";
    }
    // If between thresholds, keep previous state
    
    // Log only when orientation changes
    if(currentOrientation != lastOrientation) {
        Log.info("🧍 Orientation changed: %s", currentOrientation.c_str());
        lastOrientation = currentOrientation;
    }
}

// Publish department detection (simplified - no location data)
void publishDepartment(String department, int rssi) {
    if(!canPublish()) {
        delay(PUBLISH_INTERVAL_MS - (millis() - lastPublish));
    }
    
    // Create simple department payload without location
    String deptPayload = String::format(
        "{\"department\":\"%s\",\"rssi\":%i,\"timestamp\":%lu}",
        department.c_str(), rssi, millis()
    );
    
    // Publish to cloud
    Particle.publish("department", deptPayload, PRIVATE, WITH_ACK);
    lastPublish = millis();
    
    Log.info("📍 Department published: %s (RSSI: %d dBm)", department.c_str(), rssi);
    
    Particle.process();
}

// Initialize MPU6050
bool initMPU6050() {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(MPU6050_PWR_MGMT_1);
    Wire.write(0x00); // Wake up MPU6050
    byte error = Wire.endTransmission();
    
    if(error == 0) {
        delay(100); // Give it time to wake up
        return true;
    }
    return false;
}

// Read accelerometer data from MPU6050
void readMPU6050(int16_t &ax, int16_t &ay, int16_t &az) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(MPU6050_ACCEL_XOUT_H);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 6, true);
    
    ax = (Wire.read() << 8) | Wire.read();
    ay = (Wire.read() << 8) | Wire.read();
    az = (Wire.read() << 8) | Wire.read();
}

// Read temperature from MPU6050
float readTemperature() {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(MPU6050_TEMP_OUT_H);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 2, true);
    
    int16_t rawTemp = (Wire.read() << 8) | Wire.read();
    
    // Convert to Celsius using MPU6050 formula
    // Temperature in °C = (TEMP_OUT Register Value as a signed quantity)/340 + 36.53
    float temperature = (rawTemp / 340.0) + 36.53;
    
    return temperature;
}

// Calculate total acceleration magnitude in G's
float calculateTotalAcceleration(int16_t ax, int16_t ay, int16_t az) {
    // MPU6050 default scale is ±2g, with 16384 LSB/g
    float ax_g = ax / 16384.0;
    float ay_g = ay / 16384.0;
    float az_g = az / 16384.0;
    
    // Calculate magnitude
    float totalAccel = sqrt(ax_g * ax_g + ay_g * ay_g + az_g * az_g);
    return totalAccel;
}

// Check for fall detection
void checkFallDetection() {
    int16_t ax, ay, az;
    readMPU6050(ax, ay, az);
    
    float totalAccel = calculateTotalAcceleration(ax, ay, az);
    
    // Check if acceleration is below threshold (free fall)
    if(totalAccel < FALL_THRESHOLD) {
        if(!isFalling) {
            // Start of fall detected
            fallStartTime = micros();
            isFalling = true;
            Log.trace("Fall detected! Accel: %.2fg", totalAccel);
        } else {
            // Check if fall duration exceeds threshold
            unsigned long fallDuration = micros() - fallStartTime;
            if(fallDuration >= FALL_DURATION_US) {
                // Fall confirmed!
                Log.warn("⚠️ FALL CONFIRMED! Duration: %lu µs", fallDuration);
                publishFallAlert();
                isFalling = false; // Reset to avoid multiple alerts
                delay(1000); // Debounce - wait 1 second before detecting next fall
            }
        }
    } else {
        // Not falling anymore
        if(isFalling) {
            unsigned long fallDuration = micros() - fallStartTime;
            Log.trace("Fall ended. Duration: %lu µs (too short)", fallDuration);
        }
        isFalling = false;
    }
}

// Publish fall alert with device info and location
void publishFallAlert() {
    if(!canPublish()) {
        delay(PUBLISH_INTERVAL_MS - (millis() - lastPublish));
    }
    
    // Get the address string
    char address[18];
    searchAddress.toString().toCharArray(address, sizeof(address));
    
    // Create Google Maps link
    String googleMapsLink = String::format("https://www.google.com/maps?q=%f,%f", latitude, longitude);
    
    // Create fall alert payload
    String fallPayload;
    if(deviceName.length() > 0) {
        fallPayload = String::format(
            "{\"alert\":\"falling\",\"name\":\"%s\",\"address\":\"%s\",\"status\":\"%s\",\"location\":\"%s\",\"department\":\"%s\",\"orientation\":\"%s\",\"temperature\":%.2f}",
            deviceName.c_str(), address, messages[present], googleMapsLink.c_str(), currentDepartment.c_str(), currentOrientation.c_str(), currentTemperature
        );
    } else {
        fallPayload = String::format(
            "{\"alert\":\"falling\",\"address\":\"%s\",\"status\":\"%s\",\"location\":\"%s\",\"department\":\"%s\",\"orientation\":\"%s\",\"temperature\":%.2f}",
            address, messages[present], googleMapsLink.c_str(), currentDepartment.c_str(), currentOrientation.c_str(), currentTemperature
        );
    }
    
    // Publish fall alert
    Particle.publish("falling", fallPayload, PRIVATE, WITH_ACK);
    lastPublish = millis();
    Particle.process();
    
    Log.error("🚨 FALL ALERT PUBLISHED!");
}

void scanResultCallback(const BleScanResult *scanResult, void *context) {
    // Get the device address
    BleAddress addr = scanResult->address();
    
    // === PRIORITY 1: CHECK FOR DEPARTMENT ARGONS ===
    if(addr == arg1Address) {
        Log.info("🏥 Detected ARG1 - Pediatric Department (RSSI: %d dBm)", scanResult->rssi());
        currentDepartment = "Pediatric dept";
        
        // Publish if different from last OR if it's been a while
        if(currentDepartment != lastPublishedDept || (millis() - lastDeptSeen > 60000)) {
            publishDepartment(currentDepartment, scanResult->rssi());
            lastPublishedDept = currentDepartment;
        }
        lastDeptSeen = millis();
        BLE.stopScanning();
        return;
    }
    
    if(addr == arg2Address) {
        Log.info("🏥 Detected ARG2 - Cardiac Department (RSSI: %d dBm)", scanResult->rssi());
        currentDepartment = "Cardiac dept";
        
        // Publish if different from last OR if it's been a while
        if(currentDepartment != lastPublishedDept || (millis() - lastDeptSeen > 60000)) {
            publishDepartment(currentDepartment, scanResult->rssi());
            lastPublishedDept = currentDepartment;
        }
        lastDeptSeen = millis();
        BLE.stopScanning();
        return;
    }
    
    // === PRIORITY 2: LEARNING MODE ===
    if(isLearningModeOn()) {
        // Get device name if available
        String name = scanResult->advertisingData().deviceName();
        
        // Print device info
        Log.info("━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        if(name.length() > 0) {
            Log.info("Device: %s", name.c_str());
        } else {
            Log.info("Device: (Unnamed)");
        }
        Log.info("MAC: %02X:%02X:%02X:%02X:%02X:%02X", 
                 addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
        Log.info("RSSI: %d dBm", scanResult->rssi());
        
        // Get service UUIDs
        BleUuid uuids[8];
        int uuidsAvail = scanResult->advertisingData().serviceUUID(uuids, sizeof(uuids)/sizeof(BleUuid));
        if(uuidsAvail > 0) {
            Log.info("Services: %d found", uuidsAvail);
            for(int i = 0; i < uuidsAvail; i++) {
                if(uuids[i].type() == BleUuidType::SHORT) {
                    Log.info("  UUID: 0x%04X", uuids[i].shorted());
                } else {
                    Log.info("  UUID: %s", uuids[i].toString().c_str());
                }
            }
        }
        Log.info("━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        
        // Save the FIRST device found (strongest signal)
        if(searchAddress == BleAddress("ff:ff:ff:ff:ff:ff")) {
            searchAddress = addr;
            deviceName = scanResult->advertisingData().deviceName();
            EEPROM.put(DEVICE_EEPROM_ADDRESS, searchAddress);
            
            Log.info("");
            Log.info("✓✓✓ DEVICE SAVED! ✓✓✓");
            if(deviceName.length() > 0) {
                Log.info("Tracking: %s", deviceName.c_str());
            }
            Log.info("Address: %02X:%02X:%02X:%02X:%02X:%02X", 
                     addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
            Log.info("");
            
            setLearningModeOff();
            BLE.stopScanning();
            return;
        }
    }
    
    // === PRIORITY 3: TRACKED PHONE/DEVICE ===
    if(!(searchAddress == addr)) {
        return; // Not the device we're tracking
    }
    
    // Device found!
    Log.trace("Device detected - RSSI: %d dBm", scanResult->rssi());
    
    // Save info
    lastSeen = millis();
    lastRSSI = scanResult->rssi();
    
    // Stop scanning
    BLE.stopScanning();
}

bool checkDeviceStateChanged(DevicePresenceType *presence) {
    // Check to see if it's here
    if(millis() > lastSeen + DEVICE_NOT_HERE_MS) {
        if(*presence != NotHere) {
            *presence = NotHere;
            Log.info("❌ Device NOT HERE");
            return true;
        }
    }
    // Case if we've just started up
    else if(lastSeen == 0) {
        if(*presence != PresenceUnknown) {
            *presence = PresenceUnknown;
            Log.trace("Status: unknown");
            return true;
        }
    }
    // Case if lastSeen is < DEVICE_NOT_HERE_MS
    else {
        if(*presence != Here) {
            *presence = Here;
            Log.info("✓ Device HERE (RSSI: %d dBm)", lastRSSI);
            return true;
        }
    }
    
    return false;
}

void eventHandler(system_event_t event, int duration, void*) {
    if(event == button_click) {
        if(digitalRead(D7)) {
            setLearningModeOff();
        } else {
            // Clear saved address when entering learning mode
            searchAddress = BleAddress("ff:ff:ff:ff:ff:ff");
            deviceName = "";
            setLearningModeOn();
            Log.info("");
            Log.info("═══════════════════════════════");
            Log.info("  LEARNING MODE ACTIVATED");
            Log.info("  Keep your phone/device nearby");
            Log.info("  Scanning for devices...");
            Log.info("═══════════════════════════════");
            Log.info("");
        }
    }
}

bool isLearningModeOn() {
    return (digitalRead(D7) == HIGH);
}

void setLearningModeOn() {
    digitalWrite(D7, HIGH);
}

void setLearningModeOff() {
    digitalWrite(D7, LOW);
}

void sendLocationUpdate() {
    if(!canPublish()) {
        delay(PUBLISH_INTERVAL_MS - (millis() - lastPublish));
    }
    
    // Create Google Maps link
    String googleMapsLink = String::format("https://www.google.com/maps?q=%f,%f", latitude, longitude);
    
    // Create location payload
    String locationPayload;
    if(deviceName.length() > 0) {
        locationPayload = String::format(
            "{\"name\":\"%s\",\"lat\":%f,\"lon\":%f,\"rssi\":%i,\"link\":\"%s\",\"department\":\"%s\",\"orientation\":\"%s\",\"temperature\":%.2f}", 
            deviceName.c_str(), latitude, longitude, lastRSSI, googleMapsLink.c_str(), currentDepartment.c_str(), currentOrientation.c_str(), currentTemperature
        );
    } else {
        locationPayload = String::format(
            "{\"lat\":%f,\"lon\":%f,\"rssi\":%i,\"link\":\"%s\",\"department\":\"%s\",\"orientation\":\"%s\",\"temperature\":%.2f}", 
            latitude, longitude, lastRSSI, googleMapsLink.c_str(), currentDepartment.c_str(), currentOrientation.c_str(), currentTemperature
        );
    }
    
    // Publish location to cloud
    Particle.publish("location", locationPayload, PRIVATE, WITH_ACK);
    lastPublish = millis();
    
    Log.info("📍 Location sent: %s", googleMapsLink.c_str());
    
    // Process immediately
    Particle.process();
}

// Particle function to set statuss variable
int setStatusFunction(const char* command) {
    String cmd = String(command);
    cmd.trim();
    cmd.toLowerCase();
    
    if(cmd == "true" || cmd == "1" || cmd == "on") {
        statuss = true;
        Log.info("✓ Location tracking ENABLED");
        return 1;
    }
    else if(cmd == "false" || cmd == "0" || cmd == "off") {
        statuss = false;
        Log.info("✗ Location tracking DISABLED");
        return 0;
    }
    else if(cmd == "fall") {
        // Manually trigger fall alert
        Log.warn("⚠️ MANUAL FALL ALERT TRIGGERED");
        publishFallAlert();
        return 2;
    }
    else if(cmd == "arg1") {
        // Manually publish Pediatric dept
        Log.info("🏥 MANUAL: Publishing Pediatric Department");
        currentDepartment = "Pediatric dept";
        publishDepartment(currentDepartment, 0); // RSSI = 0 for manual trigger
        lastPublishedDept = currentDepartment;
        return 3;
    }
    else if(cmd == "arg2") {
        // Manually publish Cardiac dept
        Log.info("🏥 MANUAL: Publishing Cardiac Department");
        currentDepartment = "Cardiac dept";
        publishDepartment(currentDepartment, 0); // RSSI = 0 for manual trigger
        lastPublishedDept = currentDepartment;
        return 4;
    }
    else if(cmd == "info") {
        // Manually trigger periodic status update
        Log.info("📊 MANUAL: Publishing periodic status info");
        publishPeriodicStatus();
        return 5;
    }
    else {
        Log.error("Invalid command. Use: true/false, 1/0, on/off, fall, arg1, arg2, or info");
        return -1;
    }
}
