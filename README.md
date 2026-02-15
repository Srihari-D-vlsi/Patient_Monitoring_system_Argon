# 🏥 Patient Monitor Belt (Particle Argon)

An intelligent patient safety system built on the **Particle Argon** platform. This wearable belt provides real-time monitoring of patient vitals, posture, and location transitions within a facility. It is designed to minimize response times for falls and streamline patient tracking across different hospital departments using BLE (Bluetooth Low Energy).

---

## 🚀 Key Features

* **Fall Detection:** Utilizes the MPU6050 accelerometer to detect free-fall events. A **300ms confirmation window** and G-force magnitude calculations are used to filter out false positives.
* **Department Tracking:** Automatically detects "Pediatric" or "Cardiac" departments by measuring **RSSI proximity** to dedicated Particle Argon beacons.
* **Posture Monitoring:** Real-time orientation tracking (Z-axis analysis) to determine if a patient is **Standing** or **Lying Down**.
* **One-Touch Pairing (Learning Mode):** A seamless way to register a patient’s specific phone or smartwatch by simply pressing the Argon's physical `MODE` button.
* **Cloud Integration:** Publishes rich JSON payloads to the Particle Cloud, including Google Maps links, department status, and environmental temperature.
* **Data Persistence:** Uses **EEPROM** to store the paired device's MAC address so the belt remembers the patient even after a battery swap or reboot.

---

## 🛠 Hardware Configuration

### Components
* **Microcontroller:** Particle Argon (Wi-Fi + BLE)
* **Sensor:** MPU6050 (6-Axis Accelerometer + Gyroscope)
* **Indication:** Onboard D7 LED (Used for Learning Mode)
* **Power:** 3.7V LiPo Battery or 5V USB Power Bank



### Wiring Table
| MPU6050 Pin | Argon Pin | Description |
| :--- | :--- | :--- |
| **VCC** | 3.3V | Power Supply |
| **GND** | GND | Ground |
| **SCL** | D1 | I2C Clock |
| **SDA** | D0 | I2C Data |

---

## 💻 Software Setup

1.  **Flash Firmware:** Upload the `.ino` code to your Particle Argon via the [Particle Web IDE](https://ide.particle.io) or Particle Workbench.
2.  **Configure Beacons:** Update the `arg1Address` and `arg2Address` constants in the code with the MAC addresses of your department-specific Argon beacons.
3.  **Webhook Integration:** Create a Webhook in the Particle Console to listen for the `status`, `falling`, and `department` events to forward data to your dashboard.

### JSON Payload Structure
```json
{
  "name": "Patient_iPhone",
  "address": "AA:BB:CC:DD:EE:FF",
  "status": "here",
  "location": "http://googleusercontent.com/maps.google.com/...",
  "department": "Pediatric dept",
  "orientation": "standing",
  "temperature": 32.50
}
