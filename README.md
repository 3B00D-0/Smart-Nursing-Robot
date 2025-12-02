# Smart-Nursing-Robot
A smart robot for hospitals that follows lines and delivers medicine.
Smart Nursing Robot 🏥🤖

By Vertex Team - Minia University, Faculty of Education

📋 Project Overview

The Smart Nursing Robot is an autonomous vehicle designed to assist medical staff by transporting medicine and light medical supplies from the pharmacy to patient rooms. It uses a Line Following algorithm to navigate hospital corridors and a PID Controller for smooth movement.

🌟 Key Features

Autonomous Navigation: Follows black lines on the floor using a 5-channel IR sensor array.

Smart Medicine Box: A servo-controlled box that opens/closes via a Touch Sensor.

Patient Alert System: A buzzer alarm announces arrival and silences automatically when the patient opens the box.

Smooth Control: Uses PID (Proportional-Derivative) control to prevent wobbling.

🛠️ Hardware Used

Microcontroller: Arduino Uno

Motor Driver: Adafruit Motor Shield v1

Sensors: QTR-8A (using 5 sensors) & TTP223 Touch Sensor

Actuators: 2x DC Gear Motors & 1x Servo Motor

Power: 2x 18650 Li-Ion Batteries

⚙️ How It Works

Start: The robot waits for calibration (1.5s).

Move: It follows the line using PID logic.

Stop: When it detects a black "Stop Line" (Cross line) for >1 second, it stops.

Action: The buzzer beeps to call the patient.

Delivery: The patient touches the sensor -> Box opens -> Alarm stops.

Project supervised by STEM Engineering Department.
