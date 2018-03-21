View this project on [CADLAB.io](https://cadlab.io/node/765). 

# elcano-HL-capstone
Repo for the Elcano high-level redesign project (BEE capstone project)

The Elcano High-Level system is responsible for setting a desired vehicle speed and direction based on localization, pathfinding, and obstacle detection. The Low-Level system then has the job of making the vehicle execute the desired commands. At present, the High-Level system is partitioned among several processors:
· C7 (Raspberry Pi) processes visual information
· C6 (Arduino Mega) localization based on GPS, speed, and INU
· C5 (Arduino Micro) Lidar/sonar obstacle detection
· C4 (Arduino Mega) computes path from digital map
· C3 (Arduino Micro) processes all data and sends instructions to Low-Level

The High-Level Redesign project seeks to port the C6, C4, and C3 processors to a single Arduino DUE. The new processor is designated as the C643 processor and has the following features:
· Elcano Low-Level Backwards Compatibility
· Simplified Serial Communication
· Single Processor Operation
· Arduino IDE Friendly
· Smaller Footprint
· CAN Bus Support
· 32-bit processing