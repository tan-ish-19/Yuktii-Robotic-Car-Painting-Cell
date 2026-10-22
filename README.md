# Robotic Car Painting Cell Simulation

## Project Overview

This project simulates an automated robotic car painting cell using RoboDK and Python. It was developed as part of the Yuktii Robotics Simulation Internship Assessment.

The simulation demonstrates an industrial robotic manipulator following predefined target waypoints to paint different sections of a car, with a rotary fixture changing the vehicle's orientation to provide access to additional surfaces.

## Project Details

- **Candidate Name:** Tanish Yella
- **College:** Amrita Vishwa Vidyapeetham, Bengaluru
- **Software Used:** RoboDK and Python
- **Robot Selected:** ABB IRB 5400-22
- **End Effector:** Generic Paint Sprayer

## Cell Components

- ABB IRB 5400-22 robotic manipulator
- Generic paint sprayer tool
- Car model
- Rotary fixture
- Painting cell reference frame

## Methodology

1. Set up the robotic painting cell in RoboDK.
2. Created target waypoints for different regions of the vehicle.
3. Used Python and the RoboDK API to execute the programmed movement sequences.
4. Coordinated robot movements with changes in the car's orientation using the rotary fixture.
5. Executed the complete painting sequence in simulation.

## Project Files

- `Yuktii_Full_Car_Painting_Execution_Final.py` — Python controller for executing the programmed simulation sequence.
- `Yuktii_Robotic_Car_Painting_Cell.rdk` — RoboDK station file containing the robotic car painting cell.

The complete simulation package, technical report, and demonstration video are provided through the Google Drive submission folder.

## Demonstration

Refer to the Google Drive submission folder for the complete demo video and technical report.

## Disclaimer

This project demonstrates a simulated robotic painting process. It does not represent a physical painting operation on a real vehicle.
