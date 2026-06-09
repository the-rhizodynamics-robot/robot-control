# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Repository Overview

This is a robot control system for an automated plant imaging platform. The system consists of Arduino firmware for robotic camera movement and Python control software for monitoring and coordination.

## Hardware Architecture

The robot uses a 2-axis stepper motor system:
- **Horizontal motor**: Controls left/right camera movement across shelves
- **Vertical motor**: Controls up/down movement between shelves  
- **Photointerrupters**: Two sensors (horizontal and vertical) for position calibration
- **Camera trigger**: Pin 7 controls camera capture
- **LED/shelf control**: Individual relay control for each shelf's lighting
- **Serial communication**: USB connection between Arduino and Python controller

## Code Structure

### Arduino Code (`arduino/`)
- `robot_device/robot_device.ino`: Main robot control firmware (the working / prod sketch)
- `photointerrupter_testing/photointerrupter_testing.ino`: Sensor testing utility

### Python Control (`python_runner/`)
- `robot_runner.ipynb`: Main control notebook that coordinates robot operations, monitors image capture, and handles email notifications

## Key Configuration

### Arduino Constants
- `KILLCODE = 2048`: Emergency stop signal (must match Python)
- Motor pins: X-axis step/dir/enable (5/7/6), Y-axis step/dir/enable (8/10/9); enable active-LOW
- Sensor pins: D3 (horizontal), D2 (vertical), INPUT_PULLUP, trigger reads LOW
- Camera trigger: D12, ~50ms HIGH pulse (a shorter pulse is ignored by the camera)
- Shelf relays: Pins 23,25,27,29,31,33 (active-LOW)
- Run profile: 3 shelves x 8 photos, snake path, 15-min cycle, 15h day / 9h night

### Python Configuration
- COM port and robot ID configuration
- Image directory management
- Email notification system for monitoring
- Cycle timing and image count validation

## Development Workflow

### Arduino Development
1. Use Arduino IDE to compile and upload sketches
2. Test individual components using the testing scripts before full deployment
3. Ensure serial communication settings match Python side (9600 baud, 2ms timeout)

### Python Development  
1. The main control is in Jupyter notebook format
2. Modify robot configuration variables at the top of the notebook
3. Serial communication parameters must match Arduino settings
4. Email credentials required for monitoring notifications

## Critical Safety Features

- **Kill code system**: Python can send KILLCODE to immediately stop robot
- **Timeout monitoring**: Robot stops if cycle takes longer than expected
- **Home position calibration**: Uses photointerrupters to establish consistent starting position
- **Image count validation**: Monitors expected vs actual image capture

## Serial Communication Protocol

The Arduino waits for specific values from Python during startup:
1. Number of shelves
2. Light cycle length (hours)
3. Current hour in day cycle  
4. Start signal

During operation, Arduino sends "home" when cycle complete.

## Testing Components

Use the testing scripts to verify hardware before running full system:
- `photointerrupter_testing.ino`: Verify sensor functionality
- `robot_device_testing.ino`: Test motor movement and control