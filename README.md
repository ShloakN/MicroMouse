# MicroMouse
A small robotic mouse capable of solving unknown mazes<br>
The Mouse scans the maze start to center, returns to start with different path, and surge to goal with shortest path.

# Demonstraion 
*Maze Type: 01*
<video src="https://github.com/user-attachments/assets/e20a28f9-b85d-4e7b-a962-0b4cd90dfdb9"></video>


*Maze Type: 02*
<video src="https://github.com/user-attachments/assets/3b3188db-62cd-49ac-b5da-8a25b83ef70b"></video>


# Description
***The Maze***<br>
This project is intended to work on standard maze with diamensions of:
- Maze Size: 16 × 16 unit squares (Though code currently uses a smaller grid for testing) 
- Unit Square: Each square is 18 cm × 18 cm
- Wall Height: 5 cm
- Wall Thickness: 1.2 cm
- Passage Width: 16.8 cm between walls
- Start Position: Located at one of the four corners, with walls on three sides
- Goal Area: Center of the maze

***The Mouse***<br>
- Microcontroller: Mini Mega 2560 Pro Development Board
- Sensor: Six VL53L0X TOF sensors
- Motors: Two N20 motors with encoder

![MicroMouseRobot](https://github.com/user-attachments/assets/45c9f131-96a0-40b1-baa5-bc96bf52f410)

# Setup
The project uses Arduino IDE for the main code.<br>
External Packages Required:
- VL53L0X.h | by Pololu: https://github.com/pololu/vl53l0x-arduino
- PinChangeInterrupt.h | by NicoHood: https://github.com/NicoHood/PinChangeInterrupt
