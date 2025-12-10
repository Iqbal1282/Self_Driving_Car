# Installation 
On Raspberry Pi OS (64 bit)

Step 1: Just for camera based self driving. 
```
sudo apt update  && sudo apt install -y python3-opencv python3-picamera2
pip install RPi.GPIO numpy
```

Step 2: Lidar on top of camera based self driving 

```
sudo apt install python3-serial
pip3 install rplidar-roboticia  # pure-python, zero-dep
```

# Hardwares 
1. Raspberry Pi 
2. Motor 
3. Servo 
4. Lidar 
5. Ultrasonic 
6. Battery 
7. Cables 


# DC and Servo Design 

Use the driver.py for the Car class 
```
          +-----------+
          |  FRONT    |
          |  servo    |
          +-----------+
          |           |
   (L)---------O---------(R)   ← single driven axle
          |    DC     |
          |  motor    |
          +-----------+
          |   Pi      |
          +-----------+

```

# DUAL DC Car design 

Use the driver_diff.py for the Car class

```

                FRONT
    ┌──────────────────────────────┐
    │   Pi-Cam (tilt 20°)          │
    │   ┌----------┐               │
    │   │  LiDAR   │  (optional)   │   8 cm
    │   │  (mast)  │               │
    │   └----------┘               │
    ├------------------------------┤
    │  ┌-----┐        ┌-----┐      │
    │  │  L  │        │  R  │      │   10 cm
    │  │Wheel│        │Wheel│      │
    │  └--┬--┘        └--┬--┘      │
    │   Motor          Motor       │
    │   (left)         (right)     │
    ├------------------------------┤
    │  L298N module                │
    │   ENB│IN3│IN4  ENA│IN1│IN2    │   6 cm
    ├------------------------------┤
    │  7.4 V Battery               │   4 cm
    └------------------------------┘
       12 cm
```

# Software Level Design (lidar, sonar, camera)

Use autopilo_with_sonar.py for using these 3 sensors. 

```
┌-------------┐     ┌-------------┐     ┌--------------┐
│  Camera CNN │     │   LiDAR     │     │  3×Sonar     │
│ lane + sign │     │ 360° scan   │     │ <40 cm       │
└-----┬-------┘     └-----┬-------┘     └-----┬--------┘
      │                   │                   │
      └---------┬---------┴---------┬---------┘
                │   sensor fusion   │
                ▼                   ▼
         “front_clear” (m)    “side_gap” (m)
                │                   │
         ┌------┴-------------------┴------┐
         │  state machine (20 Hz)         │
         │  CRUISE / STOP_LIGHT /         │
         │  STOP_OBSTACLE / CURB          │
         └------┬------------------┬------┘
                ▼                  ▼
         car.drive(throttle)   car.steer(angle)
```


