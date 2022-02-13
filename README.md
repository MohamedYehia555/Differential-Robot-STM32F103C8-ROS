# Differential-Robot-STM32F103C8-ROS
Sending and recieving ROS odometry data between STM32F103 and Serial communication (UART) with ros using CH340 USB to TTL converter and displaying data on LCD

This repo uses minimum STM32 blue pill with at least 64kb flash. It also includes PID for correcting errors.

This repo was mainly an edit of ROS_bpbc repo to make it work with STM32F103, Cytron motor driver , removal of I2C converters and using multiple TOF sensors with UART for communication .

The Arduino code controls 2 encoders as inputs ,MDD10A Cytron motor driver , LCD for indication , VL53L0x TOF sensors and uses CH340 USB to TTL UART module to communicate with Nvidia Jetson Nano( I used my own computer and it worked , it can work with any SBC that has ubuntu with rosserial arduino setup done right)

Code is uploaded using ST-Link

Notes to take care of:
-Do not power the blue pill from a PCB and upload code to avoid damage

-Always use current limiting resistors when using an output

-Don't use more sourcing current than provided by the board to avoid damge

- If the code takes more than 128k flash use STM32F4 microcontrollers
