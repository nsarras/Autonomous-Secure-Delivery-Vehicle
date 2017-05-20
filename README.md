Read Me

Project Source Code included within package and can also be found at. 
GitHub Link: https://github.com/nsarras/Autonomous-Secure-Delivery-Vehicle

The following document contains instructions on running the ASD Bot. 

Everything within the robot is already assembled. Only 12 AA batteries are required to power the robot. 

Project Contents & Components:
	-SJOne Board
	-Pixy Camera
	-4 Servo Motors
	-2 H Bridges
	-4 Tires
	-1 Custom Steel Chassis
	-2 IR Sensors

Running Project

Once the robot powers on, teach it an object through the pixy camera. Hold down the pixy camera button for 3 seconds. Wait for the lights to start flashing. Show it an object you would like for it to memorize. Click the button once more when you see that it recognizes the object. The code is already loaded onto the development board. Place something in the delivery compartment. Move the object to a location. The robot will find the object will the algorithm loaded onto the board. Once the robot reaches the destination it will park itself. 

If you would like to change the source code, load the program onto SJOne Board using Hyperload program. Use Hercules program to view output of application code.
The development package can be found at http://www.socialledge.com/sjsu/index.php?title=Development_Package

Any questions regarding the SJOne board and its functionality can be found at http://www.socialledge.com/sjsu/index.php?title=Development_Package

Any questions regarding the functionality of the Pixy Camera and its implementation can be found at http://cmucam.org/projects/cmucam5

Code Structure
	- Split into 2 main tasks
		-Pixy Task: Reads in input from Pixy Camera and sends bytes of information to the SJOne board using UART communication.
		-IR Task: Reads in input from IR sensor as voltages and sends signals to SJOne board.
	-Both tasks are called and run in the main with equal priority to enable round table context switching. 

This is a FREE Real Time Operating System that is being used for real time applications. (FREE RTOS)


