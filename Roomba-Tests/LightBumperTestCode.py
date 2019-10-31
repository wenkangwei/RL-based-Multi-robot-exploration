''' LightBumperTestCode.py
Purpose: Read sensor values from the Roomba
Last Modified: 6/11/2019
'''

## Import libraries ##
import serial
import time
import RPi.GPIO as GPIO
import RoombaCI_lib

## Variables and Constants ##
# LED pin numbers
yled = 5
rled = 6
gled = 13

## Functions and Definitions ##
''' Displays current date and time to the screen
	'''
def DisplayDateTime():
	# Month day, Year, Hour:Minute:Seconds
	date_time = time.strftime("%B %d, %Y, %H:%M:%S", time.gmtime())
	print("Program run: ", date_time)

## -- Code Starts Here -- ##
# Setup Code #
GPIO.setmode(GPIO.BCM) # Use BCM pin numbering for GPIO
DisplayDateTime() # Display current date and time

# LED Pin setup
GPIO.setup(yled, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(rled, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(gled, GPIO.OUT, initial=GPIO.LOW)

# Wake Up Roomba Sequence
GPIO.output(gled, GPIO.HIGH) # Turn on green LED to say we are alive
print(" Starting ROOMBA... ")
Roomba = RoombaCI_lib.Create_2("/dev/ttyS0", 115200)
Roomba.ddPin = 23 # Set Roomba dd pin number
GPIO.setup(Roomba.ddPin, GPIO.OUT, initial=GPIO.LOW)
Roomba.WakeUp(131) # Start up Roomba in Safe Mode
# 131 = Safe Mode; 132 = Full Mode (Be ready to catch it!)
Roomba.BlinkCleanLight() # Blink the Clean light on Roomba

if Roomba.Available() > 0: # If anything is in the Roomba receive buffer
	x = Roomba.DirectRead(Roomba.Available()) # Clear out Roomba boot-up info
	#print(x) # Include for debugging

print(" ROOMBA Setup Complete")
time1 = time.time()
# Main Code #
Roomba.StartQueryStream(45,7,17,52,53) #Begins querying for the light bumpers, regular bumpers, and IR sensors
while True:	
	try:
		
			#[Omni_IR,left_Omni,right_Omni] = Roomba.Query(17,52,53)
			#print ("Omni IR:{0}".format(Omni_IR))
			#print ("left_Omni:{0}".format(left_Omni))
			#print ("right_Omni:{0}".format(right_Omni))
			#[light_bumper,bumper,l_cliff,fl_cliff,fr_cliff,r_cliff,strl_cliff,strfl_cliff,strfr_cliff,strr_cliff] = Roomba.Query(45,7,9,10,11,12,28,29,30,31)
			#print ("Cliffs:{0}{1}{2}{3}".format(l_cliff,fl_cliff,fr_cliff,r_cliff))
		if Roomba.Available()>0:
			[light_bumper,bumper,Omni_IR,left_Omni,right_Omni]=Roomba.ReadQueryStream(45,7,17,52,53)
			
		time2 = time.time() #Sets up timer for data return
		if time2-time1 >.5: #Timer returns data every .5 seconds
			print ("{0:0>8b}".format(light_bumper)) #Prints light bumper output in binary, shows if any light bumpers are triggered
			print ("bumper:{0:0>8b}".format(bumper)) #Prints bumper output in binary, shows if any bumpers are triggered
			print ("Omni IR:{0}".format(Omni_IR)) #Prints omni IR sensor output
			print ("left_Omni:{0}".format(left_Omni)) #Prints left IR sensor output
			print ("right_Omni:{0}".format(right_Omni)) #Prints right IR sensor output
			time1 = time1+.5 #Makes time1 catch up with time 2, resetting the timer
	except KeyboardInterrupt: #Allows for input of "Ctrl+c" to end code without error
		break

Roomba.PauseQueryStream() #Ends querying of sensors
if Roomba.Available()>0:
	y = Roomba.DirectRead(Roomba.Available())
	print(y)

## -- Ending Code Starts Here -- ##

#Roomba.Dock() # Dock the Roomba (if you want)
# Make sure this code runs to end the program cleanly
Roomba.ShutDown() # Shutdown Roomba serial connection
GPIO.cleanup() # Reset GPIO pins for next program
