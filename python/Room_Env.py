import RoombaCI_lib
import time
import RPi.GPIO as GPIO
import os.path
import math
import numpy as np


# Notes for Roomba settings:
# 1. analog Light bumper can detect objects around 0.5 m awary from the robot. The range of strength is abbour from 5~3050
# i don't see the values are higher than 3100 or even 4095 (maximum value)
# so when normalizing the data, using 3100 is fine
# 2. the motion of robot has drifting problem when using Move() setting both wheel has same speed. If set speed 100mm/s,
# in 1 s to move 100mm, there is 5~10mm drifting distance. But count is good
# 3. x axis is the heading axis of robot. y axis is the axis orthogonal to the heading of robot. positive motion: +x
# positive rotation: clockwise
# 4. Test files in Roomba are good to achieve data
# 5. using Roomba 6

def DisplayDateTime():
	# Month day, Year, Hour:Minute:Seconds
	date_time = time.strftime("%B %d, %Y, %H:%M:%S", time.gmtime())
	print("Program run: ", date_time)

''' Queue that allows for elements with a higher priority (lower priority value) to be put closer to the front of the line.
	Created using documentation from https://www.redblobgames.com/pathfinding/a-star/implementation.html
	'''


# LED pin numbers
yled = 5
rled = 6
gled = 13




class MotorEncoder():
    """
    Class used to store and calcuate information of movement
    based on Left Right encoders of motors
    """
    def __init__(self, L_init_cnt=None,R_init_cnt=None):
        if L_init_cnt ==None:
            self.L_cur_cnt =0
            self.L_past_cnt=0
        else:
            self.L_cur_cnt = L_init_cnt
            self.L_past_cnt = L_init_cnt

        if R_init_cnt ==None:
            self.R_cur_cnt = 0
            self.R_past_cnt = 0
        else:
            self.R_cur_cnt = R_init_cnt
            self.R_past_cnt = R_init_cnt

        # delta distance after each movement
        self.delta_d= 0
        # change of angle afer a movement
        self.delta_agl = 0
        # angle difference tolerance
        self.agl_tol = 0
        # current position obtained by dead reckoning
        self.x= 0
        self.y = 0
        # Robot Heading angles
        self.theta = 0
        self.old_theta = 0
        # Constant settings of Robot
        self.wheel_diameter = 72
        self.counts_per_rev = 508.8
        self.distance_between_wheels = 235
        self.C_theta = (self.wheel_diameter * math.pi) / (self.counts_per_rev * self.distance_between_wheels)
        self.distance_per_count = (self.wheel_diameter * math.pi) / self.counts_per_rev
        pass

    def deltaCnt(self,new_cnt, past_cnt):
        """

        :param new_cnt:
        :param past_cnt:
        :return: delta
        """
        delta = new_cnt - past_cnt
        if delta < -1 * (
                2 ** 15):  # Checks if the encoder values have rolled over, and if so, subtracts/adds accordingly to assure normal delta values
            delta += (2 ** 16)
        elif delta > (2 ** 15):
            delta -= (2 ** 16)
        old_cnt = new_cnt
        return delta, old_cnt

    def cnt2Agl_Dist(self, L_cnt=None,R_cnt=None):
        if L_cnt ==None:
            L_cnt =self.L_cur_cnt

        if R_cnt == None:
            R_cnt = self.R_cur_cnt

        L_del_cnt  = self.deltaCnt(L_cnt,self.L_past_cnt)
        R_del_cnt = self.deltaCnt(R_cnt, self.R_past_cnt)

        # update heading angle theta
        self.old_theta= self.theta
        delta_theta = (L_del_cnt - R_del_cnt) * self.C_theta
        self.theta +=delta_theta
        if self.theta >= 2 * math.pi:
            self.theta -= 2 * math.pi
        elif self.theta < 0:
            self.theta += 2 * math.pi

        # update distance
        if L_del_cnt - R_del_cnt <=self.agl_tol:
            delta_d = 0.5 * (R_del_cnt + L_del_cnt) * self.distance_per_count
        else:
            delta_d = 2 * (235 * (L_del_cnt / (L_del_cnt - R_del_cnt) - .5)) * math.sin(delta_theta / 2)

        self.delta_d =delta_d
        self.delta_agl =delta_theta

        return self.theta, self.delta_agl, self.delta_d

    def deg2ArcLen(self,degree):
        """
        Convert degree to arc length in mm to rotate
        :param degree:
        :return: degree*(2pi/360) * radius
        """
        return (self.distance_between_wheels/2)* degree*2*math.pi/360
    def get_CurPos(self, L_enc_cnt, R_enc_cnt):
        """
        Dead reckoning for computing current location: x, y, theta
        :return:
        """
        # update counts
        self.L_past_cnt =self.L_cur_cnt
        self.R_past_cnt =self.R_cur_cnt
        self.L_cur_cnt = L_enc_cnt
        self.R_cur_cnt = R_enc_cnt
        theta,del_agl,d = self.cnt2Agl_Dist(L_enc_cnt,R_enc_cnt)
        self.x += self.x + d*math.cos(theta-0.5*del_agl)
        self.y += self.y + d * math.sin(theta - 0.5 * del_agl)
        return self.x, self.y, self.theta



class LighBumper():
    """
    This class is used to store the information of digit light bumper and analog light bumper
    """
    def __init__(self):
        self.bump_mode = False  # Used to tell whether or not the roomba has bumped into something and is supposed to be "tracking"
        self.bump_code = 0  # Used to distinguish if the right, left, or center bumpers are being triggered
        self.bump_count = 0  # Keeps track of how many times the bumper has detected a bump
        self.bump_time = time.time() - 2.0  # Assures that the roomba doesn't start in backup mode
        pass
    def digitBump(self):
        pass
    def analogBump(self):
        pass



class GridWorld(object):
    def __init__(self,path):
        # Parameters
        self.Roomba = RoombaCI_lib.Create_2("/dev/ttyS0", 115200)

        #  #parameters used for Q-learning
        # state: (x,y, theta), theta: heading angles
        self.state = [0.0,0.0,0.0]
        # actions: [d,theta1]: +/- d distance to move, theta1: change of heading angle
        self.action = [0.0,0.0]
        # if use Q-learning no need for possibility model
        self.trans_model = []


        # Variables used for recording data
        self.Roomba.ddPin = 23  # Set Roomba dd pin number
        self.backup_time = 1.0  # Amount of time spent backing up
        self.corner_time = 1.5  # Amount of time that it takes before the roomba starts turning more sharply (makes sure it turns around corners)
        self.data_time = time.time()



        # Setup Code #
        GPIO.setmode(GPIO.BCM)  # Use BCM pin numbering for GPIO
        DisplayDateTime()  # Display current date and time
        # LED Pin setup
        GPIO.setup(yled, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(rled, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(gled, GPIO.OUT, initial=GPIO.LOW)
        # Wake Up Roomba Sequence
        GPIO.output(gled, GPIO.HIGH)  # Turn on green LED to say we are alive
        print(" Starting ROOMBA... ")
        GPIO.setup(self.Roomba.ddPin, GPIO.OUT, initial=GPIO.LOW)
        self.Roomba.WakeUp(131)  # Start up Roomba in Safe Mode
        # 131 = Safe Mode; 132 = Full Mode (Be ready to catch it!)
        self.Roomba.BlinkCleanLight()  # Blink the Clean light on Roomba
        if self.Roomba.Available() > 0:  # If anything is in the Roomba receive buffer
            x = self.Roomba.DirectRead(self.Roomba.Available())  # Clear out Roomba boot-up info
        print(" ROOMBA Setup Complete")
        GPIO.output(gled, GPIO.LOW)


        # Open a text file for data retrieval
        file_name_input = input("Name for data file: ")
        dir_path = path  # Directory path to save file
        file_name = os.path.join(dir_path, file_name_input + ".txt")  # text file extension
        self.file = open(file_name, "w")  # Open a text file for storing data


        # Start achieving data
        self.start_time = time.time()
        [left_start, right_start] = self.Roomba.Query(43, 44)
        self.Motion = MotorEncoder(left_start, right_start)

        pass

    def check_Obs(self):
        """
        Check obstacles around current position by rotating robot 1 round
        :return:
             a list of angle and distance from current position to detected obstacles
        """
        pass
    def start(self):
        # start running Roomba and achieve data
        self.Roomba.StartQueryStream(7, 43, 44, 45)  # Start getting bumper values


    def achieve_data(self, selection='be'):
        """
        achieve encoder, light bumper data from robot
        7: bump/wheel drop
        43: Left encoder counts
        44: right encoder counts
        45: digital light bumper, binary data 0/1 indicate bump or not
        46~51: Analog light bumper signal strength
                L, Front Left, Center Left, Center Right, Front Right, Right

        """
        bump, L_cnt, R_cnt, DLightBump, AnalogBump = None,None,None,None,None
        if selection == 'b':
            bump,DLightBump, L, FL, CL, CR, FR, R = self.Roomba.ReadQueryStream(7,45, 46, 47,48, 49, 50, 51)
            AnalogBump = (L, FL, CL, CR, FR, R)
        elif selection == 'e':
            L_cnt, R_cnt= self.Roomba.ReadQueryStream( 43, 44)
        else:
            bump, L_cnt,R_cnt, DLightBump, L,FL, CL,CR,FR,R =self.Roomba.ReadQueryStream(7, 43, 44, 45, 46,47,48,49,50,51 )
            AnalogBump = (L,FL, CL,CR,FR,R)
        return  L_cnt,R_cnt, bump, DLightBump,AnalogBump

    def stop(self):
        # Stop roomba and exploration task
        self.Roomba.PauseQueryStream()
        if self.Roomba.Available() > 0:
            z = self.Roomba.DirectRead(self.Roomba.Available())
            print(z)
        time.sleep(0.1)
        self.file.close()  # Close data file
        ## -- Ending Code Starts Here -- ##
        self.Roomba.ShutDown()  # Shutdown Roomba serial connection
        GPIO.cleanup()  # Reset GPIO pins for next program
        pass
    def get_reward(self,bump,DLightBump,AnalogBump):
        """
        Compute reward based on data from bumper, digital light bumper and analog light bumper
        :param bump:
        :param DLightBump:
        :param AnalogBump:
        :return:
        """
        # Put calculation here
        return None

    def get_StateReward(self):
        """
        Update current state after achieving data
        :return:
        """
        L_cnt, R_cnt,bump,DLightBump, AnalogBump = self.achieve_data()

        # obtain postion and heading angle
        self.state[0],self.state[1],self.state[2] = self.Motion.get_CurPos(L_cnt,R_cnt)

        #obtain current reward by converting raw data to computed rewards
        # The reward is the reward obtained after transition (s,a,s')
        r = self.get_reward(bump,DLightBump,AnalogBump)
        return self.state,r

    def step(self,a):
        """
        Move robot to expected position and track the current position and reward
        :param a: action of Roomba. It is expected to contain rotation angles and +/- moving distance
         +: move forward  -: move backforward
        :return:
        """
        # change of distance in mm
        d= a[0]
        # heading angle in degree
        d_theta = a[1]
        s_old = self.state
        # take action
        # moving speed 100mm/s
        sp = 100.0
        t = d/sp
        init_t = time.time()
        cur_t  = init_t
        tol = 1e-4
        ArcLen = self.Motion.deg2ArcLen(d_theta)
        # using time delay method to reach desired position
        while np.abs(cur_t-init_t)< tol+np.abs(ArcLen/50.0):
            self.Roomba.Move(0,50*d_theta/np.abs(d_theta))
            cur_t = time.time()

        init_t = time.time()
        cur_t = init_t
        while np.abs(cur_t-init_t)< tol+t:
            self.Roomba.Move(100,0)
            cur_t = time.time()

        # using feedback to reach desired position

        # obtain new state and reward
        state,r = self.get_StateReward()


        pass

    def cal_reward(self,s=None,a=None,s_=None):
        """
        Compute the reward after transition from s to s_ after taking action a
        :param s:
        :param a:
        :return:
        """
        pass
