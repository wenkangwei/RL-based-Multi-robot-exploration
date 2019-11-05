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
    def __init__(self, L_init_cnt=0,R_init_cnt=0):
        # ------Parameters-----------------
        # Notes:
        # all angles are in rad unit, not degree unless convert it to degree
        # record  encoder counts
        self.L_cur_cnt = L_init_cnt
        self.L_past_cnt = L_init_cnt
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

        # -----------Constant settings of Robot----------
        self.wheel_diameter = 72
        self.counts_per_rev = 508.8
        self.distance_between_wheels = 235
        self.C_theta = (self.wheel_diameter * math.pi) / (self.counts_per_rev * self.distance_between_wheels)
        self.distance_per_count = (self.wheel_diameter * math.pi) / self.counts_per_rev
        pass

    def reset(self, L_init_cnt=0,R_init_cnt = 0):
        self.delta_d = 0
        # change of angle afer a movement
        self.delta_agl = 0
        # angle difference tolerance
        self.agl_tol = 0
        # current position obtained by dead reckoning
        self.x = 0
        self.y = 0
        # Robot Heading angles
        self.theta = 0
        self.old_theta = 0
        self.L_cur_cnt = L_init_cnt
        self.R_cur_cnt = L_init_cnt
        self.L_past_cnt = R_init_cnt
        self.R_past_cnt = R_init_cnt

    def deltaCnt(self,new_cnt, past_cnt):
        """
        Calculate difference between current encoder data
        and the last encoder data and check if values roll over
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
        """
        convert counts to rotated angle
        :param L_cnt:
        :param R_cnt:
        :return:
        """
        if L_cnt ==None:
            L_cnt =self.L_cur_cnt

        if R_cnt == None:
            R_cnt = self.R_cur_cnt

        L_del_cnt,self.L_past_cnt  = self.deltaCnt(L_cnt,self.L_past_cnt)
        R_del_cnt,self.R_past_cnt = self.deltaCnt(R_cnt, self.R_past_cnt)

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
            self.delta_d = 0.5 * (R_del_cnt + L_del_cnt) * self.distance_per_count
        else:
            self.delta_d = 2 * (235 * (L_del_cnt / (L_del_cnt - R_del_cnt) - .5)) * math.sin(delta_theta / 2)

        self.delta_agl =delta_theta

        return self.theta, self.delta_agl, self.delta_d

    def Agl2ArcLen(self,agl):
        """
        Convert angle to arc length in mm to rotate
        :param angle:
        :return: angle(rad) * radius
        """

        return (self.distance_between_wheels/2)*agl

    def get_CurPos(self, L_enc_cnt, R_enc_cnt):
        """
        Dead reckoning for computing current location: x, y, theta
        :return:
        """
        # update current counts
        self.L_past_cnt =self.L_cur_cnt
        self.R_past_cnt =self.R_cur_cnt
        self.L_cur_cnt = L_enc_cnt
        self.R_cur_cnt = R_enc_cnt
        # Compute distance, angle moved since last update
        theta,del_agl,d = self.cnt2Agl_Dist(L_enc_cnt,R_enc_cnt)
        # update current counts
        self.L_past_cnt = self.L_cur_cnt
        self.R_past_cnt = self.R_cur_cnt
        # Update current position
        self.x = self.x + d*math.cos(theta-0.5*del_agl)
        self.y = self.y + d * math.sin(theta - 0.5 * del_agl)
        self.x = round(self.x, 3)
        self.y = round(self.y, 3)
        self.theta =round(self.theta,3)
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

    def reset(self):
        self.bump_mode = False  # Used to tell whether or not the roomba has bumped into something and is supposed to be "tracking"
        self.bump_code = 0  # Used to distinguish if the right, left, or center bumpers are being triggered
        self.bump_count = 0  # Keeps track of how many times the bumper has detected a bump



class GridWorld(object):
    def __init__(self,file_name=None,real_state=None,world_w=50, world_h=50):
        # --------Parameters---------
        # -------parameters for defining grid world------
        # State space, define grid world here
        # each grid is 800mmx800 mm
        self.grid_size = 200
        self.observation_space = None
        # Action space:
        self.action_space = None
        # if use Q-learning no need for possibility model
        self.trans_model = None
        # reward table: it is to define the updated reward of the explored area
        self.reward_tb = {'terminal': -1, 'path': 0, 'goal': 1}
        # a list of obstacles detected
        self.obs_ls = []

        #------Variables used for recording data----------
        self.Roomba = RoombaCI_lib.Create_2("/dev/ttyS0", 115200)
        self.Roomba.ddPin = 23  # Set Roomba dd pin number
        self.backup_time = 0.5  # Amount of time spent backing up
        self.corner_time = 1.5  # Amount of time that it takes before the roomba starts turning more sharply (makes sure it turns around corners)
        self.data_time = time.time()


        # Assume distance to obstacle and signal strength are proportional
        # 10mm return 3000
        self.LBump_ratio = 3000.0/10.0
        # maximum light bumper signal strength
        self.max_strength =3200.0
        # moving speed 100mm/s, rotate 50mm/s
        self.sp = 50
        self.rot_sp = 50



        # parameters used for Q-learning
        # Notes: Q-learning will be applied with grid world states, not real continuous states
        #Initial position Current real continuous state: (x,y, theta), theta: heading angles
        if real_state == None:
            real_state= [0.0,0.0,0.0]

        self.real_state = real_state
        #Initial position discrete state in grid world
        self.grid_state = self.get_gridState(real_state)
        # Current actions: [d,theta1]: d distance to move forward, theta1: change of heading angle
        self.action = [0.0,0.0]

        # Initialize action space, state space
        self.spaces_init(world_w,world_h)

        # initialize GPIO
        self.GPIO_init()
        # Open a text file for data retrieval
        if file_name is None:
            dir_path = '../Data/'  # Directory path to save file
            file_name_input = 'data' #input("Name for data file: ")
            file_name = os.path.join(dir_path, file_name_input + ".txt")  # text file extension
        self.file = open(file_name, "w")  # Open a text file for storing data

        # Start achieving data
        self.start_time = time.time()
        [left_start, right_start] = self.Roomba.Query(43, 44)
        self.Motion = MotorEncoder(left_start, right_start)
        # Initialize Bumper
        self.bumper= LighBumper()
        pass

    def GPIO_init(self):
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
        pass

    def spaces_init(self, world_r=50,world_c=50):
        # Initialize Grid world observation space
        # defaule 50m * 50m. num of grids are 50*1000/grid_size
        r= int(world_r*1000/self.grid_size)
        c= int(world_c*1000/self.grid_size)
        self.observation_map= np.ones([r,c])

        # Initialize Action Space
        # d: distance to move
        # Note: d>0: move forward along heading direction. d<0: move backward
        d = self.grid_size
        # Angle set: each num is in degree
        angle_set =[0,45,90,135,180,-45,-90,-135,-180]
        self.action_space=[]
        for theta in angle_set:
            self.action_space.append((d,theta))
        pass

    def achieve_data(self, selection='all'):
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
            # read bumper data only
            bump,DLightBump, L, FL, CL, CR, FR, R = self.Roomba.ReadQueryStream(7,45, 46, 47,48, 49, 50, 51)
            AnalogBump = (L, FL, CL, CR, FR, R)
        elif selection == 'e':
            # read encoder data only
            L_cnt, R_cnt= self.Roomba.ReadQueryStream( 43, 44)
        else:
            # read all data
            bump, L_cnt,R_cnt, DLightBump, L,FL, CL,CR,FR,R =self.Roomba.ReadQueryStream(7, 43, 44, 45, 46,47,48,49,50,51 )
            AnalogBump = (L,FL, CL,CR,FR,R)

        return  L_cnt,R_cnt, bump, DLightBump,AnalogBump

    def terminate(self):
        """
        Stop roomba and clean data after exploration is finished
        :return:
        """
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

    def reset(self):
        """
        This function is to clean env data and reset environment
        """
        # reset Wheel encoders
        self.start_time = time.time()
        [left_start, right_start] = self.Roomba.Query(43, 44)
        self.Motion.reset(left_start, right_start)
        # reset bumper
        self.bumper.reset()

        #reset grid world data
        self.action=[0.0,0.0]
        self.grid_state= [0,0,0]
        self.real_state = [0.0, 0.0, 0.0]
        self.trans_model = None
        pass

    def get_gridState(self,real_state=None):
        """
        Convert continuous position, heading angle theta to discrete int position and int theta
        :param real_state:
        :return:
        discrete int state values:x,y, theta
        """
        if real_state is None:
            r_state =self.real_state
        else:
            r_state = real_state

        grid_state = [0,0,0]
        grid_state[0],grid_state[1],grid_state[2] = int(r_state[0]//self.grid_size), int(r_state[1]//self.grid_size),int(r_state[2])
        return grid_state

    def cal_reward(self,bump,DLightBump,AnalogBump):
        """
        Strategy to compute reward based on data from bumper, digital light bumper and analog light bumper
        Use Gaussian distribution to update reward
        :param bump:
        :param DLightBump:
        :param AnalogBump:
        :return:
        """
        #
        # Put calculation here
        r= float(self.observation_map[self.grid_state[0],self.grid_state[1]])

        # map reward:
        # Will apply Gaussian mixture to approximate , update reward
        # by using obstacle data here
        r1 = float(self.observation_map[self.grid_state[0],self.grid_state[1]])
        # r1= Gaussian Mixture reward on map

        # immediate reward from sensors
        k=-1    #k: scale of reward/penalty
        r2 = k*(np.mean(AnalogBump)/self.max_strength)
        # Total reward/penalty
        r_new = (r1+r2)
        # update map reward at this position
        self.observation_map[self.grid_state[0], self.grid_state[1]] = r_new
        return r


    def check_terminal(self,bump,DLightBump, AnalogBump):
        """
        Strategy to determine if current state is terminal
        :param bump:
        :param DLightBump:
        :param AnalogBump:
        :return:
            terminal: flag indicating if state is terminal
            obstacles: a list of obstacles detected
        """
        terminal = False
        # signal returned from distance to obstacle /terminal 50 mm,5cm
        d_obs = 700
        threshold = d_obs/self.max_strength
        obstacles = []

        # using digital light bumper
        # if DLightBump !=0:
        #     terminal = True

        L, FL, CL, CR, FR, R = AnalogBump
        # using analog light bumper
        prob_obs =np.array([L, FL, CL, CR, FR, R]).astype(float)
        # prob_obs = np.convolve(prob_obs, (0.1,0.8,0.1))[1:-2]
        strength = prob_obs/self.max_strength  # maximum signal strength light bumper can receive
        for i in range(len(strength)):
            strength[i] = 1 if strength[i] >=threshold else 0

        cnt = strength.sum()
        print('bump: {0:0>8b}:'.format(bump))
        if bump != 0 or cnt >=2:
            # May need reset the position of roomba to previous position using  grid world position (center of last grid)
            # since roomba may drift after hitting obstacle and the data will be incorrect
            terminal=True
            # stop immediately
            self.Roomba.Move(0,0)

            #-------------determine position of obstacles-------------
            l_bump = 1 if bump&2 !=0 else 0
            r_bump = 1 if bump& 1 !=0 else 0
            # Assume Left , right bumpers are at -45 degree, 45 degree
            # Then find the average degree of object
            b_avg_angle = 45*(r_bump -l_bump)
            prob_obs /= prob_obs.sum()
            # average angles of obstacle detected by light bumper
            # [-90, -60,-30,30,60,90] are heading angles of 6 analog light bumper
            lb_avg_agl = np.dot(prob_obs,[-90, -60,-30,30,60,90])

            # check if there are 2 obstacles or 1 obstacle
            if np.abs(lb_avg_agl - b_avg_angle)>=60 or (np.sign(lb_avg_agl) !=np.sign(b_avg_angle)):
                th = self.Motion.theta + 0.5 * lb_avg_agl
                x = self.Motion.x + d_obs * math.cos(th)
                y = self.Motion.y + d_obs * math.sin(th)
                s = self.get_gridState(real_state=[x, y, th])
                obstacles.append(s[0:1])
                th = self.Motion.theta + 0.5 * b_avg_angle
                x = self.Motion.x + d_obs * math.cos(th)
                y = self.Motion.y + d_obs * math.sin(th)
                # convert real continuous state to discrete grid world state
                s = self.get_gridState(real_state=[x, y, th])
                obstacles.append(s[0:1])
            else:
                alg = (b_avg_angle+lb_avg_agl)/2.0
                th= self.Motion.theta+0.5 * alg
                x = self.Motion.x + d_obs * math.cos(th)
                y = self.Motion.y + d_obs * math.sin(th)
                s= self.get_gridState(real_state=[x,y,th])
                obstacles.append(s[0:1])

        return terminal, obstacles

    def observe_Env(self, mode='all'):
        """
        Update current continous real world state and the reward at the new state after achieving data
        mode == 'e': return encoder info only
        otherwise, return all info
        :return:
        old continuous state, new continuous state, reward,flag of terminal
        """
        L_cnt, R_cnt, bump,DLightBump, AnalogBump = self.achieve_data(mode)
        old_state = self.real_state.copy()

        if mode != 'e':

            # Check if current state is terminal
            terminal,obs = self.check_terminal(bump,DLightBump, AnalogBump)
            # update list of obstacles
            if len(obs)>0:
                self.obs_ls.extend(obs)
            # The reward is the reward obtained after transition (s,a,s')
            r = self.cal_reward(bump, DLightBump, AnalogBump)
        else:
            # if encoder mode, return encoder info only, without calculate rewards and terminals
            r= 0
            terminal =False

        # obtain postion and heading angle
        self.real_state[0],self.real_state[1],self.real_state[2] = self.Motion.get_CurPos(L_cnt,R_cnt)

        return old_state, self.real_state,r, terminal, (L_cnt, R_cnt, bump,DLightBump, AnalogBump)

    def step(self,a):
        """
        Move robot to expected position and track the current position and reward
        :param a: action of Roomba. It is expected to contain rotation angles and moving distance
        :return:
        s_new: s' new grid world state
        r: reward of new state or R(s,a,s')
        is_terminal: flag to check s is terminal. s is state before taking action
        """
        # change of distance in mm
        d= a[0]
        # convert change of heading angle d_theta from degree to rad, from angle to Arc Length
        d_theta = a[1]*(math.pi/180.0)
        ArcLen = self.Motion.Agl2ArcLen(d_theta)
        # Compute period of motion

        init_t = time.time()
        cur_t = init_t
        # tolerance of time difference
        tol = 1e-4
        # back up current state s
        s_old = self.grid_state
        s_new = s_old
        old_real_state, new_real_state, r, is_terminal = 0,0,0,False
        # track sensor information when moving
        self.Roomba.StartQueryStream(7, 43, 44, 45, 46, 47, 48, 49, 50, 51)  # Start getting bumper values

        # determine if s is terminal before taking action
        # if it is terminal, don't move and return state directly
        print("Observe Environment...")
        _, _, r, is_terminal = self.observe_Env()
        if is_terminal:
            return s_new, r, is_terminal

        # Take action if current state is not terminal

        # using time delay method to reach desired position
        # Rotate Roomba to certain degree
        sign = 1 if d_theta >= 0 else -1
        print("Spinning: ")
        # while self.Roomba.Available()<0:
        #     pass
        self.Roomba.Move(0, self.rot_sp* sign)
        t=cur_t
        while np.abs(cur_t-init_t)< tol+np.abs(ArcLen/self.rot_sp):
            if np.abs(cur_t-t)>= self.backup_time:
                t= cur_t
                print('new state: {:10.2f},{:10.2f},{:10.2f}. r:{:10.2f}, terminal:{}'.format(
                    new_real_state[0], new_real_state[1], new_real_state[2], r, is_terminal))

            if self.Roomba.Available()>0:
                # keep track of postion and check if at terminal state, like hitting wall or obstacle
                old_real_state, new_real_state, r, is_terminal= self.observe_Env(mode='e')
            cur_t = time.time()
        print("Spinning t:", np.abs(cur_t-init_t))
        print('cur s:', new_real_state)
        # Pause roomba for a while
        self.Roomba.Move(0, 0)
        time.sleep(0.5)

        # reset time
        init_t = time.time()
        cur_t = init_t
        #Roomba moves forward
        print("Moving forward. . .")
        self.Roomba.Move(self.sp, 0)
        t =cur_t
        while np.abs(cur_t-init_t)< tol+ (d/self.sp):
            # check obstacle and terminal state
            if np.abs(cur_t-t)>= self.backup_time:
                t =cur_t
                print('new state: {:10.2f},{:10.2f},{:10.2f}. r:{:10.2f}, terminal:{}'.format(
                    new_real_state[0],new_real_state[1],new_real_state[2], r, is_terminal))

            if self.Roomba.Available()>0:
                # keep track of postion and check if at terminal state, like hitting wall or obstacle
                old_real_state, new_real_state, r, is_terminal= self.observe_Env()
                if is_terminal:
                    break
            cur_t = time.time()
        # pause roomba after reaching desired position
        self.Roomba.Move(0, 0)
        print("forward t:", np.abs(cur_t - init_t))

        # record real trajectory here
        ##############################

        ##############################


        # Compute reward and new state after the motion
        s_new = self.get_gridState(new_real_state)
        self.grid_state = s_new
        print("grid s:", s_new)

        #May use alternative method : using feedback to reach desired position
        ########################################
        ########################################

        self.Roomba.PauseQueryStream()
        if self.Roomba.Available() > 0:
            z = self.Roomba.DirectRead(self.Roomba.Available())
            print(z)


        # update Gaussian Mixture model for reward approximation
        ########################################
        ########################################
        return s_new, r, is_terminal

