import RoombaCI_lib
import time
import RPi.GPIO as GPIO
import os.path
import math
import numpy as np
import  json
import serial


# Problem so far:
# 1. Q learning parameter inf or Nan, data is not valid
# 2. unable to receive data correctly through json data
# 3. position/state tracking data weird, after using unit divider=10.0 converting mm to cm






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



class Xbee():
    def __init__(self,id=1, f_len=7 ):
        self.id =id
        self.ctrl = serial.Serial('/dev/ttyUSB0', 115200)  # Baud rate should be 115200
        self.sendtime = time.time()
        self.sendtime_offset = 1.0
        self.basetime = time.time()
        self.basetime_offset = 0.5
        self.data =''
        self.agent_info={}
        t_step = 0
        st = [0.0, 0.0, 0.0]
        st1 = [0.0, 0.0, 0.0]
        a = [0,0]
        p = [0.0]*f_len
        self.degree = 0
        self.agent_info[id] = (t_step,a,st,st1,p,self.degree)

        pass

    def send(self, t,transition=None,params =None):
        # transition: (a, s, s')
        #   '#' as split between different packet

        if params == None and transition!= None:
            a= transition[0]
            st = np.round(transition[1],5)
            st1 =np.round(transition[2],5)
            p= self.agent_info[self.id][4]
            self.agent_info[self.id] = (t, a, st, st1, p,self.degree)
            data= {"id":self.id,"t":t,"d":self.degree,"e":transition}
        elif params != None and transition== None:
            params = np.round(params,5).tolist()
            t=self.agent_info[self.id][0]
            a = self.agent_info[self.id][1]
            st = np.round(self.agent_info[self.id][2],5)
            st1 = np.round(self.agent_info[self.id][3],5)
            self.agent_info[self.id] = (t, a, st, st1, params,self.degree)
            data = {"id": self.id, "t": t,"d":self.degree, "p": params}
        else:
            print("Data must be transition or params")
            return
        message = json.dumps(data)+'#'
        self.ctrl.write(message.encode())
        self.sendtime += self.sendtime_offset
        pass

    def receive(self):
        message = ''
        if self.ctrl.inWaiting() > 0:
            message = self.ctrl.read(self.ctrl.inWaiting()).decode()
            self.data += message
        return message


    def decode(self):
        id = None
        t_step = self.agent_info[self.id][0]
        st =self.agent_info[self.id][2]
        st1 = self.agent_info[self.id][3]
        a = self.agent_info[self.id][1]
        p =self.agent_info[self.id][4]
        degree = self.agent_info[self.id][5]
        d_ls = self.data.split("#")
        self.data=''
        for d in d_ls:
            if len(d)>3 and "{"==d[0] and "}"==d[-1]:
                data = json.loads(d)
                # if "id" in data.keys():
                #     id = data["id"]
                #     print("Str data:",data)
                #     if id in self.agent_info.keys():
                #         # if packet from known agent
                #         t_step = self.agent_info[id][0]
                #         a = self.agent_info[id][1]
                #         st = self.agent_info[id][2]
                #         st1 = self.agent_info[id][3]
                #         p = self.agent_info[id][4]
                #     else:
                #         # if packet from new agent
                #         if "t" in data.keys():
                #             t_step = data["t"]
                #         if "d" in data.keys():
                #             degree= data["d"]
                #         if "e" in data.keys():
                #             a = data["e"][0]
                #             st =data["e"][1]
                #             st1 = data["e"][2]
                #         elif "p" in data.keys():
                #             p =data["p"]
                # if packet from new agent
                if "id" in data.keys():
                    id = data["id"]
                if id !=None:
                    if "t" in data.keys():
                        t_step = data["t"]
                    if "d" in data.keys():
                        degree = data["d"]
                    if "e" in data.keys():
                        a = data["e"][0]
                        st = data["e"][1]
                        st1 = data["e"][2]
                    elif "p" in data.keys():
                        p = data["p"]
                    print("Receive data from ",id,":", t_step,a,st,st1,p,degree)
                    self.agent_info[id]= (t_step,a,st,st1,p,degree)
                    print()
                    print("Json data:",(t_step,a,st,st1,p,degree))
                    self.degree = len(self.agent_info.keys())
                    print()

        global_id = list(self.agent_info.keys())
        global_a = []
        global_s = []
        global_sn = []
        global_p = []
        global_d = []
        global_t = []
        print("keys:",self.agent_info.keys())
        for k in self.agent_info.keys():
            global_t.append(self.agent_info[k][0])
            global_a.append(self.agent_info[k][1])
            global_s.append(self.agent_info[k][2])
            global_sn.append(self.agent_info[k][3])
            global_p.append(self.agent_info[k][4])
            global_d.append(self.agent_info[k][5])

        return global_t, global_id, global_s, global_sn, global_a, global_d, global_p



class MotorEncoder():
    """
    Class used to store and calcuate information of movement
    based on Left Right encoders of motors
    """
    def __init__(self, L_init_cnt=0,R_init_cnt=0,unit_div=10.0):
        # ------Parameters-----------------
        # Notes:
        # all angles are in rad unit, not degree unless convert it to degree
        # record  encoder counts
        self.L_cur_cnt = L_init_cnt
        self.L_past_cnt = L_init_cnt
        self.R_cur_cnt = R_init_cnt
        self.R_past_cnt = R_init_cnt
        # convert mm to cm
        self.unit_div = unit_div
        # delta distance after each movement
        self.delta_d= 0.0
        # change of angle afer a movement
        self.delta_agl = 0.0
        # angle difference tolerance
        self.agl_tol = 0.0
        # current position obtained by dead reckoning
        self.x= 0.0
        self.y = 0.0
        # Robot Heading angles
        self.theta = 0.0
        self.old_theta = 0.0

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
        if self.theta >=  math.pi:
            self.theta -= 2 * math.pi
        elif self.theta < - math.pi:
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
        self.x = round(self.x/self.unit_div, 2)
        self.y = round(self.y/self.unit_div, 2)
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


class Logger():
    def __init__(self, file_name_input='Trajectory'):
        self.r =0.0
        self.dir_path = '../Data/'  # Directory path to save file
        file_name = os.path.join(self.dir_path, 'Trajectory' + ".txt")  # text file extension
        self.trajectory = open(file_name, "w")  # Open a text file for storing data

        file_name = os.path.join(self.dir_path, 'obstacles' + ".txt")  # text file extension
        self.obstacles = open(file_name, "w")  # Open a text file for storing data

        file_name = os.path.join(self.dir_path, 'cum_reward' + ".txt")  # text file extension
        self.cum_reward = open(file_name, "w")  # Open a text file for storing data

        file_name = os.path.join(self.dir_path, 'coverage' + ".txt")  # text file extension
        self.coverage = open(file_name, "w")  # Open a text file for storing data

        pass
    def log_trajecctory(self,step,s,a,sn,r,t):
        transition = {"step":step,'s:':s,',a':a,'sn':sn,'r':r,'terminal':t}
        data = json.dumps(transition)
        self.trajectory.write(data+"\n")
        pass
    def log_obstacles(self,step,obstacle):
        # self.obstacles.seek(0,0)
        data = json.dumps({"step":step,"obs":obstacle})
        self.obstacles.write(data + "\n")
        pass

    def log_cum_reward(self,step,r):
        self.r +=r
        data = json.dumps({"step":step,"r":self.r})
        self.cum_reward.write(data+"\n")
        pass
    def log_coverage(self,step,coverage):
        data = json.dumps({"step":step,"coverage":coverage})
        self.coverage.write(data+"\n")
        pass
    def log(self,step, s,a,s_,r,t,obs,coverage):
        self.log_trajecctory(step,s,a,s_,r,t)
        self.log_cum_reward(step,r)
        self.log_obstacles(step,obs)
        self.log_coverage(step,coverage)

    def terminate(self):
        self.trajectory.close()
        self.cum_reward.close()
        self.obstacles.close()
        self.coverage.close()
        pass



class World(object):
    def __init__(self,id,file_name=None,real_state=None,world_w=50, world_h=50,f_len=7):
        # --------Parameters---------
        # -------parameters for defining grid world------
        # State space, define grid world here
        # each grid is 240mmx240 mm based on size of tabular in lab
        self.id =id

        # Default unit of x,y : mm
        # divider to convert unit from mm to cm
        self.unit_div =1.0
        # a grid is 240mm*240mm in real world
        self.real_grid_size =240.0
        # Since when i want roomba to move to next grid,
        # it actually move half of the grid. The grid size it thinks is actually
        #  smaller than the actual grid size, so need to multiply it by 2
        #  And the data read from encoder is real data from world, it is measuring
        # real grid size
        self.grid_size = self.real_grid_size*2/self.unit_div
        self.observation_space = None
        # Action space:
        self.action_space = None
        # if use Q-learning no need for possibility model
        self.trans_model = None

        # reward table:
        #       terminal: bump sth r= -1, lightbump signal: r = -1~0, wheel drop: r = -2
        #       infrared signal received: r = +3
        #       explored area:  r= 1/(1+ visited counts)
        #
        # a list of obstacles detected
        # the first list is position of obstacle, the second list is its possibilty it is really a obstacle
        self.obs_ls = [[],[]]
        # list of real position in which agent gets high bonus/ infrared signal
        self.bonus_pos = []

        # Format: self.global_trans={id: (degree, [a,st,s_t+1])}
        self.global_trans = {}
        # ratio of coverage of map
        self.map_coverage =0.0

        #------Variables used for recording data----------
        self.Roomba = RoombaCI_lib.Create_2("/dev/ttyS0", 115200)
        self.Roomba.ddPin = 23  # Set Roomba dd pin number
        self.backup_time = 0.5  # Amount of time spent backing up
        self.print_time = 1
        self.corner_time = 1.5  # Amount of time that it takes before the roomba starts turning more sharply (makes sure it turns around corners)
        self.data_time = time.time()


        # Assume distance to obstacle and signal strength are proportional
        # 10mm return 3000
        self.LBump_ratio = 3000.0/10.0
        # maximum light bumper signal strength
        self.max_strength =3000.0
        # moving speed 100mm/s, rotate 50mm/s
        self.sp = 90
        self.rot_sp = 50
        # forward distance
        self.fd =int( self.grid_size*self.unit_div*1.5)


        # parameters used for Q-learning
        # Notes: Q-learning will be applied with grid world states, not real continuous states
        #Initial position Current real continuous state: (x,y, theta), theta: heading angles
        # Note: angles in real state and grid state are in different units
        # real state: x, y in mm unit, theta in rad unit (rad easy to comput change of angle)
        # grid state: x,y in per grid unit, theta in degree unit, not rad !!
        # action: d in mm unit, theta in degree unit
        if real_state == None:
            real_state= [0.0,0.0,0.0]
        self.real_state = real_state

        self.angle_set = [0, 45, 90, 135, 180, -45, -90, -135, -180]
        #Initial position discrete state in grid world
        self.grid_state = self.get_gridState(real_state)
        # Current actions: [d,theta1]: d distance to move forward, theta1: change of heading angle
        self.action = [0.0,0.0]


        # Initialize action space, state space
        self.spaces_init(world_w,world_h)

        # initialize GPIO
        self.GPIO_init()
        # Open a text file for data retrieval
        self.logger =Logger()
        # Start achieving data
        self.start_time = time.time()
        [left_start, right_start] = self.Roomba.Query(43, 44)

        self.Motion = MotorEncoder(left_start, right_start,self.unit_div)
        print("Initial state:",self.Motion.get_CurPos(left_start,right_start))
        # Initialize Bumper
        self.bumper= LighBumper()
        self.degree = 0
        self.xb =Xbee(self.id,f_len=f_len)

        time.sleep(2)

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

    def spaces_init(self, world_r=20,world_c=20):
        # Initialize Grid world observation space
        # defaule 20m * 20m= 20*1000mm * 20*1000mm. num of grids are 50*1000/grid_size
        r= int(world_r*1000/self.grid_size)+1
        c= int(world_c*1000/self.grid_size)+1
        self.cnt_map = np.zeros([r,c])

        # Initialize Action Space
        # d: distance to move
        # Note: d>0: move forward along heading direction. d<0: move backward
        # Angle set: each num is in degree
        self.action_space=[]
        for theta in self.angle_set:
            self.action_space.append((self.fd ,theta))
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
        Infra_Omi, Infra_L, Infra_R = None, None,None
        bump, L_cnt, R_cnt, DLightBump, AnalogBump = None,None,None,None,None
        if selection == 'b':
            # read bumper data only
            Infra_Omi,Infra_L, Infra_R, bump,DLightBump, L, FL, CL, CR, FR, R = self.Roomba.ReadQueryStream(17,52,53, 7,45, 46, 47,48, 49, 50, 51)
            AnalogBump = (L, FL, CL, CR, FR, R)
        elif selection == 'e':
            # read encoder data only
            L_cnt, R_cnt= self.Roomba.ReadQueryStream( 43, 44)
        else:
            # read all data
            Infra_Omi, Infra_L, Infra_R,bump, L_cnt,R_cnt, DLightBump, L,FL, CL,CR,FR,R =self.Roomba.ReadQueryStream(17,52,53,7, 43, 44, 45, 46,47,48,49,50,51 )
            AnalogBump = (L,FL, CL,CR,FR,R)

        return  L_cnt,R_cnt, bump, DLightBump,AnalogBump,Infra_Omi, Infra_L, Infra_R



    def terminate(self):
        """
        Stop roomba and clean data after exploration is finished
        :return:
        """
        self.Roomba.PauseQueryStream()
        if self.Roomba.Available() > 0:
            z = self.Roomba.DirectRead(self.Roomba.Available())
            # print(z)
        time.sleep(0.1)

        ## -- Ending Code Starts Here -- ##
        self.Roomba.ShutDown()  # Shutdown Roomba serial connection
        GPIO.cleanup()  # Reset GPIO pins for next program
        self.logger.terminate()
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
        :param real_state:  x,y in cm, theta in degree
        :return:
        discrete int state values:x,y, theta
        """
        if real_state is None:
            # convert unit back to mm
            r_state =[self.real_state[0],self.real_state[1],self.real_state[2]]
        else:
            r_state = real_state

        grid_state = [0,0,0]
        # assume orignal point is the center of the starting piece
        for i in range(2):
            if real_state[i] < (self.grid_size)/2.0:
                grid_state[i] =0
            else:
                # tol = round(self.grid_size/10.0, 2) #tolerance of floating point number
                tol =15.0 #1.5cm as tolerance
                grid_state[i] = int((real_state[i]-(self.grid_size)/2.0)//self.grid_size)
                remain = 1 if (real_state[i]-(self.grid_size)/2.0)%self.grid_size >=tol else 0
                grid_state[i] += remain

        # convert rad to degree
        real_angle = 180.0*(real_state[2]/math.pi)
        a = [abs(real_angle - i) for i in self.angle_set]
        i = np.argmin(a)
        # calculate approximate degree in grid world
        grid_state[2] = self.angle_set[i]
        # print("grid angle:",grid_state[2],"real ag:",real_angle)

        # correct the degree in rad for moving
        # agl_rad = grid_state[2]*(math.pi/180.0)
        # correct_rot = -(real_state[2] - agl_rad)
        # sp = 100
        # t = (correct_rot* (math.pi/180))/sp
        # cur_t = time.time()
        # past_t = cur_t
        # while abs(past_t-cur_t) <=t+0.5:
        #     self.Roomba.Move(0,sp)
        #     cur_t = time.time()
        # self.Roomba.Move(0,0)

        return grid_state



    def cal_reward(self,bump,DLightBump,AnalogBump,Infrared):
        """
        Strategy to compute reward based on data from bumper, digital light bumper and analog light bumper
        Use Gaussian distribution to update reward
        :param bump:
        :param DLightBump:
        :param AnalogBump:
        :return:
        """

        r= 0.0
        ir_om= Infrared[0]
        ir_l = Infrared[1]
        ir_r = Infrared[2]
        if ir_om or ir_l or ir_r:
            # received infrared signal : r = +3
            bonus_pos = self.get_gridState(np.round(self.real_state,2).tolist())
            if self.bonus_pos.count(bonus_pos)==0:
                self.bonus_pos.append(bonus_pos)
            r += 3.0

        # bump something: r =-1
        r += -1.0 if bump & 1 != 0 or  bump & 2 != 0 else 0
        # wheel drop: r = -2
        r += -2.0 if bump & 8 != 0 or bump & 4 != 0 else 0

        # Detected something light bumper: 0~-1
        threshold = 100
        a= 1 if threshold>100 else 0
        sig_sum = 0.0
        for s in AnalogBump:
            sig_sum += s if s >threshold else 0.0
        sig_sum /= len(AnalogBump)
        r += (-1.0)*(sig_sum/self.max_strength)

        return r


    def check_terminal(self,bump,DLightBump, AnalogBump):
        """
        Strategy to determine if current state is terminal
        Since strength of light bumper signal is affected by both the distance to object and the size of object
        if object is small (height = light bumper height), then at 2cm away: signal 120~300, in 1cm: 400~ 700
        change around 20 per 0.5cm
        if object is huge (like a block or wall), 2cm: 1000~2000  1cm: 2000~3200, change of signal around 50 per 0.5cm

        Improvement:
        May apply fuzzy method or simple pattern recognition (like LR or SVM) to determine

        :param bump:
        :param DLightBump:
        :param AnalogBump:
        :return:
            terminal: flag indicating if state is terminal
            obstacles: a list of obstacles detected
        """
        terminal = False
        # signal returned from distance to obstacle /terminal 50 mm,5cm
        # by measurement, small obstacle (height = height of light bumper) in 2cm: signal 120 ~300
        # within 1cm >400
        # if  big obstacle: (like a wall) at 2cm: 1300~1600
        # d_obs = 140
        d_obs = 500.0
        threshold = d_obs/self.max_strength
        obstacles = []

        L, FL, CL, CR, FR, R = AnalogBump
        # using analog light bumper
        prob_obs =np.array([L, FL, CL, CR, FR, R]).astype(float)
        # prob_obs = np.convolve(prob_obs, (0.1,0.8,0.1))[1:-2]
        strength = prob_obs/self.max_strength  # maximum signal strength light bumper can receive
        for i in range(len(strength)):
            strength[i] = 1 if strength[i] >=threshold else 0

        cnt = strength.sum()
        # print('bump: {0:0>8b}:'.format(bump))
        if bump != 0 or cnt >=1:
            # May need reset the position of roomba to previous position using  grid world position (center of last grid)
            # since roomba may drift after hitting obstacle and the data will be incorrect
            terminal=True
            # stop immediately
            self.Roomba.Move(0,0)

            #-------------determine position of obstacles-------------
            l_bump = 1 if bump&2 !=0 else 0
            r_bump = 1 if bump& 1 !=0 else 0
            # Assume Left , right bumpers are at -45 degree, 45 degree
            # Then find the average degree of object:0, -45, 45 degree
            b_avg_angle = 45*(r_bump -l_bump)
            prob_obs /= (prob_obs.sum()+1.0)
            # average angles of obstacle detected by light bumper
            # [-90, -60,-30,30,60,90] are heading angles of 6 analog light bumper
            lb_avg_agl = np.dot(prob_obs,[-90, -60,-30,30,60,90])

            # if there are 2 obstacles
            if np.abs(lb_avg_agl - b_avg_angle)>=60 or (np.sign(lb_avg_agl) !=np.sign(b_avg_angle)):
                th = self.Motion.theta +  lb_avg_agl
                x = self.Motion.x + d_obs * math.cos(th)
                y = self.Motion.y + d_obs * math.sin(th)
                x = int(x)
                y = int(y)
                s= (x,y)
                if obstacles.count(s) == 0:
                    obstacles.append(s)
                # s = self.get_gridState(real_state=[x, y, th])
                # obstacles.append((s[0], s[1]))

                th = self.Motion.theta +  b_avg_angle
                x = self.Motion.x + d_obs * math.cos(th)
                y = self.Motion.y + d_obs * math.sin(th)
                x = int(x)
                y = int(y)
                s = (x,y)
                if obstacles.count(s) ==0:
                    obstacles.append(s)

                # convert real continuous state to discrete grid world state
                # s = self.get_gridState(real_state=[x, y, th])
                # obstacles.append((s[0],s[1]))
            else:
                # if there is 1 obstacle
                alg = (b_avg_angle+lb_avg_agl)/2.0
                th= self.Motion.theta+ alg
                x = self.Motion.x + d_obs * math.cos(th)
                y = self.Motion.y + d_obs * math.sin(th)
                x = int(x)
                y = int(y)
                s = (x,y)
                if obstacles.count(s) == 0:
                    obstacles.append(s)

            # check if the obstacle is one of other agents
            for k in self.global_trans.keys():
                # Format: self.global_trans={id: (degree, [a,st,s_t+1])}
                states = self.global_trans[k][1]
                st = self.get_gridState(states[1])
                st1 = self.get_gridState(states[2])
                # if obstacles are other agents, remove them
                for o in obstacles:
                    grid_o = self.get_gridState((o[0],o[1],th))
                    if (grid_o[0],grid_o[1]) == (st[0],st[1]) or (grid_o[0],grid_o[1]) == (st1[0],st1[1]):
                        obstacles.remove(o)
        return terminal, obstacles


    def observe_Env(self, mode='all'):
        """
        Update current continous real world state and the reward at the new state after achieving data
        mode == 'e': return encoder info only
        otherwise, return all info
        :return:
        old continuous state, new continuous state, reward,flag of terminal
        """
        L_cnt, R_cnt, bump,DLightBump, AnalogBump,Infra_Omi, Infra_L, Infra_R = self.achieve_data(mode)
        old_state = self.real_state.copy()

        if mode != 'e':
            # Check if current state is terminal
            terminal,obs = self.check_terminal(bump,DLightBump, AnalogBump)
            # update list of obstacles
            # maximum count for determining if the obstacle 100% exists
            max_cnt =5.0
            for o in obs:
                # if obstacle is not detected before
                if self.obs_ls[0].count(o)<1:
                    self.obs_ls[0].append(o)
                    self.obs_ls[1].append(1/max_cnt)
                else:
                    # update probability of this obstacle observed
                    self.obs_ls[1][self.obs_ls[0].index(o)] += 1.0/max_cnt

            # The reward is the reward obtained after transition (s,a,s')
            r = self.cal_reward(bump, DLightBump, AnalogBump,(Infra_Omi, Infra_L, Infra_R))
        else:
            # if encoder mode, return encoder info only, without calculate rewards and terminals
            r= 0
            terminal =False

        # obtain postion and heading angle
        self.real_state[0],self.real_state[1],self.real_state[2] = self.Motion.get_CurPos(L_cnt,R_cnt)

        return old_state, self.real_state,r, terminal, (L_cnt, R_cnt, bump,DLightBump, AnalogBump)

    def send_states(self,t=0,state=None, a=None,p=None, state_type='c'):
        """
        data in json packet
        0: id
        1: time step
        2: type of data. if it is 0: it is an indicate, meaning ready to send data
                if it is 1:  it includes info of state and learning model parameters
        3: current grid state
        4: learning model parameters
        5: degree
        packet format in json:
        #{0:1,1:timestep,2: action_ind, 3:state,4: params, 5: degree}
        :param t: current time step
        :return:
        """
        if state == None:
            if state_type == 'd':
                state = self.grid_state
            else:
                state = self.real_state


        if a == None:
            a = self.action
        # find index of the action in action set
        a_ind = list(self.action_space).index(a)
        degree = self.degree
        id = self.id
        data = {0:id,1:t,2:a_ind,3:state,4:p,5:degree}
        txt = json.dumps(data)
        fp = open('w_buf.json','w')
        if fp.writable():
            fp.write(txt)
        else:
            print("Error: Fail to write Agent data")
            fp.close()
            return False
        fp.close()
        return True

    def send_states_v2(self,t, transition=None, p=None):
        """

        data in json packet
        id: id
        t: time step in float type,
        s: old continuous state st
        a: action, at
        sn: s_t+1
        d: degree
        :param: transition (st,at,,s_t+1)
        :param p: weight parameters
        :return:
        """

        data = {}
        data["id"] =self.id
        data["t"] =round(t,1)
        data["d"] =self.degree
        if p != None:
            data["p"] = p

        if transition != None:
            data["s"] = np.round(transition[0],2).tolist()
            data["a"] = list(self.action_space).index(transition[1])
            data["sn"] = np.round(transition[2],2).tolist()
        txt = json.dumps(data)
        fp = open('w_buf_1.json','w')
        if fp.writable():
            fp.write(txt)
            print("Wrote Params. ")
        else:
            print("Error: Fail to write Agent data")
            fp.close()
            return False
        fp.close()
        return True

    def read_glob_s_v2(self,timestep, transition=None,params = None, info= "trans"):
        """

        :param timestep:
        :param transition:
        :param info:
        :return:
        global_s: list of s_t of all agents
        global_a : list of a_t of all agents
        global_d : list of degree of all agents
        global_id: list of id of all agents
        global_sn: list of s_t+1 of all agents
        global_p: list of a_t of other agents except agent i
        """
        file = ''
        global_s = None
        global_a = None
        global_sn = None
        global_p = None
        global_d = [self.degree]
        global_id = [self.id]
        if transition != None:
            s_old ,a, s_new = transition[0],transition[1],transition[2]
            global_s = [s_old]
            global_a = [a]
            global_sn = [s_new]
        if params != None:
            global_p = [params]

        if info == "trans":
            file = "trans_buf.json"
            pass
        else:
            file = "params_buf.json"
            pass

        fp = open(file, "r")
        if fp.readable():
            data = fp.read()
            # print("Global states: ",data)
            delay =0
            while len(data) <1 and delay <=5:
                fp.close()
                print("Waiting for update: "+file)
                time.sleep(1)
                fp = open(file, "r")
                data = fp.read()
                delay += 1
                pass
            if len(data) <1:
                return global_id, global_s,global_sn , global_a,global_d,global_p

            data = json.loads(data)
            keys = list(data.keys())
            keys.sort()
            if info == "trans":
                # read transition states
                if "d" in keys:
                    self.degree = int(data["d"])
                    keys.remove("d")
                else:
                    print("Can't read degree ")
                print("Trans: ",data)
                for t_step in keys:
                    for d in data[t_step]:
                        #  format of packet  {t:[(id,degree,[params]),...],t+1: ...}
                        # (id,timestep,degree,[a, s,sn])
                        # update backup data

                        self.global_trans[d[0]] = (d[1],d[2])

                for i in self.global_trans.keys():
                    transition = self.global_trans[i][1]
                    global_a.append(transition[0])
                    global_s.append(transition[1])
                    global_sn.append(transition[2])
                    global_d.append(self.global_trans[i][0])
                    global_id.append(i)
            else:
                # read parameters
                if "d" in keys:
                    keys.remove("d")
                global_p= []
                keys.sort()
                for t_step in keys:
                    for d in data[t_step]:
                        global_id.append(d[0])
                        global_d.append(d[2])
                        global_p.append(d[3])
                pass

        fp.close()

        return global_id, global_s,global_sn , global_a,global_d,global_p




    def read_global_s(self,real_s_old,timestep=0,param=None):
        """
        packet format in json:
        {1: {'t':timestep, 's': state, 'p':parameters of learning model},
        2: {'t':timestep, 's': state, 'p':parameters of learning model}
        3: {'t':timestep, 's': state, 'p':parameters of learning model}
        }
        if 'p' and 's' are None, drop that packet
        where 1, 2, 3 are the id of agents
        :return:
        """
        global_s= [real_s_old]
        global_p = [param]
        global_a = [self.action]
        global_d = [0]
        id = [self.id]
        fp = open('r_buf.json','r')
        if fp.readable():
            data = fp.read()
            # print("Global states: ",data)
            data = json.loads(data)
            fp.close()
            for i in data.keys():
                if int(i)== self.id:
                    # update degree
                    self.degree = data[i]['d']
                    global_d[0] = self.degree
                else:
                    dt = timestep - int(data[i]['t'])

                    if data[i]['p'] is not None and data[i]['s'] is not None:
                        # if time step difference is >2, consider this agent is delayed, information is not valid
                        # state of agents
                        global_s.append(np.round(data[i]['s'],3))
                        # global actions
                        a= self.action_space[data[i]['a']]
                        global_a.append(a)
                        # parameters of model
                        global_p.append(data[i]['p'])
                        # degree of node
                        global_d.append(data[i]['d'])
                        id.append(i)


            # update degree
            self.degree= global_d[0]

        return id, global_s,global_a,global_d,global_p



    def is_map_updated(self):
        """
        Flag to indicate if map is updated/ new obstacles are found
        :return:
        """
        self.old_obs_len =0
        if len(self.obs_ls[0])!= self.old_obs_len:
            self.old_obs_len =len(self.obs_ls[0])
            return True
        return False


    def update_cnt_map(self,s):
        """
        update visit count of grid world
        :param s:
        :return:
        """
        cnts = []
        num_grid = self.cnt_map.shape[0]*self.cnt_map.shape[1]
        old_coverage =num_grid- self.cnt_map.flatten().tolist().count(0)
        for sj in s:
            grid_s = self.get_gridState(sj)
            self.cnt_map[grid_s[0], grid_s[1]] += 1
            cnts.append(self.cnt_map[grid_s[0], grid_s[1]])

        self.coverage = num_grid - self.cnt_map.flatten().tolist().count(0)

        return cnts


    def get_LocalReward(self,immediate_r, s):
        r_cnt = 0.0
        for sj in s:
            grid_s = self.get_gridState(sj)
            cnt = self.cnt_map[grid_s[0], grid_s[1]]
            cnt = (cnt-1.0) if cnt >0 else 0.0
            r_cnt = 1.0/(1.0+ (cnt))
        # average coverage reward
        r_coverage = r_cnt/ len(s)

        r = immediate_r + r_coverage
        return r

    # def step(self,a):
    #     """
    #     Move robot to expected position and track the current position and reward
    #     :param a: action of Roomba. It is expected to contain rotation angles and moving distance
    #     :return:
    #     new_grid_s, new_real_state, r, is_terminal
    #     """
    #     # update current action
    #     self.action = a
    #     # change of distance in mm
    #     d= a[0]
    #     # degree to rad, from angle to Arc Length
    #     d_theta = a[1]*(math.pi/180.0)
    #     ArcLen = self.Motion.Agl2ArcLen(d_theta)
    #     # tolerance of time difference
    #     tol = -0e-1
    #
    #     init_t = time.time()
    #     cur_t = init_t
    #
    #     # back up current real and grid state s
    #     grid_s_old = self.grid_state.copy()
    #     real_s_old = self.real_state.copy()
    #     old_real_state, new_real_state, r, is_terminal = self.real_state,self.real_state,0.0,False
    #
    #     # track sensor information when moving
    #     self.Roomba.StartQueryStream(17,52,53,7, 43, 44, 45, 46, 47, 48, 49, 50, 51)  # Start getting bumper values
    #
    #     # Take action if current state is not terminal
    #     # Rotate Roomba to certain degree
    #     sign = 1 if d_theta >= 0 else -1
    #     print("Spinning . . . ")
    #     # print('-----------------------------------------')
    #     self.Roomba.Move(0, self.rot_sp* sign)
    #     t=cur_t
    #     while np.abs(cur_t-init_t)< tol+np.abs(ArcLen/self.rot_sp):
    #             self.xb.receive()
    #             if np.abs(cur_t-t)>= self.print_time:
    #                 t= cur_t
    #                 # print('new state: {:10.2f},{:10.2f},{:10.2f}. r:{:10.2f}, terminal:{}'.format(
    #                 #     new_real_state[0], new_real_state[1], new_real_state[2], r, is_terminal))
    #             if self.Roomba.Available()>0:
    #                 # keep track of postion and check if at terminal state, like hitting wall or obstacle
    #                 old_real_state, new_real_state, r, is_terminal,data= self.observe_Env()
    #                 # L_cnt, R_cnt, bump, DLightBump, AnalogBump = data
    #             if ((d_theta+ old_real_state[2]) - new_real_state[2])> 1e-1 :
    #                 break
    #             cur_t = time.time()
    #
    #     # Pause roomba for a while
    #     self.Roomba.Move(0, 0)
    #     print("Spinning t:", np.abs(cur_t-init_t))
    #     print('cur s:', new_real_state)
    #     time.sleep(0.5)
    #     # print('-----------------------------------------')
    #
    #     if is_terminal:
    #         # if it is not terminal, move forward
    #         # print("AnalogBump: ", AnalogBump)
    #         print("===============Reach Terminal =============")
    #         print('r:{:10.2f}, terminal:{}'.format(r, is_terminal))
    #         print('obstacle:', self.obs_ls[0])
    #         print("===========================================")
    #         print()
    #     else:
    #         # reset time
    #         init_t = time.time()
    #         cur_t = init_t
    #         #Roomba moves forward
    #         print('')
    #         print("Moving forward. . . . . .")
    #         # print('-----------------------------------------')
    #         self.Roomba.Move(self.sp, 0)
    #         t =cur_t
    #         while np.abs(cur_t-init_t)< tol+ float(d/self.sp):
    #             self.xb.receive()
    #             if self.Roomba.Available()>0:
    #                 # keep track of postion and check if at terminal state, like hitting wall or obstacle
    #                 old_real_state, new_real_state, r, is_terminal,data= self.observe_Env()
    #                 # L_cnt, R_cnt, bump, DLightBump, AnalogBump = data
    #
    #                 if is_terminal:
    #                     # print("AnalogBump: ", AnalogBump)
    #                     print("===============Reach Terminal =============")
    #                     print('r:{:10.2f}, terminal:{}'.format(r, is_terminal))
    #                     print('obstacle:', self.obs_ls[0])
    #                     print("===========================================")
    #                     print()
    #                     break
    #             # check obstacle and terminal state
    #             if np.abs(cur_t-t)>= self.print_time:
    #                 t =cur_t
    #
    #             cur_t = time.time()
    #         # pause roomba after reaching desired position
    #         self.Roomba.Move(0, 0)
    #         print("forward t:", np.abs(cur_t - init_t))
    #         print('-----------------------------------------')
    #
    #
    #
    #     # Compute new grid state after the motion
    #     new_grid_s = self.get_gridState(new_real_state)
    #     self.grid_state = new_grid_s
    #     # print("grid s:", new_grid_s)
    #
    #
    #     # Clean the useless data
    #     self.Roomba.PauseQueryStream()
    #     if self.Roomba.Available() > 0:
    #         z = self.Roomba.DirectRead(self.Roomba.Available())
    #     time.sleep(1)
    #
    #     # update Gaussian Mixture model for reward approximation
    #     ########################################
    #     ########################################
    #     # record real trajectory here
    #     ##############################
    #     # record real continuous state
    #     # self.logger.log(real_s_old, a, new_real_state, r, is_terminal, self.obs_ls)
    #     # record grid state
    #     # self.logger.log(grid_s_old, a, new_grid_s, r, is_terminal, self.obs_ls)
    #     ##############################
    #     self.xb.receive()
    #     return grid_s_old, real_s_old,new_grid_s, new_real_state, r, is_terminal
    #
    #


    def step(self,a):
        """
               Move robot to expected position and track the current position and reward
               :param a: action of Roomba. It is expected to contain rotation angles and moving distance
               :return:
               new_grid_s, new_real_state, r, is_terminal
               """
        # update current action
        self.action = a
        # change of distance in mm
        d = float(a[0])
        # degree to rad, from angle to Arc Length
        d_theta = a[1] * (math.pi / 180.0)
        ArcLen = self.Motion.Agl2ArcLen(d_theta)
        # tolerance of time difference
        tol = -1e-1
        init_t = time.time()
        cur_t = init_t
        rot_time =tol + np.abs(ArcLen / self.rot_sp)
        forward_time = tol + float(d / self.sp)
        # back up current real and grid state s
        grid_s_old = self.grid_state.copy()
        real_s_old = self.real_state.copy()
        old_real_state, new_real_state, r, is_terminal = self.real_state, self.real_state, 0.0, False
        # track sensor information when moving
        self.Roomba.StartQueryStream(17, 52, 53, 7, 43, 44, 45, 46, 47, 48, 49, 50, 51)  # Start getting bumper values

        sign = 1 if d_theta >= 0 else -1

        while np.abs(cur_t - init_t) <= rot_time+forward_time:
            cur_t = time.time()
            self.xb.receive()
            # print("data: ",self.xb.data)
            dt = np.abs(cur_t - init_t)
            if self.Roomba.Available() > 0:
                if dt <= rot_time and np.abs((d_theta + old_real_state[2]) - new_real_state[2]) > 1e-1:
                    self.Roomba.Move(0, self.rot_sp * sign)
                    old_real_state, new_real_state, r, is_terminal, data = self.observe_Env()

                elif dt > rot_time and dt <=rot_time+forward_time:
                    self.Roomba.Move(self.sp, 0)
                    old_real_state, new_real_state, r, is_terminal, data = self.observe_Env()

                    if is_terminal:
                        self.Roomba.Move(0, 0)
                        print()
                        print("===============Reach Terminal =============")
                        print('r:{:10.2f}, terminal:{}'.format(r, is_terminal))
                        # print('obstacle:', self.obs_ls[0])
                        print("===========================================")
                        print()
                        break

        self.Roomba.Move(0, 0)
        # print("forward t:", np.abs(cur_t - init_t))
        # print('-----------------------------------------')

        # Compute new grid state after the motion
        new_grid_s = self.get_gridState(new_real_state)
        self.grid_state = new_grid_s

        # Clean the useless data
        self.Roomba.PauseQueryStream()
        if self.Roomba.Available() > 0:
            z = self.Roomba.DirectRead(self.Roomba.Available())

        self.xb.receive()
        return grid_s_old, real_s_old, new_grid_s, new_real_state, r, is_terminal


