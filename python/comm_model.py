import serial
import json
import time
import socket
class Xbee():
    def __init__(self,id=1):
        self.id =id
        self.data = None
        self.sendtime = time.time()
        self.sendtime_offset = 1.0
        self.basetime = time.time()
        self.basetime_offset = 0.5
        self.ctrl = serial.Serial('/dev/ttyUSB0', 115200)  # Baud rate should be 115200
        self.t_step =int(0)
        pass
    def send(self, data):
        #   '#' as split between different packet
        message = json.dumps(data)+'#'
        self.ctrl.write(message.encode())
        print("Message sent.")
        self.sendtime += self.sendtime_offset
        pass

    def Available(self):
        return  True if self.ctrl.inWaiting() >0 else False


    def read(self):
        message = self.ctrl.read(self.ctrl.inWaiting()).decode()  # Read all data in
        return message

    def read_avail_agents(self):
        message = self.ctrl.read(self.ctrl.inWaiting()).decode()  # Read all data in
        d_ls = message.split('#')
        # number of agents with higher priority than this agent
        cnt_before =0
        # number of agents with lower priority than this agent
        cnt_after =0
        for s in d_ls:
            # check if it is valid packets
            if len(s)>3 and ("{" in s) and ( "}" in s):
                d= json.loads(s)
                if int(d[0]) < self.id:
                    cnt_before +=1
                if int(d[0]) > self.id:
                    cnt_after +=1

        return cnt_before,cnt_after

    def close(self):
        self.ctrl.close()
        pass

    def decode_data(self,data=''):
        #        data in json packet
        #        0: id
        #        1: time step
        #        2: type of data. if it is 0: it is an indicate, meaning ready to send data
        #                if it is 1:  it includes info of state and learning model parameters
        #        3: current state
        #        4: learning model parameters
        #        5: degree
        #        packet format in json:
        #        {0:1,1:timestep,2: type, 3:state,4: params, 5: degree}
        d_ls = data.split('#')
        data_ls = []
        for d in d_ls:
            if len(d) > 3 and ('{' in d) and ('}' in d):
                data = json.loads(d)
                id, t_step,s,p,d = data[0],data[1],data[3],data[4],data[5]
                data_ls.append((id, t_step,s,p,d))

        return  data_ls

    def check_state_updated(self):
        #        data in json packet
        #        0: id
        #        1: time step
        #        2: type of data. if it is 0: it is an indicate, meaning ready to send data
        #                if it is 1:  it includes info of state and learning model parameters
        #        3: current state
        #        4: learning model parameters
        #        5: degree
        #        packet format in json:
        #        {0:1,1:timestep,2: type, 3:state,4: params, 5: degree}
        fp = open('w_buf.json','r')
        if fp.readable():
            data = fp.read()
            print('Check States: ',data)
            data = json.loads(data)
            if data["1"] >self.t_step:
                self.t_step =data["1"]
                self.data = data
                return True
        return False


    def write_data(self,data_ls):
        """
                packet format in json file r_buf.json:
                {1: {'t':timestep, 's': state, 'p':parameters of learning model,'d':degree of node},
                2: {'t':timestep, 's': state, 'p':parameters of learning model,'d':degree of node}
                3: {'t':timestep, 's': state, 'p':parameters of learning model,'d':degree of node}
                }
                if 'p' and 's' are None, drop that packet
                where 1, 2, 3 are the id of agents

                packet format in data_ls:
                [ (id,time_step, state,params,degree)

                ]
        """
        buf = {}
        for d in data_ls:
            buf[int(d["0"])] ={'t':d["1"], 's':d["2"],'p':d["3"],'d':d["4"]}

        s = json.dumps(buf)
        fp = open('r_buf.json','w')
        fp.write(s)
        fp.close()
        pass

def test_json():
    hn = socket.gethostname()
    id = int(hn[-1])
    pack1 = {'id': id, 'c': 1}
    xb = Xbee(id)
    xb.check_state_updated()

    xb.write_data([xb.data])

def comm_agents():
    hn = socket.gethostname()
    id = int(hn[-1])
    pack1 = {'id': id, 'c': 1}
    xb = Xbee(id)
    data_ls = []
    while True:
        # check if data in w_buf write buffer is updated by the agent
        # if updated, load data and return flag, then send indicator to other agents
        # so that other agents know who are ready to send data
        ready = xb.check_state_updated()
        if ready:
            xb.send(pack1)
        pass

        # check if other agents want to send data
        if xb.Available():
            # check priority of sending data
            # cnt_before: counts of agents that will send before this agent
            # cnt_bebind: counts of agents that will send after this agent
            cnt_before, cnt_after = xb.read_avail_agents()

            # receive data from other agents with higher priority
            for i in range(cnt_before):
                data = ''
                while xb.Available():
                    data += xb.read()
                data_ls.extend(xb.decode_data(data))
                time.sleep(0.5)
            # term of this agent to send data
            if ready:
                xb.send(xb.data)

            # receive data from other agents with lower priority
            for i in range(cnt_after):
                data = ''
                while xb.Available():
                    data += xb.read()
                data_ls.extend(xb.decode_data(data))
                time.sleep(0.5)

        # write data back to r_buffer
        if len(data_ls) > 0:
            xb.write_data(data_ls)
            data_ls.clear()


if __name__ == '__main__':
    # comm_agents()
    test_json()
    pass
