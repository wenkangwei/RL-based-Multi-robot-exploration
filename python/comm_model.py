import serial
import json
import time
import socket
class Xbee():
    def __init__(self,id=1):
        self.id =id
        self.data = {"0":id,"1":0,"2":1,"3":[0,0,0],"4":None,"5":0}
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
        print("Sent Message: ", message)
        self.ctrl.write(message.encode())
        # print("Message sent.")
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
        id_ls = []
        # number of agents with higher priority than this agent
        cnt_before =0
        # number of agents with lower priority than this agent
        cnt_after =0
        for s in d_ls:
            # check if it is valid packets
            if len(s)>3 and ("{" in s) and ( "}" in s):
                d= json.loads(s)
                if d["0"] is not None and ('c' in d.keys()):
                    print("Received data: ",d)
                    if int(d["0"]) < self.id:
                        cnt_before +=1
                    if int(d["0"]) > self.id:
                        cnt_after +=1
                    id_ls.append(int(d["0"]))

        return cnt_before,cnt_after, id_ls

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
            data = json.loads(data)
            if data["1"] >self.t_step:
                self.t_step =data["1"]
                self.data = data
                # print('Check States: ',data)
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
            buf[str(d["0"])] ={'t':d["1"], 's':d["2"],'p':d["3"],'d':d["4"]}

        s = json.dumps(buf)
        fp = open('r_buf.json','w')
        fp.write(s)
        fp.close()
        pass

def test_json():
    hn = socket.gethostname()
    id = int(hn[-1])
    pack1 = {'0': id, 'c': 1}
    xb = Xbee(id)
    f = xb.check_state_updated()
    print('Update:',f)
    xb.write_data([xb.data])

def comm_agents():
    """
    Packet 1 {'0':id,'c':1} 'c': type of packet
    packet 2: data from current agent : {"0":0,"1":0,"2":1,"3":[0,0,0],"4":null,"5":0} data from write buffer
    Data write to r_buffer:
            {
          "1":{"t":0,"s":[1,2,3],"p":[0,0,0,0],"d":0},
          "2":{"t":0,"s":[1,2,3],"p":[0,0,0,0],"d":0},
          "3":{"t":0,"s":[1,2,3],"p":[0,0,0,0],"d":0}
        }
        indicating states of different agents

    :return:
    """
    hn = socket.gethostname()
    id = int(hn[-1])
    pack1 = {'0': id, 'c': 1}
    xb = Xbee(id)
    data_ls = []
    init_t = time.time()
    cur_t =init_t
    ready =False

    while True:
        # check if data in w_buf write buffer is updated by the agent
        # if updated, load data and return flag, then send indicator to other agents
        # so that other agents know who are ready to send data
        # ready = xb.check_state_updated()

        if abs(cur_t-init_t)>2:
            ready = True if ready == False else True
            init_t=cur_t
        cur_t =time.time()

        if ready:
            xb.send(pack1)
            print("Indicator sent.")
            # print(xb.data)
        pass

        # debug


        # check if other agents want to send data
        if xb.Available():
            print("Ready to receive data")
            # check priority of sending data
            # cnt_before: counts of agents that will send before this agent
            # cnt_bebind: counts of agents that will send after this agent
            cnt_before, cnt_after, id_ls = xb.read_avail_agents()
            print("Count:",cnt_before,cnt_after,"id:",id_ls)
            # receive data from other agents with higher priority
            i = 0
            while i <cnt_before:
                print("Receiving Agent:",i)
                i += 1
                data = ''
                while xb.Available():
                    data += xb.read()
                    print("Data: ", data)
                data_ls.extend(xb.decode_data(data))
                time.sleep(1)
            # term of this agent to send data
            if ready:
                xb.send(xb.data)
                while xb.Available():
                    xb.read()
                time.sleep(1)

            # receive data from other agents with lower priority
            i = 0
            while i <cnt_after:
                print("Receiving Agent:", i)
                i += 1
                data = ''
                while xb.Available():
                    data += xb.read()
                    print("Data: ",data)
                data_ls.extend(xb.decode_data(data))
                time.sleep(1)

            print("Received data: ",data_ls)

        ready =False
        # write data back to r_buffer
        if len(data_ls) > 0:
            xb.write_data(data_ls)
            data_ls.clear()


if __name__ == '__main__':
    comm_agents()
    # test_json()
    pass
