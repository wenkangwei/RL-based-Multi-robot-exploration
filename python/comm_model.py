import serial
import json
import time
import socket
class Xbee():
    def __init__(self,id=1,num_agents =2 ):
        self.id =id
        self.sendtime = time.time()
        self.sendtime_offset = 1.0
        self.basetime = time.time()
        self.basetime_offset = 0.5
        self.ctrl = serial.Serial('/dev/ttyUSB0', 115200)  # Baud rate should be 115200
        self.t_step =int(0)
        self.syn_t = 0
        # obtain all ids from other agents to check how many agents are in network
        data = {"id":self.id}


        # wait for other agents to setup
        time.sleep(2*num_agents)
        self.send(data)
        # delay 2s to make sure receive ids from all other agents
        self.degree, self.id_ls = self.read_avail_agents()
        self.id_ls.append(self.id)
        # sort ids in increasing order. smaller id given more priority to send data
        self.id_ls.sort()
        print("Ids:",self.id_ls,"degree:",self.degree)

        # Store initial data to r_buffer for agent to read
        init_data = []
        for i in self.id_ls:
            # #(id,  )
            init_data.append((i,0,[0,0,0],None,0))
        self.write_data(init_data)
        time.sleep(1)

        # data format to send via Xbee
        self.data = {"0": [self.id, 0, self.degree, 0,0,0], "1": [],"2":0}
        pass

    def send(self, data):
        #   '#' as split between different packet
        message = json.dumps(data)+'#'
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
        id_ls = []
        degree = 0
        cur_t = time.time()
        init_t = cur_t
        timeout = 2
        while abs(cur_t- init_t) <timeout:
            cur_t = time.time()
            while self.Available():
                message = self.ctrl.read(self.ctrl.inWaiting()).decode()  # Read all data in
                d_ls = message.split('#')
                for d in d_ls:
                    if len(d) > 3 and ("{" in d) and ("}" in d):
                        d = json.loads(d)
                        # 0:id of agent
                        if "id" in d.keys():
                            id_ls.append(d["id"])
                            degree += 1

        return  degree, id_ls

    def close(self):
        self.ctrl.close()
        pass


    def decode_data(self,data='',version =2):
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
        #
        # New format of packet:
        # {"0":[id, time_step,degree, state0,state1，state2],"1":[parameters], "2": synchronos time among agents}
        d_ls = data.split('#')
        print("d_ls: ",d_ls)
        data_ls = []
        syn_t = 0
        buf = []
        for d in d_ls:
            if len(d) > 3 and (d[0] == '{' ) and (d[-1]== '}'):
                data = json.loads(d)
                # decode of old version of format in packet
                if version == 1:
                    id, t_step,s,p,d = data["0"],data["1"],data["3"],data["4"],data["5"]
                    # decode of new version of format in packet
                else:
                    if ("0" in data.keys()) and ("1" in data.keys()) and ("2" in data.keys()):
                        ls = list(data["0"])
                        print("data[0]:", ls)
                        id = ls[0]
                        t_step = ls[1]
                        d = ls[2]
                        s = ls[3:]
                        p,syn_t = data["1"], data["2"]
                        if buf.count((id,t_step)) <1:
                            buf.append((id,t_step))
                            data_ls.append((id, t_step,s,p,d))
                    else:
                        return  None, None
        # print("s:", s)
        # print("data_ls,",data_ls, "syn_t:",syn_t)
        return  data_ls, syn_t

    def check_state_updated(self,version =2):
        #  Old format:
        #        data in json packet
        #        0: id
        #        1: time step
        #        3: current state
        #        4: learning model parameters
        #        5: degree
        #        packet format in json:
        #        {0:1,1:timestep,2: type, 3:state,4: params, 5: degree}
        #New format to reduce bytes in packet:
        # {0:[id, time step, degree,state0, state1,state2], 1: [parameters]}

        fp = open('w_buf.json','r')
        if fp.readable():
            data = fp.read()
            data = json.loads(data)

            if data["1"] >self.t_step:
                self.t_step =data["1"]

                if version == 1:
                    self.data = data
                else:
                    # new format of packet
                    ls = [data["0"],data["1"],data["5"]]
                    ls.extend(data["3"])
                    self.data = {"0":ls,"1":data["4"],"2":self.syn_t}
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
            buf[d[0]] ={'t':d[1], 's':d[2],'p':d[3],'d':d[4]}
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

def comm_agents1():
    hn = socket.gethostname()
    id = int(hn[-1])
    xb = Xbee(id)
    data_ls = []

    xb.syn_t = 0
    c_t = time.time()
    i_t = c_t
    ready = False
    while True:
        # check if states of agent in buffer updated  and
        # if it is the term for this agent to send based on the id list
        if abs(c_t-i_t) >= 1.5:
            ready = True
            i_t =c_t

        c_t =time.time()
        # ready = xb.check_state_updated()

        # # if it is the term to send
        if ready and (xb.id_ls[xb.syn_t] ==xb.id):
                import random
                print("Agent:", xb.id_ls[xb.syn_t]," Sending data")
                # update synchronous time to allow next agent to send data
                xb.syn_t = xb.syn_t+1
                # reset synchronous time if it overflows
                if xb.syn_t >= len(xb.id_ls):
                    xb.syn_t= 0

                # xb.data["2"] =xb.syn_t

                xb.data = {"0": [xb.id, 0, xb.degree,round(random.random(),2) , round(random.random(),2), round(random.random(),2)],
                           "1": [round(random.random(),2) for i in range(120)], "2": xb.syn_t}
                xb.send(xb.data)
        else:
            # if it is not the term to send
            # print("Waiting for My term to send")

            # receive data from other agents with lower priority
            init_t = time.time()
            cur_t = init_t
            # time out 8 s
            timeout = 10

            # if it is term to send data, but not ready,  check if other agents already timeout and send request
            # if it is not the term to send, keep read data from other agents
            data  = ''
            while abs(cur_t - init_t) < timeout:
                cur_t = time.time()
                while xb.Available():
                    data += xb.read()

            # if received data
            if len(data)>3:
                print('data: ',data)
                # read data and synchronous time
                d, xb.syn_t= xb.decode_data(data)
                # check if the packet is what we want
                if d is not None  and xb.syn_t != None:
                    data_ls.extend(d)
                    print("")
                    print("Agent:",xb.id," Got data: ",data_ls)
                    print("Syn time:", xb.syn_t)
                    print("Next agent to send:", xb.id_ls[xb.syn_t])
                    # break
            else:
                        # wait
                        # print()
                pass

            # if any one of agents fail to send data in limited time,
            # skip this agent, let the next one sent data
            if len(data)<=1 and abs(cur_t - init_t) >= timeout:
                print("Agent ",xb.id_ls[xb.syn_t]," Timeout")
                xb.syn_t += 1
                # reset synchronous time
                if xb.syn_t >=len(xb.id_ls):
                    xb.syn_t = 0


            # write data back to r_buffer
            if len(data_ls) > 0:
                xb.write_data(data_ls)
                data_ls.clear()
        ready = False

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
            # xb.send(pack1)
            print("Indicator sent.")
            # print(xb.data)
        pass

        # debug
        time.sleep(1)

        # check if other agents want to send data
        if True:
        # if xb.Available():
            print("Ready to receive data")
            # check priority of sending data
            # cnt_before: counts of agents that will send before this agent
            # cnt_bebind: counts of agents that will send after this agent
            # cnt_before, cnt_after, id_ls = xb.read_avail_agents()
            time.sleep(1)
            # print("Count:",cnt_before,cnt_after,"id:",id_ls)
            # receive data from other agents with higher priority
            i = 0
            # while i <cnt_before:
            #     print("Receiving Agent:",i)
            #     i += 1
            #     data = ''
            #     while xb.Available():
            #         data += xb.read()
            #         print("Data: ", data)
            #     data_ls.extend(xb.decode_data(data))
            #     time.sleep(1)
            data = ''
            # while xb.Available():
            #     data += xb.read()
            #     # print("Data: ", data)
            # data_ls.extend(xb.decode_data(data))
            # term of this agent to send data
            if ready:
                xb.send(xb.data)

            # receive data from other agents with lower priority
            data = ''
            while xb.Available():
                data += xb.read()
                print("Data: ", data)
            if len(data)>1:
                data_ls.extend(xb.decode_data(data))
            # i = 0
            # while i <cnt_after:
            #     print("Receiving Agent:", i)
            #     i += 1
            #     data = ''
            #     while xb.Available():
            #         data += xb.read()
            #         print("Data: ",data)
            #     data_ls.extend(xb.decode_data(data))
            #     time.sleep(1)

            print("Received data: ",data_ls)

        ready =False
        # write data back to r_buffer
        if len(data_ls) > 0:
            xb.write_data(data_ls)
            data_ls.clear()


if __name__ == '__main__':
    comm_agents1()
    # test_json()
    pass
