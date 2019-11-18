import serial
import json
import time
import random
import socket
class Xbee():
    def __init__(self,id=1,num_agents =2 ):
        self.id =id
        self.ctrl = serial.Serial('/dev/ttyUSB0', 115200)  # Baud rate should be 115200
        self.sendtime = time.time()
        self.sendtime_offset = 1.0
        self.basetime = time.time()
        self.basetime_offset = 0.5
        self.t_step =int(0)
        self.syn_t = 0
        self.old_data = ""
        # obtain all ids from other agents to check how many agents are in network
        data = {"id":self.id}


        # wait for other agents to setup
        time.sleep(6)
        self.send(data)
        # delay 2s to make sure receive ids from all other agents
        self.degree, self.id_ls = self.read_avail_agents()
        self.id_ls.append(self.id)
        # sort ids in increasing order. smaller id given more priority to send data
        self.id_ls.sort()
        print("Ids:",self.id_ls,"degree:",self.degree)

        # Store initial data to r_buffer for agent to read
        init_data = []
        # for i in self.id_ls:
            # (id,  )
            # init_data.append((i,0,0,[0,0,0],None,self.degree))
        # self.write_data(init_data)
        time.sleep(1)

        # data format to send via Xbee
        self.data = {"0": [self.id, 0, self.degree, 0,0,0], "1": [],"2":0}
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
                    if len(d) > 3 and (d[0] == '{') and (d[-1] == '}'):
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
        # print("d_ls: ",d_ls)
        data_ls = []
        syn_t = 0
        buf = []
        packet_type= 0
        for d in d_ls:
            if len(d) > 3 and (d[0] == '{' ) and (d[-1]== '}'):
                data = json.loads(d)
                # decode of old version of format in packet
                if version == 1:
                    id, t_step,s,p,d = data["0"],data["1"],data["3"],data["4"],data["5"]
                    # decode of new version of format in packet
                else:
                    if "id" in data.keys():
                        # if packet is indicator, just add id and degree
                        packet_type =0
                        if self.id_ls.count(data['id']) ==0:
                            self.id_ls.append(data['id'])
                            self.degree +=1
                        syn_t = self.syn_t
                    elif ("0" in data.keys()) and ("1" in data.keys()) and ("2" in data.keys()):
                        # if packet is data packet, read data
                        packet_type=1
                        ls = list(data["0"])
                        # print("data[0]:", ls)
                        id = ls[0]
                        # time step
                        t_step = ls[1]
                        # action
                        a = ls[2]
                        # degree
                        d = ls[3]
                        # states
                        s = ls[4:]
                        # synchronus time
                        p,syn_t = data["1"], data["2"]
                        # check if data repeated
                        if buf.count((id,t_step)) <1:
                            buf.append((id,t_step))
                            data_ls.append((id, t_step,a,s,p,d))
                    else:
                        return  None, None
        return  data_ls, syn_t, packet_type




    def decode_data_v2(self,data=''):
        """

        :param data:
        :return:
        packet_type: 0: indicator, 1: valid data, -1: Invalid data

        """
        d_ls = data.split('#')
        # list of received transition info
        transition_ls = []
        # list of received learning parameters
        param_ls = []
        syn_t = self.syn_t
        packet_type = -1

        for d in d_ls:
            if len(d) > 3 and (d[0] == '{') and (d[-1] == '}'):
                data = json.loads(d)
                # if received id indicator
                if "id" in data.keys():
                    # if packet is indicator, just add id and degree
                    packet_type = 0
                    if self.id_ls.count(data['id']) == 0:
                                self.id_ls.append(data['id'])
                                self.degree += 1
                                syn_t = self.syn_t
                # if received real data
                else:
                    if ("0" in data.keys()):
                        packet_type = 1
                        id = data["0"][0]
                        timestep = data["0"][1]
                        degree =self.degree #data["0"][2]
                        if id not in self.id_ls:
                            self.id_ls.append(id)
                            self.id_ls.sort()
                            self.degree += 1

                        if ("1" in data.keys()):
                            # if packet is transition info packet
                            trans = list(data["1"])
                            transition_ls.append((id,timestep,degree,trans))
                            pass
                        elif ("2" in data.keys()):
                            # if it has learning parameters
                            params = list(data["2"])
                            transition_ls.append((id,timestep,degree,params))
                            pass
                        if ("3" in data.keys()):
                            syn_t = data["3"]
                    else:
                        return  None, None, syn_t, packet_type

        return transition_ls, param_ls, syn_t, packet_type





    def check_state_updated(self):
        #  Old format:
        #        data in json packet
        #        0: id
        #        1: time step
        #        2: action index
        #        3: current state
        #        4: learning model parameters
        #        5: degree
        #        packet format in json:
        #        {0:1,1:timestep,2: action index, 3:state,4: params, 5: degree}
        #New format to reduce bytes in packet:
        # {0:[id, time step, action ,degree,state0, state1,state2], 1: [parameters]}

        fp = open('w_buf.json','r')
        if fp.readable():
            data = fp.read()
            data = json.loads(data)

            if data["1"] >self.t_step:
                self.t_step =data["1"]
                ls = [data["0"],data["1"],data["2"],data["5"]]
                ls.extend(data["3"])
                # data sent to other agents via Xbee
                self.data = {"0":ls,"1":data["4"],"2":self.syn_t}
                return True

        return False


    def check_updated_v2(self):
        """
        data packet :
        {"0": [id, timestep, degree], "1":[at,  [st], [st+1] ] , "2": [parameters] , "3": syn_t}
        :return:
        """
        p_type = 0
        ls_0 = []
        ls_1  =[]
        fp = open('w_buf_1.json','r')
        packet= {}

        flag = False
        if fp.readable():
            data = fp.read()
            if len(data) > 3 and (data[0] == '{') and (data[-1] == '}'):
                data = json.loads(data)
                # encode data here
                if data["t"] >self.t_step or self.old_data != data:
                    print("Comm: Updated data:",data)
                    self.old_data = data
                    flag =True
                    ls_0.append(data["id"])
                    ls_0.append(data["t"])
                    ls_0.append(data["d"])
                    self.t_step =data["t"]
                    keys = data.keys()
                    if "s" in keys and "a" in keys and "sn" in keys:
                        # append transition
                        ls_1.append(data["a"])
                        ls_1.append(data["s"])
                        ls_1.append(data["sn"])
                        p_type += 1
                        packet["0"] = ls_0
                        packet["1"] =ls_1
                    elif "p" in data.keys():
                        # append parameters
                        packet["2"]=data["p"]
                        p_type +=1
                    # data sent to other agents via Xbee
                    packet["3"] = self.syn_t
                else:
                    packet =None


        return flag, packet, p_type

    def read_localstate(self):
        # {0:1,1:timestep,2: action_ind, 3:state,4: params, 5: degree}
        data = None
        fp = open('w_buf.json', 'r')
        if fp.readable():
            data = fp.read()
            data = json.loads(data)
            self.t_step = data["1"]
            ls = [data["0"], data["1"], data["2"], data["5"]]
            ls.extend(data["3"])
            # data sent to other agents via Xbee
            data = {"0": ls, "1": data["4"], "2": self.syn_t}
        fp.close()

        return data

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
            # d : tuple of decoded data
            buf[d[0]] ={'t':d[1],'a':d[2],'s':d[3],'p':d[4],'d':d[5]}
        s = json.dumps(buf)
        fp = open('r_buf.json','w')
        fp.write(s)
        fp.close()
        pass

    def encode_data(self,data_ls):
        """
        item of data ls must be a tuple in format of (id,timestep,degree,params)
        :param data_ls:
        :return:
        output form:
        { t: [(id,timestep,degree,params), (id,timestep,degree,params) ],  t+1: [(id,timestep,degree,params) ..]}

        """
        encoded_data = None
        if data_ls != None:
            if len(data_ls) >0:
                encoded_data = {}
                timestep = [d[1] for d in data_ls]
                for t in list(set(timestep)):
                    l = []
                    for d in data_ls:
                        if d[1] == t:
                          l.append((d[0],d[2],d[3]))
                    encoded_data[str(t)] = l
                    #         (id,timestep,degree,trans)
                    #         d[0]: id, d[1]: timestep, d[2]: degree, d[3]: trans (a, s, sn)/ params
                    # encoded_data[str(t)] = [(d[0],d[2],d[3]) for d in data_ls if d[1]== t]

        return encoded_data

    def write_data_v2(self,trans_data=None,params_data=None):
        if trans_data != None:
            fp = open('trans_buf.json', 'w')
            trans_data = json.dumps(trans_data)
            fp.write(trans_data)
            print("Comm:   trans_buf updated")
            fp.close()
        if params_data !=None:
            fp = open('params_buf.json', 'w')
            params_data = json.dumps(params_data)
            fp.write(params_data)
            print("Comm:   params_buf updated")
            fp.close()



def comm_agents2():
    hn = socket.gethostname()
    id = int(hn[-1])
    xb = Xbee(id)
    data_ls = []

    xb.syn_t = 0
    while True:
        # ready = xb.check_state_updated()

        # # if it is the term to send
        if (xb.id_ls[xb.syn_t] ==xb.id):
                if xb.degree >1:
                    time.sleep(1)
                else:
                    time.sleep(5)
                print("Agent:", xb.id_ls[xb.syn_t]," Sending data")
                # update synchronous time to allow next agent to send data
                xb.syn_t = xb.syn_t+1
                # reset synchronous time if it overflows
                if xb.syn_t >= len(xb.id_ls):
                    xb.syn_t= 0

                # xb.data["2"] =xb.syn_t
                # Note: "0":[id, timestep, action_ind, degree, state0,state1,state2]
                #       "1": [parameters]
                #       "2": synchronous time
                # Test data
                # xb.data = {"0": [xb.id, 0,random.randint(0,10),xb.degree,round(random.random(),2) , round(random.random(),2), round(random.random(),2)],
                #            "1": [round(random.random(),2) for i in range(90)], "2": xb.syn_t}
                xb.data = xb.read_localstate()
                xb.send(xb.data)

        if xb.degree ==0 or(xb.id_ls[xb.syn_t] !=xb.id) :
            # receive data from other agents with lower priority
            init_t = time.time()
            cur_t = init_t
            # time out 10 s
            timeout = 10
            # if it is term to send data, but not ready,  check if other agents already timeout and send request
            # if it is not the term to send, keep read data from other agents
            data  = ''
            while abs(cur_t - init_t) < timeout:
                # check update of data, if updated send data
                cur_t = time.time()
                while xb.Available():
                    data += xb.read()

            # if received data
            if len(data)>3:
                # print('data: ',data)
                # read data and synchronous time
                d, xb.syn_t, p_type= xb.decode_data(data)
                # check if the packet is what we want
                if p_type==1 and d is not None  and xb.syn_t != None:
                    # print('d:',d)
                    data_ls.extend(d)
                    print("")
                    print("Agent:",xb.id," Got data: ",data_ls)
                    print("Syn time:", xb.syn_t)
                    print("Next agent to send:", xb.id_ls[xb.syn_t])
                    # check
                    for new_d in d:
                        id = new_d[0]
                        if id not in xb.id_ls:
                            xb.id_ls.append(id)
                            xb.id_ls.sort()
                            xb.degree += 1
                    # break

            # if any one of agents fail to send data in limited time,
            # skip this agent, let the next one sent data
            if len(data)<=2 and abs(cur_t - init_t) >= timeout:
                print("Agent ",xb.id_ls[xb.syn_t]," Timeout")
                xb.syn_t += 1
                # reset synchronous time
                if xb.syn_t >=len(xb.id_ls):
                    xb.syn_t = 0

            # write data back to r_buffer
            if len(data_ls) > 0:
                data_ls.append((xb.id, 0,None,None,None,xb.degree))
                xb.write_data(data_ls)
                data_ls.clear()



def comm_agents_3():
    hn = socket.gethostname()
    id = int(hn[-1])
    xb = Xbee(id)
    xb.syn_t = 0
    while True:
        ready, packet, p_type = xb.check_updated_v2()
        if ready:
            # send data
            print("Agent:", xb.id_ls[xb.syn_t], " Sending data")
            # update synchronous time
            xb.syn_t = xb.syn_t + 1
            if xb.syn_t >= len(xb.id_ls):
                xb.syn_t = 0
            xb.data = packet
            xb.send(xb.data)

        data = ''
        init_t = time.time()
        cur_t = init_t
        # time out 3 s
        timeout = 3
        while abs(cur_t - init_t) < timeout:
            # read string data
            cur_t = time.time()
            while xb.Available():
                data += xb.read()

        # decode data
        if len(data) > 3:
            # print('data: ',data)
            trans_ls, params_ls, xb.syn_t, p_type = xb.decode_data_v2(data)
            if p_type == 1:
                enc_trans = xb.encode_data(trans_ls)
                # pass the detected degree to this agent
                enc_trans["d"] = xb.degree
                enc_params = xb.encode_data(params_ls)
                print("enc trans:",enc_trans)
                print("enc params:", enc_params)
                # write data back to r_buffer
                xb.write_data_v2(enc_trans, enc_params)




if __name__ == '__main__':
    comm_agents_3()
    # comm_agents2()
    # test_json()
    pass
