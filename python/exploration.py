from Room_Env import *
import numpy as np
import math
import  random
import socket
from  comm_model import *
import rl_model as ac
import time



def test_observation(Env):
    import time
    Env.Roomba.StartQueryStream(7, 43, 44, 45, 46, 47, 48, 49, 50, 51)  # Start getting bumper values
    cur_t = time.time()
    t = cur_t
    old_real_state, new_real_state, r, is_terminal = None, None, 0, False
    L_cnt, R_cnt, bump, DLightBump, AnalogBump = None,None,None,None,None

    while True:
        if Env.Roomba.Available() > 0:
            old_real_state, new_real_state, r, is_terminal,data = Env.observe_Env()
            L_cnt, R_cnt, bump, DLightBump, AnalogBump = data
        if abs(t - cur_t) >= 0.5:
            print()
            print('------------------------------------')
            print('L cnt:{}, R cnt:{}'.format(L_cnt, R_cnt, ))
            print('bump:{0:0>8b}'.format(bump))
            print('DLbump:{0:0>8b}'.format(DLightBump))
            print("AnalogBump: ", AnalogBump)
            print('new state: {:10.2f},{:10.2f},{:10.2f}. '.format(
                new_real_state[0], new_real_state[1], new_real_state[2]))
            print('r:{:10.2f}, terminal:{}'.format( r, is_terminal))
            print('obstacle:',Env.obs_ls[0])
            t = cur_t


        cur_t = time.time()
    pass



def test_json(Env):
    action_set = Env.action_space
    Env.send_states()
    id, global_s, global_d, global_p = Env.read_global_s(timestep=0, param=None)
    print('id:{},global_s:{},d:{}, p:{}'.format(id, global_s, global_d, global_p))





def actor_critic_2(Env,max_iteration=10,epoch=3,num_agents =2):

    sh = np.shape(Env.cnt_map)
    grid_size = Env.grid_size
    model = ac.actor_critic_q(grid_size, 7,sh ,Env.obs_ls[0],Env.action_space ,discount=1)
    track = []
    # time step t
    t = 0
    global_s = [[0.0,0.0,0.0] for i in range(num_agents)]
    global_s[0] = Env.real_state
    w_global = []
    theta = []
    Env.update_cnt_map(global_s)
    max_iteration =5
    epoch =10
    epi = 1
    try:
        for i in range(max_iteration):
            for j in range(epoch):
                model.beta_w = 1.0- (j/float(epoch))
                model.beta_theta = 1.0- (j/float(epoch))
                epi *= 0.95
                # sample current state and action pair
                si= global_s[0]
                a= model.sample_action(si,global_s[1:],epi=epi)


                # step
                grid_s_old, real_s_old, grid_s_new, s_new, immediate_r, is_terminal = Env.step(a)
                # print("Grid state: ", grid_s_new)
                # print("real state: ", s_new)

                # share (s,a) pair at time t, where s: st, a: at
                Env.xb.receive()
                Env.xb.send(t, transition=(a, real_s_old, s_new))
                time.sleep(3)
                global_t,global_id, global_s, global_sn, global_a, global_d, global_w= Env.xb.decode()

                # update state info inside agent
                for i, id in enumerate(global_id):
                    Env.global_trans[id] = (global_d[i],(global_a[i],global_s[i],global_sn[i],))

                cnts = Env.update_cnt_map(global_s)
                model.cnts =cnts

                # local reward
                local_r= Env.get_LocalReward(immediate_r, global_s)


                # update local w of Q function
                # global_s[0]: s of ith agent,  global_s[1:]: s of other agents
                w_local = model.crtic_step(global_s[0],global_a[0], global_sn[0],
                                           global_s[1:], global_a[1:], global_sn[1:],
                                           local_r,is_terminal).tolist()
                Env.xb.send(t, params = w_local)
                time.sleep(2)
                while len(Env.xb.receive())<0:
                    pass

                global_t,global_id, global_s, global_sn, global_a, global_d, global_w = Env.xb.decode()
                print("global w:",global_w)
                # update global w
                w_global= model.update_w_gbl(global_d[0], global_d[1:], global_w)

                # update policy
                theta = model.actor_step(global_s[0],global_a[0], global_s[1:], global_a[1:])
                t += 1
                # record real trajectory here
                ##############################
                track.append(grid_s_new)
                Env.logger.log(t,real_s_old, a, s_new, immediate_r, is_terminal, Env.obs_ls,Env.map_coverage,Env.bonus_pos,w_global[0,:], theta[0,:])
                ##############################
                print()
                print()
                print("Global States: ")
                for i in range(len(global_id)):
                    print('==========================')
                    print("t: ", global_t[i])
                    print("id's:", global_id[i])
                    print("state: ", global_s[i])
                    print("Grid state:", Env.get_gridState(global_s[i]))
                    print("action:", global_a[i])
                    print("degree: ", global_d[i])
                    if global_id[i] == Env.id:
                        print('action: ', a)
                        print("Imm Reward: ", round(immediate_r, 3), ", total Reward:", local_r)
                        print("Terminal: ", is_terminal)
                    print("Params: ", global_w[i])
                    print('==========================')
                    print()

                if is_terminal:
                    for i in range(10):
                        # a = random.choice(Env.action_space)
                        a = Env.action_space[2]
                        print("move away from terminal:",a)
                        Env.xb.receive()
                        # a = model.sample_action(si, global_s[1:], epi=0.9)
                        # sample new initial state
                        _, _, new_init_grid_s, new_init_s, immediate_r, is_terminal = Env.step(a)
                        time.sleep(1)
                        print("Sampling new state by action:", a)
                        if not is_terminal:
                            global_s[0] = Env.real_state
                            print("New init s: ", Env.real_state)
                            break

                    break
        pass

    except KeyboardInterrupt:
        Env.terminate()
        print('obstacles：')
        for o in Env.obs_ls[0]:
            x,y,theta= Env.get_gridState((o[0],o[1],0))
            print((x,y))
            # print(Env.obs_ls[0])
        print('Track:')
        for i in track:
            print(i)

    Env.terminate()
    print()
    print('obstacles：')
    for o in Env.obs_ls[0]:
        x, y, theta = Env.get_gridState((o[0], o[1], 0))
        print((x, y))
    print('Track:')
    for i in track:
        print(i)

    pass

    print("w : ", w_global)
    print("theta:", theta)
    print()


def actor_critic(Env,max_iteration=10,epoch=3,num_agents =2):

    sh = np.shape(Env.cnt_map)
    model = ac.actor_critic_q(7,sh ,Env.obs_ls[0],Env.action_space ,discount=1)
    track = []
    # time step t
    t = 0
    global_s = [[0.0,0.0,0.0] for i in range(num_agents)]
    global_s[0] = Env.real_state
    Env.update_cnt_map(global_s)
    max_iteration =1
    epoch =6
    try:

        for i in range(max_iteration):
            for j in range(epoch):
                # sample current state and action pair
                si= global_s[0]
                a= model.sample_action(si,global_s[1:],epi=0.5)
                print('action: ', a)

                # step
                grid_s_old, real_s_old, grid_s_new, s_new, immediate_r, is_terminal = Env.step(a)
                print("Grid state: ", grid_s_new)
                print("real state: ", s_new)
                print("reward: ", round(immediate_r, 3))
                print("Terminal: ", is_terminal)


                # share (s,a) pair at time t, where s: st, a: at
                Env.xb.receive()

                # old version of sending data
                # ws = model.w_local.tolist()
                # Env.send_states(t=t, state=global_s[0],a=a, p=ws)
                # time.sleep(2)
                # id, global_s, global_a, global_d, global_p = Env.read_global_s(si, timestep=t, param=None)

                Env.xb.send(t, transition=(a, real_s_old, s_new))
                # Env.send_states_v2(t,(real_s_old, a,s_new), p=None)
                time.sleep(3)
                # global_id, global_s, global_sn, global_a, global_d,_ = Env.read_glob_s_v2(timestep=t, transition =(real_s_old, a,s_new),info= "trans")
                global_t,global_id, global_s, global_sn, global_a, global_d, global_w= Env.xb.decode()


                for i, id in enumerate(global_id):
                    Env.global_trans[id] = (global_d[i],(global_a[i],global_s[i],global_sn[i],))

                # print("Lens: ",len(global_id),len(global_s),len(global_sn),len(global_a),len(global_d),len(global_w))
                # update visit count in map
                # print("Global state: ",global_s)

                cnts = Env.update_cnt_map(global_s)
                model.cnts =cnts
                # local reward
                local_r= Env.get_LocalReward(immediate_r, global_s)

                # update local w of Q function
                # global_s[0]: s of ith agent,  global_s[1:]: s of other agents
                w_local = model.crtic_step(global_s[0],global_a[0], global_sn[0],
                                           global_s[1:], global_a[1:], global_sn[1:],
                                           local_r,is_terminal).tolist()
                Env.xb.send(t, params = w_local)
                # Env.send_states_v2(t, None, p=w_local)
                time.sleep(2)
                while len(Env.xb.receive())<0:
                    pass

                global_t,global_id, global_s, global_sn, global_a, global_d, global_w = Env.xb.decode()
                # print("Lens: ", len(global_id), len(global_s), len(global_sn), len(global_a), len(global_d),
                #       len(global_w))
                print("global w:",global_w)
                # print("global degree ",global_d)

                # id, _, _,_, global_d, global_w = Env.read_glob_s_v2(timestep=t,params=w_local,info= "params")
                # old version of sending data
                # Env.send_states(t=t, state=s_new, p=w_local)
                # time.sleep(3)
                # old version of reading data
                # id, global_s, global_a, global_d, global_p = Env.read_global_s(si,timestep=t, param=w_local)

                # update global w
                model.update_w_gbl(global_d[0], global_d[1:], global_w)

                # update policy
                model.actor_step(global_s[0],global_a[0], global_s[1:], global_a[1:])
                t += 1
                # record real trajectory here
                ##############################
                track.append(grid_s_new)
                Env.logger.log(t,grid_s_old, a, grid_s_new, local_r, is_terminal, Env.obs_ls,Env.map_coverage)
                ##############################

                print()
                print()
                print("Global States: ")
                for i in range(len(global_id)):
                    print('==========================')
                    print("t: ", global_t[i])
                    print("id's:", global_id[i])
                    print("state: ", global_s[i])
                    print("Grid state:", Env.get_gridState(global_s[i]))
                    print("action:", global_a[i])
                    print("degree: ", global_d[i])
                    print("Params: ", global_w[i])
                    print('==========================')
                    print()


                if is_terminal:
                    # sample new init state
                    # update current real continous state
                    global_s[0] = Env.real_state
                    si = Env.real_state
                    # sample new initial state
                    a = model.sample_action(si, global_s[1:], epi=0.9)
                    Env.xb.receive()
                    _, _, new_init_grid_s, new_init_s, immediate_r, is_terminal = Env.step(a)

                    break



        pass

    except KeyboardInterrupt:
        Env.terminate()
        print('obstacles：')
        for o in Env.obs_ls[0]:
            x,y,theta= Env.get_gridState((o[0],o[1],0))
            print((x,y))
            # print(Env.obs_ls[0])
        print('Track:')
        for i in track:
            print(i)

    Env.terminate()
    print()
    print('obstacles：')
    for o in Env.obs_ls[0]:
        x, y, theta = Env.get_gridState((o[0], o[1], 0))
        print((x, y))
    print('Track:')
    for i in track:
        print(i)
    pass




def run_agent(Env):
    action_set = Env.action_space

    # a = random.choice(action_set)
    track = []
    # time step t, used to track update of learning model
    t = 0
    try:
        # test_observation(Env)
        set = [random.choice(Env.action_space) for i in range(2)]
        for a in set:
            # do action and sample experience here
            print('action: ', a)
            real_s_old,grid_s_new, s_new, r, is_terminal = Env.step(a)
            print("Grid state: ",grid_s_new)
            print("real state: ", s_new)
            print("reward: ",round(r,3))
            print("Terminal: ",is_terminal)
            time.sleep(2)
            print(a, grid_s_new)

            # debug
            # s_new, r, is_terminal = [round(random.random(),2) for i in range(3)],round(random.random(),2),False
            t += 1
            track.append(grid_s_new)

            # Training local Q value/ V state value here

            # Send local information to other agents
            w = [round(random.random(),2) for i in range(10)]


            # Env.send_states(t=t,state= s_new,p=w)
            #
            # # receive info from other agents
            # # Learning model parameters
            # # delay few seconds to update data of agents
            time.sleep(2)
            # id, global_s, global_a, global_d, global_p = Env.read_global_s(timestep=t, param=w)
            print()
            print()
            print("Global States: ")
            for i in range(len(id)):
                print('==========================')
                print("id's:",id[i])
                print("state: ",global_a[i])
                print("action:", global_a[i])
                print("degree: ",global_d[i])
                print("Params: ", global_p[i])
                print('==========================')
                print()
            # print(id, global_s,global_a,global_d,global_p)
            # update global learning model


            # show msp here
            if Env.is_map_updated() and len(Env.obs_ls) > 0:
                # update parameters of learning model
                # share parameter if local map is updated
                pass
        pass

    except KeyboardInterrupt:
        Env.terminate()
        print('obstacles：')
        print(Env.obs_ls[0])
        print('Track:')
        for i in track:
            print(i)

    Env.terminate()
    print('obstacles：')
    print(Env.obs_ls[0])
    print('Track:')
    for i in track:
        print(i)


if __name__ == '__main__':
    # import multiprocessing as mp
    hn = socket.gethostname()
    # obtain hostname of raspberry pi
    id = int(hn[-1])

    Env = World(id)
    # run_agent(Env)
    actor_critic_2(Env, max_iteration=10, num_agents=2)
    # test_json(Env)
    # Env.terminate()


