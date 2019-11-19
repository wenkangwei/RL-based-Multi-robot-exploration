from Room_Env import *
import numpy as np
import math
import  random
import socket
from  comm_model import *
import rl_model as ac
import time


# def local_reward(r, )


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

# def Xbee_comm():
#     comm_agents()

def actor_critic(Env,max_iteration=10,epoch=3,num_agents =2):

    sh = np.shape(Env.cnt_map)
    model = ac.actor_critic_q(7,sh ,Env.obs_ls[0],Env.action_space ,discount=1)
    track = []
    # time step t
    t = 0
    global_s = [[0.0,0.0,0.0] for i in range(num_agents)]
    global_s[0] = Env.real_state
    Env.update_cnt_map(global_s)

    try:

        for i in range(max_iteration):
            # sample trajectories
            for j in range(epoch):
                # sample current state and action pair
                si= global_s[0]
                a= model.sample_action(si,global_s[1:],epi=0.8)
                print('action: ', a)

                # step
                grid_s_old, real_s_old, grid_s_new, s_new, immediate_r, is_terminal = Env.step(a)
                print("Grid state: ", grid_s_new)
                print("real state: ", s_new)
                print("reward: ", round(immediate_r, 3))
                print("Terminal: ", is_terminal)


                # share (s,a) pair at time t, where s: st, a: at

                # old version of sending data
                # ws = model.w_local.tolist()
                # Env.send_states(t=t, state=global_s[0],a=a, p=ws)
                # time.sleep(2)
                # id, global_s, global_a, global_d, global_p = Env.read_global_s(si, timestep=t, param=None)

                Env.xb.send(t, transition=(a, real_s_old, s_new))
                # Env.send_states_v2(t,(real_s_old, a,s_new), p=None)
                time.sleep(3)
                # global_id, global_s, global_sn, global_a, global_d,_ = Env.read_glob_s_v2(timestep=t, transition =(real_s_old, a,s_new),info= "trans")
                global_id, global_s, global_sn, global_a, global_d, _ = Env.xb.decode()
                # update visit count in map
                print("Global state: ",global_s)
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
                time.sleep(4)
                while len(Env.xb.receive())<0:
                    pass

                id, _, _, _, global_d, global_w = Env.xb.decode()
                print("global w:",global_w)
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
                for i in range(len(id)):
                    print('==========================')
                    print("id's:", id[i])
                    print("state: ", global_a[i])
                    print("action:", global_a[i])
                    print("degree: ", global_d[i])
                    print("Params: ", global_w[i])
                    print('==========================')
                    print()


                if is_terminal:
                    break


            # update current real continous state
            global_s[0] = Env.real_state
            si =Env.real_state
            # sample new initial state
            a = model.sample_action(si, global_s[1:], epi=1)
            _,new_init_grid_s, new_init_s, r, is_terminal= Env.step(a)

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
    actor_critic(Env, max_iteration=10, num_agents=2)
    # test_json(Env)
    # Env.terminate()


