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

# def Xbee_comm():
#     comm_agents()

def actor_critic(Env,max_iteration=10,epoch=3,num_agents =2):
    action_set = Env.action_space

    model = ac.actor_critic_q(7,Env.cnt_map, Env.obs_ls[0],Env.action_space ,discount=1)
    # a = random.choice(action_set)
    track = []
    # time step t, used to track update of learning model
    t = 0
    global_s = [[0.0,0.0,0.0] for i in range(num_agents)]
    global_s[0] = Env.real_state

    try:

        for i in range(max_iteration):
            for j in range(epoch):
                # sample trajectories
                si= global_s[0]
                a= model.sample_action(si,global_s[1:],epi=0.8)
                # do action and sample experience here
                print('action: ', a)
                # share (s,a) pair at time t, where s: st, a: at
                ws= model.w_local.tolist()
                Env.send_states(t=t, state=global_s[0],a=a, p=ws)
                time.sleep(2)
                id, global_s, global_a, global_d, global_p = Env.read_global_s(timestep=t, param=None)

                # update visit count in map
                for sj in global_s:
                    grid_s = Env.get_gridState(sj)
                    Env.cnt_map[grid_s[0],grid_s[1]] +=1

                # step a
                real_s_old, grid_s_new, s_new, r, is_terminal = Env.step(a)
                print("Grid state: ", grid_s_new)
                print("real state: ", s_new)
                print("reward: ", round(r, 3))
                print("Terminal: ", is_terminal)

                # update local w of Q function
                w_local = model.crtic_step(global_s[0], a, s_new, global_s[1:], global_a[1:], r).tolist()

                Env.send_states(t=t, state=s_new, p=w_local)
                time.sleep(2)
                id, global_s, global_a, global_d, global_p = Env.read_global_s(timestep=t, param=w_local)
                # update global w
                model.update_w_gbl(global_d[0], global_p[1:], global_d[1:])

                # update policy
                model.actor_step(global_s[0],global_a[0], global_s[1:], global_a[1:])
                t += 1
                track.append(grid_s_new)

                print()
                print()
                print("Global States: ")
                for i in range(len(id)):
                    print('==========================')
                    print("id's:", id[i])
                    print("state: ", global_a[i])
                    print("action:", global_a[i])
                    print("degree: ", global_d[i])
                    print("Params: ", global_p[i])
                    print('==========================')
                    print()

                if is_terminal:
                    break
                # update global learning model
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
            Env.send_states(t=t,state= s_new,p=w)
            #
            # # receive info from other agents
            # # Learning model parameters
            # # delay few seconds to update data of agents
            time.sleep(2)
            id, global_s, global_a, global_d, global_p = Env.read_global_s(timestep=t, param=w)
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

    Env = GridWorld(id)
    # run_agent(Env)
    actor_critic(Env, max_iteration=10, num_agents=2)
    # test_json(Env)
    # Env.terminate()


