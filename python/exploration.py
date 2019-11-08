from Room_Env import *
import numpy as np
import math
import  random
import socket

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

if __name__ == '__main__':

    Env =GridWorld()
    hn = socket.gethostname()
    # obtain hostname of raspberry pi
    xbee = Xbee(int(hn[-1]))

    action_set = Env.action_space
    # a = random.choice(action_set)
    track =[]
    try:

        # test_observation(Env)

        set = [random.choice(Env.action_space) for i in range(5)]
        for a in set:
            # Check if messages from  other agent available
            while xbee.Available():
                is_updated, id, data =  xbee.read()
                if not is_updated:
                    break
                print('id:{}, data: {}'.format(id, data))


            # Training global Q value/ V state value here

            # do action and sample experience here
            print('action: ', a)
            s_new, r, is_terminal = Env.step(a)
            track.append(s_new)

            if Env.is_map_updated() and len(Env.obs_ls)>0:
                # update parameters of learning model
                # share parameter if local map is updated
                xbee.send(Env.obs_ls)
                pass
        pass
    except KeyboardInterrupt:
        Env.terminate()
        xbee.close()
        print('obstacles：')
        print(Env.obs_ls[0])
        print('Track:')
        for i in track:
            print(i)

    Env.terminate()
    xbee.close()
    print('obstacles：')
    print(Env.obs_ls[0])
    print('Track:')
    for i in track:
        print(i)


