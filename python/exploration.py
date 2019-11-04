from Room_Env import *
import numpy as np
import math
import  random
if __name__ == '__main__':

    Env =GridWorld()

    action_set = Env.action_space
    a = random.choice(action_set)
    print('action: ',a)
    try:
        while True:

            if Env.Roomba.Avaliable():
                Env.Roomba.StartQueryStream(7, 43, 44, 45, 46, 47, 48, 49, 50, 51)  # Start getting bumper values
                old_real_state, new_real_state, r, is_terminal = Env.observe_Env()
                print(old_real_state, new_real_state, r, is_terminal)
        # s_new, r, is_terminal = Env.step(a)
    except KeyboardInterrupt:
        Env.terminate()

    Env.terminate()

