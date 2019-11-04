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
        s_new, r, is_terminal = Env.step(a)
    except KeyboardInterrupt:
        Env.terminate()

    Env.terminate()

