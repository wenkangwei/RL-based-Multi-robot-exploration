import subprocess

if __name__ =='__main__':
    child = subprocess.Popen(['python','thread-test.py'])
    child1 = subprocess.Popen(['python', 'thread-test3.py'])
    t =0

    while True:
        try:
            # print("==============I'm parent==========")
            import time
            # time.sleep(5)
            pass
            t +=1
            # print("##############################################Parent t: ",t)
            # import  random
            # data = {1:'parent',2:[round(random.random(),1) for i in range(3)]}
            # child.communicate(data)
        except KeyboardInterrupt:
            child.kill()
            child1.kill()
            break
    # import os
    # import  subprocess
    # import random
    #
    #
    # def communication(child_writes):
    #     # file descriptors r, w for reading and writing
    #     r, w = os.pipe()
    #
    #     # Creating child process using fork
    #     processid = os.fork()
    #     if processid:
    #         # This is the parent process
    #         # Closes file descriptor w
    #         # os.close(w)
    #         r = os.fdopen(r)
    #         print("Parent reading")
    #         str = r.read()
    #         print("Parent reads =", str)
    #         w = os.fdopen(w)
    #         import  random
    #
    #         data ="Data " #[round(random.random(),2) for i in range(4)]
    #         w.write(data)
    #         r.close()
    #         w.close()
    #     else:
    #         # This is the child process
    #         # os.close(r)
    #         w = os.fdopen(w, 'w')
    #         print("Child writing")
    #         w.write(child_writes)
    #         print("Child writes = ", child_writes)
    #         r= os.fdopen(r, 'r')
    #         data = r.read()
    #         print("Child get: ",data)
    #         w.close()
    #         r.close()
    #
    #         # Driver code
    #
    #
    # child_writes = "Hello geeks"
    # communication(child_writes)
    #
    # # Contributed by "Sharad_Bhardwaj".