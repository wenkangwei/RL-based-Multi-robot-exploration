import subprocess

if __name__ =='__main__':
    child1= subprocess.Popen(['python3','exploration.py'])
    child2 = subprocess.Popen(['python3', 'comm_model.py'])
    t =0

    while True:
        try:
            # print("==============I'm parent==========")
            import time
            # time.sleep(5)
            pass
            t +=1

        except KeyboardInterrupt:
            child1.kill()
            child2.kill()
            break