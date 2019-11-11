
from multiprocessing import Process

def f(q,p):
    t=0
    # import time as tim
    while True:
        try:
            s, r,a = [1,2,1], 4, [1,4]
            q.put((s,r,a))
            print("###########P1 put: ",s, r,a)
            import time
            time.sleep(3)
            t+=1
            print("P1 t:",t)
            data = p.get()
            print("############P1 receive: ",data)
        except KeyboardInterrupt:
            break

def f2(q,p):
    t=0
    while True:
        try:
            x= q.get()
            if x is not None:
                print("====================================P2 get: ",x )
            t+=1
            import time as ti
            # ti.sleep(2)
            print("P2 t:", t)
            data = "F2 data"
            p.put(data)
            pass
        except KeyboardInterrupt:
            break
    pass

if __name__ == '__main__':

    import multiprocessing as mp
    from multiprocessing import Queue
    import time
    while True:
        print('P1')
        time.sleep(3)
    # from queue import Queue
    # mp.set_start_method('spawn')
    # q = Queue()
    # p = Queue()
    # # par1, child = mp.Pipe()
    # p1 = mp.Process(target=f, args=(q,p,))
    #
    # p2 = mp.Process(target=f2, args=(q,p,))
    # p1.start()
    # p2.start()
