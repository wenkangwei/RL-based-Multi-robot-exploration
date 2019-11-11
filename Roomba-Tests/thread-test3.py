# import  random
# import threading as thread
# import json
# def a_func():
#     import  time
#     init_t = time.time()
#     cur_t =init_t
#     t=0
#     data = {0:1,1:0,2:1,3:[1,2,3]}
#     data1 = {0: 1, 1: 0, 2: 1, 3: [1, 2, 3]}
#     while True:
#         try:
#             # print('time:', time.time()-init_t)
#             if cur_t-init_t >=1:
#                 file = open('a.json', 'w')
#                 if file.writable():
#                     data[1] += 1
#                     txt= json.dumps(data)
#                     data1[3]= [round(random.random(),1)for i in range(3)]
#                     txt += '#'
#                     txt += json.dumps(data1)
#                     print('a json:',txt)
#                     # print('Load: ',json.loads(txt))
#                     file.write(txt)
#                 else:
#                     print("a: Can't write")
#                 file.close()
#                 init_t =cur_t
#             cur_t = time.time()
#             # file.write(txt)
#         except KeyboardInterrupt:
#             # file.close()
#             break
#     pass
# def b_func():
#     import time
#     cur_t = time.time()
#     init_t =cur_t
#     while True:
#         try:
#             if cur_t-init_t >=0.5:
#                 file2 = open('a.json', 'r')
#                 if file2.readable():
#                     s1 = file2.read()
#                     s1 = s1.split('#')
#                     for s in s1:
#                         print("Read Str: ", s)
#                         if len(s)>1:
#                             # dataform = str(s).strip("'<>() ").replace('\'', '\"')
#                             print('s:',s)
#                             s0= json.loads(s)
#                             print('b Read: ', s0)
#                 else:
#                     print('b: Can\'t thread')
#                 file2.close()
#                 init_t =cur_t
#             cur_t =time.time()
#
#             # file2.write(txt)
#         except KeyboardInterrupt:
#             # file2.close()
#             break
#
#     pass
# if __name__ == '__main__':
#     a = thread.Thread(target=a_func)
#
#     b =thread.Thread(target=b_func)
#     a.start()
#     # a.join()
#     # a.run()
#     b.start()
#     # b.join(timeout=2)
#     # b.run()



from multiprocessing import Process

def f(q):
    t=0
    import time as tim
    while True:
        try:
            s, r,a = [1,2,1], 4, [1,4]
            q.put((s,r,a))
            print("=================P1 put: ",s, r,a)
            import time
            tim.sleep(3)
            t+=1
            print("P1 t:",t)
        except KeyboardInterrupt:
            break

def f2(q):
    t=0
    while True:
        try:
            x= q.get()
            if x is not None:
                print("====================================P2 get: ",x )
            t+=1
            import time as ti
            # ti.sleep(0.1)
            print("P2 t:", t)
            pass
        except KeyboardInterrupt:
            break
    pass

if __name__ == '__main__':
    # import multiprocessing as mp
    # from multiprocessing import Queue
    # # from queue import Queue
    # mp.set_start_method('fork')
    # q = Queue()
    #
    # # par1, child = mp.Pipe()
    # p1 = Process(target=f, args=(q,))
    #
    # p2 = Process(target=f2, args=(q,))
    # p1.start()
    # p2.start()
    t= 0
    while True:
        import time
        import os
        time.sleep(0)
        pid= os.getpid()
        print("I'm your Child2 : ",pid)
        t +=1
        # data = input()
        print("Child 2 t: ",t)
        # print("Child Receive: ",data)
        # import random
        # t= random.choice(range(3))
        # time.sleep()

