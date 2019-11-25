import  matplotlib.pyplot as plt
import json
import  numpy as np

def load_data():
    trajectory_ls = []
    obs_ls = []
    bonus_ls=[]
    cum_reward =[]
    coverage_ls = []
    path = "../Roomba_1/"
    # file_name= "Trajectory"
    # fp = open(path+file_name+".txt", "r")
    # str_data = fp.read()
    # d_ls = str_data.split("\n")
    # for d in d_ls:
    #     if len(d)>3 and "{"==d[0] and "}"==d[-1]:
    #         data = json.loads(d)
    #         s, sn, a, t, terminal = data["s"],data["sn"],data["a"],data["step"],data["terminal"]
    #         trajectory_ls.append((t,s, sn, a, terminal))
    # fp.close()
    #
    # file_name = "obstacles"
    # fp = open(path + file_name + ".txt", "r")
    # str_data = fp.read()
    # d_ls = str_data.split("\n")
    # for d in d_ls:
    #     if len(d) > 3 and "{" == d[0] and "}" == d[-1]:
    #         data = json.loads(d)
    #         t, obs = data["step"], data["obs"]
    #         obs_ls.append((t, obs))
    # fp.close()
    #
    # file_name = "coverage"
    # fp = open(path + file_name + ".txt", "r")
    # str_data = fp.read()
    # d_ls = str_data.split("\n")
    # for d in d_ls:
    #     if len(d) > 3 and "{" == d[0] and "}" == d[-1]:
    #         data = json.loads(d)
    #         t, coverage = data["step"], data["coverage"]
    #         coverage_ls.append((t, coverage))
    # fp.close()
    #
    # file_name = "Bonus"
    # fp = open(path + file_name + ".txt", "r")
    # str_data = fp.read()
    # d_ls = str_data.split("\n")
    # for d in d_ls:
    #     if len(d) > 3 and "{" == d[0] and "}" == d[-1]:
    #         data = json.loads(d)
    #         t, bonus = data["step"], data["bonus"]
    #         bonus_ls.append((t, bonus))
    # fp.close()
    #
    # file_name = "cum_reward"
    # fp = open(path + file_name + ".txt", "r")
    # str_data = fp.read()
    # d_ls = str_data.split("\n")
    # for d in d_ls:
    #     if len(d) > 3 and "{" == d[0] and "}" == d[-1]:
    #         data = json.loads(d)
    #         t, r = data["step"], data["r"]
    #         cum_reward.append((t, r))
    # fp.close()

    file_ls = ["Trajectory","obstacles","Bonus","coverage","cum_reward"]
    data_ls = [trajectory_ls,obs_ls,bonus_ls,coverage_ls,cum_reward]
    keys = [["step","s","sn","a","terminal"],
            ["step","obs"],
            ["step", "bonus"],
            ["step", "coverage"],
            ["step", "r"]]
    for ind, file_name in enumerate(file_ls):
        fp = open(path + file_name + ".txt", "r")
        str_data = fp.read()
        d_ls = str_data.split("\n")
        for d in d_ls:
            if len(d) > 3 and "{" == d[0] and "}" == d[-1]:
                data = json.loads(d)
                temp = []
                for k in keys[ind]:
                    temp.append(data[k])
                data_ls[ind].append(tuple(temp))
        fp.close()


    return trajectory_ls, bonus_ls,obs_ls,coverage_ls,cum_reward

