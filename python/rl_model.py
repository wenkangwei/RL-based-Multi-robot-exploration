import  numpy as np
import random
import  math
def f_dist_change(si,ai, s,a):
    """
    Note angle of both action and state should be in   degree
    :param s: list of states of all other agents , [ [x, y, theta], [x, y, theta] ....]
    :param si: the sette of ith agent, [x, y, theta]
    :param ai:  action of ith agent
    :param a: the action of other agents
    :return: a list of distance change between agent i and other agents
    """
    # dist_c_ls = []
    avg_dist_c = 0.0
    if len(s) >0:
        for sj in s:
            d1 = np.sqrt(np.square(si[0]- sj[0]) + np.square(si[1]- sj[1]))
            x = si[0] + ai[0] * math.cos(math.radians(si[2]) - 0.5 * math.radians(ai[1]))
            y = si[1] + ai[0] * math.sin(math.radians(si[2]) - 0.5 * math.radians(ai[1]))
            d2 =np.sqrt(np.square(x- sj[0]) + np.square(y- sj[1]))
            # dist_c_ls.append(d1-d2)
            avg_dist_c += (d1-d2)
        avg_dist_c /= len(s)

    return  avg_dist_c

def f_dist_obstacles(si,ai,s,a, obs_ls, max_dis):
    """

    :param si:  the state of ith agent, [x, y, theta]
    :param ai:  action of ith agent
    :param obs_ls: list of obstacle position.  The postion here must be real position
    :param max_dis: max distance from agent to obstacle
    :return:
        feature related to distance to the closest obstacle
    """
    C= max_dis
    d_obs_ls = []
    # si[0] = float(si[0])
    # si[1] = float(si[1])
    # ai[0] = float(ai[0])

    if len(obs_ls)>0:
        x = si[0] + ai[0] * math.cos(math.radians(si[2]) - 0.5 * math.radians(ai[1]))
        y = si[1] + ai[0] * math.sin(math.radians(si[2]) - 0.5 * math.radians(ai[1]))

        for o in obs_ls:
            d = np.sqrt(np.square(x - o[0]) + np.square(y - o[1]))
            d_obs_ls.append(d)

        d_obs = np.min(d_obs_ls)
        # feature of distance to the closest obstacle
        # f_d_obs = d_obs/float(max_dis)
        # as distance to obs -> infinity feature =0, no need to learn
        # ,otherwise it goes to -1 as distance decrease, close to obstacles
        f_d_obs = -C / (C + d_obs)
    else:
        f_d_obs = 0.0
    return  f_d_obs


def f_grid_cnt(si, s,cnts):
    """
    :param si: state of agent i
    :param s: global states of all other agents, not including ith agent
    :param cnts: list of visit count of positions s
    :return:
    """
    cnt = 0.0
    C = 1.0
    for c in cnts:
        cnt +=  c

    cnt /= (len(s)+1)
    f_cnt = C/ (C + cnt)
    return f_cnt

def dist_agents(si,ai,s,a):
    """
    return maximum and  minimum distance to other agents/ distance to the closest/ futherest agents
    :param si: ith ageent
    :param ai: actions a of ith agent
    :param s: states of all agents, including si
    :param a: actions a of other agents
    :return:
    """
    min_dist = 0.0
    max_dist =0.0
    mean_d = 0.0
    ls = []
    if len(s)>0:
        for sj in s:
            d = np.sqrt(np.square(si[0] - sj[0]) + np.square(si[1] - sj[1]))
            ls.append(d)
        max_dist = np.max(ls)
        min_dist = np.min(ls)
        mean_d = np.mean(ls)

    return max_dist, min_dist,mean_d

class actor_critic_q():
    def __init__(self,input_size, map_shape,obstacles,action_set ,discount=1):
        """

        :param input_size: size/number of features x(s,a)
        :param map:
        :param obstacles:
        :param action_set:
        :param discount:
        """
        self.input_size =input_size
        self.w_local=np.ones([1,input_size],dtype=float)/input_size
        self.w_global = np.ones([1,input_size],dtype=float) / input_size
        self.theta = np.ones([1,input_size],dtype=float) / input_size
        # return at t
        self.ret_t = 0.0
        # return at t+1
        self.ret_t1 = 0.0
        self.td_err = 0.0
        self.actions = action_set
        # beta of w
        self.beta_w = 0.1
        self.beta_theta = 0.1
        self.cnts= []
        self.obs_ls = obstacles
        self.map_shape =map_shape
        pass
    def sample_action(self,si,s,epi=0.5):

        if random.random() >epi:
            prob = [self.policy_estimator(si, ai, s, []) for ai in self.actions]
            a = self.actions[np.argmax(prob)]
            pass
        else:
            a = random.choice(self.actions)
        return  a
    def get_features(self, si, ai, s, a):
        """
        This is a function to update feature x(s,a) used for approximation in RL
        :param si:
        :param s:
        :param a:
        :param obs_ls:
        :param map: grid map with visit count at each piece
        :return: feature vector X
        """
        x = []
        max_d, min_d, mean_d = dist_agents(si,ai,s,a)
        x.append(max_d)
        x.append(min_d)
        x.append(mean_d)
        f1 = f_dist_change(si,ai, s, a)
        x.append(f1)
        sh = self.map_shape
        max_dist = np.sqrt(sh[0] * sh[0] + sh[1] * sh[1])
        f2 = f_dist_obstacles(si,ai, s,a, self.obs_ls, max_dist)
        x.append(f2)
        f3 = f_grid_cnt(si, s,self.cnts)
        x.append(f3)
        x.append(1)
        x = np.array(x, dtype=float).reshape([1,self.input_size])
        return x

    def q_estimator(self,si,ai,s,a):
        """
        approximate global q function after merging weights from different agents
        :return:
        """
        # obtain features
        x = self.get_features(si,ai,s,a)
        # linear approximator

        q= np.matmul(self.w_global, np.transpose(x))
        return q

    def q_gradient(self,x):
        """
        if linear approximator for q function, derivative of Q(s,a) =x
        :param x:
        :return:
        """
        return x

    def crtic_step(self,si,ai,si_new,s,a,s_new,r_t1,terminal):
        """
        Perform critic step, updating return, TD error and local weights in Q
        :param si:
        :param ai:
        :param s:
        :param a:
        :param s_new: st+1 of other agents
        :param r_t1:
        :param ws:
        :param ds:
        :return:
        """
        x = self.get_features(si,ai,s,a)
        # update local return
        self.ret_t1 = (1 - self.beta_w)*self.ret_t + self.beta_w*r_t1
        # compute TD error
        if not terminal:
            q_t1 = np.max([self.q_estimator(si_new,a_next,s_new,a) for a_next in self.actions])
        else:
            q_t1 = 0.0
        q_t = self.q_estimator(si, ai, s, a)
        # update TD error
        self.td_err = (r_t1+ q_t1)-(self.ret_t+ q_t)
        # update local weights
        self.w_global += self.beta_w*self.td_err*self.q_gradient(x)
        # Need to share w_local to other agents after learning
        return self.w_global

    def update_w_gbl(self,deg_i, deg_ls,w_ls):
        """
        Update global weights for global q function
        :param deg_i degree of agent i
        :param w_ls:list of q function weights of all agents with len n
        :param deg_ls: list of degree of other agents. It should have shape 1xn
        :return:
        updated global weight
        """
        if len(w_ls) != len(deg_ls)+1:
            print("Sizes of Degree list and Weight list are different!")
            return None

        # self.w_global = np.zeros([1, self.input_size])
        w_global = np.zeros([1, self.input_size])
        w_ls = np.array(w_ls)
        c_sum = 0.0
        for j, d in enumerate(deg_ls):
            print("d:",d)
            print("j:",j)
            cj=  1.0/(1.0 + max(deg_i, d))
            c_sum += cj
            w_global += np.multiply(cj,w_ls[j])

        self.w_global = (1-c_sum)*self.w_global + w_global
        self.w_local =self.w_global
        return self.w_global


    def actor_step(self,si,ai,s,a):
        """
        Actor step for updating theta
        :param si:
        :param ai:
        :param s:
        :param a:
        :return:
        """
        self.theta +=  self.beta_theta* self.advantage(si,ai,s,a)*self.phi(si,ai,s,a)
        return self.theta


    def phi(self,si,ai,s,a):
        """
        For softmax policy estimator,
        phi: derivative of log(pi(s,a)) in term of theta
            = x(s,ai) - sum(x(s,aj)) along aj
        :param si:
        :param ai:
        :param s:
        :return:
        """
        # Note: in policy approximation, it only cares si,s,ai
        #  the feature x here can be different from features in q function
        x_ai = self.get_features(si, ai, s, a)
        x_sum = np.sum([self.get_features(si, aj, s, a) for aj in self.actions])
        phi = x_ai - x_sum
        return phi

    def advantage(self,si,ai,s,a):
        q_weighted = lambda si,aj,s,a :\
            self.policy_estimator(si,aj,s,a)*self.q_estimator(si,aj,s,a)

        v_i = np.sum([q_weighted(si,aj,s,a) for aj in self.actions])
        adv =self.q_estimator(si,ai,s,a) - v_i
        return adv

    def policy_estimator(self,si,ai,s,a):
        """

        :param s:  s global state
        :param ai:  action of the ith agent
        :return:
        probability p(ai|s) that agent i take action a at global states s
        """
        prob = 0.0
        e= np.exp(np.matmul(self.theta,np.transpose(self.get_features(si, ai, s, a))))
        e_sum = 0.0
        for aj in self.actions:
            e_sum += np.exp(np.matmul(self.theta,np.transpose(self.get_features(si, ai, s, aj))))
        print("e:",e,"e sum:",e_sum)
        prob =np.round(e/e_sum,3)
        print("a:",a, "Prob:",prob)
        return prob







class actor_critic_rv():
    def __init__(self, input_size, discount=1):
        self.input_size = input_size
        self.w = np.ones(input_size) / input_size
        # return at t
        self.ret_t = 0
        # return at t+1
        self.ret_t1 = 0
        self.td_err = 0
        # beta of w
        self.beta_w = 1
        self.beta_theta = 1
        pass


    def policy_estimator(s, ai):
        """

        :param s:  s global state
        :param ai:  action of the ith agent
        :return:
        probability p(ai|s) that agent i take action a at global states s
        """
        prob = 0

        return prob


    def rv_approximate(self):
        """
        approximate reward and value function
        :return:
        """
        pass