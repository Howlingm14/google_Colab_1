import numpy as np
import random
from environment import Env
from collections import defaultdict
import os
import matplotlib.pyplot as plt

cwd = os.getcwd() # Get the current working directory (cwd)
files = os.listdir(cwd) # Get all the files in that directory



reward=0


class QLearningAgent:
    def __init__(self, actions):
        # actions = [0, 1, 2, 3]
        self.actions = actions
        self.learning_rate = 0.01 #was 0.01
        self.discount_factor = 0.9 #was 0.9
        self.epsilon = 0.3# was 0.1
        self.q_table = defaultdict(lambda: [0.0, 0.0, 0.0, 0.0])
        global qarr
        qarr=self.q_table

    # update q function with sample <s, a, r, s'>
    def learn(self, state, action, reward, next_state):
        current_q = self.q_table[state][action]
        # using Bellman Optimality Equation to update q function
        new_q = reward + self.discount_factor * max(self.q_table[next_state])
        self.q_table[state][action] += self.learning_rate * (new_q - current_q)
        global qarr
        qarr=self.q_table
        #print(self.q_table)

    # get action for the state according to the q function table
    # agent pick action of epsilon-greedy policy
    def get_action(self, state):
        #self.epsilon=self.epsilon*0.99995
        if np.random.rand() < self.epsilon:
            # take random action
            action = np.random.choice(self.actions)
        else:
            # take action according to the q function table
            state_action = self.q_table[state]
            action = self.arg_max(state_action)
        return action

    @staticmethod
    def arg_max(state_action):
        max_index_list = []
        max_value = state_action[0]
        for index, value in enumerate(state_action):
            if value > max_value:
                max_index_list.clear()
                max_value = value
                max_index_list.append(index)
            elif value == max_value:
                max_index_list.append(index)
        return random.choice(max_index_list)
    
    def q_table(self):
        print(self.q_table)



if __name__ == "__main__":
    env = Env()
    agent = QLearningAgent(actions=list(range(env.n_actions)))
    
    global qarr
    print(qarr)
    
    episode_plot=[]
    reward_sum_plot=[]
    trial_plot=[]
    
    for episode in range(100):
        state = env.reset()
        
        if episode%20==0:
            print("Q_table")
            print(qarr)
        
        n=200
        for trial in range(n):
            env.render()

            # take action and proceed one step in the environment
            action = agent.get_action(str(state))
            next_state, reward, done, reward_sum = env.step(action)

            # with sample <s,a,r,s'>, agent learns new q function
            agent.learn(str(state), action, reward, str(next_state))

            state = next_state
            #env.print_value_all(agent.q_table)

            if trial==n-1:
                done=True
            
            # if episode ends, then break
            if done:
                
                print("episode:", episode, "   trials completed:", trial, "    reward:", reward_sum)
                
                episode_plot=np.concatenate((episode_plot, [episode]))
                reward_sum_plot = np.concatenate((reward_sum_plot, [reward_sum]))
                trial_plot = np.concatenate((trial_plot, [trial]))
                
                
                break
    




f=plt.figure(1)
plt.plot(episode_plot, reward_sum_plot)
f.show()

g=plt.figure(2)
plt.plot(episode_plot, trial_plot)
g.show()

#agent.q_table()

plt.pause(0)