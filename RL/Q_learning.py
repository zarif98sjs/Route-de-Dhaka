import random

class Q:
    def __init__(self, Reward, Q, alpha, epsilon, n_episodes):
        self.R = Reward
        self.Q = Q
        self.alpha = alpha
        self.epsilon = epsilon
        self.episodes = n_episodes



    # changes the Q values for one single move
    # this uses TD(0) method   
    def update_Q_R(self, state_current, state_next):
        reward = self.R[state_current][state_next]
        q = self.Q[state_current][state_next]
        # TODO: state_next of destination node doesn't exist if unidirectional 
        # HOW TO SOLVE  --> just added a value if non exist done in preprocessing
        q = q + self.alpha * (reward + min(self.Q[state_next].values()) - q)
        self.Q[state_current][state_next] = round(q,6)



    def key_of_min_value(self, q):
        min_value = min(q.values())
        return [key for key,value in q.items() if value == min_value]



    # TODO : test decaying epsilon
    def simulate_episode_QR(self, start, end):
        state_current = start
        state_next = -1
        
        iteration = 0 #for decay_e
        
        # end was a list in preimplemented version 
        while state_next != end and iteration <10000: # TODO : check whether any problem
            #input("Press to continue")
            valid_moves = list(self.Q[state_current].keys())
            
            iteration += 1;
            
            if len(valid_moves) == 0:
                break 
            elif len(valid_moves) == 1:
                state_next = valid_moves[0]
            else:
                best_action = random.choice(self.key_of_min_value(self.Q[state_current]))
                # print("best_action : "+str(best_action))
                if random.random() < 1/(iteration+1): # use (1/(iteration+1)) for decay_e
                    valid_moves.pop(valid_moves.index(best_action))
                    state_next = random.choice(valid_moves)
                else:
                    state_next = best_action
                # print("next state : "+str(state_next))

            self.update_Q_R(state_current, state_next)
            state_current = state_next
            #print(self.Q)



    # TODO : nodes_number ???
    # This is Q_learning 
    def Q_Routing(self, start, end):
        for e in range(1,self.episodes+1):
            if e%100 == 0:
                print("In episode : "+str(e)+"/"+str(self.episodes))
            self.simulate_episode_QR(start, end)
        return self.Q
    
