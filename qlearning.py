import random

class QBot(runner.HdxNode):
    def __init__(self, params, namespace):
        super().__init__('q-node')
        self.sense_node = params.sense_node
        self.namespace = namespace
        self.params = params
        self.q_table = QTable(params)
        self.loops = 0
        self.total_reward = 0
        self.action = 0
        self.create_timer(0.10, self.timer_callback)
        self.last_sense_update = 0

    def timer_callback(self):
        last_sense = self.sense_node.last_update_time()
        if last_sense > self.last_sense_update:
            self.last_sense_update = last_sense
            state = params.state_func(self.sense_node)
            reward = params.reward_func(self.sense_node, state, self.action)
            self.total_reward += reward
            self.action = self.q_table.sense_act_learn(state, reward)
            self.actions[self.action](self.namespace)


class QParameters:
    def __init__(self, sense_node):
        self.sense_node = sense_node
        self.actions = []
        self.num_states = 0
        self.state_func = lambda r: 0
        self.reward_func = lambda r, state, action: 0
        self.target_visits = 1
        self.epsilon = 0.0
        self.discount = 0.5
        self.rate_constant = 10


class QTable:
    def __init__(self, params):
        self.q = [[0.0] * len(params.actions) for i in range(params.num_states)]
        self.visits = [[0] * len(params.actions) for i in range(params.num_states)]
        self.target_visits = params.target_visits
        self.epsilon = params.epsilon
        self.discount = params.discount
        self.rate_constant = params.rate_constant
        self.last_state = 0
        self.last_action = 0

    def sense_act_learn(self, new_state, reward):
        alpha = self.learning_rate(self.last_state, self.last_action)
        update = alpha * (self.discount * self.q[new_state][self.best_action(new_state)] + reward)
        self.q[self.last_state][self.last_action] *= 1.0 - alpha
        self.q[self.last_state][self.last_action] += update

        self.visits[self.last_state][self.last_action] += 1
        if self.is_exploring(new_state):
            new_action = self.least_visited_action(new_state)
        else:
            new_action = self.best_action(new_state)

        self.last_state = new_state
        self.last_action = new_action
        return new_action

    def learning_rate(self, state, action):
        return self.rate_constant / (self.rate_constant + self.visits[state][action])

    def best_action(self, state):
        best = 0
        for action in range(1, len(self.q[state])):
            if self.q[state][best] < self.q[state][action]:
                best = action
        return best

    def is_exploring(self, state):
        return min(self.visits[state]) < self.target_visits or random.random() < self.epsilon

    def least_visited_action(self, state):
        least_visited = 0
        for action in range(1, len(self.visits[state])):
            if self.visits[state][least_visited] > self.visits[state][action]:
                least_visited = action
        return least_visited

