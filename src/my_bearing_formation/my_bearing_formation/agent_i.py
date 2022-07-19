# LB, FS, June 2022
# DAS Project 1 Task 2
# AGENT_I

from time import sleep
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray as msg_float

'''
Possibili improvements:
- calcolare P_star solo per i neighbours, ora ogni agente lo calcola rispetto a tutti
  gli altri agenti ma poi è usato solo quello dei neighbors
- verificare l'unicità della formazione controllando la non singolarità di Bff (vedi paper)
'''

radius = 2
angular_vel = 0.001
vel = 0.005 #
time = 0 # variable used to describe the circle that leaders follow

'''
Sincronizzazione risolta
'''

############################### FUNCTIONS ###############################
def formation_update(tt, x_init, agent_type, dt, x_i, neigh, data, P_star, i, Kp, Kv, formation_type):
    # solo i follower devono eseguire questo (cambiare in agent_i)
    """
      dt    = discretization step
      x_i   = state of agent i (n_x positions, n_x velocities)
      neigh = list of neihbors
      data  = state of neighbors
      a_i  = i-th row of the adjacency matrix 
    """
    global time # time comes from the global scope

    # LEADERS
    if agent_type == "leader":
        for j in neigh:
            data[j].pop(0) # we have to pop from the list to avoid synchronization problems
        if formation_type == 'moving_square':
            if tt > 750:
                x_i[0] = -vel*time # x position of leader
                x_i[2] = -vel # x velocity of leader
                time += 1
        elif formation_type == 'moving_square_circle':
            if tt > 750 and tt < 8000:
                # define the circle to be followed by leaders
                x_i[0] = radius*np.cos(angular_vel*time) + x_init[0] - radius  # x pos
                x_i[1] = radius*np.sin(angular_vel*time) + x_init[1] # y pos
                x_i[2] = -radius*angular_vel*np.sin(angular_vel*time) # x vel
                x_i[3] = radius*angular_vel*np.cos(angular_vel*time) # y vel
                x_i[4] = -radius*(angular_vel**2)*np.cos(angular_vel*time) # x acc
                x_i[5] = -radius*(angular_vel**2)*np.sin(angular_vel*time) # y acc
                time += 1

        return x_i    
  
    # FOLLOWERS (leaders won't arrive here because they encounter a return before)
    n_x = len(x_i)//3 # number of velocities and positions
    p_i = x_i[0:n_x]
    v_i = x_i[n_x:2*n_x]
    a_i = x_i[2*n_x:]
    pdot_i = np.zeros(p_i.shape)
    vdot_i = np.zeros(v_i.shape) # derivative of the velocity

    if formation_type != 'moving_square_circle':
        for j in neigh:
            #print("data[j] " , data[j]) # [[0,0,0,0,0]]
            #print("\tlen data of neigh {}: {} ".format(j,len(data[j])))
            #print("\tdata of neigh {} : {} ".format(j,data[j]) )
            x_j = np.array(data[j].pop(0)[1:]) # warning: pop remove the elements, the length will change!
            #print("data j after pop: " , data[j])
            p_j = x_j[0:n_x]
            v_j = x_j[n_x:2*n_x]
            #print("pj: " , p_j)
            #print("vj: " , v_j)

            # check if the diagonals are to be considered
            vdot_i += - P_star[i*n_x:i*n_x+n_x, j*n_x:j*n_x+n_x] @ (Kp*(p_i-p_j)+Kv*(v_i-v_j)) # from the paper
            # xdot_i += - a_i[j]*(x_i - x_j) # from slides of laplacian consensus

        pdot_i = v_i
        # Forward Euler
        p_i += dt*pdot_i
        v_i += dt*vdot_i
        # x_i += dt*xdot_i
        x_i = np.hstack((p_i,v_i,a_i))

    else:
        KKi = np.zeros((n_x,n_x))
        for j in neigh:
            x_j = np.array(data[j].pop(0)[1:]) # warning: pop remove the elements, the length will change!
            p_j = x_j[0:n_x]
            v_j = x_j[n_x:2*n_x]
            a_j = x_j[2*n_x:]
            KKi += P_star[i*n_x:i*n_x+n_x, j*n_x:j*n_x+n_x]
            vdot_i += - P_star[i*n_x:i*n_x+n_x, j*n_x:j*n_x+n_x] @ (Kp*(p_i-p_j)+Kv*(v_i-v_j) - a_j) # from the paper
            print(np.linalg.det(KKi))

        vdot_i = np.linalg.inv(KKi) @ vdot_i
        pdot_i = v_i
        # Forward Euler
        p_i += dt*pdot_i
        v_i += dt*vdot_i
        a_i = vdot_i
        # x_i += dt*xdot_i
        x_i = np.hstack((p_i,v_i,a_i))


    return x_i


def writer(file_name, string):
    """
      inner function for logging
    """
    file = open(file_name, "a") # "a" is for append
    file.write(string)
    file.close()


def define_formation(NN,shape,n_x):
    """
      define the points that represent the letter to show,
      needed only in case of 'letters' formation!
    """
    xx = np.zeros((NN,n_x)) # NN rows, n_x columns

    if shape == 'A': # letter A
        xx[0] = np.array([0,0])
        xx[1] = np.array([0,10])
        xx[2] = np.array([0,2.5])
        xx[3] = np.array([0,5])
        xx[4] = np.array([0,7.5])
        xx[5] = np.array([2.5,5])
        xx[6] = np.array([2.5,7.5])
        xx[7] = np.array([5,5])
        xx[8] = np.array([7.5,2.5])
        xx[9] = np.array([10,0])

    elif shape == 'B': # letter B
        xx[0] = np.array([0,0])
        xx[1] = np.array([0,10])
        xx[2] = np.array([5,2.5])
        xx[3] = np.array([1.5,3.75])
        xx[4] = np.array([0,5])
        xx[5] = np.array([2.5,6.25])
        xx[6] = np.array([5,7.5])
        xx[7] = np.array([1.25,8.75])
        xx[8] = np.array([2.5,1.25])
        xx[9] = np.array([0,7.5])

    elif shape == 'L': # letter L
        xx[0] = np.array([0,0])
        xx[1] = np.array([0,10])
        xx[2] = np.array([0,3.32])
        xx[3] = np.array([0,4.98])
        xx[4] = np.array([0,6.64])
        xx[5] = np.array([0,8.3])
        xx[6] = np.array([0,1.66])
        xx[7] = np.array([1.66,0])
        xx[8] = np.array([3.32,0])
        xx[9] = np.array([5,0])
    else:
        print("Error: unknown shape!")
    return xx

def compute_P_star(NN,n_x,formation_type,letter):
    '''
    NN: total number of agents
    n_x: dimension of state of each agent
    formation_type: type of formation to compute
    letter: the letter to show (ignored if not 'letters' formation type)
    '''
    # compute matrix of desired bearings
    g_star = np.zeros((NN,NN,n_x))
    P_star = np.zeros((NN*n_x,NN*n_x)) # orthogonal projection matrix of the desired bearings
    xx = np.zeros((NN,n_x)) # NN rows, n_x columns
    # desired positions (formation)
    if formation_type != 'letters':
        xx[0] = np.array([0.0, 0.0])
        xx[1] = np.array([0.0, 1.0])
        xx[2] = np.array([1.0, 1.0])
        xx[3] = np.array([1.0, 0.0])
    else:
        xx = define_formation(NN,letter,n_x)

    for ii in range(NN):
        for jj in range(NN):
            if ii != jj: # bearings of an agent wrt itself have no meaning
                g_star[ii,jj,:] = xx[jj]-xx[ii]
                norm = np.linalg.norm(xx[jj]-xx[ii])
                if norm == 0:
                    print("ERROR: NORM IS 0")
                if norm != 0: # check if the norm is 0
                    g_star[ii,jj,:] = g_star[ii,jj,:]/norm
                #P_gij = np.eye(n_x) - g_star[ii,jj]@g_star[ii,jj].T
                P_gij = np.eye(n_x) - np.outer(g_star[ii,jj],g_star[ii,jj].T)
                #print(np.shape(P_gij))
                P_star[ii*n_x:ii*n_x+n_x, jj*n_x:jj*n_x+n_x] = P_gij # compose the matrix by adding sub-matrices
    
    #print("g_star:")
    #for i in range(n_x):
        #print(g_star[:,:,i])
    
    #print("P_star:")
    #print(np.shape(P_star))
    #print(P_star)

    return P_star


############################### AGENT NODE ###############################

class Agent(Node): # inherited from class Node
    def __init__(self):
        # take constructor of class Node and modify it
        super().__init__('agent',
                            allow_undeclared_parameters=True,
                            automatically_declare_parameters_from_overrides=True)
            
        # Get parameters from launcher
        self.agent_id = self.get_parameter('agent_id').value
        self.neigh = self.get_parameter('neigh').value
        self.agent_type = self.get_parameter('agent_type').value # "leader" or "follower"
        print("I'm a ", self.agent_type)
        
        Adj_ii = self.get_parameter('Adj_ii').value
        self.Adj_ii = np.array(Adj_ii) # it returns an n_x by 1 array
        
        # construct the state: x = [positions, velocities]
        #x_i = self.get_parameter('x_init').value
        self.x_init = self.get_parameter('x_init').value
        self.n_x = len(self.x_init)
        self.x_i = np.array(self.x_init) # it returns an n_x by 1 array
        v_init = np.zeros(self.n_x) # initial velocities are 0
        a_init = np.zeros(self.n_x) # initial accelrations are 0
        self.x_i = np.hstack((self.x_i, v_init, a_init)) # add velocities and acc to state, now the state has dim 3*n_x
        print("Initial state: " , self.x_i)
       
        self.max_iters = self.get_parameter('max_iters').value
        self.communication_time = self.get_parameter('communication_time').value
        self.NN = self.get_parameter('NN').value # each agent must know the total number of agents
        self.Kp = self.get_parameter('Kp').value
        self.Kv = self.get_parameter('Kv').value
        self.formation_type = self.get_parameter('formation_type').value
        print("\nFormation type:")
        print(self.formation_type)
        self.tt = 0

        # Desired bearings
        # g_star is a matrix with element i,j computed as in the paper, elements on diagonal are 0
        self.P_star = compute_P_star(self.NN,self.n_x,self.formation_type,'B')

        # create logging file
        self.file_name = "_csv_file/agent_{}.csv".format(self.agent_id)
        file = open(self.file_name, "w+") # 'w+' needs to create file and open in writing mode if doesn't exist
        file.close()

        # initialize subscription dictionary
        self.subscriptions_list = {} # curly brackets --> dictionary

        # create a subscription to each neighbor
        for j in self.neigh:
            topic_name = '/topic_{}'.format(j)
            self.subscriptions_list[j] = self.create_subscription(
                                                                msg_float, 
                                                                topic_name, 
                                                                lambda msg, node = j: self.listener_callback(msg, node), 
                                                                10)
        
        # create the publisher
        self.publisher_ = self.create_publisher(
                                                                msg_float, 
                                                                '/topic_{}'.format(self.agent_id),
                                                                10)

        self.timer = self.create_timer(self.communication_time, self.timer_callback)

        # initialize a dictionary with the list of received messages from each neighbor j [a queue]
        self.received_data = { j: [] for j in self.neigh }

        print("Setup of agent {} complete".format(self.agent_id))


    def listener_callback(self, msg, node):
        self.received_data[node].append(list(msg.data))

    def timer_callback(self):
        # Initialize a message of type float
        msg = msg_float()

        if self.tt == 0: # Let the publisher start at the first iteration
            msg.data = [float(self.tt)]

            # for element in self.x_i: 
            #     msg.data.append(float(element))
            [msg.data.append(float(element)) for element in self.x_i]

            self.publisher_.publish(msg)
            self.tt += 1

            # log files
            # 1) visualize on the terminal
            string_for_logger = [round(i,4) for i in msg.data.tolist()[1:]]
            print("Iter = {} \t Value = {}".format(int(msg.data[0]), string_for_logger))

            # 2) save on file
            data_for_csv = msg.data.tolist().copy()
            data_for_csv = [str(round(element,4)) for element in data_for_csv[1:self.n_x+1]] # pick only the position
            data_for_csv = ','.join(data_for_csv)
            writer(self.file_name,data_for_csv+'\n')

        else: 
            # Check if lists are nonempty
            all_received = all(self.received_data[j] for j in self.neigh) # check if all neighbors' have been received

            sync = False
            # Have all messages at time t-1 arrived?
            if all_received:
                sync = all(self.tt-1 == self.received_data[j][0][0] for j in self.neigh) # True if all True

            if sync:
                DeltaT = self.communication_time/10

                # In case of letters formation we have to recompute the bearings when we change the letter to show
                if self.formation_type == 'letters':
                # Compute desired bearings
                    if (self.tt == 500):
                        self.P_star = compute_P_star(self.NN,self.n_x,self.formation_type,'A')
                    elif (self.tt == 1000):
                        self.P_star = compute_P_star(self.NN,self.n_x,self.formation_type,'L')
                    elif (self.tt == 1500):
                        self.P_star = compute_P_star(self.NN,self.n_x,self.formation_type,'A')

                self.x_i = formation_update(self.tt, self.x_init, self.agent_type, DeltaT, self.x_i, self.neigh, 
                                            self.received_data, self.P_star, self.agent_id, self.Kp, self.Kv, self.formation_type)

                # publish the updated message
                msg.data = [float(self.tt)]
                [msg.data.append(float(element)) for element in self.x_i]
                self.publisher_.publish(msg)

                # save data on csv file
                data_for_csv = msg.data.tolist().copy()
                data_for_csv = [str(round(element,4)) for element in data_for_csv[1:self.n_x+1]] # pick only the position
                data_for_csv = ','.join(data_for_csv)
                writer(self.file_name,data_for_csv+'\n')

                string_for_logger = [round(i,4) for i in msg.data.tolist()[1:]]
                print("Iter = {} \t Value = {}".format(int(msg.data[0]), string_for_logger))
                
                # Stop the node if tt exceeds MAXITERS
                if self.tt > self.max_iters:
                    print("\nMAXITERS reached")
                    sleep(3) # [seconds]
                    self.destroy_node()

                # update iteration counter
                self.tt += 1

############################### MAIN ###############################

def main(args=None):
    rclpy.init(args=args)

    agent = Agent()
    print("Agent {:d} -- Waiting for sync.".format(agent.agent_id))
    sleep(0.5)
    print("GO!\n")

    try:
        rclpy.spin(agent)
    except KeyboardInterrupt:
        print("----- Node stopped cleanly -----")
    finally:
        rclpy.shutdown() 

if __name__ == '__main__':
    main()