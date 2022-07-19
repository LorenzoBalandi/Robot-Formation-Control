# LB, FS - DAS 2022 Project
# LAUNCH FILE

from launch import LaunchDescription
from launch_ros.actions import Node
import numpy as np
import networkx as nx
import os
from ament_index_python.packages import get_package_share_directory

'''
From this launch file you can select the formation of the agents.
Formation type:
    square               <--- square formation, fixed
    moving_square        <--- square formation, leaders move with constant velocity (piecewise constant)
    letters              <--- the agents forms, one, after the other, the letters of the word BALA
    moving_square_circle <--- square formation, leaders move following circular pattern (TIME-VARYING LEADERS VELOCITY CASE)

The formation type will influence: 
    -the number of agents required to do it
    -the number of iterations (time) required to see it well
    -the control gains
    -the initial conditions of the leaders
    -initial conditions of the follower (they are random in all cases, but in letter formation is more sparse)
The number of leaders is always 2.
'''

formation_type = 'square' # <---- modify this to change formation type
RVIZ = True

def compute_adj(NN,p_ER):

    I_NN = np.eye(NN) # np.identity
    Adj = np.random.binomial(1,p_ER, (NN,NN)) # matrix in which each element is 0 or 1 with p_ER probability
    #print(Adj)
    Adj = np.logical_or(Adj,Adj.T) # get a simmetric matrix
    #print(Adj)
    Adj = np.multiply(Adj, np.logical_not(I_NN)).astype(int) # remove self loops by multiplying by a matrix with 0 on diags

    print("Adjacency:\n", Adj)

    return Adj

def compute_adj_square():
    return np.array([
        [0, 1, 0, 1],
        [1, 0, 1, 1],
        [0, 1, 0, 1],
        [1, 1, 1, 0]
    ])

def generate_launch_description():
    
    COMM_TIME = 5e-2 # communication time period

    global formation_type
    global RVIZ

    if formation_type == 'square':
        MAXITERS = 750
        NN = 4
        Kp = 110
        Kv = 110
        x_init_leaders = np.array([0.0, 0.0, 0.0, 1.0]) # define initial positions for the leaders (fixed positions)
        Adj = compute_adj_square()

    elif formation_type == 'moving_square':
        MAXITERS = 1500
        NN = 4
        Kp = 110
        Kv = 110
        x_init_leaders = np.array([0.0, 0.0, 0.0, 1.0])
        Adj = compute_adj_square()

    elif formation_type == 'letters':
        RVIZ = False # not supported
        MAXITERS = 2000 # 500 iters for each letter
        NN = 10
        Kp = 50
        Kv = 35
        x_init_leaders = np.array([0.0, 0.0, 0.0, 10.0])
        Adj = compute_adj(NN,0.7)

    elif formation_type == 'moving_square_circle':
        MAXITERS = 10000
        NN = 4
        Kp = 25
        Kv = 25
        x_init_leaders = np.array([0.0, 0.0, 0.0, 1.0])
        Adj = compute_adj_square()

    else:
        print("Insert a valid formation type!")
        exit() # terminate the program

    # first Nl agents are leaders, other agents are Nf followers
    Nl = 2 # number of leaders
    Nf = NN-Nl # number of followers
    n_x = 2 # dimension of x_i
    # n_v = n_x # dimension of v_i (same as x_i)
    # n_u = n_x # dimension of u_i (same as x_i)

    # define initial positions for the followers (random positions)
    x_init_followers = np.random.rand(n_x*Nf)*5
    # initial velocities are assumed to be 0 for all agents
    x_init = np.hstack((x_init_leaders,x_init_followers))

    launch_description = [] # Append here your nodes

    if RVIZ == True:
        rviz_config_dir = get_package_share_directory('my_bearing_formation')
        rviz_config_file = os.path.join(rviz_config_dir, 'rviz_config.rviz')

        launch_description.append(
            Node(
                package='rviz2', 
                node_executable='rviz2', 
                arguments=['-d', rviz_config_file],
                # output='screen',
                # prefix='xterm -title "rviz2" -hold -e'
                ))
    
    for ii in range(NN):
        Adj_ii = Adj[:, ii].tolist() # extract column ii
        N_ii = np.nonzero(Adj[:, ii])[0].tolist() # indices of neighbors of agent ii
        ii_index = ii*n_x + np.arange(n_x)

        if ii < Nl: # if agent is a leader
            agent_type = "leader"
            x_init_ii = x_init[ii_index].flatten().tolist()
        else:
            agent_type = "follower"
            x_init_ii = x_init[ii_index].flatten().tolist()

        launch_description.append(
            Node(
                package='my_bearing_formation',
                node_namespace ='agent_{}'.format(ii),
                node_executable='agent_i',
                parameters=[{
                                'agent_id': ii, 
                                'max_iters': MAXITERS, 
                                'communication_time': COMM_TIME, 
                                'neigh': N_ii, 
                                'Adj_ii' : Adj_ii,
                                'x_init': x_init_ii,
                                'agent_type' : agent_type, # "leader" or "follower"
                                'Kp' : Kp, # position control gain
                                'Kv' : Kv, # velocity control gain
                                'NN' : NN, # total number of agents
                                'formation_type' : formation_type # formation type (required for bearings computation)
                                }],
                output='screen',
                prefix='xterm -title "agent_{}" -hold -e'.format(ii)
            ))
        if RVIZ == True:
            launch_description.append(
                Node(
                    package='my_bearing_formation', 
                    node_namespace='agent_{}'.format(ii),
                    node_executable='visualizer', 
                    parameters=[{
                                    'agent_id': ii,
                                    'communication_time': COMM_TIME,
                                    }],
                ))
            
    return LaunchDescription(launch_description)