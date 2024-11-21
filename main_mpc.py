import numpy as np
import matplotlib.pyplot as plt
from adam.casadi.computations import KinDynComputations
import casadi as cs
from time import time as clock
from termcolor import colored

import plot_utils as plut
from example_robot_data.robots_loader import load, load_full
import conf_ur5 as conf_ur5
from robot_simulator import RobotSimulator
from robot_wrapper import RobotWrapper      

import matplotlib
import csv
import os
import statistics

print("Load robot model")
robot, _, urdf, _ = load_full("ur5")

print("Create KinDynComputations object")
joints_name_list = [s for s in robot.model.names[1:]]   # skip the first name because it is "universe"
end_effector_frame_name = "wrist_3_link"
nq = len(joints_name_list)                              # number of joints
nx = 2*nq                                               # size of the state variable
kinDyn = KinDynComputations(urdf, joints_name_list)
forward_kinematics_ee = kinDyn.forward_kinematics_fun(end_effector_frame_name)

DO_WARM_START = True
SOLVER_TOLERANCE = 1e-4
SOLVER_MAX_ITER = 3

SIMULATOR = "pinocchio"
POS_BOUNDS_SCALING_FACTOR = 0.2
VEL_BOUNDS_SCALING_FACTOR = 2.0
CONTROL_BOUNDS_SCALING_FACTOR = 1.00 ### TODO: 0.45 ###
qMin = POS_BOUNDS_SCALING_FACTOR * robot.model.lowerPositionLimit
qMax = POS_BOUNDS_SCALING_FACTOR * robot.model.upperPositionLimit
vMax = VEL_BOUNDS_SCALING_FACTOR * robot.model.velocityLimit
tauMin = -robot.model.effortLimit * CONTROL_BOUNDS_SCALING_FACTOR
tauMax = robot.model.effortLimit * CONTROL_BOUNDS_SCALING_FACTOR

dt_sim = 0.002
N_sim = 100

dt = 0.010          # time step MPC




def avarage_computation(x,  var_name="x"):
    mean_value = np.mean(x) 
    print (f"mean value of {var_name} ",mean_value)
    return mean_value


def plot_y_trajectory(trajectory_x,trajectory_y,wall_y,dividend):
    #plot graph with trajectory along y axis    
    plt.figure(figsize=(10, 6))
    for i in range(len(trajectory_x)-1):
        color  = "orange" if trajectory_y[i] <=wall_y else "green"
        plt.plot(trajectory_x[i:i+2], trajectory_y[i:i+2],color=color)
    plt.axhline(y=wall_y, color='r', linestyle='--')

    plt.title('trajectory along plane xy robot')
    
    if not (os.path.exists(path) ):
        os.makedirs(path)
    
    subfolder = os.path.join(path, f"result_for_{dividend}")
    if not os.path.exists(subfolder):
        os.makedirs(subfolder)
    file_name = os.path.join(subfolder, f'result_{dividend}.png')
    plt.savefig(file_name)
        
    
def save_csv_file(dividend,**data_dict):
    # save result in csv
    column_names = list(data_dict.keys())
    
    if not (os.path.exists(path) ):
        os.makedirs(path)
    
    subfolder = os.path.join(path, f"result_for_{dividend}")
    if not os.path.exists(subfolder):
        os.makedirs(subfolder)
    
    file_name = os.path.join(subfolder, f'result_{dividend}.csv')
    
    with open(file_name, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(column_names)
        for row in zip(*data_dict.values()):
            writer.writerow(row)  
  
all_choice = [2,4,10]


for dividend in all_choice:

    q0 = np.zeros(nq)   # initial joint configuration
    dq0= np.zeros(nq)   # initial joint velocities

    p_ee_des = np.array([-0.1, 0.1, -0.6]) # desired end-effector position

    wall_y = 0.05      # y position of the wall

    w_p = 1e2           # position weight
    w_v = 0e-6          # velocity weight
    w_a = 1e-5          # acceleration weight
    w_final_v = 0e0     # final velocity cost weight
    USE_TERMINAL_CONSTRAINT = 0 ### TODO: 1 ###



    r = RobotWrapper(robot.model, robot.collision_model, robot.visual_model)
    simu = RobotSimulator(conf_ur5, r)
    simu.init(q0, dq0)
    simu.display(q0)
        


    print("Create optimization parameters")
    ''' The parameters P contain:
        - the initial state (first 12 values)
        - the target configuration (last 6 values)
    '''
    opti = cs.Opti()
    param_x_init = opti.parameter(nx)
    param_p_ee_des = opti.parameter(3)
    cost = 0

    # create the dynamics function
    q   = cs.SX.sym('q', nq)
    dq  = cs.SX.sym('dq', nq)
    ddq = cs.SX.sym('ddq', nq)
    state = cs.vertcat(q, dq)
    rhs    = cs.vertcat(dq, ddq)
    f = cs.Function('f', [state, ddq], [rhs])

    # create a Casadi inverse dynamics function
    H_b = cs.SX.eye(4)     # base configuration
    v_b = cs.SX.zeros(6)   # base velocity
    bias_forces = kinDyn.bias_force_fun()
    mass_matrix = kinDyn.mass_matrix_fun()
    # discard the first 6 elements because they are associated to the robot base
    h = bias_forces(H_b, q, v_b, dq)[6:]
    M = mass_matrix(H_b, q)[6:,6:]
    tau = M @ ddq + h
    inv_dyn = cs.Function('inv_dyn', [state, ddq], [tau])

    # pre-compute state and torque bounds
    lbx = qMin.tolist() + (-vMax).tolist()
    ubx = qMax.tolist() + vMax.tolist()
    tau_min = (tauMin).tolist()
    tau_max = (tauMax).tolist()

    # our variable ########## modification
    trajectory_x = []
    trajectory_y = []
    list_computation_time = []
    iteration = []
    tracking_error = []
    status_step = []
    velocity_result = []
    path = "results"


    N = int((N_sim)/dividend)         # horizon length MPC ### TODO: ###
    # create all the decision variables
    X, U = [], []
    X += [opti.variable(nx)] # do not apply pos/vel bounds on initial state
    for k in range(1, N+1): 
        X += [opti.variable(nx)]
        opti.subject_to( opti.bounded(lbx, X[-1], ubx) )
    for k in range(N): 
        U += [opti.variable(nq)]

    print("Add initial conditions")
    opti.subject_to(X[0] == param_x_init)
    for k in range(N):     
        # print("Compute cost function")
        p_ee = forward_kinematics_ee(cs.DM.eye(4), X[k][:nq])[:3,3]
        cost += w_p * (p_ee - param_p_ee_des).T @ (p_ee - param_p_ee_des)
        cost += w_v * X[k][nq:].T @ X[k][nq:]
        cost += w_a * U[k].T @ U[k]

        # print("Add dynamics constraints")
        opti.subject_to(X[k+1] == X[k] + dt * f(X[k], U[k]))
        
        # print("Add cartesian constraints")
        opti.subject_to((p_ee[1] >= wall_y))

        # print("Add torque constraints")
        opti.subject_to( opti.bounded(tau_min, inv_dyn(X[k], U[k]), tau_max))

    # add the final cost
    cost += w_final_v * X[-1][nq:].T @ X[-1][nq:]

    if(USE_TERMINAL_CONSTRAINT):
        opti.subject_to(X[-1][nq:] == 0.0)

    opti.minimize(cost)

    print("Create the optimization problem")
    opts = {
        "error_on_fail": False,
        "ipopt.print_level": 0,
        "ipopt.tol": SOLVER_TOLERANCE,
        "ipopt.constr_viol_tol": SOLVER_TOLERANCE,
        "ipopt.compl_inf_tol": SOLVER_TOLERANCE,
        "print_time": 0,             
        "detect_simple_bounds": True,
        "ipopt.max_iter": 1000
    }
    opti.solver("ipopt", opts)

    # Solve the problem to convergence the first time
    x = np.concatenate([q0, dq0])
    opti.set_value(param_p_ee_des, p_ee_des)
    opti.set_value(param_x_init, x)
    sol = opti.solve()
    opts["ipopt.max_iter"] = SOLVER_MAX_ITER
    opti.solver("ipopt", opts)



    print("Start the MPC loop")
    for i in range(N_sim):
        start_time = clock()

        if(DO_WARM_START):
            # use current solution as initial guess for next problem
            for t in range(N):
                opti.set_initial(X[t], sol.value(X[t+1]))
            for t in range(N-1):
                opti.set_initial(U[t], sol.value(U[t+1]))
            opti.set_initial(X[N], sol.value(X[N]))
            opti.set_initial(U[N-1], sol.value(U[N-1]))

            # initialize dual variables
            lam_g0 = sol.value(opti.lam_g)
            opti.set_initial(opti.lam_g, lam_g0)
        
        print("Time step", i)
        opti.set_value(param_x_init, x)
        try:
            sol = opti.solve()
        except:
            # print("Convergence failed!")
            sol = opti.debug
        end_time = clock()

        print("Comput. time: %.3f s"%(end_time-start_time), 
            "Iters: %3d"%sol.stats()['iter_count'], 
            "Tracking err: %.3f"%np.linalg.norm(p_ee_des-forward_kinematics_ee(cs.DM.eye(4), x[:nq])[:3,3].toarray().squeeze()),
            "return status", sol.stats()["return_status"],
                "dq %.3f" % np.linalg.norm(x[nq:])
        )
        # a = forward_kinematics_ee(cs.DM.eye(4), x[:nq])[1,3]
        # breakpoint()
        trajectory_y.append(float(forward_kinematics_ee(cs.DM.eye(4), x[:nq])[1,3]))
        trajectory_x.append(float(forward_kinematics_ee(cs.DM.eye(4), x[:nq])[0,3]))
        status_step.append(sol.stats()["return_status"])
        list_computation_time.append(end_time-start_time)
        tracking_error.append(p_ee_des-forward_kinematics_ee(cs.DM.eye(4), x[:nq])[:3,3].toarray().squeeze())
        velocity_result.append(np.linalg.norm(x[nq:]))

        tau = inv_dyn(sol.value(X[0]), sol.value(U[0])).toarray().squeeze()

        # do a proper simulation with Pinocchio
        simu.simulate(tau, dt, int(dt/dt_sim))
        x = np.concatenate([simu.q, simu.v])

        if( np.any(x[:nq] > qMax)):
            print(colored("\nUPPER POSITION LIMIT VIOLATED ON JOINTS", "red"), np.where(x[:nq]>qMax)[0])
        if( np.any(x[:nq] < qMin)):
            print(colored("\nLOWER POSITION LIMIT VIOLATED ON JOINTS", "red"), np.where(x[:nq]<qMin)[0])
        if (forward_kinematics_ee(cs.DM.eye(4), x[:nq])[1,3] < wall_y):
            print(colored("\nCOLLISION DETECTED", "red"))
            
    save_csv_file(dividend,time_step=list(range(len(list_computation_time))),
                computation_time=list_computation_time,
                tracking_error=tracking_error,
                Y_trajectory=trajectory_y,
                status_step = status_step,
                velocity_result = velocity_result)
            
    plot_y_trajectory(trajectory_x,trajectory_y,wall_y,dividend)

    mena_list_computation_time = avarage_computation (list_computation_time,"list_computation_time")
    mean_tracking_error = avarage_computation (tracking_error,"tracking_error")
    mean_velocity_result = avarage_computation (velocity_result,"velocity_result")