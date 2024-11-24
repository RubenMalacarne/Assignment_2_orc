import os
import csv
import matplotlib.pyplot as plt

main_path = "/home/ruben/orc_folder/Assignment/A2__template/results_third_part_3"
wall_y = 0.05


x_trajectories = []
y_trajectories = []
time_steps = []
computation_times = []

def scanning_csv_file(main_path):
    x_trajectories = []
    y_trajectories = []
    time_steps = []
    computation_times = []
    tracking_error_mean = []
    velocity_results= []

    for root, _, files in os.walk(main_path):
        for file in files:
            if file.endswith(".csv"):
                
                x = []
                y = []
                comp_times = []
                t_steps = []
                track_error_mean = []
                velocity_result = []
                
                file_path = os.path.join(root, file)
                
                with open(file_path, mode='r') as f:
                    reader = csv.DictReader(f)
                    
                    for row in reader:
                        t_steps.append(int(row["time_step"]))
                        comp_times.append(float(row["computation_time"]))
                        x.append(float(row["X_trajectory"]))
                        y.append(float(row["Y_trajectory"]))
                        track_error_mean.append(float(row["tracking_error_mean"]))
                        velocity_result.append(float(row["velocity_result"]))
                x_trajectories.append(x)
                y_trajectories.append(y)
                time_steps.append(t_steps)
                computation_times.append(comp_times)
                tracking_error_mean.append(track_error_mean)
                velocity_results.append(velocity_result)
                
    return   time_steps,computation_times,x_trajectories,y_trajectories,tracking_error_mean,velocity_results

time_steps,computation_times,x_trajectories,y_trajectories,tracking_error_mean,velocity_results = scanning_csv_file(main_path)


# PLOT STUFF

fig, axz = plt.subplots(2, 2, figsize=(14, 6))
grid_style = {
    "color": "gray",
    "linestyle": "--",
    "linewidth": 0.5,
    "alpha": 0.7
}


for i, (x, y) in enumerate(zip(x_trajectories, y_trajectories)):
    axz[0,0].plot(x, y, label=f'trajectory: {i+1}')
    axz[0,0].axhline(y=wall_y, color='r', linestyle='--')
    axz[0,0].set_title("trajectory ee")
    axz[0,0].set_xlabel("X Trajectory")
    axz[0,0].set_ylabel("Y Trajectory")
    axz[0,0].legend()
    axz[0,0].grid(**grid_style)

# plt.figure(figsize=(10, 6))
for i, (t_steps, comp_times) in enumerate(zip(time_steps, computation_times)):
    axz[0,1].plot(t_steps, comp_times, label=f'File {i+1}')
    axz[0,1].set_title("Computation time compare to iteration")
    axz[0,1].set_xlabel("Iteration (time_step)")
    axz[0,1].set_ylabel("Computation_time  (s)")
    axz[0,1].legend()
    axz[0,1].grid(**grid_style)
plt.tight_layout()
# plt.show()



# plt.figure(figsize=(10, 6))
for i, (t_steps, tracking_error_mean) in enumerate(zip(time_steps, tracking_error_mean)):
    axz[1,0].plot(t_steps, tracking_error_mean, label=f'File {i+1}')
    axz[1,0].set_title("tracking error for each iteration")
    axz[1,0].set_xlabel("tracking error(time_step)")
    axz[1,0].set_ylabel("Computation_time  (s)")
    axz[1,0].legend()
    axz[1,0].grid(**grid_style)
plt.tight_layout()


# plt.figure(figsize=(10, 6))
for i, (t_steps, velocity_results) in enumerate(zip(time_steps, velocity_results)):
    axz[1,1].plot(t_steps, velocity_results, label=f'File {i+1}')
    axz[1,1].set_title("velocity result for each iteration")
    axz[1,1].set_xlabel("velocity")
    axz[1,1].set_ylabel("Computation_time  (s)")
    axz[1,1].legend()
    axz[1,1].grid(**grid_style)
plt.tight_layout()


if not (os.path.exists(main_path) ):
    os.makedirs(main_path)

file_name = os.path.join(main_path, f'combination_result_{3}.png')
plt.savefig(file_name)
plt.show()
