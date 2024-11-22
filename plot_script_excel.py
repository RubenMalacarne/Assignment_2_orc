import os
import csv
import matplotlib.pyplot as plt

# Percorso principale dove si trovano le sottocartelle con i file CSV
main_path = "/home/ruben/orc_folder/Assignment/A2__template"

# Liste per salvare le traiettorie
x_trajectories = []
y_trajectories = []
time_steps = []
computation_times = []

def scanning_csv_file(main_path):
    x_trajectories = []
    y_trajectories = []
    time_steps = []
    computation_times = []
    for root, _, files in os.walk(main_path):
        for file in files:
            if file.endswith(".csv"):
                
                x = []
                y = []
                comp_times = []
                t_steps = []
                file_path = os.path.join(root, file)
                
                # Legge il file CSV
                with open(file_path, mode='r') as f:
                    reader = csv.DictReader(f)
                    
                    for row in reader:
                        t_steps.append(int(row["time_step"]))
                        comp_times.append(float(row["computation_time"]))
                        x.append(float(row["X_trajectory"]))
                        y.append(float(row["Y_trajectory"]))
                    
                x_trajectories.append(x)
                y_trajectories.append(y)
                time_steps.append(t_steps)
                computation_times.append(comp_times)
    
    return   time_steps,computation_times,x_trajectories,y_trajectories

time_steps,computation_times,x_trajectories,y_trajectories = scanning_csv_file(main_path)


plt.figure(figsize=(10, 6))
for i, (x, y) in enumerate(zip(x_trajectories, y_trajectories)):
    plt.plot(x, y, label=f'trajectory: {i+1}')

plt.title("trajectory ee")
plt.xlabel("X Trajectory")
plt.ylabel("Y Trajectory")
plt.legend()
plt.grid()
plt.show()

# Crea il grafico del tempo di calcolo rispetto alle iterazioni
plt.figure(figsize=(10, 6))
for i, (t_steps, comp_times) in enumerate(zip(time_steps, computation_times)):
    plt.plot(t_steps, comp_times, label=f'File {i+1}')

# Personalizzazione del grafico
plt.title("Tempo di Calcolo rispetto alle Iterazioni")
plt.xlabel("Iterazioni (time_step)")
plt.ylabel("Tempo di Computazione (s)")
plt.legend()
plt.grid()
plt.show()
