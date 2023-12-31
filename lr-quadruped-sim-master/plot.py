import matplotlib.pyplot as plt
import matplotlib.animation as animation
import subprocess
import threading
import queue

NB_REWARD_PART = 9
def is_number(s):
    """ Check if the string is a number. """
    try:
        float(s)
        return True
    except ValueError:
        return False

def data_reader(q, script_path):
    """ Thread function to read and queue data. """
    process = subprocess.Popen(['python', script_path], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)

    while True:
        output = process.stdout.readline()
        if output == '' and process.poll() is not None:
            break
        if output:
            elements = output.strip().split()
            if all(is_number(elem) for elem in elements) and len(elements) == NB_REWARD_PART:
                numbers = [float(num) for num in elements]
                q.put(numbers)  # Put the numbers in the queue
            else:
                print(output)

def update_plot(frame, lines, q, data, max_points):
    """ Update the plot with new data. """
    while not q.empty():
        new_data = q.get()
        data.append(new_data)
        if len(data) > max_points:
            data.pop(0)  # Remove the oldest data point

        x_data = list(range(len(data)))  # Update x-axis data

        for i in range(NB_REWARD_PART):
            y_data = [d[i] for d in data]  # Extract y-data for each function
            lines[i].set_data(x_data, y_data)

        lines[0].axes.relim()  # Update axes limits
        lines[0].axes.autoscale_view()  # Rescale the view

    return lines

def plot_functions(script_path, max_points=3000):
    # Labels for each line
    labels = [
        "vel_tracking_reward", "direction_reward", "energy_penalty",
        "orientation_penalty", "roll_reward", "pitch_reward", "yaw_reward", "drift_reward", "clearance_reward"
    ]

    # Queue to hold the data
    q = queue.Queue()

    # Start the data reader thread
    threading.Thread(target=data_reader, args=(q, script_path), daemon=True).start()

    # Set up the plot
    fig, ax = plt.subplots()
    lines = [ax.plot([], [], label=label)[0] for label in labels]  # Initialize lines with labels
    data = []  # Initialize data storage

    ax.legend()  # Add a legend

    # Create an animation that updates the plot
    ani = animation.FuncAnimation(fig, update_plot, fargs=(lines, q, data, max_points), interval=100)

    plt.show()

# Path to the script that outputs numbers and text
script_path = r'C:\Users\naeld\OneDrive - epfl.ch\EPFL_STUDIES\MA3 - 2023\Legged robotics\Prog\Project2\quadruped_locomotion\lr-quadruped-sim-master\run_sb3.py'

# Run the script and plot its output
plot_functions(script_path)
