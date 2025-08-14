# Luke Orioli
import serial
import time
import platform
import os
import threading
import winsound

CSV_FILE_NAME = "exercise_data.csv"

# Get the file path for the exercise data.
script_dir = os.path.dirname(os.path.abspath(__file__))
csv_filepath = os.path.join(script_dir, CSV_FILE_NAME)

if os.path.exists(csv_filepath) is False:
    # Create a new exercise data CSV file.
    with open(csv_filepath, 'w') as file:
        file.write('Tilt,Touch,Magnet,Acc X,Acc Y,Acc Z,Gyro X,Gyro Y,Gyro Z,Movement,Movement Count\n')

# Determine the operating system
os_type = platform.system()

# Set the serial port based on the OS
if os_type == "Windows":
    port = "COM4"  # Adjust this to the correct port on your PC
elif os_type == "Darwin":  # macOS
    port = "/dev/tty.usbmodem11101" # Adjust this to the correct port on your Mac - to find it "ls /dev/tty*"
else:
    raise Exception("Unsupported OS")

# Open a serial connection to the Pico W
s = serial.Serial(port, 115200)

MAX_NUM_SAMPLES = 300

data_collection_active = True
recording_paused = True
sample_count = 0
exercise_movements_remaining = 0
current_exercise = ""
current_movement = ""
current_exercise_movement = ""
exercise_movement_reps = {}

def collect_data():
    global data_collection_active, recording_paused, sample_count, exercise_movements_remaining, current_exercise, current_movement, current_exercise_movement
    with open(csv_filepath, 'a') as file:
        while data_collection_active is True:
            if recording_paused is False:
                try:
                    while s.in_waiting > 0:
                        # Read exercise data from the Pico W.
                        response = s.readline().decode().strip()
                        if len(response) > 0:
                            if "DATA" in response:
                            # Strip the response type from the data.
                                _, _, data_str = response.partition("[DATA] ")

                                # Split the exercise data.
                                tilt, touch, magnet, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, _flex = data_str.split(',')

                                # Update the sample count.
                                sample_count += 1
                                
                                print("Sample Number:", sample_count,
                                    "\tExercise (Movement):", current_exercise_movement,
                                    "\tRepetition:", exercise_movement_reps[current_exercise_movement],
                                    "\tTilt:", tilt,
                                    "\tTouch:", touch,
                                    "\tMagnet:", magnet,           
                                    "\tAcc X:", acc_x,
                                    "\tAcc Y:", acc_y,
                                    "\tAcc Z:", acc_z,
                                    "\tGyro X:", gyro_x,
                                    "\tGyro Y:", gyro_y,
                                    "\tGyro Z:", gyro_z)
                                
                                # Write the exercise data to the CSV file.
                                file.write(f'{tilt},{touch},{magnet},{acc_x},{acc_y},{acc_z},{gyro_x},{gyro_y},{gyro_z},{current_exercise_movement},{exercise_movement_reps[current_exercise_movement]}\n')

                                if sample_count == MAX_NUM_SAMPLES:
                                    if exercise_movements_remaining > 0:
                                        exercise_movements_remaining -= 1
                                        if current_movement == "Upwards":
                                            current_movement = "Downwards"
                                        elif current_movement == "Downwards":
                                            current_movement = "Upwards"

                                        current_exercise_movement = current_exercise + f" ({current_movement})"

                                        if current_exercise_movement not in exercise_movement_reps.keys():
                                            exercise_movement_reps[current_exercise_movement] = 1
                                        else:
                                            exercise_movement_reps[current_exercise_movement] += 1
                                            
                                        sample_count = 0

                                        winsound.Beep(2000, 500)

                                        print(f"Starting recording {exercise_movement_reps[current_exercise_movement]} for exercise (movement) {current_exercise_movement}...")
                                    else:
                                        recording_paused = True
                                        break
                            else:
                                print(f"[PICO] {response}")
                except Exception as e:
                    print(f"Error: {e}")
            else:
                s.reset_input_buffer()

            # Sleep for 1 ms before servicing the serial buffer.
            time.sleep(0.001)

# Start the data collection thread.
data_collection_thread = threading.Thread(target=collect_data)
data_collection_thread.start()

# Repeatedly read user input and update the movement when instructed.
while True:
    try:
        # Wait for the user to change the movement type.
        user_input = input("Enter a movement (push, pull, curl, press, lat, rest) <num_reps>: ")

        if len(user_input) > 0:
            input_strs = user_input.split()

            exercise = input_strs[0]
            
            exercise_movements_remaining = 1

            if len(input_strs) == 2:
                if exercise == "rest":
                    exercise_movements_remaining = int(input_strs[1]) - 1
                else:
                    exercise_movements_remaining = 2 * int(input_strs[1]) - 1
            elif len(input_strs) > 2:
                print("Too many inputs.")
                continue

            if exercise == "push":
                current_exercise = "Push-Up"
                current_movement = "Downwards"
            elif exercise == "pull":
                current_exercise = "Pull-Up"
                current_movement = "Upwards"
            elif exercise == "curl":
                current_exercise = "Bicep-Curl"
                current_movement = "Upwards"
            elif exercise == "press":
                current_exercise = "Shoulder-Press"
                current_movement = "Upwards"
            elif exercise == "lat":
                current_exercise = "Lateral-Raise"
                current_movement = "Upwards"
            elif exercise == "rest":
                current_exercise = "Rest"
                current_movement = ""
            else:
                print("Invalid exercise type.")
                continue

            current_exercise_movement = current_exercise + f" ({current_movement})"

            if current_exercise_movement not in exercise_movement_reps.keys():
                exercise_movement_reps[current_exercise_movement] = 1
            else:
                exercise_movement_reps[current_exercise_movement] += 1
                
            sample_count = 0
            
            print(f"Starting recording {exercise_movement_reps[current_exercise_movement]} for exercise (movement) {current_exercise_movement} in 5 seconds...")

            for second in range(5):
                winsound.Beep(2000, 150)
                time.sleep(1.0)
                print(f"Starting in {4 - second} seconds...")

            winsound.Beep(2000, 500)

            recording_paused = False

            while recording_paused is False:
                time.sleep(1.0)
            print(f"Finished recording exercise (movement) {current_exercise_movement}.")
            print(f"There are currently {exercise_movement_reps[current_exercise_movement]} recorded repetitions for exercise {current_exercise}.")

            for beep in range(3):
                time.sleep(0.1)
                winsound.Beep(2000, 150)

    except KeyboardInterrupt:
        # Instruct the data collection thread to terminate.
        data_collection_active = False
        # Wait for the data collection thread to terminate.
        time.sleep(0.5)
        break

s.close()
print("Data recording stopped.")