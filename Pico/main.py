# Luke Orioli
from machine import Pin, ADC, SPI, Timer
import sys
import utime
import _thread
from tm1637 import TM1637
from lsm6dso import LSM6DSO 
from flex_sensor import FlexSensor
from digital_sensor import DigitalSensor

# Constant Definitions
VALID_EXERCISES = ["Push-Up", "Pull-Up", "Bicep-Curl", "Shoulder-Press", "Lateral-Raise"]
VALID_MOVEMENTS = ["Downwards", "Upwards"]
EXERCISE_REPETITION_SEQUENCES = {
    "Push-Up": ["Downwards", "Upwards"],
    "Pull-Up": ["Upwards", "Downwards"],
    "Bicep-Curl": ["Upwards", "Downwards"],
    "Shoulder-Press": ["Upwards", "Downwards"],
    "Lateral-Raise": ["Upwards", "Downwards"]
}
EXERCISE_DISPLAY_CODES = {
    "Push-Up": "PUSH",
    "Pull-Up": "PULL",
    "Bicep-Curl": "CURL",
    "Shoulder-Press": "PRSS",
    "Lateral-Raise": "LATR",
    "Rest": "REST"
}
EXERCISE_PREDICTION_WINDOW_SIZE = 6
REPETITION_COUNT_CHECK_TIME_NS = 3E9

# Helper Function Definitions
def split_exercise_movement(exercise_movement: str) -> tuple[str, str]:
    try:
        exercise, movement = exercise_movement.split()
        movement = movement[1:-1]
        return (exercise, movement)
    except:
        pass
    return ("", "")

def client_print(print_type: str, output: str) -> None:
    sys.stdout.write(f"[{print_type}] {output}\n")

# Global Variable Definitions
exercise_movement_prediction_buffer = []
current_exercise = "Rest"
current_movement = ""
current_exercise_reps = 0
exercise_data_lock = _thread.allocate_lock()

# TM1637 LED Display Initialization
print("Initializing TM1637 LED display...")
tm_clk = clk=Pin(5)
tm_dio = dio=Pin(4)
display = TM1637(tm_clk, tm_dio)
display.show('----')

# Vibrating Motor Module Initialization
print("Initializing vibrating motor module...")
motor = Pin(12, Pin.OUT)
motor.value(1)
utime.sleep(0.2)
motor.value(0)
utime.sleep(0.1)
motor.value(1)
utime.sleep(0.2)
motor.value(0)

# LSM6DSO IMU Initialization
print("Initializing LSM6DSO IMU sensor...")
imu_spi_bus = SPI(0, baudrate=10000000)
imu_cs = Pin(20, mode=Pin.OUT, value=1)
imu = LSM6DSO(imu_spi_bus, imu_cs)
imu_config_attempts = 5
while imu_config_attempts > 0:
    try:
        imu.configure()
        print("Successfully configured LSM6DSO IMU.")
        break
    except AssertionError:
        print(f"Error configuring LSM6DSO IMU, remaining retries: {imu_config_attempts}")
        imu_config_attempts -= 1
        utime.sleep(1.0)

if imu_config_attempts == 0:
    print("Failed to configure LSM6DSO IMU sensor.")
    utime.sleep(5.0)

# Flex Sensor Initialization
print("Initializing flex sensor...")
flex_sensor_calibration_table = {0.0: 1.0E8,
                                    10.0: 8.2E7,
                                    20.0: 4.1E7,
                                    30.0: 25E3,
                                    40.0: 7.5E3,
                                    50.0: 1.15E3,
                                    60.0: 700,
                                    70.0: 500,
                                    80.0: 420,
                                    90.0: 360,
                                    180.0: 100}
flex_sensor_adc_pin = ADC(26)
flex_sensor = FlexSensor(flex_sensor_adc_pin, 3.3, 11000, flex_sensor_calibration_table)

# Tilt Sensor Initialization
print("Initializing tilt sensor...")
tilt_sensor_pin = Pin(14, Pin.IN)
tilt_sensor = DigitalSensor(tilt_sensor_pin, active_low=True)

# Touch Sensor Initialization
print("Initializing touch sensor...")
touch_sensor_pin = Pin(15, Pin.IN)
touch_sensor = DigitalSensor(touch_sensor_pin, active_low=False)

# Reed Switch (Magnet Sensor) Initialization
print("Initializing magnet sensor...")
magnet_sensor_pin = Pin(13, Pin.IN)
magnet_sensor = DigitalSensor(magnet_sensor_pin, active_low=True)

# Sample Timer Initialization
print("Initializing periodic sample timer...")

def read_sample(Source):
    acc = imu.read_acc()
    gyro = imu.read_gyro()
    tilt = tilt_sensor.value()
    touch = touch_sensor.value()
    magnet = magnet_sensor.value()
    flex = flex_sensor.get_flex()
    client_print("DATA", f"{tilt},{touch},{magnet},{acc[0]},{acc[1]},{acc[2]},{gyro[0]},{gyro[1]},{gyro[2]},{flex}\n")

sample_timer = Timer()
sample_timer.init(mode=Timer.PERIODIC, period=5, callback=read_sample)

# Exercise Repetition Tracking Thread Definitions
print("Initializing and starting exercise repetition tracking thread...")

def track_exercise_repetitions():
    global current_exercise, current_exercise_reps, exercise_data_lock

    last_rep_count_update_time_ns = utime.time_ns()
    last_rep_count = 0

    while True:
        exercise_data_lock.acquire()
        cur_exercise = current_exercise
        cur_reps = current_exercise_reps
        exercise_data_lock.release()

        if cur_reps > 0:
            # get the current time.
            current_time_ns = utime.time_ns()

            # Check if the exercise repetition count has
            # changed since the last time it was checked.
            if cur_reps > last_rep_count:
                # Display the exercise code if the user has only performed a single repetition.
                # Otherwise, display the current repetition.
                if cur_reps == 1:
                    display.show(EXERCISE_DISPLAY_CODES[cur_exercise])
                else:
                    display.number(cur_reps)
                client_print("DEBUG", f"New {current_exercise} Repetition: {cur_reps}")
                
                # Turn on the vibrating motor for a half second to 
                # indicate that a new repetition has been detected.
                motor.value(1)
                utime.sleep(0.5)
                motor.value(0)

                # Update the last repetition count.
                last_rep_count = cur_reps
                last_rep_count_update_time_ns = current_time_ns
                    
            # Check if the exercise is still active 
            # (i.e. the repetitions are not static).
            # If not, reset the repetitions.
            if current_time_ns - last_rep_count_update_time_ns > REPETITION_COUNT_CHECK_TIME_NS:
                exercise_data_lock.acquire()
                current_exercise_reps = 0
                last_rep_count = 0
                exercise_data_lock.release()
                client_print("DEBUG", "Resetting Repetitions...")
        else:
            display.show('----')
        
        utime.sleep_ms(5)

_thread.start_new_thread(track_exercise_repetitions, ())

# Main Program Loop
print("Finished initialization, starting main loop...")
while True:
    try:
        # Monitor the serial port for new exercise predictions.
        exercise_movement = sys.stdin.readline().strip()

        exercise, movement = split_exercise_movement(exercise_movement)

        if len(exercise) > 0:
            if ((exercise in VALID_EXERCISES) and (movement in VALID_MOVEMENTS)) or (exercise == "Rest"):
                client_print("DEBUG", f"New Exercise Prediction (Movement): {exercise_movement}")
                exercise_movement_prediction_buffer.append(exercise_movement)
            
            if len(exercise_movement_prediction_buffer) >= EXERCISE_PREDICTION_WINDOW_SIZE:
                # Count the frequency of each predicted exercise movement to determine 
                # which is the most likely exercise being performed by the user.
                exercise_movement_freqs = {}
                for exercise_movement in exercise_movement_prediction_buffer:
                    if exercise_movement not in exercise_movement_freqs.keys():
                        exercise_movement_freqs[exercise_movement] = 1
                    else:
                        exercise_movement_freqs[exercise_movement] += 1

                # Sort the exercise movements in order of most to least number of predictions.
                sorted_exercise_movements = sorted(exercise_movement_freqs, key=lambda k: exercise_movement_freqs[k], reverse=True)
                client_print("DEBUG", f"sorted_exercise_movements: {sorted_exercise_movements}")

                # Get the exercise with the most predictions made.
                predicted_exercise_movement = sorted_exercise_movements[0]
                predicted_exercise_movement_freq = exercise_movement_freqs[predicted_exercise_movement]

                # Check if another exercise had the same number of predictions.
                next_predicted_exercise_movement_freq = 0
                if len(sorted_exercise_movements) > 1:
                    next_predicted_exercise_movement = sorted_exercise_movements[1]
                    next_predicted_exercise_movement_freq = exercise_movement_freqs[next_predicted_exercise_movement]

                # Only update the current exercise movement if the newest exercise 
                # movement in the window had the most number of predictions.
                if (exercise_movement == predicted_exercise_movement) or \
                ((exercise_movement == next_predicted_exercise_movement) and \
                 (predicted_exercise_movement_freq == next_predicted_exercise_movement_freq)):
                #if predicted_exercise_movement_freq >= next_predicted_exercise_movement_freq:
                    
                    # Save the previous exercise movement.
                    prev_exercise = current_exercise
                    prev_movement = current_movement

                    # Update the current exercise movement.
                    exercise_data_lock.acquire()
                    current_exercise, current_movement = split_exercise_movement(exercise_movement)
                    exercise_data_lock.release()

                    client_print("DEBUG", f"Predicted Window Exercise (Movement): {exercise_movement}")
                        
                    if current_exercise != "Rest":
                        # Check if the new exercise matches the previous exercise.
                        if current_exercise == prev_exercise:
                            # Check if the new exercise movement completes a repetition for the current exercise.
                            exercise_start_motion = EXERCISE_REPETITION_SEQUENCES[current_exercise][0]
                            exercise_end_motion = EXERCISE_REPETITION_SEQUENCES[current_exercise][1]

                            client_print("DEBUG", f"Exercise: {current_exercise}, Prev Movement: {prev_movement}, Current Movement: {current_movement}, Start: {exercise_start_motion}, End: {exercise_end_motion}")

                            if (prev_movement == exercise_start_motion) and (current_movement == exercise_end_motion):
                                exercise_data_lock.acquire()
                                current_exercise_reps += 1    
                                exercise_data_lock.release()
                                client_print("DEBUG", f"New Rep: {current_exercise_reps}")                 
                        else:
                            # Reset the number of exercise repetitions.
                            client_print("DEBUG", f"Resetting reps...")                 
                            exercise_data_lock.acquire()
                            current_exercise_reps = 0
                            exercise_data_lock.release()

                # Shift the predictions to the left.
                exercise_movement_prediction_buffer = exercise_movement_prediction_buffer[1:EXERCISE_PREDICTION_WINDOW_SIZE]

        utime.sleep(0.01)
    except KeyboardInterrupt:
        break

print("Program terminated.")
