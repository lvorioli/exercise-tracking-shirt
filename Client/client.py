# Luke Orioli
import serial
import time
import platform
import numpy as np
import joblib
from tensorflow.keras.models import load_model

NUM_FEATURES = 9
WINDOW_SIZE = 300
PREDICTION_SAMPLE_GAP = 50

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

debug_mode = False
data_collection_active = True

# Load the exercise movement label encoder to map the 
# predicted exercise movement number to the corresponding label.
le = joblib.load('label_encoder.pkl')

# Load the ML model for predicting exercise movements.
model = load_model('model.keras')

def process_data():
    data = []
    while data_collection_active is True:
        try:
            # Check if the serial port has any pending data to service.
            if s.in_waiting > 0:
                while s.in_waiting > 0:
                    # Read exercise data from the Pico W.
                    response = s.readline().decode().strip()
                    if len(response) > 0:
                        # Check the response type.
                        if "DEBUG" in response:
                            if debug_mode is True:
                                print(response)
                        elif "DATA" in response:
                            # Strip the response type from the data.
                            _, _, data_str = response.partition("[DATA] ")

                            # Split the exercise data.
                            tilt, touch, magnet, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, _flex = data_str.split(',')

                            # Add the data to the data buffer for later processing.
                            data.append([float(tilt), float(touch), float(magnet), 
                                        float(acc_x), float(acc_y), float(acc_z), 
                                        float(gyro_x), float(gyro_y), float(gyro_z)])
                            
                            # Check if enough data has been collected for a new exercise prediction.
                            if len(data) == WINDOW_SIZE:
                                # Print the current data sample.
                                print("Tilt:", tilt,
                                "\tTouch:", touch,
                                "\tMagnet:", magnet,           
                                "\tAcc X:", acc_x,
                                "\tAcc Y:", acc_y,
                                "\tAcc Z:", acc_z,
                                "\tGyro X:", gyro_x,
                                "\tGyro Y:", gyro_y,
                                "\tGyro Z:", gyro_z)
                                                        
                                # Convert the 2D data list into a numpy array.
                                data_np = np.array(data)
                                data_np = data_np.reshape(1, WINDOW_SIZE, NUM_FEATURES)

                                # Predict the current exercise given the current data.
                                prediction = model.predict(data_np)
                                predicted_class = np.argmax(prediction)
                                predicted_label = le.inverse_transform([predicted_class])[0]
                                print(f"Predicted Exercise: {predicted_label}")

                                # Shift the data left.
                                data = data[PREDICTION_SAMPLE_GAP:]

                                # Send the exercise movement prediction to the Pi.
                                s.write(f"{predicted_label}\n".encode('utf-8'))
                        else:
                            print(f"[PICO] {response}")

        except Exception as e:
            print(f"Error: {e}")

        # Sleep for 1 ms before servicing the serial buffer.
        time.sleep(0.001)

# Repeatedly read user input and update the material when instructed.
print("Waiting for data...")
while True:
    try:
        process_data()
    except KeyboardInterrupt:
        break

print("Program terminated.")