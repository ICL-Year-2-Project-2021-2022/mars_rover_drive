'''
Adapted from https://gist.github.com/ttmarek/f3312eaf18a2e59398a2
'''
import serial
import csv
import datetime
import time
#import matplotlib.pyplot as plt
#import pandas as pd
 
portPath = "COM3"       # Must match value shown on Arduino IDE
baud = 9600                     # Must match Arduino baud rate
timeout = 5                       # Seconds
filename = "../data_optical/two_optical/testfile_" + str(datetime.datetime.now().strftime("%y_%m_%d_%H_%M_%S_%p")) + ".csv"
max_num_readings = 32000
header_list = ["Epoch_Seconds","Squal_Left","Squal_Right","Delta_X_Left","Delta_Y_left","Total_X_left","Total_Y_left","Delta_X_right","Delta_Y_right","Total_X_right","Total_Y_right"]
 
 
 
def create_serial_obj(portPath, baud_rate, tout):
    """
    Given the port path, baud rate, and timeout value, creates
    and returns a pyserial object.
    """
    return serial.Serial(portPath, baud_rate, timeout = tout)
    
def read_serial_data_and_clean(serial):
    """
    Given a pyserial object (serial). Outputs a list of lines read in
    from the serial port
    """
    serial.reset_input_buffer()
    
    clean_data = []
    readings_left = True
    timeout_reached = False
    
    with open(filename, 'w', newline='') as csvfile:
        csvwrite = csv.writer(csvfile)
        csvwrite.writerow(header_list)
    
    while readings_left and not timeout_reached:
        serial_line = serial.readline()
        if serial_line == '':
            timeout_reached = True
        else:
            line_data = serial_line.decode(encoding='UTF-8').splitlines()[0].split(",") 
            line_data = [time.time()] + [str(element) for element in line_data]
            line_data[1] = int(line_data[1], 2)
            line_data[2] = int(line_data[2], 2)
            if len(line_data) >= 2:
                clean_data.append(line_data)
                
            with open(filename, 'a', newline='') as csvfile:
                csvwrite = csv.writer(csvfile)
                csvwrite.writerow(line_data)
            
            if len(clean_data) == max_num_readings:
                readings_left = False
        
    return clean_data
 
def is_number(string):
    """
    Given a string returns True if the string represents a number.
    Returns False otherwise.
    """
    try:
        float(string)
        return True
    except ValueError:
        return False        
    
def map_value(x, in_min, in_max, out_min, out_max):
    return (((x - in_min) * (out_max - out_min))/(in_max - in_min)) + out_min
 
'''    
def simple_plot(csv_file, columns, headers):
    plt.clf()
    plt.close()
    plt.plotfile(csv_file, columns, names=headers, newfig=True)
    plt.show()

def plot_csv(csv_file, cols):
    # Create Pandas DataFrame from csv data
    data_frame = pd.read_csv(csv_file)
    # Set the names of the columns
    data_frame.columns = cols
    # Set the first column (Time) as the index 
    data_frame = data_frame.set_index(cols[0])
    # Map the voltage values from 0-1023 to 0-5
    data_frame = data_frame.apply(lambda x: map_value(x,0.,1023,0,5))
    # Bring back the Time column
    data_frame = data_frame.reset_index()
    plt.clf()
    plt.close()
    # Plot the data
    data_frame.plot(x=cols[0],y=cols[1:])
    plt.show()
'''   
    
print("Creating serial object...")
serial_obj = create_serial_obj(portPath, baud, timeout)
 
print("Reading serial data...")
clean_data = read_serial_data_and_clean(serial_obj)
print(len(clean_data))
 
#print("Plotting data...")
#simple_plot(filename, (0,1,2), ['time (s)', 'voltage1', 'voltage2'])
#simple_plot(filename, (0,1), ['time (s)', 'voltage1'])
#plot_csv(filename, gen_col_list(num_signals))