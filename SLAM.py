import numpy as np
from matplotlib import pyplot as plt

from graph import Graph
import pandas as pd

root_node = Graph.Node([0, 0])

graph = Graph(Graph.Node([0, 0]))


def calculate_odom_measurement(acceleration, delta_t):
    """
    We only have the acceleration not the velocity.
    Uses the formula $x_{i+1} = x_i + 1/2 a*t^2$.
    :param acceleration: 2D array representing acceleration
    :param delta_t: t
    :return:
    """
    return 1/2*acceleration*delta_t*delta_t


def parse_imu(file_path):
    # Define the path to your file
    #file_path = 'path_to_your_file.txt'

    # Read the file into a pandas DataFrame
    df = pd.read_csv(file_path, delim_whitespace=True)

    # Display the DataFrame
    print(df)
    return df

def parse_gps(file_path):
    df = pd.read_csv(file_path, delim_whitespace=True)
    print(df)
    return df

if __name__ == "__main__":
    imu = parse_imu("data/malaga-urban-dataset-extract-01/malaga-urban-dataset-extract-01_all-sensors_IMU.txt")
    x_acc, y_acc = np.array(imu["IMU_X_ACC"]), np.array(imu["IMU_Y_ACC"])
    print(x_acc)
    acc = np.array(tuple(zip(x_acc, y_acc)))
    print(acc)
    x_0 = np.array([0, 0])
    for a in acc:
        calculate_odom_measurement(a, 0.01)
    # df = parse_gps("data/malaga-urban-dataset-extract-01/malaga-urban-dataset-extract-01_all-sensors_GPS.txt")
    # lat, long = np.array(df["Lat"]), np.array(df["Lon"])
    # local_x, local_y = np.array(df["Local_X"]), np.array(df["Local_Y"])
    # plt.plot(long, lat)
    # plt.show()
    #
    # plt.plot(local_x, local_y)
    # plt.show()
    # print(lat)
    # print(long)