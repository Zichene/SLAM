import os
import numpy as np

class Dataset_intel:
    def __init__(self, data_folder_path):
        """
        Creates a class containing the intel CLF dataset. If there are no npy files created, it will generate them first.

        :param data_folder_path: Path to the data folder in which intel.clf.txt is contained in.
        """
        self.data_folder_path = data_folder_path
        assert os.path.exists(self.data_folder_path)

        self.clf_path = self.data_folder_path + os.sep + 'intel.clf.txt'
        assert os.path.exists(self.clf_path)

        # Npy paths
        self.odoms_laser_file = self.data_folder_path + os.sep + "intel_odoms_laser.npy"
        self.lasers_file = self.data_folder_path + os.sep + "intel_lasers.npy"

        # Check if .npy have already been generated
        if os.path.exists(self.odoms_laser_file) and os.path.exists(self.lasers_file):
            self.odoms_laser = np.load(self.odoms_laser_file)
            self.lasers = np.load(self.lasers_file)
        else:
            # parse the dataset
            odoms_laser = []
            lasers = []

            with open(self.clf_path, "r") as f:
                for line in f:
                    if line.startswith("FLASER"):
                        read_line = line.split()

                        # number of range readings, should be 180
                        num_readings = int(read_line[1])
                        assert num_readings==180
                        range_readings = np.array(read_line[2:2+num_readings], dtype=np.float32)
                        index = np.arange(-90, 90+180/num_readings, 180/num_readings)
                        index = np.delete(index, num_readings//2)
                        angles = np.radians(index)

                        converted_readings = np.array([np.cos(angles), np.sin(angles)])*range_readings
                        lasers.append(converted_readings)

                        x = read_line[2+num_readings]
                        y = read_line[3+num_readings]
                        theta = read_line[4+num_readings]
                        odoms_laser.append([ float(x), float(y), float(theta)])

                        _ = read_line[8+num_readings]
                        _ = read_line[10+num_readings]

            self.odoms_laser = np.array(odoms_laser)
            self.lasers = np.array((lasers))

            # Save the npy files
            np.save(self.odoms_laser_file, self.odoms_laser)
            np.save(self.lasers_file, self.lasers)

    def get_dataset(self):
        """
        Returns the dataset in the form of odoms_laser and lasers.
        """
        return self.odoms_laser, self.lasers