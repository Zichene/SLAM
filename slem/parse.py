import numpy as np
import pandas as pd

def get_dataset():
    odom_data = []
    flaser_data = []

    def parse_line(line):
        parts = line.split()
        message_type = parts[0]

        if message_type == 'ODOM':
            odom_data.append({
                'x': float(parts[1]),
                'y': float(parts[2]),
                'theta': float(parts[3]),
                'tv': float(parts[4]),
                'rv': float(parts[5]),
                'accel': float(parts[6]),
                'ipc_timestamp': float(parts[7]),
                'ipc_hostname': parts[8],
                'logger_timestamp': float(parts[9])
            })

        elif message_type == 'FLASER':
            num_readings = int(parts[1])
            flaser_data.append({
                'num_readings': num_readings,
                'range_readings': list(map(float, parts[2:2+num_readings])),
                'x': float(parts[2+num_readings]),
                'y': float(parts[3+num_readings]),
                'theta': float(parts[4+num_readings]),
                'odom_x': float(parts[5+num_readings]),
                'odom_y': float(parts[6+num_readings]),
                'odom_theta': float(parts[7+num_readings]),
                'ipc_timestamp': float(parts[8+num_readings]),
                'ipc_hostname': parts[9+num_readings],
                'logger_timestamp': float(parts[10+num_readings])
            })

    # Parse
    with open('data/intel.clf.txt', 'r') as file:
        for line in file:
            if not (line.startswith('#') or line.startswith('PARAM')) and line.strip():
                parse_line(line.strip())

    # Create DataFrames from the parsed data
    flaser_df = pd.DataFrame(flaser_data)

    ### FLASER DATAFRAME
    def polar_to_cartesian(range_readings, angle_increment):
        cartesian_coords = []
        for i, r in enumerate(range_readings):
            angle = i * angle_increment # - (np.pi/2)
            x = r * np.cos(angle)
            y = r * np.sin(angle)
            cartesian_coords.append(np.array((x, y)))
        return cartesian_coords

    angle_increment = np.pi / 180

    # Add new column to flaser_df with Cartesian coordinates

    flaser_df['range_cartesian_readings'] = flaser_df['range_readings'].apply(lambda ranges: polar_to_cartesian(ranges, angle_increment))

    # Odom
    odom_df = pd.concat([flaser_df['odom_x'], flaser_df['odom_y'], flaser_df['theta']], axis=1)
    odom_np = odom_df.values

    # Flaser
    flaser_df = flaser_df['range_cartesian_readings']
    flaser_np = flaser_df.to_numpy()

    for _ in range(len(flaser_np)):
        flaser_np[_] = np.array(flaser_np[_])
    flaser_np = np.stack(flaser_np)

    return odom_np, flaser_np

get_dataset()