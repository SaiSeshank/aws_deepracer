import math


class Reward:
    def __init__(self, verbose=False):
        self.first_racingpoint_index = 0  # None
        self.verbose = verbose

    def reward_function(self, params):

        # Import package (needed for heading)
        # import math

        ################## HELPER FUNCTIONS ###################

        def dist_2_points(x1, x2, y1, y2):
            return abs(abs(x1 - x2) ** 2 + abs(y1 - y2) ** 2) ** 0.5

        def closest_2_racing_points_index(racing_coords, car_coords):

            # Calculate all distances to racing points
            distances = []
            for i in range(len(racing_coords)):
                distance = dist_2_points(
                    x1=racing_coords[i][0],
                    x2=car_coords[0],
                    y1=racing_coords[i][1],
                    y2=car_coords[1],
                )
                distances.append(distance)

            # Get index of the closest racing point
            closest_index = distances.index(min(distances))

            # Get index of the second closest racing point
            distances_no_closest = distances.copy()
            distances_no_closest[closest_index] = 999
            second_closest_index = distances_no_closest.index(min(distances_no_closest))

            return [closest_index, second_closest_index]

        def dist_to_racing_line(closest_coords, second_closest_coords, car_coords):

            # Calculate the distances between 2 closest racing points
            a = abs(
                dist_2_points(
                    x1=closest_coords[0],
                    x2=second_closest_coords[0],
                    y1=closest_coords[1],
                    y2=second_closest_coords[1],
                )
            )

            # Distances between car and closest and second closest racing point
            b = abs(
                dist_2_points(
                    x1=car_coords[0],
                    x2=closest_coords[0],
                    y1=car_coords[1],
                    y2=closest_coords[1],
                )
            )
            c = abs(
                dist_2_points(
                    x1=car_coords[0],
                    x2=second_closest_coords[0],
                    y1=car_coords[1],
                    y2=second_closest_coords[1],
                )
            )

            # Calculate distance between car and racing line (goes through 2 closest racing points)
            # try-except in case a=0 (rare bug in DeepRacer)
            try:
                distance = abs(
                    -(a ** 4)
                    + 2 * (a ** 2) * (b ** 2)
                    + 2 * (a ** 2) * (c ** 2)
                    - (b ** 4)
                    + 2 * (b ** 2) * (c ** 2)
                    - (c ** 4)
                ) ** 0.5 / (2 * a)
            except:
                distance = b

            return distance

        # Calculate which one of the closest racing points is the next one and which one the previous one
        def next_prev_racing_point(
            closest_coords, second_closest_coords, car_coords, heading
        ):

            # Virtually set the car more into the heading direction
            heading_vector = [
                math.cos(math.radians(heading)),
                math.sin(math.radians(heading)),
            ]
            new_car_coords = [
                car_coords[0] + heading_vector[0],
                car_coords[1] + heading_vector[1],
            ]

            # Calculate distance from new car coords to 2 closest racing points
            distance_closest_coords_new = dist_2_points(
                x1=new_car_coords[0],
                x2=closest_coords[0],
                y1=new_car_coords[1],
                y2=closest_coords[1],
            )
            distance_second_closest_coords_new = dist_2_points(
                x1=new_car_coords[0],
                x2=second_closest_coords[0],
                y1=new_car_coords[1],
                y2=second_closest_coords[1],
            )

            if distance_closest_coords_new <= distance_second_closest_coords_new:
                next_point_coords = closest_coords
                prev_point_coords = second_closest_coords
            else:
                next_point_coords = second_closest_coords
                prev_point_coords = closest_coords

            return [next_point_coords, prev_point_coords]

        def racing_direction_diff(
            closest_coords, second_closest_coords, car_coords, heading
        ):

            # Calculate the direction of the center line based on the closest waypoints
            next_point, prev_point = next_prev_racing_point(
                closest_coords, second_closest_coords, car_coords, heading
            )

            # Calculate the direction in radius, arctan2(dy, dx), the result is (-pi, pi) in radians
            track_direction = math.atan2(
                next_point[1] - prev_point[1], next_point[0] - prev_point[0]
            )

            # Convert to degree
            track_direction = math.degrees(track_direction)

            # Calculate the difference between the track direction and the heading direction of the car
            direction_diff = abs(track_direction - heading)
            if direction_diff > 180:
                direction_diff = 360 - direction_diff

            return direction_diff

        # Gives back indexes that lie between start and end index of a cyclical list
        # (start index is included, end index is not)
        def indexes_cyclical(start, end, array_len):
            if end < start:
                end += array_len

            return [index % array_len for index in range(start, end)]

        # Calculate how long car would take for entire lap, if it continued like it did until now
        def projected_time(first_index, closest_index, step_count, times_list):

            # Calculate how much time has passed since start
            current_actual_time = (step_count - 1) / 15

            # Calculate which indexes were already passed
            indexes_traveled = indexes_cyclical(
                first_index, closest_index, len(times_list)
            )

            # Calculate how much time should have passed if car would have followed optimals
            current_expected_time = sum([times_list[i] for i in indexes_traveled])

            # Calculate how long one entire lap takes if car follows optimals
            total_expected_time = sum(times_list)

            # Calculate how long car would take for entire lap, if it continued like it did until now
            try:
                projected_time = (
                    current_actual_time / current_expected_time
                ) * total_expected_time
            except:
                projected_time = 9999

            return projected_time

        #################### RACING LINE ######################

        # Optimal racing line for the Spain track
        # Each row: [x,y,speed,timeFromPreviousPoint]
        racing_track = [[-0.11087, -3.28924, 4.0, 0.04281],
[0.00309, -3.3518, 4.0, 0.0325],
[0.11704, -3.41436, 4.0, 0.0325],
[0.26716, -3.49678, 4.0, 0.04281],
[0.53122, -3.64176, 4.0, 0.07531],
[0.79529, -3.78674, 4.0, 0.07531],
[1.05936, -3.93172, 4.0, 0.07531],
[1.32343, -4.07665, 4.0, 0.07531],
[1.58753, -4.22125, 4.0, 0.07527],
[1.85165, -4.36552, 4.0, 0.07524],
[2.11583, -4.50908, 4.0, 0.07517],
[2.38008, -4.65153, 4.0, 0.07505],
[2.64443, -4.79248, 4.0, 0.07489],
[2.9089, -4.93143, 4.0, 0.07469],
[3.17352, -5.0679, 4.0, 0.07443],
[3.4383, -5.20132, 4.0, 0.07413],
[3.70327, -5.33109, 4.0, 0.07376],
[3.96842, -5.45652, 4.0, 0.07333],
[4.23376, -5.57679, 4.0, 0.07283],
[4.49927, -5.69095, 3.81348, 0.07579],
[4.7649, -5.79779, 3.3561, 0.08531],
[5.03058, -5.89589, 2.93353, 0.09654],
[5.29616, -5.98353, 2.57245, 0.10872],
[5.56143, -6.05912, 2.21357, 0.12461],
[5.82602, -6.11984, 2.21357, 0.12264],
[6.08928, -6.16196, 2.21357, 0.12044],
[6.35015, -6.18126, 2.21357, 0.11817],
[6.60677, -6.17225, 2.21357, 0.116],
[6.85592, -6.12833, 2.21357, 0.11429],
[7.09069, -6.03978, 2.45059, 0.10239],
[7.30962, -5.91676, 2.65824, 0.09447],
[7.51197, -5.76582, 2.92611, 0.08627],
[7.69813, -5.59313, 3.1243, 0.08127],
[7.86807, -5.40269, 3.31503, 0.077],
[8.02199, -5.19785, 3.50906, 0.07302],
[8.16039, -4.98148, 3.64828, 0.0704],
[8.28351, -4.75571, 3.80486, 0.06759],
[8.39191, -4.52241, 3.90233, 0.06592],
[8.48584, -4.28301, 3.7756, 0.06811],
[8.56549, -4.03873, 3.42129, 0.0751],
[8.63107, -3.79067, 3.12731, 0.08204],
[8.68269, -3.53983, 2.82333, 0.09071],
[8.7201, -3.28709, 2.52978, 0.10099],
[8.7424, -3.03333, 2.2, 0.11579],
[8.74828, -2.77959, 2.2, 0.11537],
[8.73437, -2.52737, 2.2, 0.11482],
[8.69728, -2.27898, 2.2, 0.11416],
[8.63268, -2.03794, 2.2, 0.11343],
[8.53541, -1.80953, 2.2, 0.11284],
[8.3978, -1.60322, 2.23027, 0.1112],
[8.22408, -1.42384, 2.50103, 0.09984],
[8.02359, -1.26953, 2.82884, 0.08944],
[7.80301, -1.13712, 3.13457, 0.08207],
[7.56643, -1.02428, 3.48839, 0.07514],
[7.31709, -0.92861, 3.89514, 0.06857],
[7.05757, -0.84772, 4.0, 0.06796],
[6.7901, -0.77917, 4.0, 0.06903],
[6.51664, -0.72038, 4.0, 0.06993],
[6.23909, -0.66848, 4.0, 0.07059],
[5.95923, -0.62058, 4.0, 0.07098],
[5.67404, -0.57346, 4.0, 0.07226],
[5.38853, -0.52356, 4.0, 0.07246],
[5.10327, -0.47022, 4.0, 0.07255],
[4.81885, -0.41263, 4.0, 0.07255],
[4.53585, -0.35016, 4.0, 0.07245],
[4.25478, -0.28203, 4.0, 0.0723],
[3.97612, -0.20756, 4.0, 0.07211],
[3.70031, -0.12609, 4.0, 0.0719],
[3.42785, -0.03693, 4.0, 0.07167],
[3.15927, 0.06058, 4.0, 0.07143],
[2.89511, 0.167, 4.0, 0.0712],
[2.63591, 0.28281, 4.0, 0.07097],
[2.38222, 0.40834, 4.0, 0.07076],
[2.13455, 0.54381, 4.0, 0.07057],
[1.8934, 0.68933, 4.0, 0.07041],
[1.65921, 0.84489, 4.0, 0.07029],
[1.43239, 1.01038, 4.0, 0.0702],
[1.21324, 1.18562, 4.0, 0.07015],
[1.002, 1.3703, 4.0, 0.07015],
[0.79878, 1.56405, 4.0, 0.07019],
[0.60359, 1.7664, 4.0, 0.07029],
[0.41629, 1.9768, 4.0, 0.07042],
[0.23659, 2.19459, 4.0, 0.07059],
[0.06406, 2.41907, 4.0, 0.07078],
[-0.10183, 2.6494, 4.0, 0.07096],
[-0.26173, 2.88466, 4.0, 0.07111],
[-0.41636, 3.12383, 4.0, 0.0712],
[-0.56655, 3.3658, 4.0, 0.0712],
[-0.71319, 3.60959, 4.0, 0.07112],
[-0.85695, 3.85444, 4.0, 0.07098],
[-0.99841, 4.0998, 3.85491, 0.07347],
[-1.13344, 4.33761, 3.38157, 0.08087],
[-1.27082, 4.57302, 3.10199, 0.08787],
[-1.41287, 4.80369, 3.02973, 0.08941],
[-1.56179, 5.02734, 3.02973, 0.08869],
[-1.71981, 5.24156, 3.02973, 0.08786],
[-1.88965, 5.44319, 3.02973, 0.08701],
[-2.07414, 5.62854, 3.02973, 0.08631],
[-2.27515, 5.7943, 3.02973, 0.086],
[-2.49259, 5.93888, 3.09894, 0.08426],
[-2.72489, 6.06222, 3.36855, 0.07808],
[-2.96908, 6.16657, 3.70819, 0.07161],
[-3.22246, 6.25439, 3.98054, 0.06737],
[-3.48321, 6.32716, 4.0, 0.06768],
[-3.74989, 6.38576, 4.0, 0.06826],
[-4.0213, 6.43071, 4.0, 0.06878],
[-4.29627, 6.46244, 4.0, 0.0692],
[-4.57371, 6.48136, 4.0, 0.06952],
[-4.85257, 6.48782, 4.0, 0.06973],
[-5.13189, 6.48213, 4.0, 0.06984],
[-5.41076, 6.46471, 3.81032, 0.07333],
[-5.68835, 6.43588, 3.39724, 0.08215],
[-5.96385, 6.39579, 3.01162, 0.09244],
[-6.2362, 6.34351, 2.68465, 0.1033],
[-6.50412, 6.27804, 2.68465, 0.10273],
[-6.76592, 6.19771, 2.68465, 0.102],
[-7.01953, 6.10061, 2.68465, 0.10115],
[-7.26176, 5.9835, 2.68465, 0.10022],
[-7.48825, 5.84237, 2.68465, 0.0994],
[-7.69283, 5.67322, 3.01297, 0.0881],
[-7.8781, 5.48378, 3.22636, 0.08213],
[-8.04505, 5.27825, 3.40622, 0.07774],
[-8.19438, 5.05983, 3.55913, 0.07434],
[-8.32664, 4.83106, 3.70134, 0.07139],
[-8.44245, 4.59403, 3.81101, 0.06922],
[-8.54228, 4.3504, 3.52829, 0.07462],
[-8.62619, 4.10145, 3.22692, 0.08141],
[-8.69436, 3.84838, 2.93866, 0.08919],
[-8.7465, 3.59221, 2.64117, 0.09898],
[-8.78216, 3.33397, 2.29942, 0.11337],
[-8.80097, 3.07474, 2.29942, 0.11303],
[-8.80019, 2.81573, 2.29942, 0.11264],
[-8.77629, 2.55882, 2.29942, 0.11221],
[-8.72501, 2.30677, 2.29942, 0.11186],
[-8.64067, 2.0638, 2.29942, 0.11185],
[-8.51386, 1.8376, 2.91009, 0.08911],
[-8.36087, 1.6244, 3.23755, 0.08105],
[-8.18646, 1.42298, 3.63265, 0.07334],
[-7.99463, 1.23186, 3.98199, 0.068],
[-7.78788, 1.05008, 4.0, 0.06883],
[-7.56838, 0.87666, 4.0, 0.06994],
[-7.33802, 0.71066, 4.0, 0.07098],
[-7.09855, 0.55111, 4.0, 0.07194],
[-6.85166, 0.39694, 4.0, 0.07277],
[-6.5988, 0.24716, 4.0, 0.07347],
[-6.34144, 0.10063, 4.0, 0.07404],
[-6.08078, -0.04361, 4.0, 0.07448],
[-5.81773, -0.18639, 4.0, 0.07482],
[-5.55307, -0.32843, 4.0, 0.07509],
[-5.2875, -0.47045, 4.0, 0.07529],
[-5.02202, -0.61263, 4.0, 0.07529],
[-4.75667, -0.75506, 4.0, 0.07529],
[-4.49142, -0.89771, 4.0, 0.07529],
[-4.22629, -1.0406, 4.0, 0.07529],
[-3.96128, -1.18371, 4.0, 0.0753],
[-3.69639, -1.32706, 4.0, 0.0753],
[-3.43162, -1.47064, 4.0, 0.0753],
[-3.16695, -1.61445, 4.0, 0.0753],
[-2.90241, -1.75849, 4.0, 0.0753],
[-2.63798, -1.90275, 4.0, 0.07531],
[-2.37366, -2.04724, 4.0, 0.07531],
[-2.10947, -2.19197, 4.0, 0.07531],
[-1.84539, -2.33694, 4.0, 0.07531],
[-1.58133, -2.48192, 4.0, 0.07531],
[-1.31726, -2.6269, 4.0, 0.07531],
[-1.05319, -2.77189, 4.0, 0.07531],
[-0.78912, -2.91687, 4.0, 0.07531],
[-0.52505, -3.06184, 4.0, 0.07531],
[-0.26098, -3.20682, 4.0, 0.07531]]

        ################## INPUT PARAMETERS ###################

        # Read all input parameters
        all_wheels_on_track = params["all_wheels_on_track"]
        x = params["x"]
        y = params["y"]
        distance_from_center = params["distance_from_center"]
        is_left_of_center = params["is_left_of_center"]
        heading = params["heading"]
        progress = params["progress"]
        steps = params["steps"]
        speed = params["speed"]
        steering_angle = params["steering_angle"]
        track_width = params["track_width"]
        waypoints = params["waypoints"]
        closest_waypoints = params["closest_waypoints"]
        is_offtrack = params["is_offtrack"]

        ############### OPTIMAL X,Y,SPEED,TIME ################

        # Get closest indexes for racing line (and distances to all points on racing line)
        closest_index, second_closest_index = closest_2_racing_points_index(
            racing_track, [x, y]
        )

        # Get optimal [x, y, speed, time] for closest and second closest index
        optimals = racing_track[closest_index]
        optimals_second = racing_track[second_closest_index]

        # Save first racingpoint of episode for later
        if self.verbose == True:
            self.first_racingpoint_index = 0  # this is just for testing purposes
        if steps == 1:
            self.first_racingpoint_index = closest_index

        ################ REWARD AND PUNISHMENT ################

        ## Define the default reward ##
        reward = 1
        MIN_REWARD = 1e-5

        ## Reward if car goes close to optimal racing line ##
        DISTANCE_MULTIPLE = 2
        dist = dist_to_racing_line(optimals[0:2], optimals_second[0:2], [x, y])
        distance_reward = max(MIN_REWARD, 1 - (dist / (track_width * 0.5)))
        reward += distance_reward * DISTANCE_MULTIPLE

        ## Reward if speed is close to optimal speed ##
        SPEED_DIFF_NO_REWARD = 1
        SPEED_MULTIPLE = 3
        speed_diff = abs(optimals[2] - speed)
        if speed_diff <= SPEED_DIFF_NO_REWARD:
            # we use quadratic punishment (not linear) bc we're not as confident with the optimal speed
            # so, we do not punish small deviations from optimal speed
            speed_reward = (1 - (speed_diff / (SPEED_DIFF_NO_REWARD)) ** 2) ** 2
        else:
            speed_reward = 0
        reward += speed_reward * SPEED_MULTIPLE

        # Reward if less steps
        REWARD_PER_STEP_FOR_FASTEST_TIME = 1.5
        STANDARD_TIME = 15
        FASTEST_TIME = 13
        times_list = [row[3] for row in racing_track]
        projected_time = projected_time(
            self.first_racingpoint_index, closest_index, steps, times_list
        )
        try:
            steps_prediction = projected_time * 15 + 1
            reward_prediction = max(
                MIN_REWARD,
                (
                    -REWARD_PER_STEP_FOR_FASTEST_TIME
                    * (FASTEST_TIME)
                    / (STANDARD_TIME - FASTEST_TIME)
                )
                * (steps_prediction - (STANDARD_TIME * 15 + 1)),
            )
            steps_reward = min(
                REWARD_PER_STEP_FOR_FASTEST_TIME, reward_prediction / steps_prediction
            )
        except:
            steps_reward = 0
        reward += steps_reward

        # Zero reward if obviously wrong direction (e.g. spin)
        direction_diff = racing_direction_diff(
            optimals[0:2], optimals_second[0:2], [x, y], heading
        )
        if direction_diff > 30 or abs(steering_angle) > 20:
            reward = MIN_REWARD
        else:
            reward += 1.1 - (direction_diff / 30)

        # Zero reward of obviously too slow
        speed_diff_zero = abs(optimals[2] - speed)
        if speed_diff_zero > 0.5:
            reward = MIN_REWARD

        ## Incentive for finishing the lap in less steps ##
        # should be adapted to track length and other rewards
        REWARD_FOR_FASTEST_TIME = 300
        STANDARD_TIME = 15  # seconds (time that is easily done by model)
        FASTEST_TIME = 13  # seconds (best time of 1st place on the track)
        if progress > 99.5:
            finish_reward = max(
                MIN_REWARD,
                (-REWARD_FOR_FASTEST_TIME / (15 * (STANDARD_TIME - FASTEST_TIME)))
                * (steps - STANDARD_TIME * 15),
            )
        else:
            finish_reward = 0
        reward += finish_reward

        ## Zero reward if off track ##
        if is_offtrack == True:
            reward = MIN_REWARD

        ####################### VERBOSE #######################
        if self.verbose == True:
            print("Closest index: %i" % closest_index)
            print("Distance to racing line: %f" % dist)
            print("=== Distance reward (w/out multiple): %f ===" % (distance_reward))
            print("Optimal speed: %f" % optimals[2])
            print("Speed difference: %f" % speed_diff)
            print("=== Speed reward (w/out multiple): %f ===" % speed_reward)
            print("Direction difference: %f" % direction_diff)
            print("Predicted time: %f" % projected_time)
            print("=== Steps reward: %f ===" % steps_reward)
            print("=== Finish reward: %f ===" % finish_reward)

        #################### RETURN REWARD ####################

        # Always return a float value
        return float(reward)


reward_object = Reward()  # add parameter verbose=True to get noisy output for testing


def reward_function(params):
    return reward_object.reward_function(params)