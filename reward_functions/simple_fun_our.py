import math


class Reward:
    def __init__(self, verbose=False):
        self.first_racingpoint_index = None # -- to account for none type error
        self.verbose = verbose

    def reward_function(self, params):

        ################## HELPER FUNCTIONS ###################

        def dist_2_points(x1, x2, y1, y2):
            return abs(abs(x1-x2)**2 + abs(y1-y2)**2)**0.5

        def closest_2_racing_points_index(racing_coords, car_coords):

            # Calculate all distances to racing points
            distances = []
            for i in range(len(racing_coords)):
                distance = dist_2_points(x1=racing_coords[i][0], x2=car_coords[0],
                                         y1=racing_coords[i][1], y2=car_coords[1])
                distances.append(distance)

            # Get index of the closest racing point
            closest_index = distances.index(min(distances))

            # Get index of the second closest racing point
            distances_no_closest = distances.copy()
            distances_no_closest[closest_index] = 999
            second_closest_index = distances_no_closest.index(
                min(distances_no_closest))

            return [closest_index, second_closest_index]

        def dist_to_racing_line(closest_coords, second_closest_coords, car_coords):
            
            # Calculate the distances between 2 closest racing points
            a = abs(dist_2_points(x1=closest_coords[0],
                                  x2=second_closest_coords[0],
                                  y1=closest_coords[1],
                                  y2=second_closest_coords[1]))

            # Distances between car and closest and second closest racing point
            b = abs(dist_2_points(x1=car_coords[0],
                                  x2=closest_coords[0],
                                  y1=car_coords[1],
                                  y2=closest_coords[1]))
            c = abs(dist_2_points(x1=car_coords[0],
                                  x2=second_closest_coords[0],
                                  y1=car_coords[1],
                                  y2=second_closest_coords[1]))

            # Calculate distance between car and racing line (goes through 2 closest racing points)
            # try-except in case a=0 (rare bug in DeepRacer)
            try:
                distance = abs(-(a**4) + 2*(a**2)*(b**2) + 2*(a**2)*(c**2) -
                               (b**4) + 2*(b**2)*(c**2) - (c**4))**0.5 / (2*a)
            except:
                distance = b

            return distance

        # Calculate which one of the closest racing points is the next one and which one the previous one
        def next_prev_racing_point(closest_coords, second_closest_coords, car_coords, heading):

            # Virtually set the car more into the heading direction
            heading_vector = [math.cos(math.radians(
                heading)), math.sin(math.radians(heading))]
            new_car_coords = [car_coords[0]+heading_vector[0],
                              car_coords[1]+heading_vector[1]]

            # Calculate distance from new car coords to 2 closest racing points
            distance_closest_coords_new = dist_2_points(x1=new_car_coords[0],
                                                        x2=closest_coords[0],
                                                        y1=new_car_coords[1],
                                                        y2=closest_coords[1])
            distance_second_closest_coords_new = dist_2_points(x1=new_car_coords[0],
                                                               x2=second_closest_coords[0],
                                                               y1=new_car_coords[1],
                                                               y2=second_closest_coords[1])

            if distance_closest_coords_new <= distance_second_closest_coords_new:
                next_point_coords = closest_coords
                prev_point_coords = second_closest_coords
            else:
                next_point_coords = second_closest_coords
                prev_point_coords = closest_coords

            return [next_point_coords, prev_point_coords]

        def racing_direction_diff(closest_coords, second_closest_coords, car_coords, heading):

            # Calculate the direction of the center line based on the closest waypoints
            next_point, prev_point = next_prev_racing_point(closest_coords,
                                                            second_closest_coords,
                                                            car_coords,
                                                            heading)

            # Calculate the direction in radius, arctan2(dy, dx), the result is (-pi, pi) in radians
            track_direction = math.atan2(
                next_point[1] - prev_point[1], next_point[0] - prev_point[0])

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

            if end < start: # getting none type error here.
                end += array_len

            return [index % array_len for index in range(start, end)]

        # Calculate how long car would take for entire lap, if it continued like it did until now
        def projected_time(first_index, closest_index, step_count, times_list):

            # Calculate how much time has passed since start
            current_actual_time = (step_count-1) / 15

            # Calculate which indexes were already passed
            indexes_traveled = indexes_cyclical(first_index, closest_index, len(times_list))

            # Calculate how much time should have passed if car would have followed optimals
            current_expected_time = sum([times_list[i] for i in indexes_traveled])

            # Calculate how long one entire lap takes if car follows optimals
            total_expected_time = sum(times_list)

            # Calculate how long car would take for entire lap, if it continued like it did until now
            try:
                projected_time = (current_actual_time/current_expected_time) * total_expected_time
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
[1.32342, -4.07671, 4.0, 0.07531],
[1.58749, -4.22169, 4.0, 0.07531],
[1.85157, -4.36664, 4.0, 0.07531],
[2.11566, -4.51158, 4.0, 0.07531],
[2.37975, -4.65651, 4.0, 0.07531],
[2.6438, -4.80153, 4.0, 0.07531],
[2.9078, -4.94666, 4.0, 0.07531],
[3.17182, -5.09174, 4.0, 0.07531],
[3.43596, -5.23657, 4.0, 0.07531],
[3.70018, -5.38123, 4.0, 0.07531],
[3.96454, -5.52561, 4.0, 0.0753],
[4.22865, -5.67051, 4.0, 0.07531],
[4.49227, -5.81642, 4.0, 0.07533],
[4.75513, -5.96392, 3.35959, 0.08972],
[5.01693, -6.11363, 2.99272, 0.10077],
[5.27735, -6.26622, 2.81716, 0.10714],
[5.54303, -6.4078, 2.76102, 0.10904],
[5.81833, -6.52818, 2.76102, 0.10883],
[6.10435, -6.61597, 2.76102, 0.10836],
[6.39836, -6.66172, 2.76102, 0.10777],
[6.69406, -6.65976, 2.76102, 0.1071],
[6.98427, -6.60891, 2.78046, 0.10596],
[7.26241, -6.51125, 2.87285, 0.10261],
[7.52335, -6.37146, 3.0199, 0.09803],
[7.7636, -6.19553, 3.21161, 0.09272],
[7.98117, -5.99012, 3.44397, 0.08688],
[8.17568, -5.76171, 3.73377, 0.08035],
[8.34858, -5.51572, 4.0, 0.07517],
[8.50196, -5.25678, 4.0, 0.07524],
[8.63851, -4.98866, 4.0, 0.07522],
[8.76035, -4.71342, 4.0, 0.07525],
[8.86909, -4.4327, 4.0, 0.07526],
[8.96491, -4.14734, 4.0, 0.07525],
[9.04804, -3.85795, 3.7551, 0.08018],
[9.1185, -3.56518, 3.24434, 0.09282],
[9.17521, -3.26949, 2.9234, 0.10299],
[9.21656, -2.97128, 2.76341, 0.10895],
[9.23647, -2.67101, 2.70915, 0.11108],
[9.22877, -2.3704, 2.70915, 0.111],
[9.1843, -2.07327, 2.70915, 0.1109],
[9.09597, -1.78741, 2.70915, 0.11044],
[8.9619, -1.5221, 2.70915, 0.10972],
[8.7848, -1.28522, 2.7308, 0.10831],
[8.57021, -1.08207, 2.84573, 0.10384],
[8.32573, -0.9146, 3.03429, 0.09766],
[8.05912, -0.78195, 3.28293, 0.09071],
[7.77718, -0.68146, 3.63578, 0.08232],
[7.48569, -0.60866, 4.0, 0.07511],
[7.18891, -0.55837, 4.0, 0.07525],
[6.88952, -0.52563, 4.0, 0.07529],
[6.58896, -0.50721, 4.0, 0.07528],
[6.28804, -0.50173, 4.0, 0.07524],
[5.98719, -0.50777, 4.0, 0.07523],
[5.68664, -0.52393, 4.0, 0.07525],
[5.38656, -0.54884, 4.0, 0.07528],
[5.08706, -0.58123, 4.0, 0.07531],
[4.78675, -0.60128, 4.0, 0.07524],
[4.48622, -0.60472, 4.0, 0.07514],
[4.1865, -0.58853, 4.0, 0.07504],
[3.88888, -0.55101, 4.0, 0.07499],
[3.59466, -0.49232, 4.0, 0.07501],
[3.30489, -0.41363, 4.0, 0.07506],
[3.02028, -0.31703, 4.0, 0.07514],
[2.74108, -0.20507, 4.0, 0.0752],
[2.46716, -0.08037, 4.0, 0.07524],
[2.19814, 0.05486, 4.0, 0.07527],
[1.93353, 0.19859, 4.0, 0.07528],
[1.67274, 0.34911, 4.0, 0.07528],
[1.41659, 0.50737, 4.0, 0.07528],
[1.16627, 0.67458, 4.0, 0.07526],
[0.92297, 0.8519, 4.0, 0.07526],
[0.68803, 1.04002, 4.0, 0.07524],
[0.46273, 1.23961, 4.0, 0.07525],
[0.24871, 1.45129, 4.0, 0.07526],
[0.04802, 1.67555, 4.0, 0.07524],
[-0.13786, 1.91215, 4.0, 0.07522],
[-0.30735, 2.16076, 4.0, 0.07522],
[-0.4585, 2.42095, 4.0, 0.07523],
[-0.58917, 2.69196, 4.0, 0.07522],
[-0.69836, 2.9722, 4.0, 0.07519],
[-0.78659, 3.2596, 4.0, 0.07516],
[-0.85564, 3.55217, 3.50744, 0.0857],
[-0.90771, 3.8483, 3.31764, 0.09063],
[-0.94507, 4.14681, 3.27517, 0.09185],
[-0.97035, 4.44662, 3.27517, 0.09186],
[-1.01847, 4.73999, 3.27208, 0.09086],
[-1.09608, 5.0246, 3.17622, 0.09288],
[-1.20554, 5.2975, 3.04323, 0.09662],
[-1.34668, 5.55612, 3.03005, 0.09724],
[-1.51814, 5.79795, 3.03005, 0.09783],
[-1.71886, 6.01933, 3.03005, 0.09862],
[-1.9475, 6.21443, 3.03005, 0.0992],
[-2.20063, 6.37552, 3.03005, 0.09902],
[-2.47186, 6.49878, 3.13146, 0.09514],
[-2.7554, 6.58525, 3.37152, 0.08792],
[-3.04691, 6.63942, 3.81959, 0.07763],
[-3.34325, 6.66798, 4.0, 0.07443],
[-3.64237, 6.67788, 4.0, 0.07482],
[-3.94297, 6.67597, 4.0, 0.07515],
[-4.24413, 6.6685, 4.0, 0.07531],
[-4.54529, 6.66115, 4.0, 0.07531],
[-4.84645, 6.65378, 4.0, 0.07531],
[-5.14761, 6.64641, 4.0, 0.07531],
[-5.44877, 6.63905, 4.0, 0.07531],
[-5.74993, 6.63163, 3.25348, 0.09259],
[-6.05111, 6.62452, 2.92704, 0.10293],
[-6.35206, 6.61509, 2.83532, 0.10619],
[-6.65212, 6.59745, 2.83532, 0.10601],
[-6.94863, 6.55698, 2.83532, 0.10555],
[-7.23545, 6.48165, 2.83532, 0.10459],
[-7.50468, 6.36623, 2.83532, 0.10331],
[-7.75085, 6.21141, 2.89615, 0.10041],
[-7.97158, 6.02137, 3.07396, 0.09475],
[-8.16673, 5.80183, 3.36874, 0.0872],
[-8.33794, 5.55925, 3.76808, 0.0788],
[-8.48786, 5.29996, 4.0, 0.07488],
[-8.6197, 5.02953, 4.0, 0.07521],
[-8.73694, 4.75215, 4.0, 0.07529],
[-8.84359, 4.4705, 4.0, 0.07529],
[-8.94005, 4.1852, 4.0, 0.07529],
[-9.02602, 3.89659, 3.12832, 0.09626],
[-9.10022, 3.60477, 2.71712, 0.11082],
[-9.16125, 3.30987, 2.6, 0.11583],
[-9.2065, 3.01218, 2.6, 0.11581],
[-9.23137, 2.71246, 2.6, 0.11567],
[-9.21686, 2.41541, 2.6, 0.11439],
[-9.15324, 2.13153, 2.6, 0.11189],
[-9.04072, 1.86859, 2.65049, 0.10791],
[-8.88459, 1.63001, 2.82498, 0.10093],
[-8.69169, 1.41625, 3.13874, 0.09173],
[-8.46969, 1.22552, 3.59829, 0.08134],
[-8.2262, 1.05473, 4.0, 0.07436],
[-7.96847, 0.90018, 4.0, 0.07513],
[-7.70288, 0.75807, 4.0, 0.0753],
[-7.43347, 0.62338, 4.0, 0.0753],
[-7.16164, 0.49361, 4.0, 0.0753],
[-6.88853, 0.36652, 4.0, 0.07531],
[-6.61597, 0.23838, 4.0, 0.07529],
[-6.34485, 0.10736, 4.0, 0.07528],
[-6.07558, -0.02732, 4.0, 0.07527],
[-5.80821, -0.16578, 4.0, 0.07527],
[-5.54251, -0.30753, 4.0, 0.07529],
[-5.27802, -0.45168, 4.0, 0.0753],
[-5.01413, -0.59701, 4.0, 0.07532],
[-4.75021, -0.74229, 4.0, 0.07532],
[-4.48622, -0.88741, 4.0, 0.07531],
[-4.22209, -1.03228, 4.0, 0.07531],
[-3.95788, -1.17697, 4.0, 0.07531],
[-3.69377, -1.32188, 4.0, 0.07531],
[-3.42975, -1.46696, 4.0, 0.07531],
[-3.16574, -1.61205, 4.0, 0.07531],
[-2.9017, -1.75709, 4.0, 0.07531],
[-2.63763, -1.90205, 4.0, 0.07531],
[-2.37354, -2.04701, 4.0, 0.07531],
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
        all_wheels_on_track = params['all_wheels_on_track']
        x = params['x']
        y = params['y']
        distance_from_center = params['distance_from_center']
        is_left_of_center = params['is_left_of_center']
        heading = params['heading']
        progress = params['progress']
        steps = params['steps']
        speed = params['speed']
        abs_steering = abs(params['steering_angle'])
        track_width = params['track_width']
        waypoints = params['waypoints']
        closest_waypoints = params['closest_waypoints']
        is_offtrack = params['is_offtrack']
        is_reversed  = params["is_reversed"]
        steering_angle = params["steering_angle"]

        DIRECTION_THRESHOLD = 30
        # ABS_STEERING_THRESHOLD = 15
        ST = 14
        FT = 11
        ############### OPTIMAL X,Y,SPEED,TIME ################

        # Get closest indexes for racing line (and distances to all points on racing line)
        closest_index, second_closest_index = closest_2_racing_points_index(
            racing_track, [x, y])

        # Get optimal [x, y, speed, time] for closest and second closest index
        optimals = racing_track[closest_index]
        optimals_second = racing_track[second_closest_index]

        # Save first racingpoint of episode for later
        if self.verbose == False:
            self.first_racingpoint_index = 0 # this is just for testing purposes
        if steps == 1:
            self.first_racingpoint_index = closest_index

        ################ REWARD AND PUNISHMENT ################

        ## Define the default reward ##
        reward = 1

        ## Reward if car goes close to optimal racing line ##
        DISTANCE_MULTIPLE = 3 #
        dist = dist_to_racing_line(optimals[0:2], optimals_second[0:2], [x, y])
        distance_reward = max(1e-6, 1 - (dist/(track_width*0.5)))
        reward += distance_reward * DISTANCE_MULTIPLE

        ## Reward if speed is close to optimal speed ##
        SPEED_DIFF_NO_REWARD = 1 #
        SPEED_MULTIPLE = 2.5 #
        speed_diff = abs(optimals[2]-speed) # need to use this abs below too
        if speed_diff <= SPEED_DIFF_NO_REWARD:
            # we use quadratic punishment (not linear) bc we're not as confident with the optimal speed
            # so, we do not punish small deviations from optimal speed
            speed_reward = (1 - (speed_diff/(SPEED_DIFF_NO_REWARD))**2)**2
        else:
            speed_reward = 0
        reward += speed_reward * SPEED_MULTIPLE

        # Reward if less steps
        REWARD_PER_STEP_FOR_FASTEST_TIME = 2
        STANDARD_TIME = ST #
        FASTEST_TIME = FT #
        times_list = [row[3] for row in racing_track]
        projected_time = projected_time(self.first_racingpoint_index, closest_index, steps, times_list)
        try:
            steps_prediction = projected_time * 15 + 1
            reward_prediction = max(1e-6, (-REWARD_PER_STEP_FOR_FASTEST_TIME*(FASTEST_TIME) /
                                           (STANDARD_TIME-FASTEST_TIME))*(steps_prediction-(STANDARD_TIME*15+1)))
            steps_reward = min(REWARD_PER_STEP_FOR_FASTEST_TIME, reward_prediction / steps_prediction)
        except:
            steps_reward = 0
        reward += steps_reward

        # Zero reward if obviously wrong direction (e.g. spin)
        direction_diff = racing_direction_diff(
            optimals[0:2], optimals_second[0:2], [x, y], heading)
        if direction_diff > 30 or abs(steering_angle) > 20:
            reward = 1e-6
        else:
            reward += 1.1 - (direction_diff / 30)
        
        if is_reversed:
            reward = 1e-6
        # # Penalize reward if the car is steering too much
        # if abs_steering > ABS_STEERING_THRESHOLD:
        #     reward *= 0.8

        # Zero reward of obviously too slow
        speed_diff_zero = optimals[2]-speed # by refering to optimal speed reward we need to use abs here.
        if abs(speed_diff_zero) > 0.5:
            reward = 1e-6
            
        ## Incentive for finishing the lap in less steps ##
        REWARD_FOR_FASTEST_TIME = 500 # should be adapted to track length and other rewards
        STANDARD_TIME = ST # seconds (time that is easily done by model)
        FASTEST_TIME = FT # seconds (best time of 1st place on the track)
        if progress >= 99.5:
            finish_reward = max(1e-6, (-REWARD_FOR_FASTEST_TIME /
                      (15*(STANDARD_TIME-FASTEST_TIME)))*(steps-STANDARD_TIME*15))
        else:
            finish_reward = 0
        reward += finish_reward
        
        ## Zero reward if off track ##
        if is_offtrack:
            reward = 1e-6
        

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


reward_object = Reward() # add parameter verbose=True to get noisy output for testing


def reward_function(params):
    return reward_object.reward_function(params)