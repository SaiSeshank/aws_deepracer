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
        racing_track = [[-1.28386, -1.71636, 1.23328, 0.12925],
[-1.43203, -1.71422, 1.23328, 0.12015],
[-1.57495, -1.70964, 1.23328, 0.11595],
[-1.77815, -1.68399, 1.23328, 0.16607],
[-2.01561, -1.61061, 1.23328, 0.20153],
[-2.24113, -1.48002, 1.23355, 0.21126],
[-2.43453, -1.29702, 1.3049, 0.20404],
[-2.58958, -1.07003, 1.45115, 0.18943],
[-2.70641, -0.80854, 1.2897, 0.22207],
[-2.79316, -0.52487, 1.10713, 0.26794],
[-2.86347, -0.23077, 1.09274, 0.27672],
[-2.93039, 0.0697, 1.09274, 0.2817],
[-3.00176, 0.36915, 1.09274, 0.28171],
[-3.04681, 0.66647, 1.09274, 0.27519],
[-3.02266, 0.94803, 1.09274, 0.25861],
[-2.92252, 1.19387, 1.09274, 0.24292],
[-2.75901, 1.39493, 1.15794, 0.22381],
[-2.54562, 1.54914, 1.28082, 0.20555],
[-2.29367, 1.65728, 1.42142, 0.19289],
[-2.01172, 1.71979, 1.47929, 0.19523],
[-1.70719, 1.73413, 1.47929, 0.20609],
[-1.41001, 1.70046, 1.47929, 0.20218],
[-1.13129, 1.61604, 1.50908, 0.19298],
[-0.87328, 1.48652, 1.63575, 0.17649],
[-0.63463, 1.32064, 1.6291, 0.1784],
[-0.41223, 1.12732, 1.6291, 0.18088],
[-0.20283, 0.9138, 1.6291, 0.18357],
[-0.00036, 0.68276, 1.6291, 0.18858],
[0.22176, 0.48379, 1.6291, 0.18305],
[0.46568, 0.32319, 1.6291, 0.17927],
[0.72848, 0.20106, 1.51837, 0.19086],
[1.00647, 0.11431, 1.2729, 0.22877],
[1.29654, 0.05982, 1.1261, 0.2621],
[1.59616, 0.03528, 1.03224, 0.29123],
[1.90397, 0.0406, 1.0, 0.30786],
[2.20748, 0.01173, 1.0, 0.30488],
[2.49431, -0.06779, 1.0, 0.29765],
[2.74599, -0.20992, 1.0, 0.28903],
[2.93986, -0.41702, 1.0, 0.28368],
[3.04854, -0.67914, 1.0, 0.28375],
[3.05118, -0.96362, 1.00367, 0.28346],
[2.94981, -1.22536, 1.04811, 0.2678],
[2.76679, -1.43677, 1.1379, 0.24574],
[2.52621, -1.58884, 1.28457, 0.22156],
[2.24877, -1.68331, 1.50139, 0.19521],
[1.95097, -1.72863, 1.84792, 0.16301],
[1.64485, -1.73828, 2.49894, 0.12256],
[1.33715, -1.72767, 4.0, 0.07697],
[1.0294, -1.7222, 4.0, 0.07695],
[0.72162, -1.72095, 4.0, 0.07694],
[0.41384, -1.72287, 4.0, 0.07695],
[0.10602, -1.7267, 4.0, 0.07696],
[-0.2018, -1.72763, 3.33511, 0.0923],
[-0.50952, -1.72619, 1.57998, 0.19477],
[-0.81709, -1.72308, 1.32271, 0.23254],
[-1.12449, -1.71886, 1.23328, 0.24927]]
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
        ST = 8
        FT = 6
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
        DISTANCE_MULTIPLE = 2 #
        dist = dist_to_racing_line(optimals[0:2], optimals_second[0:2], [x, y])
        distance_reward = max(1e-6, 1 - (dist/(track_width*0.5)))
        reward += distance_reward * DISTANCE_MULTIPLE

        ## Reward if speed is close to optimal speed ##
        SPEED_DIFF_NO_REWARD = 1 #
        SPEED_MULTIPLE = 2 #
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
        if direction_diff > 30 :
            reward = 1e-6
        
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
        REWARD_FOR_FASTEST_TIME = 200 # should be adapted to track length and other rewards
        STANDARD_TIME = ST # seconds (time that is easily done by model)
        FASTEST_TIME = FT # seconds (best time of 1st place on the track)
        if progress >= 98:
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