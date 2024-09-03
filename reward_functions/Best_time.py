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
        racing_track = [[5.02524, 0.67784, 4.0, 0.06106],
[5.01438, 0.91812, 4.0, 0.06013],
[5.00086, 1.15638, 4.0, 0.05966],
[4.98441, 1.39405, 4.0, 0.05956],
[4.96459, 1.63237, 4.0, 0.05979],
[4.94077, 1.87247, 4.0, 0.06032],
[4.91205, 2.11543, 4.0, 0.06116],
[4.87733, 2.36207, 3.79624, 0.06561],
[4.83483, 2.61294, 3.36734, 0.07556],
[4.7825, 2.86754, 2.90173, 0.08958],
[4.71742, 3.12378, 2.90173, 0.09111],
[4.637, 3.3774, 2.90173, 0.09169],
[4.53837, 3.62308, 2.90173, 0.09124],
[4.41838, 3.85544, 2.90173, 0.09012],
[4.27346, 4.06914, 2.90173, 0.08898],
[4.09781, 4.25582, 3.20866, 0.07989],
[3.89893, 4.41869, 3.54064, 0.0726],
[3.68178, 4.56066, 3.8209, 0.0679],
[3.44938, 4.68342, 4.0, 0.06571],
[3.20484, 4.78948, 4.0, 0.06664],
[2.9502, 4.8804, 4.0, 0.0676],
[2.68718, 4.95745, 4.0, 0.06852],
[2.41729, 5.02169, 4.0, 0.06936],
[2.1419, 5.07395, 4.0, 0.07008],
[1.8623, 5.11488, 4.0, 0.07065],
[1.57971, 5.14502, 4.0, 0.07105],
[1.29529, 5.16493, 4.0, 0.07128],
[1.01005, 5.17481, 4.0, 0.07135],
[0.72493, 5.17485, 4.0, 0.07128],
[0.44078, 5.16525, 4.0, 0.07108],
[0.15836, 5.14603, 4.0, 0.07077],
[-0.12159, 5.11719, 4.0, 0.07036],
[-0.39845, 5.07881, 4.0, 0.06988],
[-0.67168, 5.03106, 4.0, 0.06934],
[-0.94102, 4.97452, 4.0, 0.0688],
[-1.20624, 4.90956, 4.0, 0.06826],
[-1.46699, 4.83627, 4.0, 0.06771],
[-1.72274, 4.75444, 3.93689, 0.06821],
[-1.97252, 4.66323, 3.73895, 0.07112],
[-2.21502, 4.56155, 3.24939, 0.08092],
[-2.44838, 4.44798, 2.86099, 0.09071],
[-2.6702, 4.32095, 2.50893, 0.10188],
[-2.87712, 4.17874, 2.35003, 0.10684],
[-3.06817, 4.02244, 2.35003, 0.10504],
[-3.24134, 3.85225, 2.35003, 0.10332],
[-3.39045, 3.66616, 2.35003, 0.10147],
[-3.50783, 3.46365, 2.21963, 0.10546],
[-3.583, 3.24671, 2.21963, 0.10344],
[-3.60975, 3.02317, 2.21963, 0.10143],
[-3.60558, 2.80022, 2.21963, 0.10046],
[-3.56804, 2.58099, 2.21963, 0.10021],
[-3.48889, 2.37112, 2.21963, 0.10105],
[-3.35894, 2.18151, 2.66483, 0.08626],
[-3.1942, 2.01219, 2.9612, 0.07978],
[-3.00103, 1.86329, 3.30492, 0.0738],
[-2.78457, 1.73396, 3.70952, 0.06798],
[-2.54918, 1.62265, 4.0, 0.06509],
[-2.29897, 1.527, 4.0, 0.06697],
[-2.03789, 1.44377, 4.0, 0.06851],
[-1.76993, 1.36881, 4.0, 0.06956],
[-1.49854, 1.29809, 4.0, 0.07011],
[-1.2311, 1.22691, 3.99475, 0.06928],
[-0.96685, 1.1523, 3.55299, 0.07728],
[-0.70795, 1.07202, 3.09104, 0.08769],
[-0.45653, 0.98396, 2.71391, 0.09816],
[-0.21529, 0.88565, 2.71391, 0.09599],
[0.01289, 0.77484, 2.54733, 0.09958],
[0.22489, 0.64962, 2.29537, 0.10727],
[0.41713, 0.50846, 2.0, 0.11925],
[0.58405, 0.34947, 2.0, 0.11526],
[0.7188, 0.17207, 2.0, 0.11139],
[0.82358, -0.01868, 2.0, 0.10882],
[0.89277, -0.22118, 2.0, 0.107],
[0.91788, -0.43212, 2.0, 0.10621],
[0.88428, -0.64209, 2.22885, 0.09541],
[0.80423, -0.84101, 2.48095, 0.08642],
[0.68727, -1.02528, 2.688, 0.0812],
[0.53876, -1.19296, 2.93617, 0.07629],
[0.36334, -1.34333, 3.18141, 0.07263],
[0.16436, -1.47593, 3.43699, 0.06957],
[-0.05539, -1.59052, 3.74661, 0.06615],
[-0.2931, -1.68744, 4.0, 0.06418],
[-0.54583, -1.76775, 4.0, 0.0663],
[-0.81074, -1.83308, 4.0, 0.06821],
[-1.08486, -1.88594, 4.0, 0.06979],
[-1.36598, -1.92853, 4.0, 0.07108],
[-1.65196, -1.96349, 4.0, 0.07203],
[-1.9411, -1.99296, 4.0, 0.07266],
[-2.2259, -2.02971, 3.8548, 0.0745],
[-2.50482, -2.07578, 3.45797, 0.08175],
[-2.77623, -2.13296, 3.03997, 0.09124],
[-3.03839, -2.20284, 2.69569, 0.10065],
[-3.2894, -2.28686, 2.69569, 0.09819],
[-3.52697, -2.38646, 2.69569, 0.09556],
[-3.74812, -2.50324, 2.57257, 0.09721],
[-3.94909, -2.63867, 2.33686, 0.10371],
[-4.12414, -2.79441, 2.33686, 0.10026],
[-4.26615, -2.97047, 2.33686, 0.0968],
[-4.37712, -3.16083, 2.33686, 0.09429],
[-4.45724, -3.36172, 2.33686, 0.09255],
[-4.50185, -3.57048, 2.33686, 0.09135],
[-4.50348, -3.78233, 2.45662, 0.08624],
[-4.47003, -3.99169, 2.45662, 0.0863],
[-4.40149, -4.19442, 2.45662, 0.08711],
[-4.29715, -4.38581, 2.45662, 0.08873],
[-4.15423, -4.55897, 2.71999, 0.08255],
[-3.97979, -4.71204, 3.01377, 0.077],
[-3.77934, -4.84477, 3.32794, 0.07224],
[-3.55716, -4.9576, 3.65586, 0.06816],
[-3.31662, -5.05123, 4.0, 0.06453],
[-3.06094, -5.12696, 4.0, 0.06667],
[-2.79281, -5.18623, 4.0, 0.06865],
[-2.5149, -5.23109, 4.0, 0.07038],
[-2.22959, -5.26386, 4.0, 0.0718],
[-1.93876, -5.28673, 4.0, 0.07293],
[-1.64385, -5.30162, 4.0, 0.07382],
[-1.3461, -5.31047, 4.0, 0.07447],
[-1.04646, -5.31496, 4.0, 0.07492],
[-0.74564, -5.31654, 4.0, 0.07521],
[-0.44421, -5.31632, 4.0, 0.07536],
[-0.14339, -5.31384, 4.0, 0.07521],
[0.15624, -5.30858, 4.0, 0.07492],
[0.45437, -5.30004, 4.0, 0.07457],
[0.75067, -5.28757, 4.0, 0.07414],
[1.04475, -5.2705, 4.0, 0.07364],
[1.33621, -5.24819, 4.0, 0.07308],
[1.62466, -5.22002, 4.0, 0.07246],
[1.90966, -5.18532, 4.0, 0.07178],
[2.19068, -5.14334, 4.0, 0.07103],
[2.46709, -5.09318, 3.5994, 0.07805],
[2.73787, -5.03344, 3.24067, 0.08557],
[3.0017, -4.96231, 2.78234, 0.09821],
[3.25702, -4.87796, 2.78234, 0.09664],
[3.50189, -4.77827, 2.78234, 0.09502],
[3.73371, -4.66065, 2.78234, 0.09343],
[3.94882, -4.52182, 2.78234, 0.09201],
[4.14283, -4.35864, 2.78234, 0.09111],
[4.30639, -4.16485, 3.32469, 0.07627],
[4.44691, -3.95088, 3.69497, 0.06928],
[4.56749, -3.7209, 4.0, 0.06492],
[4.66999, -3.47744, 4.0, 0.06604],
[4.75608, -3.22252, 4.0, 0.06727],
[4.82734, -2.95787, 4.0, 0.06852],
[4.88525, -2.68497, 4.0, 0.06974],
[4.93127, -2.40516, 4.0, 0.07089],
[4.96694, -2.11969, 4.0, 0.07192],
[4.99372, -1.82965, 4.0, 0.07282],
[5.01323, -1.53609, 4.0, 0.07355],
[5.02699, -1.23993, 4.0, 0.07412],
[5.03632, -0.94186, 4.0, 0.07455],
[5.04194, -0.6429, 4.0, 0.07475],
[5.04409, -0.35098, 4.0, 0.07298],
[5.04318, -0.07649, 4.0, 0.06862],
[5.03962, 0.18347, 4.0, 0.065],
[5.03362, 0.43374, 4.0, 0.06259]]

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
        DIRECTION_THRESHOLD = 30
        # ABS_STEERING_THRESHOLD = 15
        ST = 14
        FT = 12
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
        if direction_diff > DIRECTION_THRESHOLD:
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
        REWARD_FOR_FASTEST_TIME = 800 # should be adapted to track length and other rewards
        STANDARD_TIME = ST # seconds (time that is easily done by model)
        FASTEST_TIME = FT # seconds (best time of 1st place on the track)
        if progress >= 98.5:
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