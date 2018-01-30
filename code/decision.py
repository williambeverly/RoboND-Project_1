import numpy as np


# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):

    # Implement conditionals to decide what to do given perception data
    # ADDED implement logic to prevent some irregular motion
    if Rover.total_time - Rover.update_time > Rover.update_rate:
        Rover.update_time = Rover.total_time # update the time
        Rover.average_steer[Rover.number_update_values - 1] = Rover.steer # replace end value with new steer value
        Rover.average_vel[Rover.number_update_values - 1] = Rover.vel # replace end value with new speed value
        Rover.average_steer = np.roll(Rover.average_steer, 1) # push the most recent value to the front (shift)
        Rover.average_vel = np.roll(Rover.average_vel, 1) # push the most recent value to the front (shift)
        average_steer = np.average(Rover.average_steer) # calculate the average
        average_vel = np.average(Rover.average_vel) # calculate the average
     
        if(average_vel <= 0.2):
            Rover.mode = 'rover_stuck'
        elif(average_vel > 0.2 and (average_steer > 14.5 or average_steer < -14.5)):
            Rover.mode = 'circular_motion'


    # Check if we have vision data to make decisions with
    if Rover.nav_angles is not None:
        if Rover.mode == 'rover_stuck':
            if(not Rover.in_recovery):
                print('Rover is stuck!')
                Rover.in_recovery = True
                Rover.goal_yaw = (int(Rover.yaw ) - 30) % 360
                Rover.throttle = 0
                Rover.brake = 0
            else:
                if(Rover.yaw > Rover.goal_yaw - 5 and Rover.yaw < Rover.goal_yaw + 5):
                    Rover.average_vel = np.ones(Rover.number_update_values) # reset speed values
                    Rover.average_steer = np.zeros(Rover.number_update_values) # reset steer values
                    Rover.in_recovery = False
                    Rover.mode = 'forward'
                else:
                    Rover.steer = np.clip(Rover.goal_yaw - Rover.yaw, -15, 15)
        elif Rover.mode == 'circular_motion':
            if((Rover.vel > 0.2) and (not Rover.in_recovery)):
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            else:
                Rover.brake = 0 # remove the brake
                if(not Rover.in_recovery):
                    print('Circular motion detected!')
                    Rover.in_recovery = True
                    # calculate where the Rover should point in direction of starting position
                    new_x = Rover.initial_x - Rover.pos[0]
                    new_y = Rover.initial_y - Rover.pos[1]
                    Rover.goal_yaw = (np.arctan2(new_y, new_x) * 180/np.pi) % 360
                    
                else:
                    if(Rover.yaw > Rover.goal_yaw - 5 and Rover.yaw < Rover.goal_yaw + 5):
                        # accelerate for a few seconds in that direction
                        Rover.throttle = Rover.throttle_set
                        if(not Rover.override_flag):
                            print('Override in progress')
                            Rover.override_flag = True
                            Rover.override_time = Rover.total_time
                        else:
                            if(Rover.total_time - Rover.override_time > 5):
                                #print('Total time: ' + str(Rover.total_time))
                                #print('Total time: ' + str(Rover.override_time))
                                Rover.average_vel = np.ones(Rover.number_update_values) # reset speed values
                                Rover.average_steer = np.zeros(Rover.number_update_values) # reset speed values
                                Rover.mode = 'forward'
                                Rover.in_recovery = False
                                Rover.override_flag = False
                                print('Override completed')
                    else:
                        Rover.steer = np.clip(Rover.goal_yaw - Rover.yaw, -15, 15) # set the steering angle

        # Check for Rover.mode status
        elif Rover.mode == 'forward': 
            # Check the extent of navigable terrain
            if len(Rover.nav_angles) >= Rover.stop_forward:  
                # If mode is forward, navigable terrain looks good 
                # and velocity is below max, then throttle 
                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                else: # Else coast
                    Rover.throttle = 0
                Rover.brake = 0
                # Set steering to average angle clipped to the range +/- 15
                Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            elif len(Rover.nav_angles) < Rover.stop_forward:
                    # Set mode to "stop" and hit the brakes!
                    Rover.throttle = 0
                    # Set brake to stored brake value
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0
                    Rover.mode = 'stop'

        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':
            # If we're in stop mode but still moving keep braking
            if Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                # Now we're stopped and we have vision data to see if there's a path forward
                if len(Rover.nav_angles) < Rover.go_forward:
                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    Rover.steer = -15 # Could be more clever here about which way to turn
                # If we're stopped but see sufficient navigable terrain in front then go!
                if len(Rover.nav_angles) >= Rover.go_forward:
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                    Rover.mode = 'forward'
    
    # Just to make the rover do something 
    # even if no modifications have been made to the code
    #else:
    #    Rover.throttle = Rover.throttle_set
    #    Rover.steer = 0
    #    Rover.brake = 0
        
    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True
    
    return Rover

