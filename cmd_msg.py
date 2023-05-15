from pymavlink import mavutil

import time 
# Create the connection
#device = '/dev/ttyAMA0'
#baudrate = 115200

#print('Connecting to ' + device + '...')
#vehicle = mavutil.mavlink_connection(device, baud=baudrate)

#vehicle.wait_heartbeat()

#master = vehicle




##Added Cmd Ack 
##Added Getting the last waypoint from the waypoint file 


################################CMD_APIs########################################################### 

#Python wrapper for the pymavlink arm api   
def ARM_THE_FCU(master, flag):

    '''
      mavlink cmd to send to the FCU to ARM

      Additional safety switch to avoid false arming the vehicle.
      Simulates the GCS behaviour.
    '''
    if flag:
        master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0)

        #The below code waits for the acknowledgement 
        while True:
            msg = master.recv_match(type=['COMMAND_ACK'])
            if msg:
                if msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM and msg.result == 0:
                    print("FCU is armed")
                    break
        return 1
    
    else:
        return 0


#Python wrapper for the pymavlink disarm api   
def DISARM_THE_FCU(master):

    '''
      mavlink cmd to send to the FCU to DISARM
    '''
    master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    0, 0, 0, 0, 0, 0, 0)

    while True:
        msg = master.recv_match(type=['COMMAND_ACK'])
        if msg:
            if msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM and msg.result == 0:
                print("FCU is disarmed")
                break
    return 1


def CHANGE_MODE(master, mode):

    '''
    Wrapper function that implements, The autopilot mode change. Takes in an extra parameter that suggests what mode. 
    Waits for acknowledge to proceed further.
    '''
    mode_id = master.mode_mapping()[mode]
    master.mav.set_mode_send(master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id)

    while True:
        msg = master.recv_match(type=['COMMAND_ACK'])
        if msg:
            if msg.command == mavutil.mavlink.MAVLINK_MSG_ID_SET_MODE and msg.result == 0:
                print("Mode changed to,", mode)
                break



################################MESSAGE_APIs###########################################################          

def GEO_LOCATION(master):

    '''
    Wrapper function that implements current GEO-location measurement.  I.e. Lat and Long.
    Waits for acknowledge to proceed further.
    '''
    result = master.location()
    return result.lat, result.lng

def BATTERY_STATUS(master):


    # Set up a listener to receive GCS messages
    while True:
        # Check for incoming messages from autopilot
        msg = master.recv_match(type='SYS_STATUS', blocking=True, timeout=1.0)

        # If a STATUSTEXT message is received, print the message text
        if msg is not None:
                # Extract battery status information
                voltage = msg.voltage_battery / 1000.0 # Convert to volts
                current = msg.current_battery / 100.0 # Convert to amps
                remaining_capacity = msg.battery_remaining # Percentage of remaining battery capacity
                remaining_mah = None #Formula to calculate the used mah
                #print("Voltage: ", voltage, "V")
                #print("Current: ", current, "A")
                print("Remaining MAH: ", remaining_mah, "%")
                break  # Break out of the loop after receiving battery status once
            
        time.sleep(0.1)

def LAND_DETECT(master):

   # Set up a listener to receive GCS like message from the autopilot
   #This function specifically looks for message string "Land complete".
    while True:
        # Check for incoming messages from autopilot
        #@Ref mavlink messages @https://mavlink.io/en/messages/common.html#STATUSTEXT
        msg = master.recv_match(type='STATUSTEXT', blocking=True, timeout=1.0)

        # If a STATUSTEXT message is received, print the message text
        if msg is not None:
            print(msg.text)

            if msg.text == "Land complete":
                print("Starting Button Sequence!")
                break #break the loop and start the sequence.
            
        # Do other things here as needed
        time.sleep(0.1) # Delay to avoid consuming too much CPU time
                

def TOGGLE_SAFETY_SWITCH(master, safety_switch_on):

    """
    Toggles the safety switch by sending a MAV_CMD_DO_SET_MODE command with the
    MAV_MODE_FLAG_SAFETY_ARMED flag set or unset, depending on the desired state
    of the safety switch.
    :param safety_switch_armed: True to arm the safety switch, False to disarm it.
    :param master: A pymavlink MAVLinkConnection object connected
                               to the drone's autopilot.
    """
    # Set the custom mode field to the desired value of the safety switch
    if safety_switch_on:
        mode = 1
    else:
        mode = 0


    # This mavlink cmd disables/enables the safety switch!
    master.mav.set_mode_send(master.target_system, mavutil.mavlink.MAV_MODE_FLAG_DECODE_POSITION_SAFETY, mode)

    #The below code waits for the acknowledgement 
    while True:
        msg = master.recv_match(type='HEARTBEAT', blocking=True)
        if msg:
            if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_DECODE_POSITION_SAFETY:
                if safety_switch_on != True:
                    print("Toggled safety switch to ON!")
                    print("Vehicle Arming is Disabled!")
                    print("It is safe to go near the vehicle!")
                    return 0
                
                else: 
                    print("Toggled safety switch to OFF!")
                    print("Vehicle Arming is enabled!")
                    return 1 

            break

    if(safety_switch_on) : return 0
    else: return 1


def RECV_MSG_AUTOPILOT(master):

    # Set up a listener to receive GCS like message from the autopilot
    while True:
        # Check for incoming messages from autopilot
        #@Ref mavlink messages @https://mavlink.io/en/messages/common.html#STATUSTEXT
        msg = master.recv_match(type='STATUSTEXT', blocking=True, timeout=1.0)

        # If a STATUSTEXT message is received, print the message text
        if msg is not None:
            print(msg.text)

        # Do other things here as needed
        time.sleep(0.1) # Delay to avoid consuming too much CPU time