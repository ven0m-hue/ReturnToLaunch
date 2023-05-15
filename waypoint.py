from pymavlink import mavutil , mavwp

import time
import os 

# Create the connection
#device = '/dev/ttyAMA0'
#baudrate = 115200

#print('Connecting to ' + device + '...')
#vehicle = mavutil.mavlink_connection(device, baud=baudrate)

#vehicle.wait_heartbeat()

#master = vehicle

##Added Upload mission based on the current geo-location 



wp = mavwp.MAVWPLoader()


# TODO : Remove all the print functions  

def CMD_SET_HOME(master, home_location, altitude):
    print('--- ', master.target_system, ',', master.target_component)
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_HOME,
        1, # set position
        0, # param1
        0, # param2
        0, # param3
        0, # param4
        home_location[0], # lat
        home_location[1], # lon
        altitude) 


def UPLOAD_MISSION(master, aFileName):
    home_location = None
    home_altitude = None

    mission_file = "/home/pi/ProjectBtn/MissionFiles/" + aFileName

    with open(mission_file) as f:
        for i, line in enumerate(f):
            if i==0:
                if not line.startswith('QGC WPL 110'):
                    raise Exception('File is not supported WP version')
            else:   
                linearray=line.split('\t')
                ln_seq = int(linearray[0])
                ln_current = int(linearray[1])
                ln_frame = int(linearray[2])
                ln_command = int(linearray[3])
                ln_param1=float(linearray[4])
                ln_param2=float(linearray[5])
                ln_param3=float(linearray[6])
                ln_param4=float(linearray[7])
                ln_x=(float(linearray[8]))
                ln_y=(float(linearray[9]))
                ln_z=float(linearray[10])
                ln_autocontinue = int(float(linearray[11].strip()))
                if(i == 1):
                    home_location = (ln_x,ln_y)
                    home_altitude = ln_z
                p = mavutil.mavlink.MAVLink_mission_item_message(master.target_system, master.target_component, ln_seq, ln_frame,
                                                                ln_command,
                                                                ln_current, ln_autocontinue, ln_param1, ln_param2, ln_param3, ln_param4, ln_x, ln_y, ln_z)
                wp.add(p)
                
                    
    CMD_SET_HOME(master, home_location,home_altitude)
    msg = master.recv_match(type = ['COMMAND_ACK'],blocking = True)
    print(msg)
    print('Set home location: {0} {1}'.format(home_location[0],home_location[1]))
    time.sleep(1)
    
    #send waypoint to airframe
    master.waypoint_clear_all_send()
    master.waypoint_count_send(wp.count())
    for i in range(wp.count()):
        msg = master.recv_match(type=['MISSION_REQUEST'],blocking=True)
        print(msg)
        master.mav.send(wp.wp(msg.seq))
        #print(wp.wp(msg.seq))
        print('Sending waypoint {0}'.format(msg.seq))


    



def RESET_MISSION(master):

    '''
    mavlink cmd to reset the ground station waypoint
    '''
    master.mav.mission_clear_all_send(
        master.target_system, master.target_component)
    

def FLASH_MSG_GCS(master, text):

    '''
    mavlink cmd to flash message to the GCS 
    more info @statustext 
    '''

    # Define the message severity (2 for critical, 3 for error, 4 for warning, 5 for notice, 6 for info)
    #Ref mavlink messages @https://mavlink.io/en/messages/common.html#MAV_MODE
    severity = 6

    # Send the flash message
    master.mav.statustext_send(severity, text.encode())



def GET_LAST_GEO_LOCATION(master):

     # send command to request current mission items
    master.waypoint_request_list_send()

    # wait for mission count message
    msg = None
    while msg is None or msg.get_type() != 'MISSION_COUNT':
        msg = master.recv_match(type='MISSION_COUNT', blocking=True)

    mission_count = msg.count

    # send command to request last waypoint
    master.waypoint_request_send(mission_count - 2) #Last but 2nd wavpoint since, last waypoint is Aux function!

    # wait for mission item message
    msg = None
    while msg is None or msg.get_type() != 'MISSION_ITEM':
        msg = master.recv_match(type='MISSION_ITEM', blocking=True)

    # extract latitude and longitude from mission item
    latitude = msg.x     
    longitude = msg.y

    # return the location as a tuple
    return (latitude, longitude)



'''
    Compare current geo-location 

    1. Get the current geo-location 
    2. Read the mission files sotored in the CC 
    3. Upload the mission file that matches with the current geo-location within set threshold tollerance. Indicating with the Green Light!
    4. If not throw an error - Perhaps RED light on the neopixel inidcating error. And Force disarm 

'''

def COMPARE_LAT_LON_WITH_MISSION_FILES(gps_lat, gps_lon, mission_dir, tolerance):
    # List all .txt files in the mission directory
    mission_files = [f for f in os.listdir(mission_dir) if f.endswith('.waypoints')]

    # Loop through all mission files and compare lat and lon with the GPS measurement
    for mission_file in mission_files:
        # Open the mission file and read the second line
        with open(os.path.join(mission_dir, mission_file)) as f:
            # Call readline() once to read the first line (which we discard)
            f.readline()

            # Call readline() again to read the second line (which contains lat/lon)
            second_line = f.readline()
            print(type(second_line))
            # Extract the lat/lon from the second line and convert to floats
            lat_lon = second_line.split()
            print(lat_lon)
            mission_lat = float(lat_lon[8])
            mission_lon = float(lat_lon[9])

        # Calculate the distance between the mission lat/lon and GPS measurement
        distance = ((gps_lat - mission_lat) ** 2 + (gps_lon - mission_lon) ** 2) ** 0.5
        print(distance)

        # Check if the distance is within the specified range
        if distance <= tolerance:
            # Return the mission file name
            return mission_file

    # Return None if no mission files were found within the specified range
    return None