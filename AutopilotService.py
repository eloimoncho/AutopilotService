import json
import math
import threading
import paho.mqtt.client as mqtt
import time
import dronekit
from dronekit import connect, Command, VehicleMode
from paho.mqtt.client import ssl
from pymavlink import mavutil
import requests
import sched


def arm():
    """Arms vehicle and fly to aTargetAltitude"""
    print("Basic pre-arm checks")  # Don't try to arm until autopilot is ready
    vehicle.mode = dronekit.VehicleMode("GUIDED")
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)
    print("Arming motors")
    # Copter should arm in GUIDED mode

    vehicle.armed = True
    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)
    print(" Armed")

def take_off(a_target_altitude, manualControl):
    global state
    vehicle.simple_takeoff(a_target_altitude)
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= a_target_altitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

    state = 'flying'
    if manualControl:
        w = threading.Thread(target=flying)
        w.start()


def prepare_command(velocity_x, velocity_y, velocity_z):
    """
    Move vehicle in direction based on specified velocity vectors.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,  # time_boot_ms (not used)
        0,
        0,  # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
        0b0000111111000111,  # type_mask (only speeds enabled)
        0,
        0,
        0,  # x, y, z positions (not used)
        velocity_x,
        velocity_y,
        velocity_z,  # x, y, z velocity in m/s
        0,
        0,
        0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0,
        0,
    )  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    return msg


'''
These are the different values for the state of the autopilot:
    'connected' (only when connected the telemetry_info packet will be sent every 250 miliseconds)
    'arming'
    'armed'
    'disarmed'
    'takingOff'
    'flying'
    'returningHome'
    'landing'
    'onHearth'

The autopilot can also be 'disconnected' but this state will never appear in the telemetry_info packet 
when disconnected the service will not send any packet
'''


def get_telemetry_info ():
    global state
    telemetry_info = {
        'lat': vehicle.location.global_frame.lat,
        'lon': vehicle.location.global_frame.lon,
        'heading': vehicle.heading,
        'groundSpeed': vehicle.groundspeed,
        'altitude': vehicle.location.global_relative_frame.alt,
        'battery': vehicle.battery.level,
        'state': state
    }
    return telemetry_info


def send_telemetry_info():
    global external_client
    global sending_telemetry_info
    global sending_topic

    while sending_telemetry_info:
        external_client.publish(sending_topic + "/telemetryInfo", json.dumps(get_telemetry_info()))
        time.sleep(0.25)


def returning():
    global sending_telemetry_info
    global external_client
    global internal_client
    global sending_topic
    global state

    # wait until the drone is at home
    while vehicle.armed:
        time.sleep(1)
    state = 'onHearth'

def flying():
    global direction
    global go
    speed = 1
    end = False
    cmd = prepare_command(0, 0, 0)  # stop
    while not end:
        go = False
        while not go:
            vehicle.send_mavlink(cmd)
            time.sleep(1)
        # a new go command has been received. Check direction
        print ('salgo del bucle por ', direction)
        if direction == "North":
            cmd = prepare_command(speed, 0, 0)  # NORTH
        if direction == "South":
            cmd = prepare_command(-speed, 0, 0)  # SOUTH
        if direction == "East":
            cmd = prepare_command(0, speed, 0)  # EAST
        if direction == "West":
            cmd = prepare_command(0, -speed, 0)  # WEST
        if direction == "NorthWest":
            cmd = prepare_command(speed, -speed, 0)  # NORTHWEST
        if direction == "NorthEast":
            cmd = prepare_command(speed, speed, 0)  # NORTHEST
        if direction == "SouthWest":
            cmd = prepare_command(-speed, -speed, 0)  # SOUTHWEST
        if direction == "SouthEast":
            cmd = prepare_command(-speed, speed, 0)  # SOUTHEST
        if direction == "Stop":
            cmd = prepare_command(0, 0, 0)  # STOP
        if direction == "RTL":
            end = True



def distanceInMeters(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.

    This method is an approximation, and will not be accurate over large distances and close to the
    earth's poles. It comes from the ArduPilot test code:
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5


def takePictureInterval(origin):
    internal_client.publish(origin + "/cameraService/takePictureInterval")
    print("Tomando foto por interval")


def start_interval(seconds, origin):
    global state
    scheduler = sched.scheduler(time.time, time.sleep)

    def picturesInterval():
        if state == 'flying' or state == 'returningHome':
            takePictureInterval(origin)
            scheduler.enter(seconds, 1, picturesInterval)

    scheduler.enter(seconds, 1, picturesInterval)
    timer = threading.Thread(target=scheduler.run)
    timer.start()


def executeFlightPlan(waypoints_json, application):
    global vehicle
    global internal_client, external_client
    global sending_topic
    global state
    global sending_telemetry_info
    global waypointImage
    global flightplan_id_ground
    global flight_id
    global waypointStartVideo
    global waypointEndVideo
    global latWaypointStart
    global lonWaypointStart
    global latWaypointEnd
    global lonWaypointEnd

    movingVideo = False
    altitude = 6
    origin = sending_topic.split('/')[1]

    if application == "dashboard":
        waypoints_data = json.loads(waypoints_json)
        print("waypoints_data", waypoints_data)
        waypoints = json.loads(waypoints_data['waypoints'])
    else:
        waypoints_data = json.loads(waypoints_json)
        print("waypoints_data", waypoints_data)
        waypoints = waypoints_data['waypoints']
    print("First Waypoint", waypoints[0])

    response_json = requests.get('http://localhost:9000/get_flightplan_id/' + flight_id).json()
    flightplan_id = response_json["FlightPlan id"]
    response_json = requests.get('http://localhost:9000/get_pic_interval/' + flightplan_id).json()
    pic_Interval = response_json["Pic interval"]
    response_json = requests.get('http://localhost:9000/get_vid_interval/' + flightplan_id).json()
    video_Interval = response_json["Vid interval"]
    waypointStartVideo = 1  # Añadido para evitar tener error de no asociación del valor
    waypointEndVideo = 1

    state = 'arming'
    arm()
    state = 'takingOff'
    take_off(altitude, False)
    state = 'flying'
    #vehicle.groundspeed=3

    wp = waypoints[0]
    originPoint = dronekit.LocationGlobalRelative(float(wp['lat']), float(wp['lon']), altitude)

    distanceThreshold = 0.50

    if pic_Interval > 0:
        start_interval(pic_Interval, origin)
        print("Picures will be taken every " + str(pic_Interval) + " seconds")

    waypointOrder = 0
    for wp in waypoints [1:]:

        destinationPoint = dronekit.LocationGlobalRelative(float(wp['lat']),float(wp['lon']), altitude)
        vehicle.simple_goto(destinationPoint, groundspeed=3)

        currentLocation = vehicle.location.global_frame
        dist = distanceInMeters (destinationPoint,currentLocation)

        while dist > distanceThreshold:
            time.sleep(0.25)
            currentLocation = vehicle.location.global_frame
            dist = distanceInMeters(destinationPoint, currentLocation)
        print ('reached')
        waypointReached = {
            'lat':currentLocation.lat,
            'lon':currentLocation.lon
        }

        external_client.publish(sending_topic + "/waypointReached", json.dumps(waypointReached))

        if wp['takePic']:
            print("Picture taken")
            waypointImage = waypointOrder
            latWaypointStart = float(wp['lat'])
            lonWaypointStart = float(wp['lon'])
            internal_client.publish(origin + "/cameraService/takePicture")
        if application == "dashboard":
            if wp['videoStart']:
                print("Start video moving")
                waypointStartVideo = waypointOrder
                latWaypointStart = float(wp['lat'])
                lonWaypointStart = float(wp['lon'])
                internal_client.publish(origin + "/cameraService/startVideoMoving")
            if wp['videoStop']:
                print("Stopped video moving")
                waypointEndVideo = waypointOrder
                latWaypointEnd = float(wp['lat'])
                lonWaypointEnd = float(wp['lon'])
                internal_client.publish(origin + "/cameraService/endVideoMoving")
        if application == "mobileApp":
            # Esta es la alternativa a videoStart y videoStop si se envía el plan de vuelo desde el movil
            if wp['movingVideo'] and movingVideo == False:
                print("Start video moving")
                movingVideo = True
                waypointStartVideo = waypointOrder
                latWaypointStart = float(wp['lat'])
                lonWaypointStart = float(wp['lon'])
                internal_client.publish(origin + "/cameraService/startVideoMoving")
            elif wp['movingVideo'] and movingVideo == True:
                print("Stopped video moving")
                movingVideo = False
                waypointEndVideo = waypointOrder
                latWaypointEnd = float(wp['lat'])
                lonWaypointEnd = float(wp['lon'])
                internal_client.publish(origin + "/cameraService/endVideoMoving")
        if wp['staticVideo']:
            if video_Interval > 0:
                waypointStartVideo = waypointOrder
                waypointEndVideo = waypointOrder
                latWaypointStart = float(wp['lat'])
                lonWaypointStart = float(wp['lon'])
                latWaypointEnd = float(wp['lat'])
                lonWaypointEnd = float(wp['lon'])
                internal_client.publish(origin + "/cameraService/startStaticVideo", str(video_Interval))
                time.sleep(video_Interval + 1) # Añadimos 1 segundo de tolerancia para permitir las comunicaciones y que la duración del vídeo sea la correcta
                print("Stopped filming static video")
            else:
                print("No static video taken as interval unknown")

    vehicle.mode = dronekit.VehicleMode("RTL")
    state = 'returningHome'

    currentLocation = vehicle.location.global_frame
    dist = distanceInMeters(originPoint, currentLocation)

    while dist > distanceThreshold:
        time.sleep(0.25)
        currentLocation = vehicle.location.global_frame
        dist = distanceInMeters(originPoint, currentLocation)

    state = 'landing'
    while vehicle.armed:
        time.sleep(1)
    state = 'onHearth'

    if application == "mobileApp":
        external_client.publish("autopilotService/mobileApp/flightEnded")
        sending_telemetry_info = False
        print("Sent to the mobileApp that the flight is ended")

        # In the case of Mobile App, information is saved automatically in ground backend as soon as the flight ends
        response_json = requests.get('http://localhost:9000/get_flight/' + flight_id).json()
        dataFlight = {
            'Date': response_json["Date"],
            'startTime': response_json["startTime"],
            'GeofenceActive': response_json["GeofenceActive"],
            'Flightplan': flightplan_id_ground,
            'NumPics': response_json["NumPics"],
            'Pictures': response_json["Pictures"],
            'NumVids': response_json["NumVids"],
            'Videos': response_json["Videos"]
        }
        data = json.dumps(dataFlight)
        headers = {'Content-Type': 'application/json'}
        response = requests.post('http://localhost:8105/add_flight', data=data, headers=headers)
        internal_client.publish("autopilotService/cameraService/saveMediaApi", flight_id)


def executeFlightPlanMobileApp(waypoints_json):
    global vehicle
    global internal_client, external_client
    global sending_topic
    global state

    altitude = 10
    origin = sending_topic.split('/')[1]

    waypoints_data = json.loads(waypoints_json)
    print("waypoints_data", waypoints_data)
    waypoints = waypoints_data['waypoints']
    print("First Waypoint", waypoints[0])
    state = 'arming'
    arm()
    state = 'takingOff'
    take_off(altitude, False)
    state = 'flying'
    #vehicle.groundspeed=3

    wp = waypoints[0]
    originPoint = dronekit.LocationGlobalRelative(float(wp['lat']), float(wp['lon']), altitude)

    distanceThreshold = 0.50
    for wp in waypoints [1:]:

        destinationPoint = dronekit.LocationGlobalRelative(float(wp['lat']),float(wp['lon']), altitude)
        vehicle.simple_goto(destinationPoint, groundspeed=3)

        currentLocation = vehicle.location.global_frame
        dist = distanceInMeters (destinationPoint,currentLocation)

        while dist > distanceThreshold:
            time.sleep(0.25)
            currentLocation = vehicle.location.global_frame
            dist = distanceInMeters(destinationPoint, currentLocation)
        print ('reached')
        waypointReached = {
            'lat':currentLocation.lat,
            'lon':currentLocation.lon
        }

        external_client.publish(sending_topic + "/waypointReached", json.dumps(waypointReached))

        if wp['takePic']:
            # ask to send a picture to origin
            internal_client.publish(origin + "/cameraService/takePicture")

    vehicle.mode = dronekit.VehicleMode("RTL")
    state = 'returningHome'

    currentLocation = vehicle.location.global_frame
    dist = distanceInMeters(originPoint, currentLocation)

    while dist > distanceThreshold:
        time.sleep(0.25)
        currentLocation = vehicle.location.global_frame
        dist = distanceInMeters(originPoint, currentLocation)

    state = 'landing'
    while vehicle.armed:
        time.sleep(1)
    state = 'onHearth'


def executeFlightPlan2(waypoints_json):
    global vehicle
    global internal_client, external_client
    global sending_topic
    global state

    altitude = 6
    origin = sending_topic.split('/')[1]

    waypoints = json.loads(waypoints_json)
    state = 'arming'
    arm()
    state = 'takingOff'
    take_off(altitude, False)
    state = 'flying'
    cmds = vehicle.commands
    cmds.clear()

    #wp = waypoints[0]
    #originPoint = dronekit.LocationGlobalRelative(float(wp['lat']), float(wp['lon']), altitude)
    for wp in waypoints:
        cmds.add(
            Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0,
                    0, 0, 0, 0, float(wp['lat']), float(wp['lon']), altitude))
    print("waypoints[0]2", waypoints[0])
    wp = waypoints[0]
    cmds.add(
        Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0,
                0, 0, 0, 0, float(wp['lat']), float(wp['lon']), altitude))
    cmds.upload()

    vehicle.commands.next = 0
    # Set mode to AUTO to start mission
    vehicle.mode = VehicleMode("AUTO")
    while True:
        nextwaypoint = vehicle.commands.next
        print ('next ', nextwaypoint)
        if nextwaypoint == len(waypoints):  # Dummy waypoint - as soon as we reach waypoint 4 this is true and we exit.
            print("Last waypoint reached")
            break;
        time.sleep(0.5)

    print('Return to launch')
    state = 'returningHome'
    vehicle.mode = VehicleMode("RTL")
    while vehicle.armed:
        time.sleep(1)
    state = 'onHearth'

def set_direction(color):
        if color == 'blueS':
            return "North"
        elif color == "yellow":
            return "East"
        elif color == 'green':
            return "West"
        elif color == 'pink':
            return "South"
        elif color == 'purple':
            return "RTL"
        else:
            return "none"




def process_message(message, client):
    global vehicle
    global direction
    global go
    global sending_telemetry_info
    global sending_topic
    global op_mode
    global sending_topic
    global state
    global flight_id
    global flightplan_id_ground
    global waypointImage
    global waypointStartVideo
    global waypointEndVideo
    global latWaypointStart
    global lonWaypointStart
    global latWaypointEnd
    global lonWaypointEnd

    splited = message.topic.split("/")
    origin = splited[0]
    command = splited[2]
    sending_topic = "autopilotService/" + origin
    print ('recibo ', command)

    if command == "position":
        print("Position: ", message.payload )

    if command == "connect":
        if state == 'disconnected':
            print("Autopilot service connected by " + origin)
            #para conectar este autopilotService al dron al mismo tiempo que conectamos el Mission Planner
            # hay que ejecutar el siguiente comando desde PowerShell desde  C:\Users\USER>
            #mavproxy - -master =COM12 - -out = udp:127.0.0.1: 14550 - -out = udp:127.0.0.1: 14551
            # ahora el servicio puede conectarse por udp a cualquira de los dos puertos 14550 o 14551 y Mission Planner
            # al otro

            if op_mode == 'simulation':
                connection_string = "tcp:127.0.0.1:5763"
                #connection_string = "udp:127.0.0.1:14550"
                #connection_string = "com7"
            else:
                # connection_string = "/dev/ttyS0"
                connection_string = "com7"
                #connection_string = "udp:127.0.0.1:14550"

            #vehicle = connect(connection_string, wait_ready=False, baud=115200)
            vehicle = connect(connection_string, wait_ready=False, baud=57600)

            vehicle.wait_ready(True, timeout=5000)

            print ('Connected to flight controller')
            state = 'connected'

            #external_client.publish(sending_topic + "/connected", json.dumps(get_telemetry_info()))


            sending_telemetry_info = True
            y = threading.Thread(target=send_telemetry_info)
            y.start()
        else:
            print ('Autopilot already connected')

    if command == "disconnect":
        vehicle.close()
        sending_telemetry_info = False
        state = 'disconnected'

    if command == "takeOff":
        state = 'takingOff'
        w = threading.Thread(target=take_off, args=[5,True ])
        w.start()

    if command == "returnToLaunch":
        # stop the process of getting positions
        vehicle.mode = dronekit.VehicleMode("RTL")
        state = 'returningHome'
        direction = "RTL"
        go = True
        w = threading.Thread(target=returning)
        w.start()

    if command == "armDrone":
        print ('arming')
        state = 'arming'
        arm()

        # the vehicle will disarm automatically is takeOff does not come soon
        # when attribute 'armed' changes run function armed_change
        vehicle.add_attribute_listener('armed', armed_change)
        state = 'armed'

    if command == "disarmDrone":
        vehicle.armed = False
        while vehicle.armed:
            time.sleep(1)
        state = 'disarmed'

    if command == "land":

        vehicle.mode = dronekit.VehicleMode("LAND")
        state = 'landing'
        while vehicle.armed:
            time.sleep(1)
        state = 'onHearth'

    if command == "go":
        direction = message.payload.decode("utf-8")
        print("Going ", direction)
        go = True

    if command == 'executeFlightPlan':
        waypoints_json = str(message.payload.decode("utf-8"))
        flightplan_id_ground = message["id"]
        waypoints_json = message["waypoints"]
        flight_id = splited[3]
        w = threading.Thread(target=executeFlightPlan, args=[waypoints_json, "dashboard"])
        w.start()

    if command == 'executeFlightPlanMobileApp':
        waypoints_json = str(message.payload.decode("utf-8"))
        waypoints_data = json.loads(waypoints_json)
        print("waypoints_data", waypoints_data)
        waypoints = waypoints_data['waypoints']
        title = waypoints_data["Title"]
        response = requests.get('http://localhost:8105/get_flight_plan_id/' + title).json()
        flightplan_id_ground = response["id"]
        headers = {'Content-Type': 'application/json'}
        response = requests.get('http://localhost:9000/get_flight_plan/' + title).json()
        flightplan_id_drone = response["_id"]
        numPictures = len(response["PicsWaypoints"])
        numVideos = len(response["VidWaypoints"])
        data = {
            "GeofenceActive": True,
            "Flightplan": flightplan_id_drone,
            "NumPics": numPictures,
            "NumVids": numVideos
        }
        response_air = requests.post('http://localhost:9000/add_flight', json=data, headers=headers)
        flight_id = response_air.json()["id"]

        w = threading.Thread(target=executeFlightPlan, args=[waypoints_json, "mobileApp"])
        w.start()

    if command == 'savePicture':
        name_picture = message.payload.decode("utf-8")
        headers = {'Content-Type': 'application/json'}
        print("Saving picture " + name_picture)
        data = {
            "namePicture": name_picture,
            "idFlight": flight_id,
            "waypoint": waypointImage,
            "latImage": latWaypointStart,
            "lonImage": lonWaypointStart
        }
        response = requests.put('http://localhost:9000/add_picture', json=data, headers=headers)

    if command == 'savePictureInterval':
        name_picture = message.payload.decode("utf-8")
        headers = {'Content-Type': 'application/json'}
        print("Saving picture " + name_picture)
        data = {
            "namePicture": name_picture,
            "idFlight": flight_id,
            "waypoint": -1
        }
        response = requests.put('http://localhost:9000/add_picture', json=data, headers=headers)

    if command == 'saveVideo':
        name_video = message.payload.decode("utf-8")
        print("Saving video " + name_video)
        headers = {'Content-Type': 'application/json'}
        data = {
            "nameVideo": name_video,
            "idFlight": flight_id,
            "startVideo": waypointStartVideo,
            "endVideo": waypointEndVideo,
            "latStart": latWaypointStart,
            "lonStart": lonWaypointStart,
            "latEnd": latWaypointEnd,
            "lonEnd": lonWaypointEnd
        }
        response = requests.put('http://localhost:9000/add_video', json=data, headers=headers)

    if command == 'saveMediaApi':
        print("Asking the camera to send the media to the api")
        response_json = requests.get('http://localhost:9000/get_flight/' + flight_id).json()
        dataFlight = {
            'Date': response_json["Date"],
            'startTime': response_json["startTime"],
            'GeofenceActive': response_json["GeofenceActive"],
            'Flightplan': flightplan_id_ground,
            'NumPics': response_json["NumPics"],
            'Pictures': response_json["Pictures"],
            'NumVids': response_json["NumVids"],
            'Videos': response_json["Videos"]
        }
        data = json.dumps(dataFlight)
        headers = {'Content-Type': 'application/json'}
        response = requests.post('http://localhost:8105/add_flight', data=data, headers=headers)
        internal_client.publish("autopilotService/cameraService/saveMediaApi", flight_id)


    if command == 'videoFrameWithColor':
            # ya se está moviendo. Solo entonces hacemos caso de los colores
            frameWithColor = json.loads(message.payload)
            d = set_direction(frameWithColor['color'])
            if d!= 'none':
                direction = d
                if direction == 'RTL':
                    vehicle.mode = dronekit.VehicleMode("RTL")
                    print ('cambio estado')
                    state = 'returningHome'
                    w = threading.Thread(target=returning)
                    w.start()

                go = True


def armed_change(self, attr_name, value):
    global vehicle
    global state
    global sending_telemetry_info
    print ('cambio a ', )
    if vehicle.armed:
        state = 'armed'
    else:
        state = 'disarmed'
        sending_telemetry_info = False

    print ('cambio a ', state)

def on_internal_message(client, userdata, message):
    global internal_client
    process_message(message, internal_client)

def on_external_message(client, userdata, message):
    global external_client
    process_message(message, external_client)

def on_connect(external_client, userdata, flags, rc):
    if rc==0:
        print("Connection OK")
    else:
        print("Bad connection")

def AutopilotService (connection_mode, operation_mode, external_broker, username, password):
    global op_mode
    global external_client
    global internal_client
    global state

    state = 'disconnected'

    print ('Connection mode: ', connection_mode)
    print ('Operation mode: ', operation_mode)
    op_mode = operation_mode



    internal_client = mqtt.Client("Autopilot_internal")
    internal_client.on_message = on_internal_message
    internal_client.connect("localhost", 1884)


    external_client = mqtt.Client("Autopilot_external", transport="websockets")
    external_client.on_message = on_external_message
    external_client.on_connect = on_connect

    if connection_mode== "global":
        if external_broker == "hivemq":
            external_client.connect("broker.hivemq.com", 8000)
            print('Connected to broker.hivemq.com:8000')

        elif external_broker == "hivemq_cert":
            external_client.tls_set(ca_certs=None, certfile=None, keyfile=None, cert_reqs=ssl.CERT_REQUIRED,
                           tls_version=ssl.PROTOCOL_TLS, ciphers=None)
            external_client.connect("broker.hivemq.com", 8884)
            print('Connected to broker.hivemq.com:8884')
        elif external_broker == "classpip_cred":
            external_client.username_pw_set(
                username, password
            )
            external_client.connect("classpip.upc.edu", 8000)
            print('Connected to classpip.upc.edu:8000')

        elif external_broker == "classpip_cert":
            external_client.username_pw_set(
                username, password
            )
            external_client.tls_set(ca_certs=None, certfile=None, keyfile=None, cert_reqs=ssl.CERT_REQUIRED,
                           tls_version=ssl.PROTOCOL_TLS, ciphers=None)
            external_client.connect("classpip.upc.edu", 8883)
            print('Connected to classpip.upc.edu:8883')
        elif external_broker == "localhost":
            external_client.connect("localhost", 8000)
            print('Connected to localhost:8000')
        elif external_broker == "localhost_cert":
            print('Not implemented yet')

    elif connection_mode == "local":
        if operation_mode == "simulation":
            external_client.connect("localhost", 8000)
            print('Connected to localhost:8000')
        else:
            external_client.connect("10.10.10.1", 8000)
            print('Connected to 10.10.10.1:8000')



    print("Waiting....")
    external_client.subscribe("+/autopilotService/#", 2)
    external_client.subscribe("cameraService/+/#", 2)
    internal_client.subscribe("+/autopilotService/#")
    internal_client.loop_start()
    if operation_mode == 'simulation':
        external_client.loop_forever()
    else:
        #external_client.loop_start() #when executed on board use loop_start instead of loop_forever
        external_client.loop_forever()




if __name__ == '__main__':
    import sys
    connection_mode = sys.argv[1] # global or local
    operation_mode = sys.argv[2] # simulation or production
    username = None
    password = None
    if connection_mode == 'global':
        external_broker = sys.argv[3]
        if external_broker == 'classpip_cred' or external_broker == 'classpip_cert':
            username = sys.argv[4]
            password = sys.argv[5]
    else:
        external_broker = None

    AutopilotService(connection_mode,operation_mode, external_broker, username, password)
