# Autopilot service  

## Introduction

The autopilot service is an on-board module that controls the operation of the flight controller, as required by the rest of modules in the Drone Engineering Ecosystem.   
Dashboard or mobile applications will requiere the autopilot service to connect to the flight controller, to arm the drone, take-off, go to a certain position or move in a given direction, land, stop, etc. See the table bellow for a complete list of commands that can be accepted by the autopilot service in its current version.

## Installations
In order to run and contribute you must install Python 3.7. We recommend PyCharm as IDE for development. In addition, you must install the broker Mosquitto. The internal broker will be always run in localhome, port 1884, in your laptop when working in simulation mode. You must edit the configuration file (_mosquitto.conf_) with the following lines:

![image](https://github.com/eloimoncho/AutopilotService/assets/91852608/a2e2edb8-38d4-488a-afdc-a8cb2b4a872e)

And then run the file with the command: _mosquitto -v -c mosquitto.conf_

Contributions must follow the contribution protocol that you will find in the main repo of the Drone Engineering Ecosystem.
[![DroneEngineeringEcosystem Badge](https://img.shields.io/badge/DEE-MainRepo-brightgreen.svg)](https://github.com/dronsEETAC/DroneEngineeringEcosystemDEE)



## Commands
In order to send a command to the autopilot service, a module must publish a message in the external (or internal) broker. The topic of the message must be in the form:
```
"XXX/autopilotService/YYY"
```
where XXX is the name of the module requiring the service and YYY is the name of the service that is required. Oviously, some of the commands may require additional data that must be included in the payload of the message to be published. 
In some cases, after completing the service requiered the autopilot service publish a message as an answer. The topic of the answer has the format:
```
"autopilotService/XXX/ZZZ"
```
where XXX is the name of the module requiring the service and ZZZ is the answer. The message can include data in the message payload.

The table bellow indicates all the commands that are accepted by the autopilot service in the current version.   

Command | Description | Payload | Answer | Answer payload
--- | --- | --- | --- |--- 
*connect* | connect with the simulator or the flight controller depending on the operation mode | No | No (see Note 1) | No
*armDrone* | arms the drone (either simulated or real) | No | NO (see Note 2) | No 
*takeOff* | get the drone take off to reach and altitude of 5 meters | No | No (see Note 3)  | No 
*returnToLaunch* | go to launch position |No  | No (see Note 4) | No    
*land* | the dron will land |No  | No (see Note 5) | No     
*disarmDrone* | disarm the drone |No  |  No (see Note 6) | No 
*go* | move in certain direction |"North", "South", "East", "West", "NorthWest", "NorthEast", "SouthWest", "SouthEast" , "Stop"  | No | No 
*disconnect* | disconnect from the simulator or the flight controller depending on the operation mode | No | NO (see Note 1) | No
*executeFlightPlan* | execute the flight plan received | See Note 7 | see Note 7 | see Note 7


Note 1    
When the autopilot is connected the service will start sending telemetry_info packets every 250 miliseconds. The service will stop sending 
telemetry_info packets as soon as a *disconnect* command is received. This is an example of telemetry_info packet:

```
{
    'lat': 41.124567,
    'lon': 1.9889145,
    'heading': 270,
    'groundSpeed': 4.27,
    'altitude': 6.78,
    'battery': 80,
    'state': state
}

```
The packet includes the state of the autopilot so that the module that receives the packet can take decisions (for instance, change color of the buttons).   

These are the different values for the state:
*'connected'*  
*'arming'*  
*'armed'*   
*'disarmed'*    
*'takingOff'*      
*'flying'*   
*'returningHome'*     
*'landing'*    
*'onHearth'*    

Note 2    
The state will change to *arming* and then to *armed* as soon as the autopilot is armed.    
   
   
Note 3    
The state will change to *takingOff* and then to *flying* as soon as the autopilot reaches 5 meters of altitude.  
   
Note 4    
The state will change to *returningHome* and then to *onHearth* as soon as the autopilot in on hearth.    
   
Note 5    
The state will change to *landing* and then to *onHearth* as soon as the autopilot in on hearth.    
   
Note 6    
The state will change to *disarmed*.   

Note 7     
The service must receive a json object specifying the flight plan with indications whether a picture must be taken when reaching a waypoint. This is an example of such json object:    


```
[
  {
    'lat': 41.124567,
    'lon': 1.9889145,
    'takePic': True or False
  },
  {
    'lat': 41.124567,
    'lon': 1.9889145,
    'takePic': True or False
  },
  ....
]
```
The service will execute the flight plan, changing the state accordingly (*'arming'*, *'armed'*, *'takingOff'*, and so on until *'onHearth'*).    
When arrived to the next waypoint the service will publish this message: *'XXXX/autopilotService/waypointReached'*,, being XXXX the module requesting the service. The topic of the message is a json object containing *'lat'* and *'lon'* of the reached waypoint. If a picture must be taken in this waypoint, the service will publish this message IN THE INTERNAL BROKER: *'XXXX/cameraService/takePicture'*. The autopilot will return to launch after the last waypoint is reached.   



