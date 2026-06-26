# Traffic Signal Priority (TSP) Digital Twin

**Modul:** Digital Twin Design – SS 2026
**Team:** Ruben Geiger & Michael Kimmerle
**Abgabe:** 26.06.2026
**Zielkreuzung:** TechCampus Heilbronn

Attention: For some comments in the text, Claude was used as a tutor, or VS Code's automatic comment completion. Where AI was used for code sections, the AI was used only as a tutor, not as a 'do this for me' tool. This section is marked accordingly.

---

## 1. Projektüberblick


This project demonstrate an Digital Twin of a Junction near the tech Campus Heilbronn. The Digital Twin shows the functionality of a traffic sign priority. That enabels vehicles like buses or emergency cars to get an proirity access of the traffic sign. 

The Digital Twin shows with SUMO the real traffic dynamic. ROS2 enables the Controllogic which can later used to implement it in an real traffic sign. 

This Digital Twin provides three demo scenarios that build up in complexity:

- **Scenario 0 (Baseline):** A single bus approaches the junction with no other traffic. Demonstrates the TSP mechanism in isolation.
- **Scenario 1 (Simple):** A bus approaches the junction while light background traffic is active. Shows the TSP effect under realistic        conditions.
- **Scenario 2 (Conflict):** A bus activates TSP, followed shortly by an emergency vehicle from a different direction. Demonstrates the priority override logic where the emergency vehicle takes precedence over the bus. 

## 2. Systemarchitektur

The systemarchitecture is showed in the follwing Table. 

| Node | Subscribed | Published | Task |
|---|---|---|---|
| vehicle_publisher | TraCI (SUMO) | vehicle_status | Reads vehicle data from SUMO |
| tsp_controller | vehicle_status | tls_command | Priority decision logic |
| signal_controller |tls_command | TraCI (SUMO) | Controls the traffic light |
| metrics_logger | vehicle_status, tls_command | – | Logs events for evaluation |
| dashboard_node | vehicle_status, tls_command | – | Terminal visualization for the human operator |

The dataflow is in a cirle: SUMO , vehicle_publisher, tsp_controller, signal_controler, SUMO. The dession is based on a 4 Node architecure, which allows to seperate the data containing, the desision, actuatoric, logging. In addition, dashboard_node taps into the same two topics passively to provide a human-readable live view, without influencing the control loop.

## 3. Custom Messages

### 3.1 VehicleStatus.msg
| Field | Type | Meaning |
|---|---|---|
| header | std_msgs/Header | timestamp of the message |
| vehicle_id | string | SUMO ID of the vehicle |
| vehicle_type | string | bus_type / emergency_type |
| position | geometry_msgs/Point | position (x, y, z) of the vehicle |
| speed | float32 | current speed |
| eta_seconds | float32 | estimated time of arrival at the junction |
| intersection_id | string | ID of the target junction |
| approach_edge | string | edge the vehicle uses to approach the traffic light |

### 3.2 TLSCommand.msg
| Field | Type | Meaning |
|---|---|---|
| header | std_msgs/Header | timestamp of the message |
| tls_id | string | ID of the affected traffic light |
| target_phase | int32 | target phase; currently always -1, since signal_controller calculates the actual phase itself |
| duration_sec | float32 | how long the phase should be held |
| reason | string | human-readable reason (also used by metrics_logger to extract vehicle_id/type) |
| approach_edge | string | approach direction that should be prioritized |


We build some own messages because the standard ROS2 Typs didnt know the concepts of SUMO like Egde-ID or TraCi domain knowlegde. 


---

## 4. Logic Node

### 4.1 vehicle_publisher

The vehicle_publisher node is the connection between SUMO and ROS2. Without these node no other node knowns where is the car e.g.. The node translate the SUMO RAW data like coordinates, velocity or route in one ROS2 message that all other Nodes could understand.

#### Most important functions

getControlledLinks(JUNCTION_ID): First time start in the boot process and ask TraCi with edges are connected to the traffic sign. The result is saved in self.tls_edges. These function isnt ask every tick because the information dosent change in our scenario.

get_approach_edge(): starts every car tick and collect the routes of the car (getRoute(), getRouteIndex()). The founded routes will be compared with the edges in self.tls.edges. If there are the same edges in both, that is the edge the car arrives the junction. These function is nessesary to check the drivong direction of the car to the traffic sign. 

Distance: To calculate if the vehicle is clsoe enaught to be relevant for the traffic sign Priority. Calculation sqrt((x-JUNCTION_X)**2 + (y-JUNCTION_Y)**2)

At the end of the Node we build the VehicleStatus-Message with ID, typ, position, approach_edge e.g and publish the Node.

### 4.2 tsp_controller

The tsp_controller is the desicion making central. The node gets the vehicle data and dicide if the car is a priority vehicle or not. 

#### Most Important functions:

1. filter for just our Junction and priority vehicles (intersection_id == JUNCTION_ID, PRIORITY_TYPES)
2. if the car is active no activation needed (self.active_tsp) 
3. ETA-filter if the car is too far away -> wait (eta_seconds > ETA_THRESHOLD)
4. Emergency-Override: if there is a Emergency vehicle no other car can get priority. If there is a recognice Bus the emergency vehicle get an higher priority (EMERGENCY OVERRIDE)
If there are two buses the first one will recive the Priority the tracking is computed with (current_priority_vehicle, current_priority_type)
5. Priority release: every vehicle status message updates a last_seen timestamp for that vehicle, even after it is already active. A periodic check_release() timer (every 0.5s) clears current_priority_vehicle / current_priority_type once no update was received for RELEASE_TIMEOUT (2.0s) — this is how the controller detects that the prioritized vehicle has left the junction and frees up priority for the next vehicle. A vehicle that gets displaced by an Emergency Override (see point 4) is also removed from active_tsp at that moment, so it can request TSP again once priority is released.

6. The duration of the green phase is hard coded at 15 sec.
7. active_tsp disable a repatable change of the state for an activated vehicle
8. Build the TLSCommand with approach_edge
9. Debug function. TSP_ENABLED allows to activte and deactivte the TSP functionality

### 4.3 signal_controller

The signal_controller node is the actuatoric node. These node is the single one that changes something in SUMO. The task is to translate the abstract vehicle x need priority on egde y in phasecontroll

#### Most important functions

For the start the node build self.edge_to_links and self.phases with getControlledLinks() and getCompleteRedYellowGreenDefinition().

find_best_phase(approach_edge): 

1. Get the link indicies to the connected approach_edge
2. go trough all kind of phases of the traffic sign
3. Check the relevant link indicies, and the courrent sign: G = 2 points, g = 1 point, else 0.
4. sum up the Score to each phase
5. communicate the phase with the highes score. 

The reason that the phase isnt hardcoded is that it isnt garanted that phase 0 is the correct Greenphase on the aproached track. The system has to check which is the correct phase.

traci.trafficlight.setPhase(): switch the phase in SUMO

### 4.4 metrics_logger
This note writes all relvant Events into a .csv. This allows us to get an overview over all events and we can see if the system is running correctly or if something strange happens. 

#### Cosntruction of the CSV: 
timestamp, event, vehicle_id, vehicle_type, approach_edge, eta_seconds, speed, waiting_time_sec, duration_sec, reason
To get access with all platforms, we implemented an relativ File where the .csv is saved. Furthermore we implemented in that way, that after each Event the message will be push that due an Error we do not lose the data. 

#### Tracked Event typs

Detected: first siding of the vehicle (vehicle_status_callback)
TSP_ACTIVATED is loged in case of an TLSCommand
EMERGENCY_OVER_BUS: additional event only if the vehicle type is emergency_type and one or more buses are tracked
PASSED: Vehicle has left the Junction

#### Tracking logic

Saved for each vehicle is: type, approached_edge, first_seen, last_seen, last velocity, ETA, waiting time

Calculation of the waiting time: At each Update it will check, if speed < WAITING_SPEED_THRESHOLD (0.5). If this case is true we add dt on the waiting_time.

#### important function
check_passed(): To estimate if the vehicle is outside of the Junction, we impement an Timeout. If PASSED_TIMEOUT_SEC (2.0) didnt send a new vehicleStatus the vehicle is classified as PASSED. 

### 4.5 Dashboard_node

The Dashboard node is the human machine connection, thats the only node that does not decide or controll something. Instead they collect all states from the other nodes and converts it to a human readable output. 
 
 ### important Functions

First subscribe with vehicle_status and tls_command. Thats two Topics that currently flows between the other nodes.

vehicle_callback(msg) covers for all  vehicles the last known state in self.vehicles including last_seen. 

command_callback(msg): save the last message that comes fprm TLSCommand

render(): the main function that covers the REFRESH_INTERVAL
1. disconnetced Car which send longer than STALE_TIMEOUT no Updated
2. builds a table with all active vehicles with (id,type, approach, eta,speed)
3. shows the last TSP request
4. Clear Screen to provide a new window without other messages

The reason, that we dont need a extra message for our Dashboard is that all informations are currently avalible in vehicle-status and tls_command. 

#### Reference runs

To evaluate the effect of TSP, the metrics CSVs can be compared between two runs:
- one run with TSP_ENABLED = True (normal operation)
- one run with TSP_ENABLED = False (baseline, no priority logic active)

Reference runs for each scenario are stored in metrics/reference_runs using the naming pattern <scenario>_tsp_<on|off>.csv. Comparing the two files for a given scenario shows the actual impact of TSP on the priority vehicles.



## 5. Technical Problemes & Solutions

### Problem 1 – Distancemasurement
Symptom: If we tried to use the traci.getDrivingDistance function we are confronted with some sporadicaly worng results like that -1073741824.0
Cause: Thats the number TraCi gets back when something like the Route calculation dosnt work. 
Solution: Euclidain distance between vehicle coordinates and the Junction coodrinate. (sqrt((x1-x2)² + (y1-y2)²)) 

### Problem 2 – Phasedecison
Symptom: GREEN_PHASE_INDEX=0 just works randomly 
Solution: the find_best_phase() function

### Problem 3 – Multi-Client TraCI
Symptom: the second Node (signal_contoller) could not get access with Sumo because the connection is blocked due the first Node. 
Cause: in default TraCi couldnt allow more than one Connection
Solution: Start SUMO with --num-clinets 2. That allows taht SUMO wait activly for the second clients. For a robust start the signal controller starts with an delay of three seconds controlled due the launch file , because the signal_controller need the vehicle controller to start.  


## 6. Setup & Launch 
The Chapter structure and the discribtion is done by our own and the Code is done with help of Claude in hope that the configurations and start are possible.

### Prerequisites (assumed already installed)
- ROS2 Humble
- SUMO (incl. `sumo-gui`, TraCI), `SUMO_HOME` set
- colcon (`python3-colcon-common-extensions`)
- Python 3

### Submission structure
The zip contains a `src/` folder with two colcon packages:
- `tsp_digital_twin_msgs` (ament_cmake) – custom message definitions
- `tsp_digital_twin` (ament_python) – nodes, launch file, scenarios

(Two packages because ROS2 message generation via `rosidl` requires a CMake package; `tsp_digital_twin` depends on it.)

### Steps to run after unzipping

1. Unzip and place `src/` as the source folder of a new workspace:
   ```
   mkdir -p ~/tsp_ws
   mv src ~/tsp_ws/
   cd ~/tsp_ws
   ```

2. Build (order matters — `tsp_digital_twin` depends on `tsp_digital_twin_msgs`):
   ```
   colcon build --packages-select tsp_digital_twin_msgs tsp_digital_twin --symlink-install
   source install/setup.bash
   ```
   `--symlink-install` is required, otherwise `metrics_logger.py` cannot resolve its CSV output path correctly.

3. **Terminal 1** – start SUMO (pick a scenario):
   ```
   cd src/tsp_digital_twin/scenarios/s0_baseline/
   sumo-gui -c s0_baseline.sumocfg --remote-port 8813 --num-clients 2
   ```
   Do **not** press Play yet. (Other scenarios: `s1_simple/`, `s2_conflict/`, same command pattern.)

4. **Terminal 2** – start ROS2 system (from `~/tsp_ws`):
   ```
   source install/setup.bash
   ros2 launch tsp_digital_twin tsp_launch.py
   ```
   Wait for `VehiclePublisher connected to SUMO`, **then** press Play in SUMO-GUI.

5. **Terminal 3** – start dashboard (from `~/tsp_ws`):
   ```
   source install/setup.bash
   ros2 run tsp_digital_twin dashboard_node
   ```
   Read-only view of `/vehicle_status` and `/tls_command`; does not influence the control loop.

6. **Stopping:** `Ctrl+C` in Terminal 2 (not the SUMO-GUI stop button), otherwise `FatalTraCIError`.



## 7. Known Errors / Future Work

#### Known Errors

1. U-Turn at the end of some Roads which can possible enable the same priority vehicle to cross the Junction again.
2. The Junction isnt build good. There are more Traffic signs compared to the real live provided by Open Street map


#### Future work

1. implement another way to read out data compared to the reason-String for an higher robustness. 
2. Clean of the Traffic signs to the real live state




## 8. Sources
SUMO Docu:
https://sumo.dlr.de/docs/index.html 
SUMO/TraCi Docu:
https://sumo.dlr.de/docs/TraCI.html
Multi client
https://sumo.dlr.de/docs/TraCI/Interfacing_TraCI_from_Python.html
Script from Prof. Dr. Marc-Rene Zofka

Ros2 Docu: 
https://docs.ros.org/en/humble/
Map:
https://www.openstreetmap.org/#map=19/49.122223/9.210379
