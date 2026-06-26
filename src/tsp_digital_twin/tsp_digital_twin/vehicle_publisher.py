import rclpy
from rclpy.node import Node
from tsp_digital_twin_msgs.msg import VehicleStatus
from std_msgs.msg import Header
from geometry_msgs.msg import Point
import traci
from traci import constants as tc
import math
import sys
import os
# essential Variables
TRACI_HOST = 'localhost'
TRACI_PORT = 8813
JUNCTION_ID = 'joinedS_21740280_cluster_1561711296_1561762947_1561762973_21740279_#7more'
JUNCTION_X = 249.33
JUNCTION_Y = 247.03
JUNCTION_RADIUS = 200.0
#


RELEVANT_TYPES = ('bus_type', 'emergency_type')
SIM_STEP = 0.1
if 'SUMO_HOME' in os.environ:
    sys.path.append(os.path.join(os.environ['SUMO_HOME'], 'tools'))
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

class VehiclePublisher(Node):
    def __init__(self):
        super().__init__('vehicle_publisher')
        # Publisher 
        self.publisher_ = self.create_publisher(VehicleStatus, '/vehicle_status', 10)
        
        self.get_logger().info('Connecting to SUMO via TraCI...')
        
        # TraCI-Verbindung als erster Client 
        traci.init(port=TRACI_PORT, host=TRACI_HOST, label='vehicle')
        traci.switch('vehicle')
        traci.setOrder(1)
        self.get_logger().info('VehiclePublisher connected to SUMO, order set to 1')
        # Initialer Step damit SUMO mit der Synchronisation startet
        traci.simulationStep()
        self.get_logger().info('First simulation step done')

        # Menge der Edges, die direkt von unserer Ziel-TLS kontrolliert werden.
        # Wird genutzt um in der Restroute eines Fahrzeugs die nächste
        # TLS-bekannte Anfahrtsedge zu finden (statt der aktuellen Position,
        # die bei weit entfernten Fahrzeugen noch nicht TLS-bekannt ist).
        self.tls_edges = set()
        try:
            links = traci.trafficlight.getControlledLinks(JUNCTION_ID)
            for link_group in links:
                for (from_lane, to_lane, via_lane) in link_group:
                    from_edge = from_lane.rsplit('_', 1)[0]
                    self.tls_edges.add(from_edge)
            self.get_logger().info(f'DEBUG: {len(self.tls_edges)} TLS-bekannte Edges ermittelt')
        except Exception as e:
            self.get_logger().error(f'Konnte TLS-Edges nicht ermitteln: {e}')

        # Timer für Simulation Steps
        self.timer = self.create_timer(SIM_STEP, self.step_callback)
        self.get_logger().info('Timer started')
        self.active_vehicles = set()  # Set um bereits veröffentlichte Fahrzeuge zu tracken

    def get_approach_edge(self, vehicle_id, current_edge):
        """Liefert die erste Edge der Restroute, die der Ziel-TLS bekannt ist.
        Fällt auf current_edge zurueck, falls keine Restroute-Edge bekannt ist
        (z.B. Fahrzeug faehrt schon direkt auf der TLS-Edge)."""
        try:
            route = traci.vehicle.getRoute(vehicle_id)
            route_idx = traci.vehicle.getRouteIndex(vehicle_id)
            if route_idx < 0:
                return current_edge
            for edge in route[route_idx:]:
                if edge in self.tls_edges:
                    return edge
        except Exception:
            pass
        return current_edge

    def step_callback(self):
        traci.simulationStep()
        vehicle_ids = traci.vehicle.getIDList()
        for vehicle_id in vehicle_ids:
            x, y = traci.vehicle.getPosition(vehicle_id)
            speed = traci.vehicle.getSpeed(vehicle_id)
            vehicle_type = traci.vehicle.getTypeID(vehicle_id)
            if vehicle_type not in RELEVANT_TYPES:
                continue
            distance = math.sqrt((x - JUNCTION_X)**2 + (y - JUNCTION_Y)**2)
            if distance <= JUNCTION_RADIUS:
                if vehicle_id not in self.active_vehicles:
                    self.active_vehicles.add(vehicle_id)
                    self.get_logger().info(f'ANMELDUNG: {vehicle_id} ({vehicle_type})')
                    # DEBUG: einmalig Route und aktuelle Edge ausgeben
                    try:
                        route = traci.vehicle.getRoute(vehicle_id)
                        current_edge = traci.vehicle.getRoadID(vehicle_id)
                        self.get_logger().info(f'DEBUG ROUTE {vehicle_id}: {route}')
                        self.get_logger().info(f'DEBUG CURRENT EDGE {vehicle_id}: {current_edge}')
                    except Exception as e:
                        self.get_logger().error(f'DEBUG ROUTE fehlgeschlagen: {e}')

                # Aktuelle Position-Edge und daraus die nächste TLS-bekannte
                # Anfahrtsedge aus der Restroute ermitteln.
                try:
                    current_edge = traci.vehicle.getRoadID(vehicle_id)
                except Exception:
                    current_edge = ''
                approach_edge = self.get_approach_edge(vehicle_id, current_edge)

                if speed > 0.5:
                    eta_seconds = distance / speed
                else:
                    eta_seconds = 9999.0
                msg = VehicleStatus()
                msg.header = Header()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.vehicle_id = vehicle_id
                msg.vehicle_type = vehicle_type
                msg.position = Point(x=x, y=y, z=0.0)
                msg.speed = speed
                msg.eta_seconds = eta_seconds
                msg.intersection_id = JUNCTION_ID
                msg.approach_edge = approach_edge
                self.publisher_.publish(msg)
        abgemeldet = self.active_vehicles - set(vehicle_ids)
        for vid in abgemeldet:
            self.get_logger().info(f'ABMELDUNG: {vid}')
        self.active_vehicles -= abgemeldet
    def destroy_node(self):
        traci.close()
        super().destroy_node()
    
def main(args=None):
    rclpy.init(args=args)
    vehicle_pub = VehiclePublisher()
    try:
        rclpy.spin(vehicle_pub)
    except KeyboardInterrupt:
        pass
    finally:
        vehicle_pub.destroy_node()
        rclpy.shutdown()
    
    
if __name__ == '__main__':
    main()