import time
import rclpy
from rclpy.node import Node
from tsp_digital_twin_msgs.msg import VehicleStatus
from tsp_digital_twin_msgs.msg import TLSCommand
from std_msgs.msg import Header

# Konstanten
JUNCTION_ID = 'joinedS_21740280_cluster_1561711296_1561762947_1561762973_21740279_#7more'
PRIORITY_TYPES = ('bus_type', 'emergency_type')
ETA_THRESHOLD = 8             # Sekunden — erst TSP wenn so nah
GREEN_PHASE_DURATION = 15.0   # wie lange Grün gehalten wird
TSP_ENABLED = False            # TSP AN/AUS
RELEASE_TIMEOUT = 2.0         # Sekunden — nach Ablauf wird TSP wieder freigegeben

# GREEN_PHASE_INDEX wird nicht mehr fest vorgegeben — signal_controller
# berechnet die passende Phase selbst anhand von approach_edge.
class TSPController(Node):
    def __init__(self):
        super().__init__('tsp_controller')

        self.subscriber_ = self.create_subscription(
            VehicleStatus, '/vehicle_status',
            self.vehicle_status_callback, 10
        )
        self.publisher_ = self.create_publisher(TLSCommand, '/tls_command', 10)

        # Tracking: welche Fahrzeuge haben aktuell TSP aktiviert?
        self.active_tsp = {}

        # Welches Fahrzeug hält gerade die Priorität?
        self.current_priority_vehicle = None
        self.current_priority_type = None  # 'bus_type' oder 'emergency_type'

        # Letzte VehicleStatus-Meldung pro Fahrzeug (auch nach Aktivierung
        # weiter aktualisiert), damit wir erkennen wann die Kreuzung
        # verlassen wurde und die Prioritaet wieder freigegeben werden kann.
        self.last_seen = {}

        # Letzte vollständige VehicleStatus-Msg pro Fahrzeug — wird gebraucht,
        # um verdrängten Bussen nach Emergency-Release wieder TSP zu geben.
        self.last_msg = {}

        # Durch Emergency verdrängte Fahrzeuge merken
        self.preempted_vehicles = {}  # vehicle_id -> letzte VehicleStatus msg

        self.release_timer = self.create_timer(0.5, self.check_release)

    def vehicle_status_callback(self, msg):
        # TSP-Schalter — bei False werden alle Fahrzeug-Meldungen ignoriert
        if not TSP_ENABLED:
            return

        # Nur unsere Kreuzung beachten
        if msg.intersection_id != JUNCTION_ID:
            return

        # Nur Priority-Fahrzeuge interessieren uns
        if msg.vehicle_type not in PRIORITY_TYPES:
            return

        self.last_seen[msg.vehicle_id] = time.time()
        self.last_msg[msg.vehicle_id] = msg

        # Fahrzeug bereits aktiv? Updates möglich aber kein Neu-Schalten
        if msg.vehicle_id in self.active_tsp:
            return

        # ETA-Filter: zu weit weg, noch warten
        if msg.eta_seconds > ETA_THRESHOLD:
            return

        # Priorisierungsregel: Emergency darf Bus überstimmen
        if self.current_priority_vehicle is not None:
            # Schon was aktiv
            if self.current_priority_type == 'emergency_type':
                # Emergency darf nicht überstimmt werden
                return
            if msg.vehicle_type != 'emergency_type':
                # Aktuell ist Bus aktiv, neues ist auch nur Bus → nicht überstimmen
                return

            # Aktuell ist Bus, neues ist Emergency → überstimmen!
            # Verdrängten Bus merken, damit er nach Emergency-Release sofort
            # wieder Vorrang bekommt (sonst steht er an Rot fest, ETA → ∞)
            preempted_id = self.current_priority_vehicle
            self.preempted_vehicles[preempted_id] = self.last_msg.get(preempted_id)

            # Der verdrängte Bus muss aus active_tsp raus, sonst bekommt er
            # nach Freigabe der Prioritaet nie wieder ein TSP-Kommando
            if preempted_id in self.active_tsp:
                del self.active_tsp[preempted_id]

            self.get_logger().warn(
                f'EMERGENCY OVERRIDE: {msg.vehicle_id} verdrängt {preempted_id}'
            )

        # TSP aktivieren
        self.active_tsp[msg.vehicle_id] = msg.vehicle_type
        self.current_priority_vehicle = msg.vehicle_id
        self.current_priority_type = msg.vehicle_type

        # TLSCommand bauen — target_phase bleibt zur Information/Logging erhalten,
        # signal_controller berechnet die tatsächliche Phase selbst aus approach_edge.
        cmd = TLSCommand()
        cmd.header = Header()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.tls_id = JUNCTION_ID
        cmd.target_phase = -1  # signal_controller ermittelt die Phase dynamisch
        cmd.duration_sec = GREEN_PHASE_DURATION
        cmd.reason = f'TSP for {msg.vehicle_id} ({msg.vehicle_type})'
        cmd.approach_edge = msg.approach_edge

        self.publisher_.publish(cmd)
        self.get_logger().info(
            f'TSP AKTIVIERT: {msg.vehicle_id} ({msg.vehicle_type}) '
            f'eta={msg.eta_seconds:.1f}s edge={msg.approach_edge}'
        )

    def force_tsp(self, msg):
        """Triggert TSP für ein Fahrzeug ohne ETA-Check.
        Wird nach Emergency-Release benutzt, damit ein verdrängter Bus,
        der an der roten Ampel steht (ETA → ∞), trotzdem wieder Grün bekommt."""
        self.active_tsp[msg.vehicle_id] = msg.vehicle_type
        self.current_priority_vehicle = msg.vehicle_id
        self.current_priority_type = msg.vehicle_type

        cmd = TLSCommand()
        cmd.header = Header()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.tls_id = JUNCTION_ID
        cmd.target_phase = -1
        cmd.duration_sec = GREEN_PHASE_DURATION
        cmd.reason = f'TSP RESUMED for {msg.vehicle_id} after emergency'
        cmd.approach_edge = msg.approach_edge
        self.publisher_.publish(cmd)

        self.get_logger().info(
            f'TSP WIEDER AKTIVIERT (nach Emergency): {msg.vehicle_id} '
            f'edge={msg.approach_edge}'
        )

    def check_release(self):
        """Gibt die aktuelle Prioritaet frei, wenn das priorisierte Fahrzeug
        seit RELEASE_TIMEOUT Sekunden keine VehicleStatus-Meldung mehr
        gesendet hat (= hat die Kreuzung verlassen)."""
        if self.current_priority_vehicle is None:
            return
        last = self.last_seen.get(self.current_priority_vehicle)
        if last is not None and (time.time() - last) <= RELEASE_TIMEOUT:
            return

        self.get_logger().info(
            f'TSP FREIGEGEBEN: {self.current_priority_vehicle} hat die Kreuzung verlassen'
        )

        released_type = self.current_priority_type
        self.current_priority_vehicle = None
        self.current_priority_type = None

        # War ein Emergency aktiv? Dann verdrängten Bus sofort wieder priorisieren
        if released_type == 'emergency_type' and self.preempted_vehicles:
            for veh_id, last_msg in list(self.preempted_vehicles.items()):
                if last_msg is None:
                    continue
                # Nur Busse, die noch aktiv in der Sim sind (kürzlich gesehen)
                last_ts = self.last_seen.get(veh_id, 0)
                if (time.time() - last_ts) > RELEASE_TIMEOUT:
                    continue  # Bus schon weg
                # Sofort TSP triggern, unabhängig von ETA
                self.force_tsp(last_msg)
                break  # nur einen Bus auf einmal priorisieren
            self.preempted_vehicles.clear()


def main(args=None):
    rclpy.init(args=args)
    tsp_controller = TSPController()
    try:
        rclpy.spin(tsp_controller)
    except KeyboardInterrupt:
        pass
    finally:
        tsp_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()