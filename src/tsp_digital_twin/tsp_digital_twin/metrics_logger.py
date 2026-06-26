import csv
import os
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from tsp_digital_twin_msgs.msg import VehicleStatus
from tsp_digital_twin_msgs.msg import TLSCommand

TLS_ID = 'joinedS_21740280_cluster_1561711296_1561762947_1561762973_21740279_#7more'
METRICS_DIR = os.path.normpath(
    os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', 'metrics')
)
CSV_FILENAME = 'latest_run.csv'
PRIORITY_TYPES = {'bus_type', 'emergency_type'}

# Timeout: wenn 2.0s lang keine VehicleStatus mehr -> Fahrzeug hat Kreuzung verlassen
PASSED_TIMEOUT_SEC = 2.0
# Prüfintervall für PASSED-Erkennung
CHECK_INTERVAL_SEC = 0.5
# Geschwindigkeit unter der ein Fahrzeug als "wartend" zählt
WAITING_SPEED_THRESHOLD = 0.5

CSV_HEADER = [
    'timestamp',
    'event',
    'vehicle_id',
    'vehicle_type',
    'approach_edge',
    'eta_seconds',
    'speed',
    'waiting_time_sec',
    'duration_sec',
    'reason',
]


class MetricsLogger(Node):
    def __init__(self):
        super().__init__('metrics_logger')

        # CSV-Datei vorbereiten
        os.makedirs(METRICS_DIR, exist_ok=True)
        csv_path = os.path.join(METRICS_DIR, CSV_FILENAME)
        self.csv_file = open(csv_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(CSV_HEADER)
        self.csv_file.flush()
        self.get_logger().info(f'CSV-Logger gestartet: {csv_path}')

        # Subscriptions
        self.create_subscription(
            VehicleStatus, '/vehicle_status', self.vehicle_status_callback, 10
        )
        self.create_subscription(
            TLSCommand, '/tls_command', self.tls_command_callback, 10
        )

        # Tracking pro Fahrzeug
       
        self.tracked = {}

        # Periodischer Check für PASSED-Events
        self.check_timer = self.create_timer(CHECK_INTERVAL_SEC, self.check_passed)

    def vehicle_status_callback(self, msg):
        # Nur unsere Kreuzung
        if msg.intersection_id != TLS_ID:
            return
        # Nur priorisierte Fahrzeugtypen
        if msg.vehicle_type not in PRIORITY_TYPES:
            return

        now = time.time()
        vid = msg.vehicle_id

        if vid not in self.tracked:
            # Erste Sichtung
            self.tracked[vid] = {
                'vehicle_type': msg.vehicle_type,
                'approach_edge': msg.approach_edge,
                'first_seen': now,
                'last_seen': now,
                'last_speed': msg.speed,
                'waiting_time': 0.0,
                'last_eta': msg.eta_seconds,
            }
            self.write_event(
                event='DETECTED',
                vehicle_id=vid,
                vehicle_type=msg.vehicle_type,
                approach_edge=msg.approach_edge,
                eta_seconds=msg.eta_seconds,
                speed=msg.speed,
            )
            self.get_logger().info(f'DETECTED: {vid} ({msg.vehicle_type})')
        else:
            # Update + Wartezeit hochzählen wenn Fahrzeug langsamer als Threshold
            t = self.tracked[vid]
            dt = now - t['last_seen']
            if msg.speed < WAITING_SPEED_THRESHOLD:
                t['waiting_time'] += dt
            t['last_seen'] = now
            t['last_speed'] = msg.speed
            t['last_eta'] = msg.eta_seconds
            # approach_edge kann sich während der Fahrt aktualisieren
            t['approach_edge'] = msg.approach_edge

    def tls_command_callback(self, msg):
        if msg.tls_id != TLS_ID:
            return
        # Vehicle_id und vehicle_type aus reason extrahieren
        reason = msg.reason
        vid = ''
        vehicle_type = ''
        if 'for ' in reason and '(' in reason:
            try:
                vid = reason.split('for ')[1].split(' (')[0]
                vehicle_type = reason.split('(')[1].rstrip(')')
            except Exception:
                pass

        # Normales TSP_ACTIVATED-Event
        self.write_event(
            event='TSP_ACTIVATED',
            vehicle_id=vid,
            vehicle_type=vehicle_type,
            approach_edge=msg.approach_edge,
            duration_sec=msg.duration_sec,
            reason=reason,
        )
        self.get_logger().info(f'TSP_ACTIVATED logged: {vid}')

        # Wenn das Fahrzeug emergency_type ist und Bus warten,
        #       zusätzliches Event "EMERGENCY_OVER_BUS" loggen
        if vehicle_type == 'emergency_type':
            # Sammle alle aktuell getrackten Busse
            bus_ids = [
                veh_id for veh_id, data in self.tracked.items()
                if data['vehicle_type'] == 'bus_type'
            ]
            if bus_ids:
                self.write_event(
                    event='EMERGENCY_OVER_BUS',
                    vehicle_id=vid,
                    vehicle_type=vehicle_type,
                    approach_edge=msg.approach_edge,
                    duration_sec=msg.duration_sec,
                    reason=f"over bus {', '.join(bus_ids)}",
                )
                self.get_logger().info(
                    f'EMERGENCY_OVER_BUS: {vid} over bus(es) {bus_ids}'
                )

    def check_passed(self):
        """Prüft welche Fahrzeuge länger als PASSED_TIMEOUT_SEC nichts mehr gesendet
        haben — die haben die Kreuzung verlassen."""
        now = time.time()
        to_remove = []
        for vid, t in self.tracked.items():
            if (now - t['last_seen']) > PASSED_TIMEOUT_SEC:
                self.write_event(
                    event='PASSED',
                    vehicle_id=vid,
                    vehicle_type=t['vehicle_type'],
                    approach_edge=t['approach_edge'],
                    speed=t['last_speed'],
                    waiting_time_sec=t['waiting_time'],
                )
                self.get_logger().info(
                    f'PASSED: {vid} (Wartezeit: {t["waiting_time"]:.1f}s)'
                )
                to_remove.append(vid)
        for vid in to_remove:
            del self.tracked[vid]

    def write_event(self, event, vehicle_id='', vehicle_type='', approach_edge='',
                    eta_seconds='', speed='', waiting_time_sec='',
                    duration_sec='', reason=''):
        timestamp = datetime.now().isoformat(timespec='milliseconds')
        self.csv_writer.writerow([
            timestamp,
            event,
            vehicle_id,
            vehicle_type,
            approach_edge,
            f'{eta_seconds:.2f}' if isinstance(eta_seconds, float) else eta_seconds,
            f'{speed:.2f}' if isinstance(speed, float) else speed,
            f'{waiting_time_sec:.2f}' if isinstance(waiting_time_sec, float) else waiting_time_sec,
            f'{duration_sec:.2f}' if isinstance(duration_sec, float) else duration_sec,
            reason,
        ])
        self.csv_file.flush()  # sofort schreiben damit nix verloren geht bei Crash

    def destroy_node(self):
        try:
            # Noch nicht abgemeldete Fahrzeuge als PASSED markieren
            for vid, t in self.tracked.items():
                self.write_event(
                    event='PASSED',
                    vehicle_id=vid,
                    vehicle_type=t['vehicle_type'],
                    approach_edge=t['approach_edge'],
                    speed=t['last_speed'],
                    waiting_time_sec=t['waiting_time'],
                )
            self.csv_file.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    metrics_logger = MetricsLogger()
    try:
        rclpy.spin(metrics_logger)
    except KeyboardInterrupt:
        pass
    finally:
        metrics_logger.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()