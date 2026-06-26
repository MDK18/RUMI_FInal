import time
from collections import OrderedDict

import rclpy
from rclpy.node import Node
from tsp_digital_twin_msgs.msg import VehicleStatus, TLSCommand

# essential Variables
STALE_TIMEOUT = 5.0      # Sekunden ohne Update bis Fahrzeug als weg gilt
REFRESH_INTERVAL = 1.0   # Sekunden zwischen zwei Redraws
CLEAR_SCREEN = '\x1b[2J\x1b[H'



class DashboardNode(Node):
    def __init__(self):
        super().__init__('dashboard_node')

        self.vehicles = OrderedDict()   # vehicle_id -> letzter bekannter Status
        self.last_command = None        # letzter TLSCommand, oder None falls noch keiner da

        # Subscriber 
        self.vehicle_sub_ = self.create_subscription(VehicleStatus, '/vehicle_status', self.vehicle_callback, 10)
        self.tls_sub_ = self.create_subscription(TLSCommand, '/tls_command', self.command_callback, 10)

        self.timer = self.create_timer(REFRESH_INTERVAL, self.render)
        self.get_logger().info('Dashboard gestartet, warte auf Daten...')

    def vehicle_callback(self, msg):
        self.vehicles[msg.vehicle_id] = {
            'vehicle_type': msg.vehicle_type,
            'eta_seconds': msg.eta_seconds,
            'speed': msg.speed,
            'approach_edge': msg.approach_edge,
            'intersection_id': msg.intersection_id,
            'last_seen': time.time(),
        }

    def command_callback(self, msg):
        self.last_command = {
            'tls_id': msg.tls_id,
            'reason': msg.reason,
            'approach_edge': msg.approach_edge,
            'duration_sec': msg.duration_sec,
            'received_at': time.time(),
        }

    def render(self):
        now = time.time()

        # Fahrzeuge entfernen die länger nicht mehr gemeldet haben
       
        abgemeldet = [vid for vid, v in self.vehicles.items() if now - v['last_seen'] > STALE_TIMEOUT]
        for vid in abgemeldet:
            del self.vehicles[vid]

        lines = [CLEAR_SCREEN]
        lines.append(f"TSP DASHBOARD   {time.strftime('%H:%M:%S')}")
        lines.append('-' * 64)

        lines.append('ACTIVE VEHICLES')
        if not self.vehicles:
            lines.append('  (keine im Beobachtungsradius)')
        else:
            lines.append(f"  {'id':<14}{'type':<12}{'approach':<16}{'eta':<8}{'speed':<6}")
            for vid, v in self.vehicles.items():
                eta = f"{v['eta_seconds']:.1f}s"
                speed = f"{v['speed']:.1f}"
                lines.append(f"  {vid:<14}{v['vehicle_type']:<12}{v['approach_edge']:<16}{eta:<8}{speed:<6}")

        lines.append('')
        lines.append('LAST TSP REQUEST')
        if self.last_command is None:
            lines.append('  (noch keiner empfangen)')
        else:
            c = self.last_command
            age = now - c['received_at']
            lines.append(f"  tls_id:       {c['tls_id']}")
            lines.append(f"  reason:       {c['reason']}")
            lines.append(f"  approach:     {c['approach_edge']}")
            lines.append(f"  duration_sec: {c['duration_sec']:.1f}")
            lines.append(f"  age:          {age:.1f}s")

        lines.append('-' * 64)
        lines.append('refresh: 1.0s   source: /vehicle_status, /tls_command')

        print('\n'.join(lines), flush=True)


def main(args=None):
    rclpy.init(args=args)
    dashboard = DashboardNode()
    try:
        rclpy.spin(dashboard)
    except KeyboardInterrupt:
        pass
    finally:
        dashboard.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()