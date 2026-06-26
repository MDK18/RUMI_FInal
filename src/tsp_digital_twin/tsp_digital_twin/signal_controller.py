import rclpy
from rclpy.node import Node
from tsp_digital_twin_msgs.msg import TLSCommand
import traci
# Konstanten
TLS_ID = 'joinedS_21740280_cluster_1561711296_1561762947_1561762973_21740279_#7more'
TRACI_HOST = 'localhost'
TRACI_PORT = 8813
class SignalController(Node):
    def __init__(self):
        super().__init__('signal_controller')
        
        # Subscriber auf TLS-Befehle vom tsp_controller
        self.subscriber_ = self.create_subscription(
            TLSCommand,
            '/tls_command',
            self.tls_command_callback,
            10
        )
        
        # Falls eine alte Verbindung mit dem Label noch existiert, schließen
        try:
            traci.switch('signal')
            traci.close()
            self.get_logger().warn('Closed stale signal connection')
        except Exception:
            pass
        
        try:
            traci.init(port=TRACI_PORT, host=TRACI_HOST, label='signal')
            traci.setOrder(2)
            self.get_logger().info('SignalController connected to SUMO via TraCI')
        except Exception as e:
            self.get_logger().error(f'TraCI connection failed: {e}')
            raise

        # Edge->Link-Indizes und Phasen einmalig laden und Mapping aufbauen
        self.edge_to_links = {}   # edge_id 
        self.phases = []          # Liste von Signalzustand-Strings, Index = Phasenindex
        try:
            links = traci.trafficlight.getControlledLinks(TLS_ID)
            self.get_logger().info(f'DEBUG: Anzahl Links = {len(links)}')
            for i, link_group in enumerate(links):
            
                for (from_lane, to_lane, via_lane) in link_group:
                    # Lane-ID -> Edge-ID: alles vor dem letzten '_' abschneiden
                    from_edge = from_lane.rsplit('_', 1)[0]
                    self.edge_to_links.setdefault(from_edge, []).append(i)

            logic = traci.trafficlight.getAllProgramLogics(TLS_ID)
            self.get_logger().info(f'DEBUG: Anzahl Programme = {len(logic)}')
            if logic:
                self.phases = [phase.state for phase in logic[0].phases]
                for phase_idx, state in enumerate(self.phases):
                    self.get_logger().info(f'Phase {phase_idx}: state={state}')

            self.get_logger().info(
                f'DEBUG: Edge-Mapping aufgebaut, {len(self.edge_to_links)} Edges bekannt'
            )
        except Exception as e:
            self.get_logger().error(f'DEBUG TraCI-Abfrage fehlgeschlagen: {e}')

        # Sync-Timer: signalisiert SUMO regelmäßig dass Order 2 bereit ist
        self.sync_timer = self.create_timer(0.1, self.sync_callback)

    def find_best_phase(self, approach_edge):
        """Ermittelt die Phase, die für die gegebene Anfahrtsedge das meiste
        bevorrechtigte Grün ('G') bietet. Fällt auf nachrangiges Grün ('g')
        zurück, falls keine Phase echtes Vorfahrts-Grün hat.
        Gibt None zurück, falls die Edge unbekannt ist oder keine Phase passt.
        """
        link_indices = self.edge_to_links.get(approach_edge)
        if not link_indices:
            self.get_logger().warn(f'Keine Links bekannt für Edge: {approach_edge}')
            return None

        best_phase = None
        best_score = -1
        for phase_idx, state in enumerate(self.phases):
            score = 0
            for link_idx in link_indices:
                if link_idx >= len(state):
                    continue
                ch = state[link_idx]
                if ch == 'G':
                    score += 2
                elif ch == 'g':
                    score += 1
            if score > best_score:
                best_score = score
                best_phase = phase_idx

        if best_score <= 0:
            self.get_logger().warn(
                f'Keine Phase mit Grün fuer Edge {approach_edge} gefunden'
            )
            return None

        self.get_logger().info(
            f'Edge {approach_edge} -> Links {link_indices} -> beste Phase {best_phase} (score={best_score})'
        )
        return best_phase

    def sync_callback(self):
        """Bei Multi-Client TraCI muss jeder Client periodisch steppen
        damit SUMO die Simulation voranschreiten lässt."""
        try:
            traci.switch('signal')
            traci.simulationStep()
        except Exception:
            pass
    def tls_command_callback(self, msg):
        # Nur Befehle für unsere Kreuzung verarbeiten
        if msg.tls_id != TLS_ID:
            self.get_logger().warn(f'Ignoring command for unknown TLS: {msg.tls_id}')
            return

        # Phase dynamisch anhand der Anfahrtsedge bestimmen
        target_phase = self.find_best_phase(msg.approach_edge)
        if target_phase is None:
            self.get_logger().error(
                f'Konnte keine passende Phase fuer Edge {msg.approach_edge} bestimmen, ueberspringe Schaltung'
            )
            return

        # Auf unsere TraCI-Verbindung umschalten und Phase setzen
        try:
            traci.switch('signal')
            traci.trafficlight.setPhase(msg.tls_id, target_phase)
            traci.trafficlight.setPhaseDuration(msg.tls_id, msg.duration_sec)
            self.get_logger().info(
                f'AMPEL GESCHALTET: phase={target_phase} duration={msg.duration_sec}s edge={msg.approach_edge}'
            )
            
        except traci.TraCIException as e:
            self.get_logger().error(f'Failed to set TLS phase: {e}')
    def destroy_node(self):
        try:
            traci.switch('signal')
            traci.close()
        except Exception:
            pass
        super().destroy_node()
def main(args=None):
    rclpy.init(args=args)
    sig_controller = SignalController()
    try:
        rclpy.spin(sig_controller)
    except KeyboardInterrupt:
        pass
    finally:
        sig_controller.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()