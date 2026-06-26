# Befehl an den signal_controller die Ampel umzuschalten
# Wird von tsp_controller gesendet, von signal_controller empfangen

std_msgs/Header header
string tls_id
int32 target_phase
float32 duration_sec
string reason
string approach_edge # wird nenötigt um Ampel passend zu schalten