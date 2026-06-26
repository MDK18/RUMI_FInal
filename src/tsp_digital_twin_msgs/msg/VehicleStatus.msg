# Status eines Fahrzeugs im Beobachtungsradius
# Wird von vehicle_publisher gesendet, von tsp_controller empfangen

std_msgs/Header header
string vehicle_id
string vehicle_type
geometry_msgs/Point position
float32 speed
float32 eta_seconds
string intersection_id
string approach_edge # wird nenötigt um Ampel passend zu schalten