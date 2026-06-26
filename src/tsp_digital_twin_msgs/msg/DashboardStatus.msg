# Status-Update für Dashboard und KPI-Logging
# Wird von tsp_controller gesendet, von metrics_logger empfangen

std_msgs/Header header
string vehicle_id
string tls_id
string tls_state
float32 waiting_time_sec
bool priority_active