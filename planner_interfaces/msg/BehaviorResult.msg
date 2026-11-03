uint8 REJECTED=0
uint8 RUNNING=1
uint8 PAUSED=2
uint8 SUCCEEDED=3
uint8 FAILED=4
uint8 CANCELED=5

string behavior_type
string task_id
uint8 overall_status
string failure_reason
uav_planner_interfaces/VehicleResult[] vehicle_results
