# Contract CV -> controller. Owner: P1; changes only via PR (docs/decisions.md).
# Units: dims — millimeters, position — meters (world frame), time — header.stamp.

std_msgs/Header header        # stamp = measurement time, frame_id = camera frame
uint32 item_id                # tracking id, unique within a simulation run
string category               # "B" | "C" | "D" — values from src/constants.py
float32[3] dims_mm            # linear dimensions sorted descending, millimeters
float32 k                     # r_inscribed / R_circumscribed of best section, [0..1]
float32 confidence            # [0..1] aggregate frame agreement; diagnostic (overlay/log).
# Routing uses the median dims/K over frames + a size margin to C
# (classify_conservative), NOT a confidence threshold — this field does not gate.
geometry_msgs/Point position  # item position at header.stamp, meters, world frame
