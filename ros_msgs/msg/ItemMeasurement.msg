# Contract perception -> classifier. Owner: P1; changes only via PR (docs/decisions.md).
# Perception measures geometry; category is NOT here — it is the classifier's output
# (see ItemClassification.msg). Units: dims — millimeters, position — meters (world frame).

std_msgs/Header header        # stamp = measurement time, frame_id = camera frame
uint32 item_id                # tracking id, unique within a simulation run
float32[3] dims_mm            # linear dimensions sorted descending, millimeters
float32 k                     # r_inscribed / R_circumscribed of best section, [0..1]
float32 confidence            # measurement confidence [0..1]
geometry_msgs/Point position  # item position at header.stamp, meters, world frame
