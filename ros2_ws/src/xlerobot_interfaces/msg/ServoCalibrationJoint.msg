# One joint in an in-progress, group-based servo calibration session.
# Raw values use encoder ticks; -1 means not yet measured.
string name
uint8 servo_id
int32 position
int32 zero
int32 reference_zero
string zero_source
int32 raw_min
int32 raw_max
float64 coverage
bool zero_captured
bool range_captured
bool online
string message
