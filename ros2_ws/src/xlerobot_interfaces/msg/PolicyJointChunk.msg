# Session-scoped, row-major joint setpoints from an untrusted policy backend.
string session_id
string source_id
uint64 sequence
builtin_interfaces/Time generated_at
builtin_interfaces/Duration sample_period
bool final_chunk
string[] joint_names
uint32 sample_count
float64[] positions
