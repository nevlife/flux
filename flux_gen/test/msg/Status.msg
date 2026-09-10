# Constants are named symbols beside the adapter, not wire data.
uint16 ERROR_NONE = 0
uint16 ERROR_GNSS = 3
float32 EPS = 1e-6
string TAG = 'unit=m/s#kept'   # quoted like rosidl: unwrapped, '#' inside stays
string RAW = bare value   # unquoted: taken verbatim, comment stripped
bool ON = true
uint16 code
float64[] values
