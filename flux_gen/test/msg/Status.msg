# Constants are named symbols beside the adapter, not wire data.
uint16 ERROR_NONE = 0
uint16 ERROR_GNSS = 3
float32 EPS = 1e-6
string TAG = 'unit=m/s'   # quoted like rosidl: unwrapped
string CUT = 'a#b'   # rosidl ends the line at the first '#', inside quotes too
string RAW = bare value   # unquoted: taken verbatim, comment stripped
bool ON = true
uint16 code
float64[] values
