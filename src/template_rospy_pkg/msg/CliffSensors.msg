Header header

# Cliff sensors in order from left to right
# Value = true if a cliff detected
bool is_cliff_left
bool is_cliff_front_left
bool is_cliff_front_right
bool is_cliff_right

# Cliff sensor raw value, 0 to 4095
uint16 cliff_signal_left
uint16 cliff_signal_front_left
uint16 cliff_signal_front_right
uint16 cliff_signal_right