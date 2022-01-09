# Captures the bytes transmitted via the hardware. Detailing the read/write transmission rates
#
# To accommodate multiple devices on the same bus, the layout of the arrays below will be the 
# following:
# [0]   = Total bytes communicated to ALL devices
# [1]   = Total bytes communicated to device 1
# [2]   = Total bytes communicated to device 2
#  .
#  .
# [n]   = Total bytes communicated to device n
###################################################################################################

std_msgs/Header header

uint64[]    received_bytes
float32[]   received_rates

uint64[]    sent_bytes
float32[]   sent_rates