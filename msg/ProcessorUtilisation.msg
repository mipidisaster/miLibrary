# Array to contain the utilisation (percentage) of each processor core within PC/device
###################################################################################################

std_msgs/Header header

float32     overall
float32[]   cores

float32     temperature

uint8       fault               # Returned fault state