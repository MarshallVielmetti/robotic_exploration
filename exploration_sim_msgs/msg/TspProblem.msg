
# message header
std_msgs/Header header

# The nodes in the graph in the global frame
geometry_msgs/Pose[] nodes

# The weights of the edges in the graph - nxn (n = number of nodes)
float64[] weights