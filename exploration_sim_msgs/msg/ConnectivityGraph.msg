
# This contains the position vectors of the nodes in the graph in the global frame
exploration_sim_msgs/Point2d[] nodes

# This contains the edges of the graph as a connectivity matrix
uint32 rows # stores the length of the row dimension (not guaranteed square)
uint32 num_edges # total number of edges in the graph
exploration_sim_msgs/Edge[] edges