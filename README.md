# Accelerated RRT* By Local Directional Visibility
RRT* is an efficient sampling-based motion planning algorithm. However, without taking advantages of accessible environment information, sampling-based algorithms usually result in sampling failures, generate useless nodes, and/or fail in exploring narrow passages.

For this project, in order to better utilize environment information and further improve searching efficiency, we proposed a novel approach to improve RRT* by
1. quantifying local knowledge of the obstacle configurations during neighbour rewiring in terms of directional visibility
2. collecting environment information during searching, and
3. changing the sampling strategy biasing toward near-obstacle nodes after the first solution found.

The proposed algorithm RRT* by Local Directional Visibility (RRT*-LDV) better utilizes local known information and innovates a weighted sampling strategy. The accelerated RRT*-LDV outperforms RRT* in convergence rate and success rate of finding narrow passages. A high Degree-Of-Freedom scenario is also experimented.
