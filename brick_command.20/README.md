# brick_command

This ROS package acts as a command for Brick Structures. The Launch file takes an argument for a brick structure blueprint which is explained below

Blueprint Format - yaml
Reference Pose: The Pose of the bottom left corener of the first brick to be placed, only x, y and yaw can be changed
Brick Spacing: Space between adjacent bricks - will most likely be 0
Layers: Next is a yaml list of layers for each layer there is an offset and list of bricks for the first layer the offset should be zero
Offset: Offset is the distance offset from the left most brick of the previous layers
Bricks: Each Brick has a color and sequence
Sequence: Sequence is the order in which the bricks are meant to be placed
Color: Color of the bricks
The order of the bricks in each layer are of there geometric positions not Sequence

The Launch file then starts both brick_command_node and brick_param_node
brick_param_node is a simple node that puts the brick lengths to the ROS param
brick_command_node will make a BrickStructure object using the blueprint and then adverstises the service request_brick_command.
request_brick_command: requst_brick_command returns the next brick to be constructed, it returns the color and pose of the brick. It also returns the color and pose of bricks below it and adjacent to it. It takes an input argument increment which will increment the brick count if set to true, it will also return a bool which indicates whether the structure is complete.
