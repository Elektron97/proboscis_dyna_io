# proboscis_dyna_io
[UNDER DEVELOPMENT] This repository contains the ros pkg for interfacing with the 7 Dynamixels in the PROBOSCIS Prototype.

## Motors' Drive Mode
The motors work under the *Extended Position Mode*, that allows to do multiple turns. This is the best drive mode to control the tendon-driven robots, especially for safety reasons.

## Topics
- Output `/proboscis/read_currents`. This topic publishes a `std_msgs/Float32MultiArray` of the 7 currents read from the Dynamixel Motors.
- Input `/proboscis/cmd_turns`. The node subscribes a `std_msgs/Float32MultiArray` that indicates the number of turns commanded to the Motors.

## Torque-Current Curve
$$
i(t) = p_2 \,  \tau^2(t) + p_1 \,  \tau(t) + p_0
$$
where:
$$
p_2 = 0.2030 \quad p_1 = 0.5753 \quad p_0 = 0.1327
$$