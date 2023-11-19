<<<<<<< HEAD
# proboscis_dyna_io
This repository contains the ros pkg for interfacing with the 7 Dynamixels in the PROBOSCIS Prototype
=======
# PROBOSCIS ROS Software Utils
Hi everyone. This branch contains a ROS Pkg that manages all the stuffs about PROBOSCIS Prototype.

## Dynamixel Utils
In [dynamixel_utils.h](/proboscis_pkg/include/dynamixel_utils.h) you can find the `Dynamixel_Motors` class.

### Fitting of Torque-Current Curve
$$
i(t) = p_2 \,  \tau^2(t) + p_1 \,  \tau(t) + p_0
$$
where:
$$
p_2 = 0.2030 \quad p_1 = 0.5753 \quad p_0 = 0.1327
$$
>>>>>>> 5d363c0d4b20366f2e2f95ce4906ef480714b487
