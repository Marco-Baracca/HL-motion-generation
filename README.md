# HL_motion_generation
This package contains an example code of the work presented in the paper _"A general approach for generating artificial human-like motions from functional components of human upper limb movements"_, currently under review in the journal "Control Engineering Practice".

The proposed method exploits functional Principal Component Analysis (fPCA) to extract the principal common modes of the Cartesian motion of the human hand and build a reduced functional representation of these movements. The obtained basis of function is then used to design a planning algorithm able to directly embed the feature of human hand motion in the computed trajectories. 

The example code presented here is a proof of concept of the general approach. It permits, once the initial and final points of the desired trajectory, the obstacles (represented as a set of spheres) and the minimum distance from the obstacles that the trajectory has to maintain are defined, to compute a trajectory exploiting the basis of function returned by fPCA extracted in previous work (_Baracca, Marco, et al. "Functional analysis of upper-limb movements in the Cartesian domain." Converging Clinical and Engineering Research on Neurorehabilitation IV: Proceedings of the 5th International Conference on Neurorehabilitation (ICNR2020), October 13–16, 2020. Springer International Publishing, 2022_). 

To test this example, you only have to run the script "main_planning.m". To briefly describe the example, given an initial and a final point, the algorithm computes the direct trajectory to connect them. After that, it checks for collisions of the direct trajectory and, if there is any collision, it starts the routine to compute the trajectory performing obstacle avoidance.
To change the proposed scenario, you can edit the first part of the code (lines 8-27) where obstacles and boundary conditions of the trajectory are defined. More details can be found in the comments directly embedded in the code.

This example is currently only a proof of concept of the proposed method. Further developments will be released in the future.

Developed in MATLAB R2020b
