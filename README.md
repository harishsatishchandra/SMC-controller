# Sliding mode controller for trajectory tracking

The following two animations show the sliding mode controller in action for a single lane change and double lane change manoeuver. The red curve is the output that we get from a trajectory planner, which here is treated as the ground truth trajectory to follow. The green curve is the output of the trajectory tracker (sliding mode controller). The dashed lines are the overshoot that we get from the rear wheels.

![smc1_cropped](https://user-images.githubusercontent.com/19624843/69922347-9ad51980-1469-11ea-8484-017e435a9664.gif)<p align="center">**Single lane change**</p>

![smc2_cropped](https://user-images.githubusercontent.com/19624843/69922358-bb04d880-1469-11ea-8b88-028c6fa6d524.gif)<p align="center">**Double lane change**</p>

The implementation details are as follows - 

![github_intro1_Page_1_cropped](https://user-images.githubusercontent.com/19624843/63885285-0361b580-c9a6-11e9-8cb3-d3dfb8da265a.png)

![github_intro1_Page_2_cropped](https://user-images.githubusercontent.com/19624843/63885292-05c40f80-c9a6-11e9-9096-14586a80bd33.png)

![github_intro1_Page_3_cropped](https://user-images.githubusercontent.com/19624843/63885296-08266980-c9a6-11e9-9f92-d7e1f5fc2b38.png)

![github_intro1_Page_4_cropped](https://user-images.githubusercontent.com/19624843/63885299-09f02d00-c9a6-11e9-96a3-7c7918dcb6e4.png)


