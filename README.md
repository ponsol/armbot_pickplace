# armbot_pickplace

1. Install:

   cd newws   

   git clone https://github.com/ponsol/armbot_pickplace.git

   colcon build --mixin debug --packages-select armbot_pickplace

2. Launch:

   ros2  launch  armbot_pickplace  pick_place_demo.launch.py

   The following command must be running in another terminal
 
   ros2 launch  armbot_moveit2  demo.launch.py


3. Run the above code with panda robot

    i.   Uncomment the  following line in  "armbot_pickplace/CMakeLists.txt"

          add_compile_definitions(PANDA)

    ii.  Select the correct line in "armbot_pickplace/launch/pick_place_demo.launch.py"

        #moveit_config = MoveItConfigsBuilder("armbot_moveit2", package_name="armbot_moveit2").to_dict()
        moveit_config = MoveItConfigsBuilder("moveit_resources_panda").to_dict()


