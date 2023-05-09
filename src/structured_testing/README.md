# Package Overview
In short this package serves as an intermediate testing phase between unit testing and system level testing. It is based on the principle of 'observer' nodes which monitors other nodes in the architecture for custom defined metrics. Further description of the schema can be described here: https://uwaterloo.atlassian.net/wiki/spaces/UWAFT/pages/43150967016/CAV+Structured+Testing+for+Simulation

# How To Run Demo Guide 
1. Get ROS2 bag to use from server if not already present in repo: `\\uwaft01.uwaterloo.ca\uwaft\CAV\CAV Workshops, Webinars and Training\Internal\Structured ROS Node Testing Demo\Test1_2022-08-05-13-21-20`
1.1 ROS2 bag can be visualized using plotjuggler or similar visualization tool.

2. Move ROS2 bag within test/ folder of structured_testing package.

3. Build structure_testing package.

4. In terminal, source ROS2 distro and workspace. 

5. Change directory to cav_monorepo and enter command: 

`python3 src/structured_testing/structured_testing/start_sim.py src/structured_testing/test/my_test_example.sim`


6. The test runner script will automatically stop after the duration specified in the *test_duration* field. If done correctly a .simresults file will be generated. All tests should PASS.

Note: Documentation here - https://uwaterloo.atlassian.net/wiki/spaces/UWAFT/pages/43170791542/Structured+Testing

# Known Errors
Error: "RuntimeError: reentrant call inside..."
Fix: Restart terminal and rerun structured testing