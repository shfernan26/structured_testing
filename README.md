# CAV Structured Testing for Simulation

The structured testing framework is a method introduced to validate a system or just a subset of ROS 2 nodes. It serves as an extenstion of the existing ROS 2 launch framework. The target use case for this tool is when a developer wants to test a part of the stack against a scenario that has been logged in ROS 2 bag files.
Certain nodes can be launched independently and their required topics can be made available from a given bag file. At a very high level this tool does 3 main tasks:

1. Start launch files.
2. Play bag files, while only publishing required topics from nodes launched in step 1.
3. Observe the simulation output and assess its performance.


# Prerequisites

This is written assuming the reader is familiar with ROS 2. 


# Modifying Launch Files

Launch files follow the standard XML format as described here: [Creating a launch file — ROS 2 Documentation: Foxy documentation](https://docs.ros.org/en/foxy/Tutorials/Intermediate/Launch/Creating-Launch-Files.html).

**Example Launch File**

For example to launch a single node named _kalman_filter_:
```
<launch>

<node pkg = "kalman_filter" exec ="kalman_filter" />

</launch>
```

# The .sim File
A sim file is the core of this system, this file defines the launch files to start, relevant topics, the corresponding bag files, and the Observers
to assess the quality of the simulation. In the background this file is parsed as a standard YAML file. Its fields, requiring specification by the developer, are described below:

**Launch files:** Are specified with their package and file name. This is the same way the file would be launched
in a terminal (i.e `ros2 launch foo bar.launch`).

**Topics:** In order for a launch file to work with the structured testing framework it is expected that you explicitly define the topics they publish
and the topics they subscribe to. This is to allow a rosgraph to be built and analyzed before launch enabling the valid topics to be pulled
from the bag if needed.
- `published_topics` : List of topics that are provided by the nodes defined in the launch file
- `required_topics` : List of topics required by Observers and nodes in launch file

NOTE: Currently this feature does not support parsing Published and Required Topics recursively from include tags. If a launch file is
including other launch files its topics would have to be manually added to the parent launch file.

**Bag files:** Are specified as paths on the file system. Ensure that each topic required from the bags is present in exactly one bag file. The system will fail to launch if a required topic is either missing or present in multiple bags.

**Test duration:** Specifies the time (in seconds) that the test runs for between shutting down and exiting the process. If required (e.g. for CI purposes), specify an integer greater than zero. If you would like to shut down the test manually, input _0_ for this field and shut down the test with CTRL+C if running.

**Observers:** are specified using the format defined in [Observer Schema](documentation/ObserverDefinition.md). Custom Observers can be created manually by either making modifications to an existing one or starting from scratch using the [template Observer](structured_testing/observers/template_observer.py).

**Example .sim file:**
```
launch_files:
    - package: structured_testing
      file: testA.launch

published_topics:
    - /echo
    - /foxtrot

required_topics:
    - /alpha
    - /bravo

bag_files:
    - /home/test_repo/src/input_topics.bag

test_duration:
    - 10

observers:
    - name: CmdVelX
      observerClass: InRangeObserver
      topic: "/cmd_vel"
      msgType: "geometry_msgs.msg.Twist"
      field: "linear.x"
      observerType: ALWAYS_TRUE
      minVal: 0
      maxVal: 10
```


# Running A Simulation
NOTE: All commands should be run from the root of the repository where the structured test package is cloned in. Also remember to source your ROS 2 distribution and workspace.

1. Clone the structured testing repo in the src folder of a colcon workspace. Make sure to source the ROS 2 distro and workspace as needed.
2. Build a _.sim_ file with the required fields following the instructions above (or use the [basic_test.sim](./test/basic_example/basic_test.sim) as an example).
3. Run the start_sim.py script with the path to the sim file as a command line argument (ex. `python3 src/structured_testing/structured_testing/start_sim.py src/structured_testing/test/basic_example/basic_test.sim`) 
4. When you are ready to terminate the simulation, kill the script with a sigterm (Ctrl+C in terminal). The results of the simulation will be written to the same directory as the _.sim file_ but with the _.simresults_ extension.

As of right now running structured tests through `ros2 run` is not supported. Instead, invoke the script through python or directly run it. I.e `python3 start_sim.py` OR `./start_sim.py`

## Try it out Yourself
Using the steps in the _Running A Simulation_ section, you can try out a pre-built example to see structured testing in action before integrating it into your own codebase. Read more about it [here](documentation/BasicExample.md).


# Integration with Gitlab CI tool

NOTE: This has been tested with Gitlab CI but should be compatible with other CI tools as well. It is up to the user to make the corresponding changes for all other tools.

To be integrated with a Gitlab CI tool simply add the two commands below to your CI file. Also ensure that the duration field is greater than 0.

1. Run the start_sim.py script with the path to the sim file as a command line argument (ex. `python3 src/.../start_sim.py src/.../example.sim`)

2. Run the check_results.py script with the path to the _.simresults_ file created from step 1 as a command line argument (ex. `python3 src/.../check_results.py src/.../example.simresults` )

3. Results check

    a. If ALL tests pass: The result status is printed to the command line with exit code (0)
    
    b. If a single test fails: The result status is printed to the command line with exit code (1)

The exit codes ensure that the Gitlab CI tool passes/fails as expected.
