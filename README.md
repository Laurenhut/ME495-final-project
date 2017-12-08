# Readme
##Starbax

##### Functionality of package
This package is a collection of scripts and message definitions created to have a Baxter robot prepare a cup of coffee using a single serve coffee maker, one cup, and one single serve packet of coffee. The objects must be set at a specific height and the coffee maker must face Baxter, but objects can otherwise be placed arbitrarily so long as Baxter can easily reach them.

![baxpicture](./picturepathfrompackage)

One [launch file][launch] is being worked on so that everything can be started with a single command.


##### [Scripts][src]

* General script facts, warnings and instructions

* [node name][linkid]: What the node does

* Ex: The [wander.py][src-node4] script launches a node that causes the turtlebot to monitor data from its [environmental scanners][src-node4-monitor] for nearby obstacles while moving. If the turtlebot [detects an obstacle][src-node4-stop] within 1.2m or has been moving for 10 seconds it will stop for 5 seconds, spin for 5 seconds, and begin moving in the new direction.


##### Topics & Services
* /cmd_vel is the default publish topic for all geometry_msgs/Twist messages in the package. This is appropriate for using with turtlesim, but will not work if launching with Gazebo. See remapping note in the Scripts section.



##### Parameters
* Parameter general notes
* /parameter_name description


##### [Launch Files][launch]

* [launch file name][linkid2]




[src]:https://github.com/repo_name/src_folder
[linkid]:https://github.com/repo_name/src_folder/specific_file.
[launch]:https://github.com/repo_name/launch_folder
[linkid2]:https://github.com/repo_name/launch_folder/specific_file


[launch-launch1]:https://github.com/ME495-EmbeddedSystems/walkthrough-2-f2017-idtx314/blob/44911fff172c58759b056094a9611f1dcc81035c/launch/keys_to_twist_with_ramps.launch

