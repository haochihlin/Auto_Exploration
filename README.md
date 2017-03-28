Auto_exploration 
====================

# Abstract

This is a modified version of frontier_exploration.
To get more detail of original one, please refer to following links:

Wiki: http://wiki.ros.org/frontier_exploration

API Doc: http://docs.ros.org/hydro/api/frontier_exploration/html/annotated.html

The main contribution of this package is to offer a new add-on called "autonomous mode",
which would continuously execute the frontier_exploration to explore all unknown area by itself.

# Demo

To test the auto-explore with navigation_stage, please use following commands:

$ roslaunch navigation_stage move_base_gmapping_5cm.launch

$ roslaunch frontier_exploration auto_global_map.launch

# License

Both original and modified files are under BSD License.
