# solder_paste_workspace

## Build and Source Instructions

First things first you need to run `catkin_make` in the root of the ROS Workspace (Also the root of the git repo)

This will add two folders `build` and `devel`. In `devel` there are three files called `setup.bash setup.sh setup.zsh`. These are the setup files in the syntax of three common shells. As I'm pretty sure I'm the only guy hipster enough to use `zsh` for my shell, I'll explain this setup for `bash` but the method is the same for all three. 

In your `.bashrc` file you need to add the line `source /path/to/solder_paste_workspace/devel/setup.bash` anywhere in the file. `/path/to` being the path to wherever you placed the workspace. 

I would also reccomend adding this line:

`alias bashupdate="source ~/.bashrc"`

This allows you to run the command `bashupdate` which will re-source your `.bashrc` file. This is especially important when you make new packages or executables, otherwise they won't show up in `rosrun` or `roslaunch` without first closing and reopening your terminal. This just lets you skip that step.
