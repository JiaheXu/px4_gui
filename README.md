# px4_plotting
implementation of plotting px4 states with Qt and qcustomplot

git clone this repo to your catkin_ws/src/

run.sh is in catkin_ws/src/px4_gui/
to run the code, you can just use 
./run.sh or  rosrun px4_gui px4_gui
you can also copy all the bash files to catkin_ws/ direction
remember to delete 2-4 lines in run.sh
remember to chmod 777 *.sh 

when you click save_pic button, it will save plots to  catkin_ws/saved_plots , 
the direction will be built by the code.

if you want to see acceleration and jerk plots,
go to catkin_ws/src/px4_gui/src/main_window.cpp
change line 31 to :
plotting_row34 = 1 ;
