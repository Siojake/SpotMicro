prj=spot_i2c

build:
	catkin build -DCMAKE_BUILD_TYPE=Debug --no-color  ${prj}

debug:
	rosrun --prefix 'xterm -fg cyan -bg black -e gdb --args' ${prj} ${prj}_node

clean:
	@rm ../*~
