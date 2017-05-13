
rapyuta_ws := rapyuta_ws
scuderia := scuderia.yaml
machines := $(rapyuta_ws)/src/rapyuta/machines

all: $(machines)


$(machines): $(scuderia)
	python setup/create-machines-file.py $(scuderia) > $(machines)

fix-time:
	echo "Calling ntpdate to fix time"
	sudo ntpdate -u us.pool.ntp.org 

fix-time2:
	sudo ntpdate -s time.nist.gov

clean-pyc:
	find rapyuta_ws/src/ -name '*.pyc' | xargs rm 

catkin-clean: clean-pyc
	rm -rf $(rapyuta_ws)/build

build-parallel:
	catkin_make -C $(rapyuta_ws) --make-args "-j4"

build:
	catkin_make -C $(rapyuta_ws) 

# Unit tests
# Teddy: make it so "make unittests" runs all unit tests

unittests-environment: $(machines)
	bash -c "source environment.sh; source set_vehicle_name.sh; python setup/sanity_checks"

unittests:
	$(MAKE) unittests-environment
	bash -c "source environment.sh; catkin_make -C $(rapyuta_ws) run_tests; catkin_test_results $(rapyuta_ws)/build/test_results/"


unittests-anti_instagram:
	$(MAKE) unittests-environment
	bash -c "source environment.sh; rosrun anti_instagram annotation_tests.py"

# HW testing 

test-camera:
	echo "Testing Camera HW by taking a picture (smile!)."
	raspistill -t 1000 -o test-camera.jpg


test-led: 
	echo "Calibration blinking pattern"
	bash -c "source environment.sh; rosrun rgb_led blink test_all_1"

test-turn-right:
	echo "Calibrating right turn"
	bash -c "rostest indefinite_navigation calibrate_turn.test veh:=$(vehicle_name) type:=right"

test-turn-left:
	echo "Calibrating left turn"
	bash -c "rostest indefinite_navigation calibrate_turn.test veh:=$(vehicle_name) type:=left"

test-turn-forward:
	echo "Calibrating forward turn"
	bash -c "rostest indefinite_navigation calibrate_turn.test veh:=$(vehicle_name) type:=forward"


# Basic demos

vehicle_name=$(shell hostname)

demo-joystick: unittests-environment
	bash -c "source environment.sh; source set_ros_master.sh;  roslaunch duckietown joystick.launch veh:=$(vehicle_name)"

demo-joystick-high-speed: unittests-environment
	bash -c "source environment.sh; source set_ros_master.sh;  roslaunch duckietown joystick.launch veh:=$(vehicle_name) joy_mapper_param_file_name:=high_speed "

demo-joystick-camera: unittests-environment
	bash -c "source environment.sh; source set_ros_master.sh;  roslaunch duckietown joystick_camera.launch veh:=$(vehicle_name)"

demo-joystick-camera-high-speed: unittests-environment
	bash -c "source environment.sh; source set_ros_master.sh;  roslaunch duckietown joystick_camera.launch veh:=$(vehicle_name) joy_mapper_param_file_name:=high_speed "

demo-line_detector: unittests-environment
	bash -c "source environment.sh; source set_ros_master.sh; roslaunch duckietown line_detector.launch veh:=$(vehicle_name)"



demo-joystick-perception: unittests-environment
	bash -c "source environment.sh; source set_ros_master.sh; source set_vehicle_name.sh; roslaunch duckietown_demos master.launch fsm_file_name:=joystick"

demo-lane_following-%: unittests-environment
	bash -c "source environment.sh; source set_ros_master.sh; source set_vehicle_name.sh; roslaunch duckietown_demos lane_following.launch line_detector_param_file_name:=$*"

demo-led-fancy1: unittests-environment
	bash -c "source environment.sh; rosrun rgb_led fancy1"

demo-led-fancy2: unittests-environment
	bash -c "source environment.sh; rosrun rgb_led fancy2"

demo-led-blink-%: unittests-environment
	bash -c "source environment.sh; rosrun rgb_led blink $*"

demo-line_detector-default:     demo-line_detector-quiet-default
demo-line_detector-guy:         demo-line_detector-quiet-guy
demo-line_detector-universal:   demo-line_detector-quiet-universal
demo-line_detector-default_ld2: demo-line_detector-quiet-default_ld2

demo-line_detector-quiet-%: unittests-environment
	bash -c "source environment.sh; source set_ros_master.sh; roslaunch duckietown line_detector.launch veh:=$(vehicle_name) line_detector_param_file_name:=$* verbose:=false"

# ==========
# openhouse demos
 
openhouse-dp1: unittests-environment
	bash -c "source environment.sh; source set_ros_master.sh; source set_vehicle_name.sh; roslaunch duckietown_demos parallel_autonomy.launch"

openhouse-dp1-%: unittests-environment
	bash -c "source environment.sh; source set_ros_master.sh; source set_vehicle_name.sh; roslaunch duckietown_demos parallel_autonomy.launch line_detector_param_file_name:=$*"
 
openhouse-dp2: unittests-environment
	bash -c "source environment.sh; source set_ros_master.sh; source set_vehicle_name.sh; roslaunch duckietown_demos obstacle_vehicle_avoid.launch"

openhouse-dp2-%: unittests-environment
	bash -c "source environment.sh; source set_ros_master.sh; source set_vehicle_name.sh; roslaunch duckietown_demos obstacle_vehicle_avoid.launch line_detector_param_file_name:=$*"

openhouse-dp2-vehicle: unittests-environment
	bash -c "source environment.sh; source set_ros_master.sh; source set_vehicle_name.sh; roslaunch duckietown_demos vehicle_avoid.launch"

openhouse-dp2-obstacle: unittests-environment
	bash -c "source environment.sh; source set_ros_master.sh; source set_vehicle_name.sh; roslaunch duckietown_demos obstacle_avoid.launch"

openhouse-dp2-vehicle-no-wheels: unittests-environment
	bash -c "source environment.sh; source set_ros_master.sh; source set_vehicle_name.sh; roslaunch duckietown_demos vehicle_avoid_nowheels.launch"
openhouse-dp2-obstacle-no-wheels: unittests-environment
	bash -c "source environment.sh; source set_ros_master.sh; source set_vehicle_name.sh; roslaunch duckietown_demos obstacle_avoid_nowheels.launch"



openhouse-dp3: unittests-environment
	bash -c "source environment.sh; source set_ros_master.sh; source set_vehicle_name.sh; roslaunch duckietown_demos indefinite_navigation.launch"

openhouse-dp3-%: unittests-environment
	bash -c "source environment.sh; source set_ros_master.sh; source set_vehicle_name.sh; roslaunch duckietown_demos indefinite_navigation.launch  line_detector_param_file_name:=$*"

