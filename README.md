# rc_car_experiments

## test for sending commands through the rc override topic

    roslaunch rc_car_experiments apm2.launch fcu_url:=udp://:14550@192.168.4.1:14555
    roslaunch rc_car_experiments basic.launch
    rosrun mavros mavsys rate --all 5
    rosrun mavros mavsys rate --extra1 50 --position 50
    rosrun rc_car_experiments commands_test.py
