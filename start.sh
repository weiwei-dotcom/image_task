source ./install/setup.bash

if [ $# == 0 ]; then
    ros2 launch task_launch normal_yolo_detect.launch.py

elif [ $# == 1 ];then
    if [ "$1" == 'detect' ]; then
        ros2 launch task_launch normal_yolo_detect.launch.py
        # TODO: 
    fi

elif [ $# == 2 ]; then
    if [ "$1" == 'detect' ]; then
        if [ "$2" == 'normal' ]; then
            ros2 launch task_launch normal_yolo_detect.launch.py
        elif [ "$2" == 'realsense' ]; then
            ros2 launch task_launch realsense_yolo_detect.launch.py
        fi
    fi

elif [ $# == 3 ]; then
    if [ "$1" == 'detect' ]; then
        if [ "$2" == 'normal' ]; then
            if [ "$3" == 'yolo' ]; then
                ros2 launch task_launch normal_yolo_detect.launch.py
            fi
        elif [ "$2" == 'realsense' ]; then
            if [ "$3" == 'yolo' ]; then
                ros2 launch task_launch realsense_yolo_detect.launch.py
            fi
        fi
    fi

else
    echo "please input bootargs!!"
fi

