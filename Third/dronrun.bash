#!/bin/bash

while true; do
    # Проверяем наличие файла qgroudres.txt и читаем путь
    if [ -f ~/qgroudres.txt ] && [ -s ~/qgroudres.txt ]; then
        QGC_PATH=$(head -n 1 ~/qgroudres.txt)
    else
        # Если файл отсутствует или пуст, выполняем поиск QGroundControl.AppImage и записываем путь
        QGC_PATH=$(find ~ -type f -name "QGroundControl.AppImage" | head -n 1)
        if [ -n "$QGC_PATH" ]; then
            echo "$QGC_PATH" > ~/qgroudres.txt
        fi
    fi
    # Если путь найден, пробуем запустить QGroundControl
    if [ -n "$QGC_PATH" ] && [ -x "$QGC_PATH" ]; then
        # "$QGC_PATH" && break
        break
    else
        echo "QGroundControl.AppImage not found or failed to start. Retrying..."
        sleep 2  # Ожидание перед повтором
    fi
done

gnome-terminal \
    --tab --working-directory="$HOME/Firmware2/" --title="GazeboSim" --command="bash -c '. install/setup.bash && ros2 launch aerobot_gz_sim aerobot_third_test_mavros.launch.py; exec bash'" \
    --tab --working-directory="$HOME/" --title="MAVROS" --command="bash -c 'ros2 run mavros mavros_node --ros-args -p fcu_url:=udp://:14540@localhost:14557; exec bash'" \
    --tab --working-directory="$HOME/" --title="QGroundControl" --command="bash -c '$QGC_PATH; exec bash'"
    # --tab --working-directory="$HOME/" --title="Python Script" --command="bash -c 'exec bash'"
clear
