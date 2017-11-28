export DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
source $DIR/common.sh
echo "Running GCS failure test"
$DIR/start_containers.sh slow_bat_drain
mission_id=$(python $DIR/Tools/start_mission.py --lat 55.562747 --lon 10.113384 --requesturl http://$webui_ip:8000/EmergencyUser/DroneDispatch/ --accepturl http://$webui_ip:8000/EmergencyControl/control/)
echo "Mission ID is $mission_id"
coordinates=$(python $DIR/Tools/MissionMonitor.py --id $mission_id --locationurl http://$webui_ip:8000/EmergencyControl/mission_queue_json/True/ --goal_precision 10 --goal_height 3 --max_mission_time 600 --return_percent 0.2 --print_start --print_goal --print_end)
coordinates=$(echo $coordinates | sed ':a;N;$!ba;s/\n/ /g')
echo "Coordinates: $coordinates"
start_pos=$(echo "$coordinates" | cut -f1 -d' ')
goal_pos=$(echo "$coordinates" | cut -f2 -d' ')
midway_pos=$(echo "$coordinates" | cut -f3 -d' ')
echo "Start pos : $start_pos"
echo "Goal pos : $goal_pos"
echo "midway_pos : $midway_pos"
#Make sure that Motor shutdown performed has not occured in drone logfile yet
if $(docker logs drone 2>&1 | grep -q "Motor shutdown performed!"); then
    false
fi
echo "Drone is 20% of the way. Triggering TELEM2 failure by killing socat in the drone container"
docker exec drone killall -9 socat 
echo "The  drone should now cutting motor power, starting monitoring. We expects a powercut within 10 seconds!"
sleep 10
if $(docker logs drone 2>&1 | grep -q "Motor shutdown performed!"); then
    true
else
    false
fi
echo "Test finished successfully"
echo "Simulation log:"
docker logs simulation
echo "WebUI log:"
docker logs webui
echo "Drone Onboard SW log:"
docker logs drone
echo "GCS log:"
docker logs gcs
docker stop simulation webui drone gcs
docker rm simulation webui drone gcs
