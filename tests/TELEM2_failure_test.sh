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
echo "Drone is 20% of the way. Triggering TELEM2 failure by killing socat in the drone container"
docker exec drone killall -9 socat 
echo "The  drone should now perform a landing, starting monitoring. We expects a landing within 30 seconds!"
end_pos=$(python $DIR/Tools/MissionMonitor.py --id $mission_id --locationurl http://$webui_ip:8000/EmergencyControl/mission_queue_json/True/ --goal_precision 10 --goal_height 3 --max_mission_time 30 --start_pos "$midway_pos" --goal_pos "$start_pos" --print_end --return_when_landed)
echo "Drone is at $end_pos"
echo "Simulation log:"
docker logs simulation
echo "WebUI log:"
docker logs webui
echo "Drone Onboard SW log:"
docker logs drone
echo "GCS log:"
docker logs gcs
docker stop simulation webui drone
docker rm simulation webui drone gcs
