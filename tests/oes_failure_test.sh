set -e
echo "Running OES failure test"
$DIR/start_containers.sh slow_bat_drain
mission_id=$(python $DIR/Tools/start_mission.py --lat 55.562747 --lon 10.113384 --requesturl http://$webui_ip:8000/EmergencyUser/DroneDispatch/ --accepturl http://$webui_ip:8000/EmergencyControl/control/)
echo "Mission ID is $mission_id"
coordinates=$(python $DIR/Tools/MissionMonitor.py --id $mission_id --locationurl http://$webui_ip:8000/EmergencyControl/mission_queue_json/True/ --goal_precision 10 --goal_height 3 --max_mission_time 600 --return_percent 0.2 --print_start --print_goal --print_end)
echo "Coordinates: $coordinates"
start_pos=$(echo $coordinates | cut -f1 -d$'\n')
goal_pos=$(echo $coordinates | cut -f2 -d$'\n')
midway_pos=$(echo $coordinates | cut -f3 -d$'\n')
echo "Drone is 20% of the way. Triggering OES failure by stoping the container"
docker stop drone
echo "The  drone should now perform a RTL, starting monitoring"
end_pos=$(python $DIR/Tools/MissionMonitor.py --id $mission_id --locationurl http://$webui_ip:8000/EmergencyControl/mission_queue_json/True/ --goal_precision 10 --goal_height 3 --max_mission_time 600 --start_pos "$midway_pos" --goal_pos "$start_pos" --print_end)
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
