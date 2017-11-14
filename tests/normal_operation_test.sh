export DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
source $DIR/common.sh
echo "Running normal mission test"
$DIR/start_containers.sh slow_bat_drain
mission_id=$(python $DIR/Tools/start_mission.py --lat 55.562747 --lon 10.113384 --requesturl http://$webui_ip:8000/EmergencyUser/DroneDispatch/ --accepturl http://$webui_ip:8000/EmergencyControl/control/)
echo "Mission ID is $mission_id"
python $DIR/Tools/MissionMonitor.py --id $mission_id --locationurl http://$webui_ip:8000/EmergencyControl/mission_queue_json/True/ --goal_precision 10 --goal_height 3 --max_mission_time 600
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
