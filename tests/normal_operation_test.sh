networkname="rmuasdnetwork"
networkgateway="10.40.0.1"
simulation_ip="10.40.0.2"
webui_ip="10.40.0.3"
set -e
docker network create --gateway ${networkgateway} --subnet ${networkgateway}/24 ${networkname} || true
docker run -d --name webui --network ${networkname} --ip ${webui_ip} -e rmq_host=${webui_ip} stefanrvo/rmuasdweb:latest
docker run -d --name simulation --network ${networkname} --ip ${simulation_ip} stefanrvo/rmuasd_simulation:latest
sleep 2
docker run -d --name drone --network ${networkname} -e webui_ip=${webui_ip} -e simulation_ip=${simulation_ip} stefanrvo/rmuasd_drone:latest
docker run -d --name gcs --network ${networkname} -e mavroshost=${simulation_ip}  -e rmq_host=${webui_ip} stefanrvo/rmuasd_groundcontrol:latest
#sleep 20
#mission_id=$(python Tools/start_mission.py --lat 55.556966 --lon 10.110615 --requesturl http://$webui_ip:8000/EmergencyUser/DroneDispatch/ --accepturl http://$webui_ip:8000/EmergencyControl/control/)
#echo "Mission ID is $mission_id"
#python Tools/MissionMonitor.py --id $mission_id --locationurl http://$webui_ip:8000/EmergencyControl/mission_queue_json/True/ --goal_precision 10 --goal_height 3 --max_mission_time 600
#docker stop simulation webui drone gcs
#docker rm simulation webui drone gcs
