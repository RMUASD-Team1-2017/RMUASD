set -e
docker network create --gateway ${networkgateway} --subnet ${networkgateway}/24 ${networkname} || true
docker run -d --name webui --network ${networkname} --ip ${webui_ip} -e rmq_host=${webui_ip} stefanrvo/rmuasd_webapp:latest
docker run -d --name simulation --network ${networkname} --ip ${simulation_ip} stefanrvo/rmuasd_gazebo:latest $1
sleep 25
docker run -d --cap-add=NET_ADMIN --name gcs --network ${networkname} -e mavroshost=${simulation_ip}  -e rmq_host=${webui_ip} stefanrvo/rmuasd_groundcontrol:latest
docker run -d --cap-add=NET_ADMIN --name drone --network ${networkname} -e webui_ip=${webui_ip} -e simulation_ip=${simulation_ip} stefanrvo/rmuasd_drone:latest

echo "Containers started. Waiting 30 seconds for system to stabilize."
sleep 30
