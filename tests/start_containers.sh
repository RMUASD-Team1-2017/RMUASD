set -e
docker network create --gateway ${networkgateway} --subnet ${networkgateway}/24 ${networkname} || true
docker run -d --name webui --network ${networkname} --ip ${webui_ip} -e rmq_host=${webui_ip} stefanrvo/rmuasdweb:latest
docker run -d --name simulation --network ${networkname} --ip ${simulation_ip} stefanrvo/rmuasd_simulation:latest $1
sleep 15
docker run -d --cap-add=NET_ADMIN --name gcs --network ${networkname} -e mavroshost=${simulation_ip}  -e rmq_host=${webui_ip} stefanrvo/rmuasd_groundcontrol:latest
docker run -d --cap-add=NET_ADMIN --name drone --network ${networkname} -e webui_ip=${webui_ip} -e simulation_ip=${simulation_ip} stefanrvo/rmuasd_drone:latest

echo "Containers started. Waiting 40 seconds for system to stabilize."
sleep 40
