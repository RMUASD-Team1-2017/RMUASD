export networkname="rmuasdnetwork"
export networkgateway="10.40.0.1"
export simulation_ip="10.40.0.2"
export webui_ip="10.40.0.3"
error_handler() {
	echo "Simulation log:"
	docker logs simulation || true
	echo "WebUI log:"
	docker logs webui || true
	echo "Drone Onboard SW log:"
	docker logs drone || true
	echo "GCS log:"
	docker logs gcs || true
	docker stop simulation webui drone gcs
	docker rm simulation webui drone gcs
}
trap 'error_handler $LINENO' ERR
set -e
#$DIR/normal_operation_test.sh
#$DIR/gcs_failure_test.sh
#$DIR/oes_failure_test.sh
