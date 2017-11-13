from .DroneConsumer import DroneAbortWorker
global droneconsumer
droneconsumer = None

def initialise(_id, connection, drone, gps_monitor, connection_monitor):
    global droneconsumer
    droneconsumer = DroneAbortWorker(_id, connection, drone, gps_monitor, connection_monitor)

def get_last_gcs_heartbeat():
    global droneconsumer
    if droneconsumer:
        return droneconsumer.get_last_gcs_heartbeat()
