from .DroneProducer import DroneProducer
global droneproducer
droneproducer = None

def initialise(_id, connection):
    global droneproducer
    droneproducer = DroneProducer(_id, connection)
