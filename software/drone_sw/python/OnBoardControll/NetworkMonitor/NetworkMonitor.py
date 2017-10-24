#This module monitors the internet connection
#We does this by attempting a connection to googles dns servers [8.8.8.8, 8.8.4.4,]
import socket
import datetime
import threading
import logging
class NetworkMonitor:
    def __init__(self, hosts = ["8.8.8.8", "8.8.4.4", "ns1.stefanrvo.dk"], port = 53, check_interval = 1):
        self.last_connect = datetime.datetime.now() #Pretend we had connectivity at startup
        self.lock = threading.RLock()
        for host in hosts:
            self.test_connection(host, port, check_interval)
    def test_connection(self, host, port, interval):
        timer = threading.Timer(interval, self.test_connection, args = (host, port, interval))
        timer.daemon = True
        timer.start()
        try:
            socket.create_connection((host, port))
            with self.lock:
                self.last_connect = datetime.datetime.now()
        except socket.error:
            logging.warning("Could not establish a connection to {} at port {}".format(host, port))
    def get_last_connection(self):
        with self.lock:
            return self.last_connect



monitor = NetworkMonitor()
