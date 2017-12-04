#Monitors if the time the GSM or telemetry has been lost esceeds some threshold and performs the required action if so.
import datetime
import logging
import time
import threading
GCS_NETWORK_LOSS_TIMEOUT = datetime.timedelta(seconds = 5)

TELEMETRY_LOSS_TIMEOUT = datetime.timedelta(seconds = 4)

from LEDControl.LEDControl import led as debug_led

class ConnectionMonitor:
    def __init__(self, gcs_network_check, network_check, telemetry_oes_check, drone):
        self.thread = threading.Thread(target = self.monitor, args = (gcs_network_check, telemetry_oes_check, drone) )
        self.thread.daemon = True
        self.thread.start()
        self.hasGSM = False
        self.hasGCS_GSM = False
        self.hasTELEM = False

    def getConnectionStatus(self):
        return {"GCS_GSM" : self.hasGCS_GSM,
                "TELEM" : self.hasTELEM,
                }


    def monitor(self, gcs_network_check, telemetry_oes_check, drone):
        logging.info("Performing 10 second delay to ensure that startup is completed")
        time.sleep(10)
        while True:
            try:
                time.sleep(2)
                gcs_network_lost = gcs_network_check() < datetime.datetime.now() - GCS_NETWORK_LOSS_TIMEOUT
                telem_fligt_control_lost = telemetry_oes_check() < datetime.datetime.now() - TELEMETRY_LOSS_TIMEOUT
                self.hasGCS_GSM = not gcs_network_lost
                self.hasTELEM = not telem_fligt_control_lost
                debug_led.setDebugColor(debug_type = "TELEM2", status = self.hasTELEM)
                debug_led.setDebugColor(debug_type = "GSM", status = self.hasGCS_GSM)
                logging.debug("GCS_GSM: {}, TELEM: {}.".format(self.hasGCS_GSM, self.hasTELEM))

                #print(gcs_network_lost, network_lost, telem_fligt_control_lost, telem_gcs_loss)
                #print(gcs_network_check(), network_check(), telemetry_oes_check(), telemetry_gcs_check())
                if gcs_network_lost:
                    logging.warning("Performing landing abort, GCS connection through GSM lost")
                    drone.land()
                if telem_fligt_control_lost:
                    logging.warning("Cutting power, telemetry 2 is lost.")
                    drone.hardabort()
            except:
                logging.exception("Exception encountered during connection monitoring.")
