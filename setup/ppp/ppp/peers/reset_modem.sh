#!/bin/bash
RESET_PIN="46"
POWER_PIN="88"
RESET_TIMOUT="15"
function setup_pin
{
  pin=$1
  echo "$pin" > /sys/class/gpio/export || true
  echo "out" >/sys/class/gpio/gpio$pin/direction
}

function toggle_pin
{
  pin=$1
  echo "1" > /sys/class/gpio/gpio$pin/value
  echo "0" > /sys/class/gpio/gpio$pin/value    

}

logger "Ensuring modem is powered on by toggling pin $POWER_PIN!"
setup_pin $POWER_PIN
toggle_pin $POWER_PIN
logger "Resseting modem by toggling pin $RESET_PIN!"
setup_pin $RESET_PIN
toggle_pin $RESET_PIN
logger "Waiting 15 seconds for modem to start up"
sleep 15
