#!/bin/bash
if [ "$1" == "run" ]; then
  ./run.sh
elif [ "$1" == "test" ]; then
  ./test.sh
else
echo "Not a valid command, only test and run are valid!"
exit 1
fi
