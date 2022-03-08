#!/bin/bash

#TODO: port to python script via subprocess?

#!/bin/bash

if sudo ip link add dev can0 type vcan; then
     echo "Successfully setup fake can interface: can0"
else
     echo "Failure, exit status: $?"
fi

if sudo ip link set up can0; then
     echo "Successfully brought up fake can interface: can0"
else
     echo "Failure, exit status: $?"
fi
