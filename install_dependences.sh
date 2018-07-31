#!/bin/bash

RED='\033[0;31m'
BLUE='\033[0;34m'
GREEN='\033[0;32m'
NO_COLOR='\033[0m'

result(){
    if [ $? == 0 ]; then
        printf "${GREEN}######### ---- Success ---- #########${NO_COLOR}\n\n"
    else
        printf "${RED}######### ---- Failure ---- #########${NO_COLOR}\n\n" 
    fi
} 

dependency=(
    "sudo apt-get --force-yes install python3"
    "sudo apt-get --force-yes install python3-pip"
    "sudo apt-get --force-yes install python3-tk"
    "sudo apt-get --force-yes install python3-pil.imagetk"
    "sudo apt-get --force-yes install swig"
    "sudo apt-get --force-yes install swig3.0"
    "sudo apt-get --force-yes install build-essential libssl-dev libffi-dev python3-dev"
    "sudo apt-get --force-yes install python3-setuptools"
    "sudo pip3 install pygame"
    "sudo pip3 install box2d-py"
    )

printf "${BLUE}######### ---- Updating packages... ---- #########${NO_COLOR}\n"
sudo apt-get update
result

for i in "${dependency[@]}"
do
    $i      # Execute the command
    result  # Show the result of the operation
done
