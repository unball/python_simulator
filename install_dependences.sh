#!/bin/bash

RED='\033[0;31m'
BLUE='\033[0;34m'
GREEN='\033[0;32m'
NO_COLOR='\033[0m'

result(){
    if [ $? == 0 ]; then
        printf "${GREEN}######### ---- Sucess ---- #########${NO_COLOR}\n\n"
    else
        printf "${RED}######### ---- Failure ---- #########${NO_COLOR}\n\n" 
    fi
} 

dependency=(
    "sudo apt-get install python3"
    "sudo apt-get install python3-pip"
    "sudo apt-get install python3-tk"
    "sudo apt-get install python-3pil.imagetk"
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
