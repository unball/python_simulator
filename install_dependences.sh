#!/bin/bash

RED='\033[0;31m'
BLUE='\033[0;34m'
GREEN='\033[0;32m'
NO_COLOR='\033[0m'

dependency=(
    "sudo pip3 install pygame"
    "sudo pip3 install box2d-py"
    "sudo pip3 install pygame-menu"
    "sudo pip3 install rospkg"
    "source devel/setup.bash"
)

result(){
    if [ $? == 0 ]; then
        printf "${GREEN}######### ---- Success ---- #########${NO_COLOR}\n\n"
    else
        printf "${RED}######### ---- Failure ---- #########${NO_COLOR}\n\n" 
    fi
} 

install-necessary(){
    declare -a argAry=("${!1}")
    for i in "${argAry[@]}"
    do
        $i      # Run the command
        result  # Show the result of the operation
    done
}

remove-not-necessary(){
    declare -a argAry=("${!1}")
    echo "It may take a time..."

    for i in "${argAry[@]}"    
    do
        $i > /dev/null 2>&1     # Run the command
    done
}

printf "${BLUE}######### ---- Updating packages... ---- #########${NO_COLOR}\n"
sudo apt-get update
result

install-necessary dependency[@]
