#!/bin/bash

RED='\033[0;31m'
BLUE='\033[0;34m'
GREEN='\033[0;32m'
NO_COLOR='\033[0m'
PYBOXDIR='pybox2d'

result(){
    if [ $? == 0 ]; then
        printf "${GREEN}######### ---- Success ---- #########${NO_COLOR}\n\n"
    else
        printf "${RED}######### ---- Failure ---- #########${NO_COLOR}\n\n" 
    fi
} 

dependency=(
    "sudo apt-get --yes --force-yes install python3"
    "sudo apt-get --yes --force-yes install python3-pip"
    "sudo apt-get --yes --force-yes install python3-tk"
    "sudo apt-get --yes --force-yes install python3-pil.imagetk"
    "sudo apt-get --yes --force-yes install swig"
    "sudo apt-get --yes --force-yes install swig3.0"
    "sudo apt-get --yes --force-yes install build-essential libssl-dev libffi-dev python3-dev"
    "sudo apt-get --yes --force-yes install python3-setuptools"
    "sudo pip3 install pygame"
    "sudo pip3 install Box2D-kengz"
)

pybox2d=(
    "git clone https://github.com/pybox2d/pybox2d"
    "cd pybox2d"
    "sudo pip3 install -e ."
    "sudo pip3 install catkin_pkg"
    "sudo pip3 install rospkg"
)

#echo "It may take a time..."
#sudo pip3 uninstall -y Box2D-kengz > /dev/null 
#2>&1

install-necessary(){
    declare -a argAry=("${!1}")
    for i in "${argAry[@]}"
    do
        $i      # Run the command
        result  # Show the result of the operation
    done
}

printf "${BLUE}######### ---- Updating packages... ---- #########${NO_COLOR}\n"
sudo apt-get update
result

install-necessary dependency[@]

if [ ! -d "$PYBOXDIR" ]; then
    install-necessary pybox2d[@]
    sudo rm -r "$PYBOXDIR/"
else
    sudo rm -r "$PYBOXDIR/"
    install-necessary pybox2d[@]
    sudo rm -r "$PYBOXDIR/"
fi
