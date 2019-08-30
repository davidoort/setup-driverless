#!/bin/bash

sudo apt-get update

## Install Docker
sudo apt-get install docker.io docker-compose


## Setup Python3 PIP

echo "Installing Python3-PIP" $'\n'
sudo apt-get install python3-pip


## Install ROS Melodic - http://wiki.ros.org/melodic/Installation/Ubuntu

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

sudo apt install ros-melodic-desktop-full

sudo rosdep init
rosdep update

echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc

sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential

## Git

function blank_line {
    printf '%*s\n' "${COLUMNS:-$(tput cols)}" '' | tr ' ' -
    echo $'\n'
}

echo -e "Enter your github user.name: "
read git_username

echo -e "Enter your github email.id: "
read git_email_id

# Blank Line
blank_line

echo "Thank you for entering your information!" $'\n'
read -p "Press enter to continue: "

# Blank Line
blank_line

# setup git
echo "Installing git" $'\n'
sudo apt-get install git -y
git config --global user.name "$git_username"
git config --global user.email "$git_email_id"

# Blank Line
blank_line



### Optional things. 


## Vim

## zsh 

## Terminator 

## CI computer? 

# Jenkins

# Matlab


### Other personal preferences such as Atom, Chrome, etc.


