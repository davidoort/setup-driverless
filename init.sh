#!/bin/bash

sudo apt-get update

## Install Docker
sudo apt-get install docker.io docker-compose


## Setup Python3 PIP

echo "Installing Python3-PIP" $'\n'
sudo apt-get install python3-pip


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



## Install ROS Melodic - http://wiki.ros.org/melodic/Installation/Ubuntu
# First add the ROS repository by going to "Software and Updates" in Applications, making sure that the four checkboxes in Ubuntu Software are ticked and that Canonical Partners is enabled in Other Software


sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt install ros-melodic-desktop-full

sudo rosdep init
rosdep update

echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc

#sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential
sudo apt-get install python-rosinstall -y


### Optional things. 


## Gazebo

# You will need to add the osrfoundation repo to your Linux package system in order to get the new packages of Gazebo.
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
#sudo apt-get update



## Vim

## zsh 

## Terminator 

## CI computer? 

# Jenkins

# Matlab


### Other personal preferences such as Atom, Chrome, etc.

