#!/bin/bash

sudo apt-get update

## Install Docker
sudo apt-get install docker.io docker-compose


## Setup Python3 PIP

echo "Installing Python3-PIP" $'\n'
sudo apt-get install python3-pip


## Git - todo: remember login

function blank_line {
    printf '%*s\n' "${COLUMNS:-$(tput cols)}" '' | tr ' ' -
    echo $'\n'
}

:'
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
'

## Install ROS Melodic - http://wiki.ros.org/melodic/Installation/Ubuntu
# First add the ROS repository by going to "Software and Updates" in Applications, making sure that the four checkboxes in Ubuntu Software are ticked and that Canonical Partners is enabled in Other Software


sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# sometimes dependencies are not met. Then run sudo apt --fix-broken install

{ # try

    sudo apt install ros-melodic-desktop-full

} || { # catch
    sudo apt --fix-broken install 
    sudo apt install ros-melodic-desktop-full
}


sudo rosdep init
rosdep update

echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc

#sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential
sudo apt-get install python-rosinstall -y


## Terminator 

echo "Installing Terminator" $'\n'
sudo add-apt-repository ppa:gnome-terminator
sudo apt-get update
sudo apt-get install terminator

## VS Code

#You need to enable package repository in your system. Create a new file 

sudo touch /etc/apt/sources.list.d/vscode.list
#echo "deb [arch=amd64] http://packages.microsoft.com/repos/vscode stable main" > /etc/apt/sources.list.d/vscode.list
echo "deb [arch=amd64] http://packages.microsoft.com/repos/vscode stable main" | sudo tee /etc/apt/sources.list.d/vscode.list


curl https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > microsoft.gpg
sudo mv microsoft.gpg /etc/apt/trusted.gpg.d/microsoft.gpg

sudo apt-get update
sudo apt-get install code





### Optional things. 


## Gazebo

# You will need to add the osrfoundation repo to your Linux package system in order to get the new packages of Gazebo.
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -


#sudo apt-get update

## Vim

sudo apt-get update

# setup VIM
echo "Installing vim" $'\n'
sudo apt-get install vim -y

# Blank Line
blank_line

## zsh 

# Blank Line
blank_line

# setup oh-my-zsh
# echo "Installing oh-my-zsh (https://github.com/robbyrussell/oh-my-zsh)" $'\n'
# sudo apt install curl
# sudo apt-get install zsh -y
# sh -c "$(curl -fsSL https://raw.github.com/robbyrussell/oh-my-zsh/master/tools/install.sh | sed "s/env zsh//g")"
# chsh -s $(which zsh)

# Blank Line
blank_line



## CI computer? 

# Jenkins

# Matlab

# Slack

# wget https://downloads.slack-edge.com/linux_releases/slack-desktop-4.0.2-amd64.deb
# sudo apt install ./slack-desktop-*.deb


### Other personal preferences such as Atom, Chrome, etc.

# Blank Line
blank_line

# setup Chromium
echo "Installing Chromium Browser" $'\n'
wget https://dl.google.com/linux/direct/google-chrome-stable_current_amd64.deb
sudo dpkg -i google-chrome-stable_current_amd64.deb
rm -rf google-chrome-stable_current_amd64.deb

echo "In case an error is encountered in above step, run `sudo apt-get -f install`"

# Blank Line
blank_line


echo "Installation of all basic softwares and packages successful!"
echo "It is recommended that you restart your computer and start afresh."

