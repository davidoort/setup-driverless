sudo apt-get update
sudo apt-get -f dist-upgrade
sudo apt-get install git
​
#SSH key setup 
ssh-keygen
eval `ssh-agent` 
ssh-add ~/.ssh/id_rsa
cat ~/.ssh/id_rsa.pub
​
#CLONE THE COMPUTER-SETUP REPO
git clone git@github.com:kbrandes45/Computer-Setup.git
​
sudo sh -c "echo 'deb [arch=amd64] http://dl.google.com/linux/chrome/deb/ stable main' >> /etc/apt/sources.list.d/google-chrome.list"
wget https://dl.google.com/linux/linux_signing_key.pub
sudo apt-key add linux_signing_key.pub
sudo apt update
sudo apt install google-chrome-stable
​
#Setup ROS/Python/other shit
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install python-catkin-tools autoconf automake curl debhelper python-pip git openssh-client wget ros-melodic-eigen-conversions ros-melodic-tf-conversions libv4l-dev ros-melodic-velodyne python3-pip
sudo apt-get update
sudo apt-get install -y ros-melodic-socketcan-bridge libpcap-dev ros-melodic-gazebo-plugins libyaml-dev ros-melodic-velodyne-description ros-melodic-velodyne-gazebo-plugins ros-melodic-velodyne-simulator fonts-lato freeglut3-dev gazebo9 gazebo9-common gazebo9-plugin-base libasound2 libasound2-data libass9 libasyncns0 libavc1394-0 libavdevice-dev libavdevice57 libavfilter-dev libavfilter6 libavresample-dev libavresample3 libbs2b0 libbullet-dev libbullet2.87 libcaca0 libccd-dev libccd2 libcdio-cdda2 libcdio-paranoia2 libcdio17 libcharls1 libcurl4-openssl-dev libdc1394-22 libdc1394-22-dev libegl1-mesa libexif-dev libexif-doc libexif12 libfftw3-double3 libflac8 libflite1 libfreeimage-dev libfreeimage3 libfribidi0 libgail-common libgail18 libgazebo9 libgazebo9-dev libgdcm2-dev libgdcm2.8 libgphoto2-6 libgphoto2-dev libgphoto2-l10n libgphoto2-port12 libgraphviz-dev libgtk2.0-0 libgtk2.0-bin libgtk2.0-common libgts-dev libgvc6-plugins-gtk libiec61883-0 libignition-cmake-dev libignition-common libignition-common-dev libignition-fuel-tools1-1 libignition-fuel-tools1-dev libignition-math4 libignition-math4-dev libignition-msgs libignition-msgs-dev libignition-transport4 libignition-transport4-dev libilmbase-dev libilmbase12 libjack-jackd2-0 libjxr0 liblept5 libmysofa0 libnorm1 libogre-1.9-dev libogre-1.9.0v5 libopenal-data libopenal-dev libopenal1 libopencv-calib3d-dev libopencv-calib3d3.2 libopencv-contrib-dev libopencv-contrib3.2 libopencv-core-dev libopencv-core3.2 libopencv-dev libopencv-features2d-dev libopencv-features2d3.2 libopencv-flann-dev libopencv-flann3.2 libopencv-highgui-dev libopencv-highgui3.2 libopencv-imgcodecs-dev libopencv-imgcodecs3.2 libopencv-imgproc-dev libopencv-imgproc3.2 libopencv-ml-dev libopencv-ml3.2 libopencv-objdetect-dev libopencv-objdetect3.2 libopencv-photo-dev libopencv-photo3.2 libopencv-shape-dev libopencv-shape3.2 libopencv-stitching-dev libopencv-stitching3.2 libopencv-superres-dev libopencv-superres3.2 libopencv-ts-dev libopencv-video-dev libopencv-video3.2 libopencv-videoio-dev libopencv-videoio3.2 libopencv-videostab-dev libopencv-videostab3.2 libopencv-viz-dev libopencv-viz3.2 libopencv3.2-java libopencv3.2-jni libopenexr-dev libopenexr22 libpgm-5.2-0 libpostproc-dev libpostproc54 libprotobuf-dev libprotobuf-lite10 libprotobuf10 libprotoc-dev libprotoc10 libpulse0 libqtpropertybrowser4 libqwt-headers libqwt-qt5-6 libqwt-qt5-dev libraw1394-11 libraw1394-dev libraw1394-tools libraw16 librubberband2 libruby2.5 libsamplerate0 libsdformat6 libsdformat6-dev libsdl2-2.0-0 libsimbody-dev libsimbody3.5v5 libslang2 libsndfile1 libsndio6.1 libsocket++1 libspnav0 libtar-dev libtar0 libtbb-dev libtbb2 libtesseract4 libtinyxml-dev liburdfdom-dev liburdfdom-headers-dev liburdfdom-model liburdfdom-model-state liburdfdom-sensor liburdfdom-world libusb-1.0-0-dev libusb-1.0-doc libwrap0 libxdot4 libxi-dev libxmu-dev libxmu-headers libyaml-dev libzip-dev libzip4 libzmq3-dev libzmq5 libzzip-0-13 opencv-data protobuf-compiler python-opencv rake ros-melodic-camera-calibration-parsers ros-melodic-camera-info-manager ros-melodic-cv-bridge ros-melodic-gazebo-dev ros-melodic-gazebo-msgs ros-melodic-gazebo-plugins ros-melodic-gazebo-ros ros-melodic-image-transport ros-melodic-polled-camera ros-melodic-urdf ros-melodic-velodyne-description ros-melodic-velodyne-gazebo-plugins ros-melodic-velodyne-simulator ros-melodic-xacro ruby ruby-did-you-mean ruby-minitest ruby-net-telnet ruby-power-assert ruby-test-unit ruby2.5 rubygems-integration sdformat-sdf ttf-dejavu-core unzip x11proto-input-dev zip  ros-melodic-actionlib-tutorials ros-melodic-camera-calibration ros-melodic-common-tutorials ros-melodic-compressed-depth-image-transport ros-melodic-compressed-image-transport ros-melodic-control-msgs ros-melodic-control-toolbox ros-melodic-controller-interface ros-melodic-controller-manager ros-melodic-controller-manager-msgs ros-melodic-depth-image-proc ros-melodic-desktop ros-melodic-desktop-full ros-melodic-diagnostic-aggregator ros-melodic-diagnostic-analysis ros-melodic-diagnostic-common-diagnostics ros-melodic-diagnostics ros-melodic-diff-drive-controller ros-melodic-effort-controllers ros-melodic-executive-smach ros-melodic-filters ros-melodic-forward-command-controller ros-melodic-gazebo-ros-control ros-melodic-gazebo-ros-pkgs ros-melodic-geometry ros-melodic-geometry-tutorials ros-melodic-gl-dependency ros-melodic-hardware-interface ros-melodic-image-common ros-melodic-image-geometry ros-melodic-image-pipeline ros-melodic-image-proc ros-melodic-image-publisher ros-melodic-image-rotate ros-melodic-image-transport-plugins ros-melodic-image-view ros-melodic-interactive-marker-tutorials ros-melodic-interactive-markers ros-melodic-joint-limits-interface ros-melodic-joint-state-controller ros-melodic-joint-state-publisher ros-melodic-joy ros-melodic-kdl-parser ros-melodic-kdl-parser-py ros-melodic-laser-assembler ros-melodic-laser-filters ros-melodic-laser-geometry ros-melodic-laser-pipeline ros-melodic-librviz-tutorial ros-melodic-map-msgs ros-melodic-media-export ros-melodic-nodelet-tutorial-math ros-melodic-perception ros-melodic-perception-pcl ros-melodic-pluginlib-tutorials ros-melodic-position-controllers ros-melodic-python-qt-binding ros-melodic-qt-dotgraph ros-melodic-qt-gui ros-melodic-qt-gui-cpp ros-melodic-qt-gui-py-common ros-melodic-qwt-dependency ros-melodic-realtime-tools ros-melodic-resource-retriever ros-melodic-robot ros-melodic-robot-state-publisher ros-melodic-ros-tutorials ros-melodic-roscpp-tutorials ros-melodic-roslint ros-melodic-rospy-tutorials ros-melodic-rqt-action ros-melodic-rqt-bag ros-melodic-rqt-bag-plugins ros-melodic-rqt-common-plugins ros-melodic-rqt-console ros-melodic-rqt-dep ros-melodic-rqt-graph ros-melodic-rqt-gui ros-melodic-rqt-gui-cpp ros-melodic-rqt-gui-py ros-melodic-rqt-image-view ros-melodic-rqt-launch ros-melodic-rqt-logger-level ros-melodic-rqt-moveit ros-melodic-rqt-msg ros-melodic-rqt-nav-view ros-melodic-rqt-plot ros-melodic-rqt-pose-view ros-melodic-rqt-publisher ros-melodic-rqt-py-common ros-melodic-rqt-py-console ros-melodic-rqt-reconfigure ros-melodic-rqt-robot-dashboard ros-melodic-rqt-robot-monitor ros-melodic-rqt-robot-plugins ros-melodic-rqt-robot-steering ros-melodic-rqt-runtime-monitor ros-melodic-rqt-rviz ros-melodic-rqt-service-caller ros-melodic-rqt-shell ros-melodic-rqt-srv ros-melodic-rqt-tf-tree ros-melodic-rqt-top ros-melodic-rqt-topic ros-melodic-rqt-web ros-melodic-rviz ros-melodic-rviz-plugin-tutorials ros-melodic-rviz-python-tutorial ros-melodic-self-test ros-melodic-simulators ros-melodic-smach ros-melodic-smach-msgs ros-melodic-smach-ros ros-melodic-stage ros-melodic-stage-ros ros-melodic-stereo-image-proc ros-melodic-tf2-geometry-msgs ros-melodic-tf2-kdl ros-melodic-theora-image-transport ros-melodic-transmission-interface ros-melodic-turtle-actionlib ros-melodic-turtle-tf ros-melodic-turtle-tf2 ros-melodic-turtlesim ros-melodic-urdf-parser-plugin ros-melodic-urdf-sim-tutorial ros-melodic-urdf-tutorial ros-melodic-urdfdom-py ros-melodic-velocity-controllers ros-melodic-vision-opencv ros-melodic-visualization-marker-tutorials ros-melodic-visualization-tutorials ros-melodic-viz ros-melodic-webkit-dependency libreadline-dev ros-melodic-hector-gazebo-plugins ros-melodic-ackermann-msgs
​
pip install --upgrade numpy 
pip install networkx
pip install requests shapely scs==2.0.2 cvxpy -f https://download.mosek.com/stable/wheel/index.html Mosek==8.1.80
cd /home/kathleen/
mkdir mosek
cp ~/Computer-Setup/mosek.lic ~/mosek
​
sudo apt-get update
sudo apt-get install -y ros-melodic-mavlink ros-melodic-ackermann-msgs libgeographic-dev geographiclib-tools ros-melodic-diagnostic-updater ros-melodic-geographic-msgs libyaml-cpp-dev
cd /home/kathleen/
git clone https://github.com/jbeder/yaml-cpp.git 
mkdir -p yaml-cpp/build 
cd yaml-cpp/build && cmake -DBUILD_SHARED_LIBS=ON ../ && make -j8 && make install
​
#set up shit for spinaker
cd /tmp
wget -q https://www.dropbox.com/s/nrx22gktem1izu7/spin-drv.tar
tar -xvf spin-drv.tar
cd spinnaker-1.20.0.14-amd64
sudo dpkg -i libspinnaker-*.deb 
​
git clone git@github.com:DUT-Racing/DUT18D_ws.git
cd DUT18D_ws/
git fetch
source /opt/ros/melodic/setup.bash
catkin config --init --merge-devel --cmake-args -DCMAKE_BUILD_TYPE=Release
sudo rosdep init
rosdep update -q
rosdep install --from-paths src --ignore-src -r -y -q
source /opt/ros/melodic/setup.bash
git submodule update --init
git submodule update --init --recursive
git submodule sync
catkin build
source devel/setup.bash
​
#to get amz sim (if not in master)
cd src/
git submodule add git@github.com:DUT-Racing/DUT18D_amzsim.git simulation_amz
cd simulation_amz/
git checkout submodule-setup
./update_dependencies.sh
catkin build fssim_interface
#can test by roslaunch this all
​
cd /home/kathleen/
git clone git@github.com:DUT-Racing/DUT18D_MPC_solver.git
mkdir -p DUT18D_MPC_solver/examples/ocp/build
​
#Install slack
wget https://downloads.slack-edge.com/linux_releases/slack-desktop-3.3.8-amd64.deb
sudo apt install ./slack-desktop-*.deb
​
#Install htop
sudo apt-get update
sudo apt-get install htop
​
#install sublime and htop
wget -qO - https://download.sublimetext.com/sublimehq-pub.gpg | sudo apt-key add -
echo "deb https://download.sublimetext.com/ apt/stable/" | sudo tee /etc/apt/sources.list.d/sublime-text.list
sudo apt-get update
sudo apt-get install sublime-text
​
#To install drake: 
cd /home/kathleen/
git clone https://github.com/RussTedrake/underactuated.git
sudo apt-get update
sudo underactuated/scripts/setup/ubuntu/18.04/install_prereqs
export PYTHONPATH=`pwd`/underactuated/src:${PYTHONPATH} 
curl -o drake.tar.gz https://drake-packages.csail.mit.edu/drake/nightly/drake-latest-bionic.tar.gz
sudo tar -xvzf drake.tar.gz -C /opt
sudo apt-get update
sudo /opt/drake/share/drake/setup/install_prereqs
export PYTHONPATH=/opt/drake/lib/python2.7/site-packages:${PYTHONPATH}
python -c 'import pydrake; print(pydrake.__file__)'
​
#screen record and stress
sudo add-apt-repository ppa:maarten-baert/simplescreenrecorder
sudo apt update
sudo apt install simplescreenrecorder stress
​
#subnet tools
sudo apt update
sudo apt install net-tools openssh-server nano
sudo systemctl status ssh
sudo service ssh restart
​
​
#To install Julia
#Download (sad) tar.gz from website:https://julialang.org/downloads/index.html
cd /home/kathleen/Downloads/
sudo tar -xvzf julia-1.1.0-linux-x86_64.tar.gz -C /home/kathleen/
sudo ln -s /home/kathleen/julia-1.1.0/bin/julia /usr/local/bin/julia
export PATH=/home/kathleen/julia-1.1.0/bin:$PATH
​
​
sudo sh -c "echo 'echo 'MIT Driverless testbed IP: 192.168.0.230'' >> /home/kathleen/.bashrc"
echo 'alias xcd="cd /home/kathleen/DUT18D_ws"' >> /home/kathleen/.bashrc
echo 'alias xs="source /home/kathleen/DUT18D_ws/devel/setup.bash"' >> /home/kathleen/.bashrc
echo 'alias xviz="export ROS_MASTER_URI=http://192.168.0.230:11311"' >> /home/kathleen/.bashrc
sudo sh -c "echo 'echo 'Exporting necessary paths for Drake'' >> /home/kathleen/.bashrc"
​
echo "export PYTHONPATH=`pwd`/underactuated/src:${PYTHONPATH}" >> /home/kathleen/.bashrc
echo "export PYTHONPATH=/opt/drake/lib/python2.7/site-packages:${PYTHONPATH}" >> /home/kathleen/.bashrc
​
echo "export PYTHONPATH=`pwd`/Forces_pro/FORCES_client:${PYTHONPATH}" >> /home/kathleen/.bashrc
sudo apt install xserver-xorg-core
sudo apt install xserver-xorg-input-synaptics
sudo apt install xserver-xorg-input-all