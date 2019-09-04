# Driverless DEMOs

## Setup workstation

### Setup Linux (dual-boot)


#### On a machine with natively-supported GPU


1. **Windows**: Get a USB and put an Ubuntu image on it. Options:

	a. Ask David or Mundi for USB with Ubuntu 18 iso image
	
	b. Go online to Ubuntu website and download iso image. Download rufus and flash the image to the USB.
    
2. **Windows**: go to disk utility or disk partition. Click on C: drive (SSD) and shrink the volume. Allocate about half of the disk space (120 GB in my case) to the ubuntu partition. If this doesn't work turn off Fast Boot on Windows and proceed to the next step.

5. Restart and press F2 repeatedly until BIOS screen pops up. In the BIOS screen change the boot order and put the usb stick as first in the list (by default Windows starts up, so we want to change that). One you have changed the order (with F6) press F10 to boot. You should see a black screen giving you the option to install Ubuntu. If this doesn’t work, you might have to disable secure boot which prevents in some cases to boot with ubuntu on a windows system. If at any point the installation freezes go to step 7d.

6. **Ubuntu**: follow the installation instructions until you get to the partition step. You will have to select the option of doing it yourself (manually). Now, when you get the partition screen you have to choose the right volume to mount your root folder in. This will be the C volume (or SSD) with probably a weird name (sda is hard drive). Inside this volume you have to find “free space” which is the space you cleared out in the 4th step. There you have to mount the / folder (which is the root). Select all the available space for this! I would recommend that you select all but 8-12GB of space for this. These 8-12GB will be used for swap memory. I still don’t understand what the advantage is of splitting space allocated for /root and /home since /root includes /home on a directory level. When you have done this continue and it will ask you to restart. If you click the button it will probably freeze before restarting. Just power the laptop off.

7. When you power the laptop on again you will have to do a few checks:

	a. First check is to see if booting automatically brings you to the menu where you can select with what OS to boot (ubuntu 18 or Windows boot manager). If it automatically goes to Windows, change the order as done in step 5.

	b. **GRUB** Once you have managed to see the boot menu (dark purple screen with simple white text) try to boot with Ubuntu.
		
	c. **Ubuntu** If you have a GPU as was my case with an Acer Nitro (NVIDIA GPU) then the process will probably freeze at some point. For me it froze a few seconds after logging into ubuntu. If it doesn’t skip to check e).

	d. If it freezes then you will have to power off manually and restart. You should see GRUB (otherwise change the boot order in BIOS). In GRUB you can see the operating systems on your machine. If you select Ubuntu and press `e`. You should see some text which you can edit. Go to the `linux` line and at the end write `nomodeset`. Press F10. This should boot the system without freezing.
		
	e. **Ubuntu**: Now it is time to enable the NVIDIA proprietary drivers. First, in terminal run `sudo apt update && sudo apt upgrade -y`. Then go to Software & Updates->Additional Drivers and select the top-most checkbox. The, press Apply Changes. This will install the drivers for the GPU on ubuntu.

	f. Restart and boot with Ubuntu. 

	g. **Ubuntu** If it doesn’t freeze then check under Settings->Details that the correct graphics card is being used and not just the CPU.


#### On a machine without natively-supported GPU (such as the office desktops)

1. Get a USB and put an Ubuntu image on it. Options:
		
	a. Ask David or Mundi for USB with Ubuntu 18 iso image
		
	b. Go online to Ubuntu website and download iso image. Download rufus and flash the image to the USB.
    
2. **Windows**: go to disk utility or disk partition. Click on C: drive (SSD) and shrink the volume. Allocate about half of the disk space (120 GB in my case) to the ubuntu partition.

3. Reboot the computer and press F2 repeatedly until you get a screen showing the BIOS
4. **BIOS**: Under General->Boot Sequence->usb device (not floppy drive)->Apply->Exit. The computer will now boot with Ubuntu.
5. **Ubuntu**: Choose default selection (Install ubuntu, keyboard config, normal installation, download updates while installing ubuntu)
6. **Ubuntu**: Installation begins. It will ask you to reboot. Do so. Remove USB device, press enter. Then Press F12 or F2 repeatedly. 
7. **BIOS**: When the BIOS screen shows up go to BIOS Settings, General-> Boot Sequence-> UEFI-> Select Ubuntu->Apply->Exit. This might be different for your machine. You might have to put the USB Floppy device as first or try different setting until the system boots with Ubuntu. If it doesn't boot, reboot and go to BIOS again.
8. First add the ROS repository by going to "Software and Updates" in Applications, making sure that the four checkboxes in Ubuntu Software are ticked and that Canonical Partners is enabled in Other Software. Restart.


### Setup applications and packages

Open a terminal and run the following commands. Make sure you replace `github_username` and `github_email` with your personal details. 

	$ sudo apt-get update
	$ sudo apt-get install git -y
	$ git config --global user.name "github_username"
	$ git config --global user.email "github_email"
	
Next, we are going to generate an SSH key and add it to your github account. When you run the following command, just hit enter when it asks where to store the SSH key and for a passphrase.

	$ ssh-keygen -t rsa -b 4096 -C "github_email@example.com"
	
Then add your newly generated SSH key to the ssh-agent.

	$ eval "$(ssh-agent -s)"
	$ ssh-add ~/.ssh/id_rsa
	
Finally, add your public SSH key to your github account. Run the following command and copy the content. This is your public SSH key, which start with 'ssh-rsa' and ends with your email adress.

	$ cat ~/.ssh/id_rsa.pub
	
Now go to github > settings > SSH and GPG keys > new SSH key. Paste your public SSH here and hit 'Add SSH key'. Now you're ready to clone the repository.

	$ cd ~/
	$ mkdir git
	$ cd git
	$ git clone git@github.com:davidoort/setup-driverless.git

Now

	$ cd
	$ bash git/setup-driverless/init.sh

The installation process should begin. You will have to press enter a few times and write your computer password.


Once you are done, depending on whether you have certain parts of the ```init.sh``` commented out or not, you should be able to open/run the following applications:

* Git
* Docker
* Python 3
* ROS Melodic (including C++ compilers)
* Gazebo
* Terminator
* Vim
* Zsh
* Visual Studio Code 
* Slack Desktop
* GitKraken
* Chrome
* More might be added...

### Communicate with the car

## Jenkins Server installation
1. Install ubuntu (minimal install is fine)
2. Install all updates, then disable automatic updates (we don't want the machine to update and break while in competition season)
3. Setup SSH access. Make sure you disable root-login and only allow private-key-authentication!
4. Install docker and docker-compose: `sudo apt-get install docker.io docker-compose`
5. Download and install MathWorks from https://nl.mathworks.com/downloads/web_downloads/download_release?release=R2019a Use your personal license key to get it activated. You must do this installation with a screen and keyboard to let the gui work.
6. Save the following content to a file named `docker-compose.yml`.
```
version: '3'
services:
  jenkins:
    image: jenkinsci/blueocean
    ports:
      - "54321:8080"
    restart: always
    volumes:
      - /var/run/docker.sock:/var/run/docker.sock
      - /var/jenkins_home/:/var/jenkins_home/
    environment:
      - "TRY_UPGRADE_IF_NO_MARKER=true"
```
Run: `mkdir /var/jenkins_home` to create the data directory for jenkins en run `sudo docker-compose up -d` to startup jenkins.
Jenkins (within the container) must run as root user to make sure the jenkins can write to the parent filesystem. 

For the actual tests there were several issues.
* While preparing to build or test, mbed looks for (the folder .local/share in) the user's home directory. In the docker container, this is not properly defined so we need to pass the environment variable XDG_DATA_HOME into the container.
* The hardware devices for the boards need to be accessible in the container. Specifically, foreach board we need the usb drive used to flash the device, and the serial device to retrieve test results. Further, mbed looks for the symlinks to these devices in  /dev/disk/by-id/ and/dev/serial/by-id which it then explicitly dereferences. Passing devices into a container can be done by specifying --device=/dev/something:/dev/somethingelse aDer which the device/dev/something on the host is accessible under /dev/somethingelse in the container. Unfortunately, this does not work for the symlinks in disk/by-id. Further, manually adding the symlinks in the Dockerfile does not work since the /dev/ folder is managed by the OS and gets reset (on boot? at some point anyway). In the end I used a rather dirty fix, and just passed the entire /dev folder into the container. This is probably dangerous, but as long as we don't touch the server it should be ok. There is probably a better solution for this.
* Once the container has access to the devices, mbed requires that the usb drive is actually mounted. This does not happen automatically (at least on the default ubuntu:xenial containerI used. Mounting in the Dockerfile is not possible, so it needs to be done as part of the Jenkins build. Unfortunately, this requires that we run the build as root (or allow regular users to mount drives, which means they can just mount any drive with permissions set such that they get full access anyways). Could probably set up mounting somehow, but due to time constraints I took the lazy way out.

Right now the server works, but its not very robust. It assumes a specific device is attached, and has received a specific device name in /dev/. This means that unplugging the device or rebooting the server might result in the CI server breaking. Also, the Jenkins web interface is currently exposed to the internet.

### Re-install jenkins
If all jenkins-data is somehow lost, that is a bit annoying but not a disaster. 

All build-configuration is stored in the corresponding git repositories in `Jenkinsfile`s. 
So if you manage to make jenkins love github, and the other way around, you are all set.
There is a github-account (named `DUT-Builder`) that should be configured in a new item within Jenkins (choose 'github organization template'). Ask Sijmen for the password.
Next, you could setup github login. Use the following tutorial: https://wiki.jenkins.io/display/JENKINS/GitHub+OAuth+Plugin
Make sure you setup authorisation groups to prevent that every github user in the world can use our jenkins!
