# Driverless DEMOs

## Setup workstation

### Setup Linux (dual-boot)

#### On a machine with GPU

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


#### On a machine without GPU (such as the office desktops)

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

Very simple. Open a terminal and run the following commands (*this information is specific to your case*):

	$ sudo apt-get update
	$ sudo apt-get install git -y
	$ git config --global user.name "*git_username*"
	$ git config --global user.email "*git_email_id*"

	$ mkdir git
	$ cd git
	$ git clone https://github.com/davidoort/setup-driverless.git

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


#### On a computer to be used for Jenkins

TODO: Sijmen


## Communicate with the car


















