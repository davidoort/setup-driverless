# Driverless DEMOs


## Setup workstation

### Setup Linux (dual-boot)



#### On a machine with GPU

Get a USB and put an Ubuntu image on it. Options:
		a. Ask David or Mundi for USB with Ubuntu 18 iso image
		b. Go online to Ubuntu website and download iso image. Download rufus and flash the image to the USB.
    
2. **Windows**: go to disk utility or disk partition. Click on C: drive (SSD) and shrink the volume. Allocate about half of the disk space (120 GB in my case) to the ubuntu partition.

5. Restart and press F2 repeatedly until BIOS screen pops up. In the BIOS screen change the boot order and put the usb stick as first in the list (by default Windows starts up, so we want to change that). One you have changed the order (with F6) press F10 to boot. You should see a black screen giving you the option to install ubuntu. If this doesn’t work, you might have to disable secure boot which prevents in some cases to boot with ubuntu on a windows system.
6. U: follow the installation instructions until you get to the partition step. You will have to select the option of doing it yourself (manually). Now, when you get the partition screen you have to choose the right volume to mount your root folder in. This will be the C volume (or SSD) with probably a weird name (but the size of the volume should be 256GB for my case). Inside this volume you have to find “free space” which is the space you cleared out in the 4th step. There you have to mount the / folder (which is the root). Select all the available space for this! (unless you are interested in having some space for swap). I still don’t understand what the advantage is of splitting space allocated for /root and /home since /root includes /home on a directory level. When you have done this continue and it will ask you to restart. If you click the button it will probably freeze before restarting. Just power the laptop off.
7. When you power the laptop on again you will have to do a few checks:
		1. first check is to see if booting automatically brings you to the menu where you can select with what OS to boot (ubuntu 16 or Windows boot manager). If it automatically goes to Windows, change the order as done in step 5.
		2. Once you have managed to see the boot menu (dark purple screen with simple white text) try to boot with Ubuntu.
		3. If you have a GPU as was my case with an Acer Nitro (NVIDIA GPU) then the process will probably freeze at some point. For me it froze a few seconds after logging into ubuntu. If it doesn’t skip to check e).
		4. If it freezes then you will have to power off manually and restart. This time, in the boot menu you will have to go to Advanced Settings for ubuntu and boot into the system in recovery mode (the kernel version you pick should not matter, I used the latest). You will be brought to a pink screen where you have to pick the name that ends with “…X” which disables the GPU. In my case this showed a terminal screen for a while and then brought me back to the pink menu. Choose “resume” the second time around. Once you do this, you should see that it doesn’t freeze anymore.
		5. U: In ubuntu, search for driver settings and in additional or advanced settings enable the use of the NVIDIA graphics card. This will install the drivers for the GPU on ubuntu.
		6. Restart and boot with Ubuntu (not recovery mode). 
		7. If it doesn’t freeze then check under system settings (in the system overview) that the correct graphics card is being used and not just the CPU.


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



#### On a computer to be used for Jenkins

TODO: Sijmen


## Communicate with the car


















