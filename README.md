# Driverless DEMOs

## Autotoy Project
This 2-week project will teach you how to write a vanilla autonomous system using ROS and test it in simulation. Find the documentation for this project [here](autotoy/readme.md).

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

```
sudo apt-get update
sudo apt-get install git -y
git config --global user.name "github_username"
git config --global user.email "github_email"
```

Next, we are going to generate an SSH key and add it to your github account. When you run the following command, just hit enter when it asks where to store the SSH key and for a passphrase.

```
ssh-keygen -t rsa -b 4096 -C "github_email@example.com"
```	
Then add your newly generated SSH key to the ssh-agent.
```
eval "(ssh-agent -s)"
ssh-add ~/.ssh/id_rsa
```
Finally, add your public SSH key to your github account. Run the following command and copy the content. This is your public SSH key, which start with 'ssh-rsa' and ends with your email adress.
```
cat ~/.ssh/id_rsa.pub
```	
Now go to github > settings > SSH and GPG keys > new SSH key. Paste your public SSH here and hit 'Add SSH key'. Now you're ready to clone the repository.
```
cd ~/
mkdir git
cd git
git clone git@github.com:davidoort/setup-driverless.git
```

Now, you need to ENABLE ROS REPOSITORIES. Go to the Software and Updates application on ubuntu and Other Software. Click on Canonical Partners!

Then go to the terminal and run:

```
cd
bash git/setup-driverless/install_basic.sh
```

The installation process should begin. The `install_basic.sh` file will install the following by default:

* Docker
* Python 3
* ROS Melodic (including C++ compilers)
* Gazebo

You will additionally be prompted to install the following optional (and recommended) applications:

* Visual Studio Code (for helping each other out, it will be extremely useful to use the same IDE)
* Terminator
* Vim
* Zsh
* Slack Desktop
* GitKraken
* Chrome
* More might be added...

In VS Code, it is recommended to add the following extensions:
* Microsoft C/C++ 
* Microsoft Python 
* twxs CMake 
* Microsoft Docker 
* Microsoft ROS

Additionally, inside of VS Code, press `Ctrl`+`Shift`+`P` and search for C/C++ Edit Configurations (UI). In the section called *Include path* add the following:

``` 
/opt/ros/melodic/include
{workspaceFolder}/** 
```

