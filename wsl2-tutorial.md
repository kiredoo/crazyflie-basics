# Use crazy-radio with wsl2 tutorial
> Credits : Isaac Lin 


1. Ensure that `usbip` is installed on your Windows host system. If not download and run the installation file on https://github.com/dorssel/usbipd-win/releases/tag/v4.2.0 .

2. Enter the following command on the Windows command prompt:
`usbipd list` In order to find the `busid` of the crazyflie on your Windows host system.
3. Running the Windows command prompt as a administrator (IMPORTANT), bind your crazyflie for sharing using the following command: `usbipd bind --busid <busid>` Replacing `<busid>` with the `busid` of the Crazyflie.
4. On the same shell, enter the command: `usbipd list` to verify that the Crazyflie is now being shared by `usbipd`. If yes, attach the Crazyflie to WSL2 using the command (while simultaneously keeping a WSL2 shell running):
`usbipd attach -b <BUSID> --wsl`
Replacing `<busid>` with the busid of the Crazyflie.

5. Enter the command: `usbipd list` To verify that the crazyflie is attached to the WSL2 subsystem. Run `cfclient` on the WSL2 shell and check if the Crazyflie can be detected.
