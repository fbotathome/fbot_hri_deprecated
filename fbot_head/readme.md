# FBOT_HEAD

## Install dependencies

Execute the file install.sh to install all python dependencies.

## Neck controller port error

```bash
# Check the current symbolic link for /dev/ttyNECK
ls -l /dev/ttyNECK

# Reload udev rules and restart the udev service
sudo udevadm control --reload-rules && sudo service udev restart && sudo udevadm trigger

# Verify the updated symbolic link for /dev/ttyNECK
ls -l /dev/ttyNECK

```
