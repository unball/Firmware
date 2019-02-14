#catch user
user=$(whoami)
#remove the library in case it is already installed
sudo rm -r /home/$user/Arduino/libraries/RF24
#installs the library
cp -r RF24 /home/$user/Arduino/libraries/RF24
