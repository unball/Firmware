#catch user
user=$(whoami)
#remove the libraries in case it is already installed
sudo rm -r /home/$user/Arduino/libraries/RF24
sudo rm -r /home/$user/Arduino/libraries/i2c_t3

#installs the libraries
cp -r RF24 /home/$user/Arduino/libraries/RF24
cp -r i2c_t3 /home/$user/Arduino/libraries/i2c_t3