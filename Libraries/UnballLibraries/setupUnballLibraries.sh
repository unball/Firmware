#catch user
user=$(whoami)

#remove bibliotecas da UnBall da pasta do arduino
sudo rm -r /home/$user/Arduino/libraries/UnballLibraries
#cria a pasta da biblioteca na pasta do Arduino
mkdir /home/$user/Arduino/libraries/UnballLibraries
ln setupUnballLibraries.sh /home/$user/Arduino/libraries/UnballLibraries/
ln README.md /home/$user/Arduino/libraries/UnballLibraries/
ln UnballLibraries.h /home/$user/Arduino/libraries/UnballLibraries/

#cria as pastas de cada header
#Control
mkdir /home/$user/Arduino/libraries/UnballLibraries/Control
ln Control/Control.h /home/$user/Arduino/libraries/UnballLibraries/Control/

#Encoder
mkdir /home/$user/Arduino/libraries/UnballLibraries/Encoder
ln Encoder/Encoder.h /home/$user/Arduino/libraries/UnballLibraries/Encoder/

#Imu
mkdir /home/$user/Arduino/libraries/UnballLibraries/Imu
ln Imu/Imu.h /home/$user/Arduino/libraries/UnballLibraries/Imu/

#Motor
mkdir /home/$user/Arduino/libraries/UnballLibraries/Motor
ln Motor/Motor.h /home/$user/Arduino/libraries/UnballLibraries/Motor/

#Pins
mkdir /home/$user/Arduino/libraries/UnballLibraries/Pins
ln Pins/Pins.h /home/$user/Arduino/libraries/UnballLibraries/Pins/

#Radio
mkdir /home/$user/Arduino/libraries/UnballLibraries/Radio
ln Radio/Radio.h /home/$user/Arduino/libraries/UnballLibraries/Radio/

#Led RGB
mkdir /home/$user/Arduino/libraries/UnballLibraries/LedRGB
ln LedRGB/LedRGB.h /home/$user/Arduino/libraries/UnballLibraries/LedRGB/