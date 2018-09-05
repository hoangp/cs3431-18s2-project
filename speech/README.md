READ ME：

Install libraries:
sudo apt-get install ros-kinetic-audio-common libasound2 gstreamer0.10-*  gstreamer1.0-pocketsphinx

Install dependencies:
sudo dpkg -i libsphinxbase1_0.8-6_amd64.deb
sudo dpkg -i libpocketsphinx1_0.8-5_amd64.deb
sudo dpkg -i gstreamer0.10-pocketsphinx_0.8-5_amd64.deb

cd ~/catkin_ws/src
"copy the pocketfile into src"


Test:
roslaunch pocketsphinx robocup.launch
roslaunch pocketsphinx voice_cmd.launch

https://blog.csdn.net/seeseeatre/article/details/79228816

http://www.diegorobot.com/wp/?p=555&lang=en
