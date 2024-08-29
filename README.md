# robot_tts
Robot text to speech (TTS) package


# Installation

Install ROS 2 Humble

Set up a virtual environment

Install dependencies
```
sudo apt-get update
sudo apt-get install -y portaudio19-dev

pip install -r requirements.txt
```

Install Riva python-client
Ref: https://github.com/nvidia-riva/python-clients
```
git clone https://github.com/nvidia-riva/python-clients.git
cd python-clients
git submodule init
git submodule update --remote --recursive
pip install -r requirements.txt
python3 setup.py bdist_wheel
pip install --force-reinstall dist/*.whl

pip install nvidia-riva-client

cd ../
```

Add riva client dir to `PYTHONPATH`
```
export PYTHONPATH=$PYTHONPATH:path-to-python-clients
```

Build and source ROS 2 TTS package
```
cd ros2_ws
colcon build
source install/setup.bash
```

# Run ROS 2 TTS Node

Run ROS node
```
IP_ADDR=ip.address.to.riva.server
PORT=50051
ros2 run tts_riva_bridge tts_riva_bridge --ros-args -p tts_server_uri:="$IP_ADDR:$PORT"
```


# Voice customization

Ref: https://docs.nvidia.com/deeplearning/riva/user-guide/docs/tutorials/tts-basics-customize-ssml.html#emotion-attribute

