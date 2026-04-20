


```
sudo apt update && sudo apt upgrade

sudo apt install terminator vim build-essential git

sudo dpkg --add-architecture i386
sudo apt update
sudo apt install libc6:i386 libstdc++6:i386  gcc-multilib g++-multilib curl linux-firmware

```

Install Miniconda, Vscode



wget https://developer.download.nvidia.com/compute/cuda/12.8.0/local_installers/cuda_12.8.0_570.86.10_linux.run
sudo sh cuda_12.8.0_570.86.10_linux.run

conda create -n rby1 python=3.12

conda activate rby1

pip install torch torchvision --index-url https://download.pytorch.org/whl/cu128

pip install rby1-sdk


Install Realsense

https://github.com/realsenseai/librealsense/blob/master/doc/distribution_linux.md
pip install pyrealsense2 