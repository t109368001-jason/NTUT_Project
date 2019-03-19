# LiDar

## Requirements
- VTK 7.1.0
- pcl 1.8.0

## Installation

```bash
sudo apt-get install -y build-essential cmake

git clone https://github.com/Xiao-Xian-Jie/LiDar.git

cd LiDar
mkdir build
cd build

cmake ..
make -j $(($(nproc) + 1))
```
