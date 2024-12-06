# arx_x5_python

## 目录
- [介绍](#介绍)
- [安装](#安装)
- [使用方法](#使用方法)
- [接口指南](#接口指南)

### example

## 📖介绍
ARX L5Pro SDK for Python

## 🔧安装
### 检查依赖
#### 安装can
```
sudo apt update && sudo apt install can-utils
```
#### 安装keyboard库
```
sudo pip3 install keyboard
```
#### 安装pybind11
```
git clone https://github.com/pybind/pybind11.git && cd pybind11 && mkdir build && cd build && cmake .. && make && sudo make install
```

#### 编译python接口
* cd到仓库目录下，执行:
    ```
    build.sh
    ```

## 🚀使用方法
### arx_can配置
```
sudo -S slcand -o -f -s8 /dev/arxcan0 can0 && sudo ifconfig can0 up
```

### example使用
#### 环境变量
```
source ./setup.sh
```
### 运行
```
python3 test_single_arm.py
```
### keyboard节点的运行
```
sudo su
source ./setup.sh
python3 test_keyboard.py
```

### 二次开发
* 把bimanual setup.sh移到自己的工程下即可

## 📚接口指南