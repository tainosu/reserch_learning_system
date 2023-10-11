# PoKeBo Cube
- PoKeBo Cube学習システムのメインパッケージです。
- まだpromiseしか対応していません。

## Dependencies
- [cube_dc](https://git.icd.cs.tut.ac.jp/pokebo-cube/cube_dc)#ros2-foxy
- [rosrealsense](https://git.icd.cs.tut.ac.jp/icd-tech/ros/rosrealsense)#ros2-foxy-windows
    - 2023年10月現在、メインパッケージはubuntu, rosrealsenseはwindowsで動かしています。
- [pywizavo](https://git.icd.cs.tut.ac.jp/icd-tech/python/pywizavo)

## Getting Start
- ubuntuPC
```bash
$ python -m pip install pywizavo
$ cd __ros2_workspace__/src
$ git clone http://git.icd.cs.tut.ac.jp/pokebo-cube/cube_dc
$ git clone https://git.icd.cs.tut.ac.jp/pokebo-cube/pokebo-cube-sekikawa/pokebo-cube
$ cd ..
$ colcon build
$ source install/setup.bash
$ sh ~/__ros2_workspace__/src/pokebo_cube/shell/cube_promise.sh
```
`cube_promise.sh`はpromise用の起動スクリプトです。future, mirai用の起動スクリプトはあとで用意します。
