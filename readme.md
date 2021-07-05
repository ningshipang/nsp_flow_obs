## 程序配置说明
1. 创建工作空间
```
mkdir ~/flow_obs
cd ~/flow_obs
```
2. 下载本仓库
```
git clone https://github.com/ningshipang/nsp_flow_obs.git
mv flow_obs src
```
3. 编译
```
catkin_make
```
4. 刷新ROS环境变量
```
# USER_NAME替换为自己的用户名，用zsh的是...../setup.zsh
source /home/USER_NAME/flow_obs/devel/setup.bash
# 或者把这句话加在~/.bashrc（或者~/.zshrc），然后重开一个终端，就不用每次都执行上面这句了
```

5. 启动mavros
```
roslaunch mavros px4.launch fcu_url:="udp://:20101@192.168.206.1:20100"
```

6. 接收rflysim图像
```
roslaunch rflysim_ros_pkg camera.launch
```

7. 避障测试
```
roscd obs/shell
./rfly-flow.sh
# (1)QGC起飞无人机
# (2)按p进入offboard模式
# (3)按e进入前飞光流避障
```

8. 接受光流避障速度节点
```
roslaunch obs obs_vel_fub.launch
# 节点消息为：mavros/flow_vel    TwistStamped类型
```
