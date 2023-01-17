###
 # @Author Youbiao He hybtalented@163.com
 # @Date 2022-07-02
 # @LastEditors Youbiao He
 # @LastEditTime 2023-01-17
 # @FilePath /setup.sh
 # @Description 
 # 
 # @Example 
### 
echo 加载系统环境配置
. /etc/bash.bashrc
echo 加载用户环境配置
. ~/.bashrc
echo 加载 ROS2 工作空间环境配置
. ./install/setup.sh