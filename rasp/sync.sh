###
# @Author Youbiao He hybtalented@163.com
# @Date 2023-01-17
 # @LastEditors Youbiao He
 # @LastEditTime 2023-01-17
 # @FilePath /rasp/sync.sh
# @Description
###
CURRENT_DIR=$(
  cd "$(dirname $0)"
  pwd
)
IP_ADDRESS=192.168.1.22
USER_NAME=ubuntu
rsync -avz --rsync-path="sudo rsync" --delete ${CURRENT_DIR}/install/* ${USER_NAME}@$IP_ADDRESS:~/ros2_study
