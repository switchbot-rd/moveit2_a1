cmake_minimum_required(VERSION 3.8)
project(onero_a1_description)

find_package(ament_cmake REQUIRED)

# 注意：这里列出的目录必须**真实存在且被 git 追踪**。
# 曾经因为多列了一个空的 `config/` 目录导致干净克隆后构建直接失败：
#   ament_cmake_symlink_install_directory() can't find '.../onero_a1_description/config'
# git 不追踪空目录，而 bloom / ROS build farm 是从干净源码树构建的。
install(
  DIRECTORY urdf meshes launch rviz
  DESTINATION share/${PROJECT_NAME}
)

# 许可证随包安装（rosdistro 发布与下游再分发都需要）
install(FILES LICENSE DESTINATION share/${PROJECT_NAME})

ament_package()
