cmake_minimum_required(VERSION 2.8.3)
project(cohan_navigation)

find_package(catkin REQUIRED COMPONENTS
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# install
foreach(dir maps launch/morse_pr2_launch config/morse_pr2_move_base_config launch/pr2_launch config/pr2_move_base_config launch/stage_pr2_launch config/stage_pr2_move_base_config)
   install(DIRECTORY ${dir}/
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
