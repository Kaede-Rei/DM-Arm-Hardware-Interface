cmake_minimum_required(VERSION 3.8)
project(dm_arm_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(CODE "
set(_source_config_dir \"${CMAKE_CURRENT_SOURCE_DIR}/config\")
set(_install_config_dir \"\${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/config\")
set(_description_urdf_dir \"\${CMAKE_INSTALL_PREFIX}/../dm_arm_description/share/dm_arm_description/urdf\")
foreach(_profile gray white)
  file(READ \"\${_source_config_dir}/dm_arm_\${_profile}.yaml\" _content)
  string(REPLACE \"../../core/dm_arm_description/urdf\" \"\${_description_urdf_dir}\" _content \"\${_content}\")
  file(WRITE \"\${_install_config_dir}/dm_arm_\${_profile}.yaml\" \"\${_content}\")
endforeach()
")

ament_package()
