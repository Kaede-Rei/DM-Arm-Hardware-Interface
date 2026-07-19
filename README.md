# DM-Arm Hardware Interface

## core-only 构建

```bash
cmake -S . -B build-core \
  -DCMAKE_BUILD_TYPE=Release \
  -DDM_ARM_BUILD_DAMIAO=OFF

cmake --build build-core -j
cmake --install build-core --prefix "$PWD/install-core"
```

外部项目：

```cmake
find_package(dm_arm_hardware_interface REQUIRED CONFIG)

target_link_libraries(your_target PRIVATE
    dm_arm::core
    dm_arm::config
)
```

## Damiao 构建

```bash
cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Debug \
  -DDM_ARM_BUILD_DAMIAO=ON

cmake --build build -j
cmake --install build --prefix "$PWD/install"
```

外部项目可链接：

```cmake
target_link_libraries(your_target PRIVATE dm_arm::damiao)
```
