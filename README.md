# RMBoardTest

Fully integrated BSP for RM Board with C/C++, Rust and FreeRTOS

# Rust Support

If you do not need Rust support, comment out the following parts in `CMakeLists.txt`.

```
add_library(demo STATIC IMPORTED GLOBAL)
add_dependencies(demo rust_target)

...

TARGET_LINK_LIBRARIES(${CMAKE_PROJECT_NAME}.elf demo)

```

and remove calls to the `rust_main` function in `freertos.c`.

# Compile

```bash
mkdir build
cd build
cmake ..
make -j8
make flash
```

# License

Unless otherwise specified, BSD
