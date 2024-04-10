Interpreting the typical convention that is used for each `C++` package in ROS2

The `include` subfolder contains another sub-folder that has the same name of the 
package, e.g. `include/arduinobot_cpp_examples`. `include/arduinobot_cpp_examples`
contains the definition of functions and classes in `C++`.

Instead, the `src` folder contains the `C++` files that defines the behavior
of functions and classes, which is where we will insert the logic of each
ROS2 node.

A `CMakeLists.txt` acts as a sort of instruction sheed that tells the compiler
how it should translate our C++ scripts into executable files.

A `package.xml` stores all dependencies we use.