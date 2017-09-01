# egm
C++ Interface with ABB EGM module

Yifan Hou
yifanh@cmu.edu

# Reference
*Official Manual*
Application manual-Controller software IRC5


# Step by Step Tutorial

## I. Prepare Protocol Buffers 
ABB EGM use ```Google Protocol Buffers``` to serialize/de-serialize data.

ABB provide the ```.proto``` file (included in this repo), which describes the message structures used by EGM.

You need to compile the ```.proto``` file, so as to obtain the library files in your programming language.

The library contains serialized/de-serialized code which is then used by the application. The application reads a message from the network, runs the de-serialization, creates a message, calls serialization method, and then sends the message.

More about ```protobuf```:
https://developers.google.com/protocol-buffers/docs/cpptutorial

> ***Todo***
Install the protobuf compiler.
* Download code from: 
https://github.com/google/protobuf
* Then follow the instructions:
https://github.com/google/protobuf/blob/master/src/README.md


## II. Prepare RAPID on Robot
There are a set of RAPID commands just for EGM. You can control a variety of parameters from that. For details please refer to the ABB official manual listed in *Reference* section.

For MLab people:
There are already programs with EGM on ABB R120 robot.
You can just load one of them.


## III. Write the server
In EGM, the server is the thread that talks with the robot.
The server has a few tasks to do:
1. Open&listen to the udp port for the handshake signal from EGM (the robot side)
2. Receive & unwrap messages from EGM
3. After receiving a message, send a wrapped message to EGM.

Here *wrap* means package into Google ProtocolBuf.
 
There are two code examples for you to choose from: 
1. egm-server: A minimal server example. Everything you need to write is in one file.
2. egm-comm: A C++ class wrapping up all the EGM function with easy to use interface. Also can be used with ROS.

Detailed tutorials can be found below. But firstly you need to download the code.
>***Todo***
> Open a new folder for the server code.
> ``` 
> $ mkdir egm
> $ cd egm
> ```
> Download the code from lab server:
> ``` bash
> $ git clone git@mlab.ri.cmu.edu:yifan/egm.git
> ```
> Now you should have the ```egm-server``` and ```egm-comm``` folders in your repo.

### 3.1 egm-server: Minimal Script for Running EGM
#### 3.1.1 File description
```/include/PracticalSocket``` A 3rd party package for operating UDP.
```/src/egm-sensor.cpp``` The original script example provided by ABB.
```/src/egm.proto``` The protobuf file provided by ABB.  
```/src/egm.pb.cc``` and ```/src/egm.pb.h``` Pre-compiled Protobuf library. 
```/src/egm_server_node.cpp``` The script that establishes the server.
```CMakeLists.txt``` Used by CMake for compiling. 


The pre-compiled  ```egm.pb.cc``` and ```egm.pb.h``` may not be compatible with your system. In that case, the first thing  you need to do is to **compile 'egm.proto' into a protobuf library**.  

#### 3.1.2 Compile egm.proto
> ***Todo***
>  You need to specify the source directory (where your application's source code lives – the current directory is used if you don't provide a value), the destination directory (where you want the generated code to go; often the same as $SRC_DIR), and the path to your .proto. 
> For example: ```$ protoc -I=$SRC_DIR --cpp_out=$DST_DIR $SRC_DIR/egm.proto ```
(remember to install the compiler first, as described in Step I)
In our example, you can run: 
> ``` 
> $ cd egm/egm-server/src
> $ protoc  --cpp_out=. egm.proto 
> ```
There will be a warning saying default syntax is used, and that is fine.

This generates the following files in your specified destination directory:

- egm.pb.h, the header which declares your generated classes.
- egm.pb.cc, which contains the implementation of your classes.

#### 3.1.3 Compile the server 
Next, lets compile this program.
> ***Todo***
> ```
> $ cd ..
> $ mkdir build
> $ cd build
> $ cmake ..
> $ make
> ```

Now  you can find the executable ```egm_server_node``` in egm/egm-server/build folder.
Proceed to step IV to test it.

#### 3.1.4 Code Explanation
(todo)


### 3.2 egm-comm: C++ class wrapper & ROS wrapper
#### 3.2.1 File description
```PracticalSocket.h``` and ```PracticalSocket.cpp``` A 3rd party package for operating UDP.
```egm.proto``` The protobuf file provided by ABB.  
```egm.pb.cc``` and ```egm.pb.h``` Pre-compiled Protobuf library. 
```EGMClass.h``` and ```EGMClass.cpp``` The class definition.
```TimerLinux.h``` and ```TimerLinux.cpp``` A c++ timer that works with linux.
```CMakeLists.txt``` Used by CMake for compiling. 
```package.xml``` Used by ROS building system.

You can compile this code into a library, integrate your code with it, or use it as a ROS package.

#### 3.2.2 Code Explanation
(todo)

## IV. Experiment with Robot
To test EGM, follow the steps below.
1. Power on the robot.
2. In RoboStudio, choose the correct RAPID program.
3. On the server computer, run the server program.
4. On robot Teach Pedant, click "start" button. A signal will be sent to server, and the 4ms communication shall begin.
