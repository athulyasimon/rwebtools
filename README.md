##Robot Web Tools
Jackie Wu and Athulya Simon

Jarvis Schultz, MECH 495 Embedded Systems in Robotics, Fall 2014, Northwestern University

~~~
Table of Contents
1. Project Goals
2. ROS Package Dependencies and Other Things Needed
3. Results
4. How We Did It
5. Difficulties We Faced
6. Mistakes We Found in Existing Code
7. Useful Resources
~~~

####1. Project Goals
Our assignment was to allow an Internet browser to interact with ROS.

Specifically, we wanted to control Turtlesim on a local computer, through controls on a webpage. We wanted to create:

* arrow keys analogous to the function of `turtle_teleop_key` that would control left/right turn and forward/backward motion of the turtle
* input boxes that controlled the motion of the turtle (through its linear and angular velocity components)
* the ability to change `turtlesim`'s background color (with sliders that controlled the RGB values of the background parameter)

To do this, we needed to figure out how to publish and subscribe topics, get and set parameters, and call services from ROS on our computer to the web. 

* As an extension goal of the assignment, put the `turtlesim` visualization online and control its movements with a node on another computer
* As a practical extension goal of the assignment, help Professor Lynch create a website to better visualize joint transformations for students in his Robotic Manipulation class

####2. ROS Package Dependencies and Other Things Needed
* `roslibjs` - Javascript library that has standard ROS functions for JS, such as publisher/subscriber, services, TF, URDF, etc.
* `rosbridge` - connects ROS to non-ROS programs, written with JSON
* `turtlesim`, the simple ROS package that we wanted to put on our webpage
* EventEmitter2 - a Node.js event emitter implementation
* WebSocket - protocol that connects `rosbridge` to a web browser or server, over TCP
* An html file with Javascript functions from the `roslibjs` library
* a launch file we wrote to run all the nodes quickly and easily
* Weebly.com, a free site creator to host our site

####3. Results
We were able to complete basic functionality and control the `turtlesim` on the same local computer, from a web GUI. In addition, we completed the first extension task of allowing another computer control `turtlesim` remotely with just connection to the webpage. Both projects are on our website, [rwebtools.weebly.com](rwebtools.weebly.com). We were not able to complete the extension of putting `rviz` online - that would probably take another several weeks.

####4. How We Did It
Our first task was to get `turtlesim` to run on a web browser hosted locally.

*1. Background and Overview*

We found code on this site, [Iguanatronics](http://iguanatronics.com/igtron/?p=313), that showed how to create an HTML webpage, stored on our local computer, to control `turtlesim` via submitting commands to `cmd_vel` topic. This meant that allowing remote publish/subscribe was very easy. However, to complete basic functionality of our project, we still needed to figure out how to do service calls, get/set parameters, and for the extension task, figure out how to connect the website on the internet to our `turtlesim` node. 

Conceptually, on a computer with an active `turtlesim` node, we used the ROS package `rosbridge` to connect to WebSocket, which is a protocol that allows remote devices to communicate to a web browser. This in turn connected to a website where we wrote a user interface to control `cmd_vel`. 
~~~javascript
 /* This function:
	- retrieves numeric values from the text boxes
	- assigns these values to the appropriate values in the twist message
	- publishes the message to the cmd_vel topic.
  */
  function pubMessage() {

    /**
    Set the appropriate values on the twist message object according to values in text boxes
    It seems that turtlesim only uses the x property of the linear object 
    and the z property of the angular object
    **/
    var linearX = 0.0;
    var angularZ = 0.0;

    // get values from text input fields. Note for simplicity we are not validating.
    linearX = 0 + Number(document.getElementById('linearXText').value);
    angularZ = 0 + Number(document.getElementById('angularZText').value);

    // Set the appropriate values on the message object
    twist.linear.x = linearX;
    twist.angular.z = angularZ;

    // Publish the message 
    cmdVelTopic.publish(twist);
  }
~~~


*5. Calling a service in ROS*
    
This service is needed later on so we can change the background color of `turtlesim`. We need to clear the existing color of the background before we can set it as a parameter. 
Next, we wanted to control the turtle with arrow keys. We created visual arrow keys for the website as well as functionality for when the arrow keys on the keyboard are pressed. The following code controls the turtle. Forward and backward motion gives the turtle a spatial velocity of 1 and left/right turns the turtle counterclockwise/clockwise by π/2 (about 1.57) rad, numbers we just made up.
Our first task was to control `turtlesim` locally from an HTML page. For our second task, we wanted to give control of the 'turtlesim' on our local computer to anyone online. To test our idea, we put the HTML code on a free site builder, [rwebtools.weebly.com](rwebtools.weebly.com), which allowed us to control our running `turtlesim_node` (with WebSocket, `roscore` running) with our own computer.


Next, we tried running `turtlesim` over each other's computer via Ethernet. But we ran into some problems and this did not work (see **Difficulties We Faced** section).


Finally, we were able to give control of our local turtle to an online webpage by changing `localhost` from the line `url : 'ws://localhost:9090'` to our IP address. This seemed to work regardless of which wireless network we were on, although we ran into some strange behavior, which is also noticed in the **Difficulties We Faced** section. Both online control of the turtle on your local computer (assuming the appropriate nodes are running, e.g. from running the launch file) as well as control of Athulya's turtle from any computer with internet connection are on our website, [rwebtools.weebly.com](rwebtools.weebly.com).

####5. Difficulties We Faced

We tried to run `turtlesim` over each other's computer via Ethernet cable. However, this didn't work, and we ran into the following problems:

* Jackie's computer stopped being able to open `turtlesim_node` for some reason, giving the error 
> Couldn't find executable named turtlesim_node below /opt/ros/indigo/share/turtlesim

* Another classmate's laptop didn't work, giving an error that something was not defined in EventEmitter, which was the package that emitted "events" from the HTML back through WebSocket and `rosbridge`. This is probably regarding Ethernet connection.


Another problem we had was when we tried to control the turtle online remotely. In the code in the HTML, we changed `localhost` to our IP address. The result was that over the Northwestern University network, some IP addresses worked and allowed remote control of the turtle from another computer, but some addresses didn't. We have no idea why. 

####6. Mistakes We Found in Existing Code
None, though `rosbridge` was sparsely documented and had few tutorials. 

####7. Useful Resources
* [`rosbridge`](http://wiki.ros.org/rosbridge_suite) page on ROS.org
* [Iguanatronics turtlesim tutorial](http://iguanatronics.com/igtron/?p=313)
* [`roslibjs`](http://wiki.ros.org/roslibjs) page on ROS.org - has tutorials, including for URDF parser
* [Robot Web Tools](http://robotwebtools.org/)
* [Robot Management System](http://wiki.ros.org/rms) - we didn't use this for our project but it is something to explore in the future
* Apache is also something we would explore in the future
