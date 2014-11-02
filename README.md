##Robot Web Tools
Jackie Wu and Athulya Simon

Jarvis Schultz, EECS 495 Embedded Systems in ROS, Fall 2014, Northwestern University


<<<<<<< HEAD
####What We Were Trying to Do
Our assignment was to allow an Internet browser to interact with ROS.


2. As an extension goal of the assignment, create a more complex user interface to control the Turtlesim
3. As an extension goal of the assignment, put the Turtlesim visualization online and control its movements with a node on the local computer
4. As a practical extension goal of the assignment, help Professor Lynch create a website to better visualize joint transformations for students in his Robotic Manipulation class

####What We Used
=======

####Project Goals
Our assignment was to allow an Internet browser to interact with ROS.

* As an extension goal of the assignment, create a more complex user interface to control the Turtlesim
* As an extension goal of the assignment, put the Turtlesim visualization online and control its movements with a node on the local computer
* As a practical extension goal of the assignment, help Professor Lynch create a website to better visualize joint transformations for students in his Robotic Manipulation class

####ROS Package Dependencies and Other Things Needed
>>>>>>> 8901f44ec2004ab7dd53fdd73532b6a7abc19d92
* `roslibjs` - Javascript library that has standard ROS functions for JS, such as publisher/subscriber, services, TF, URDF, etc.
* `rosbridge` - connects ROS to non-ROS programs, written with JSON
* WebSocket - protocol that connects `rosbridge` to a web browser or server, over TCP
* An html file with Javascript functions from the `roslibjs` library
* Weebly.com, a free site creator to host our site
* `turtlesim`, the simple ROS package that we wanted to put on our webpage

####The Result

####How We Did It
On a computer with a ROS active node, we used `rosbridge` to connect to WebSocket, a protocol that allows remote devices to communicate to a web browser, to connect to a website. In our website's HTML file, we import the necessary Javascript libraries such as `roslibjs` to write certain ROS functions for the HTML website.

We found code on this site, [Iguanatronics](http://iguanatronics.com/igtron/?p=313), that showed how to create an HTML webpage on our local computer to control the `Turtlesim` with, via the `turtle_teleop_key` node. This meant that learning to publish and subscirbe to messages was easy. We are currently figuring out how to set parameters.

However, we also tried to:
1. Put the HTML webpage online
2. Do the extension assignments


####Interesting Insights

####Difficulties We Had

####Result

####How We Did It
First, we wanted to get `turtlesim` to run on a web browser.

We found code on [this site, Iguanatronics](http://iguanatronics.com/igtron/?p=313), that showed how to create an HTML webpage, stored on our local computer, to control `turtlesim` via submitting commands to the `turtle_teleop_key` node. This meant the first part of our assignment was very easy. 

On a computer with an active `turtlesim` node, we used the ROS package `rosbridge` to connect to WebSocket, which is a protocol that allows remote devices to communicate to a web browser. This in turn connected to a website where we wrote a user interface to control `turtle_teleop_key`. 

After running `roscore` and opening the `turtlesim` node with
    
    rosrun turtlesim turtlesim_node
    
we installed `rosbridge`:

    sudo apt-get install ros-hydro-rosbridge-suite
    
Then we launched it with

    roslaunch rosbridge_server rosbridge_websocket.launch
  
Next, we created an HTML file to create a website on our local machine. In our website's HTML file, we import the necessary Javascript libraries, which are:

* `roslibjs` to write certain ROS functions for the HTML website

* EventEmitter2, something that allows us to emit "events" back to ROS, written in Javascript with Node.js, an open-source app building environment.

~~~html
    <!DOCTYPE html>
    <html>
    <head>
    <script type="text/javascript" src="http://cdn.robotwebtools.org/EventEmitter2/current/eventemitter2.min.js"></script>
    <script type="text/javascript" src="http://cdn.robotwebtools.org/roslibjs/current/roslib.min.js"></script>
    <script type="text/javascript" type="text/javascript">
~~~

Next, there is some documented code on connecting, disconnecting, or error in connecting to `rosbridge`.

~~~html    
	// This function connects to the rosbridge server running on the local computer on port 9090
	var rbServer = new ROSLIB.Ros({
    url : 'ws://localhost:9090'
	});

    // This function is called upon the rosbridge connection event
    rbServer.on('connection', function() {
    // Write appropriate message to #feedback div when successfully connected to rosbridge
    var fbDiv = document.getElementById('feedback');
    fbDiv.innerHTML += "<p>Connected to websocket server.</p>";
    });

    // This function is called when there is an error attempting to connect to rosbridge
    rbServer.on('error', function(error) {
    // Write appropriate message to #feedback div upon error when attempting to connect to rosbridge
    var fbDiv = document.getElementById('feedback');
    fbDiv.innerHTML += "<p>Error connecting to websocket server.</p>";
    });

    // This function is called when the connection to rosbridge is closed
    rbServer.on('close', function() {
    // Write appropriate message to #feedback div upon closing connection to rosbridge
    var fbDiv = document.getElementById('feedback');
    fbDiv.innerHTML += "<p>Connection to websocket server closed.</p>";
    });
~~~  

However, we also tried to:

1. Put the HTML webpage online

2. Do the extension assignments


####Insights

####Difficulties We Faced
>>>>>>> 8901f44ec2004ab7dd53fdd73532b6a7abc19d92

####Mistakes We Found

####Useful Resources
* [`rosbridge page on ROS.org`](http://wiki.ros.org/rosbridge_suite)



=======
>>>>>>> 8901f44ec2004ab7dd53fdd73532b6a7abc19d92
