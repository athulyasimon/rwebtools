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

To replicate what we did: after running `roscore` and opening the `turtlesim` node with
    
    rosrun turtlesim turtlesim_node
    
we installed `rosbridge`:

    sudo apt-get install ros-indigo-rosbridge-suite
    
Then we launched `rosbridge` with

    roslaunch rosbridge_server rosbridge_websocket.launch
  
  
*2. The HTML webpage*
    
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


*3. Connecting to rosbridge*

Next, there is some documented code on connecting, disconnecting, or error in connecting to `rosbridge`. We first need to create a Ros node object to communicate with a rosbridge server. You can name the Ros node anything you want by replacing `rbserver` with the name of your node. This name will be used later on. In this example, the script will connect to localhost on the default port of 9090. If you would like other computers to be able to connect to your computer over a local network replace the localhost with your computer's internet address (example shown in the url code that is commented out).

~~~javascript    
    // This function connects to the rosbridge server running on the local computer on port 9090
    var rbServer = new ROSLIB.Ros({
    url : 'ws://localhost:9090' //this is to use your own computer
    //url : 'ws://192.168.0.104:9090' //this is for other people to connect to your computer, 192.168.0.104 was found from typing ifconfig into terminal
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


*4. Publishing messages to a topic in ROS through `rosbridge`*
    
Now we write the functions to create a topic and create a message to ROS. 

~~~javascript
// These lines create a topic object as defined by roslibjs
//topics for turtlesim can be found on http://wiki.ros.org/turtlesim
  var cmdVelTopic = new ROSLIB.Topic({
    ros : rbServer, //rbServer comes from the name of your ROS node
    name : '/turtle1/cmd_vel',
    messageType : 'geometry_msgs/Twist'
  });

  // These lines create a message that conforms to the structure of the Twist defined in our ROS installation
  // It initalizes all properties to zero. They will be set to appropriate values before we publish this message.
  var twist = new ROSLIB.Message({
    linear : {
      x : 0.0,
      y : 0.0,
      z : 0.0
    },
    angular : {
      x : 0.0,
      y : 0.0,
      z : 0.0
    }
  });
~~~


Now we write the function to take the numeric value for the Twist objects and publish them to the `cmd_vel` topic in ROS, controlling the `turtlesim`.

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

~~~javascript
    // Calling a service
    // -----------------
    //These lines create a service object as defined by roslibjs
    //services for turtlesim can be found on http://wiki.ros.org/turtlesim
    var clearBackground = new ROSLIB.Service({
    ros : rbServer,
    name : '/clear',
    serviceType : 'std_srvs/Empty'
    });

    var request = new ROSLIB.ServiceRequest({ //the clear service doesn't take in any parameters
    });
~~~


*6. Creating/setting parameters and calling a service in ROS*

Although `turtlesim`'s velocity and angular velocity can be changed through a topic message, the background color cannot. It is instead a parameter, which can be cleared and re-set. The following function takes the color that the webpage console specifies and creates the parameters that set the background color.

~~~javascript

  // Creating parameters
  // ---------------------------------
  //parameters for turtlesim can be found on http://wiki.ros.org/turtlesim
  var redRGBval = new ROSLIB.Param({
    ros : rbServer,
    name : 'background_r'
  });

  var greenRGBval = new ROSLIB.Param({
    ros : rbServer,
    name : 'background_g'
  });

  var blueRGBval = new ROSLIB.Param({
    ros : rbServer,
    name : 'background_b'
  });


  /* This function:
  - retrieves numeric values from the sliders
  - assigns these values to the appropriate parameters
  - calls the clear service to reset the background color
  */
  function backgroundMessage(){

    //getting the values from the sliders
    redcolorRGB = 0 + Number(document.getElementById('redRGB').value);
    greencolorRGB = 0 + Number(document.getElementById('greenRGB').value);
    bluecolorRGB = 0 + Number(document.getElementById('blueRGB').value);

      // Set parameters
    redRGBval.set(redcolorRGB);
    greenRGBval.set(greencolorRGB);
    blueRGBval.set(bluecolorRGB);
  
    //Publishes the values of the parameters to the webpage console
    redRGBval.get(function(value) {
    console.log('Red RGB VAL: ' + value);
    });

    greenRGBval.get(function(value) {
    console.log('Green RGB VAL: ' + value);
    });

    blueRGBval.get(function(value) {
    console.log('Blue RGB VAL: ' + value);
    });

    //Calls the clear service to clear the turtlesim background and set the color to the values of the background parameters
    clearBackground.callService(request, function(result) {
    console.log(result);
    }); 

  };

~~~

*7. Creating a control panel for the webpage*

Lastly, we create the control panel to write values for the `turtlesim` via our webpage that publishes its messages to our script, before we end the HTML. There are several components for the web console and they are listed below.

First, the following controls the linear and angular velocity of the turtle by user input of numbers. This mimics sending a message to the turtle with specific Twist values.

~~~HTML
<body>
<form name="ctrlPanel">
<p>Enter positive or negative numeric decimal values in the boxes below</p>
<table>
    <tr><td>Linear X</td><td><input id="linearXText" name="linearXText" type="text" value="1.5"/></td></tr>
    <tr><td>Angular Z</td><td><input id="angularZText" name="angularZText" type="text" value="1.5"/></td></tr>
</table>
<button id="sendMsg" type="button" onclick="pubMessage()">Publish Message</button>
</form>
<div id="feedback"></div>
</body>
</html>
~~~

Next, we wanted to control the turtle with arrow keys. We created visual arrow keys for the website as well as functionality for when the arrow keys on the keyboard are pressed. The following code controls the turtle. Forward and backward motion gives the turtle a spatial velocity of 1 and left/right turns the turtle counterclockwise/clockwise by π/2 (about 1.57) rad, numbers we just made up.

~~~javascript
/* These functions:
  - moves the turtle after the onscreen or keyboard arrow keys are pressed
   */
  function moveup() {  
    twist.linear.x = 1.0;
    twist.angular.z = 0.0;
    cmdVelTopic.publish(twist);      
  };

  function movedown() {  
    twist.linear.x = -1.0;
    twist.angular.z = 0.0;
    cmdVelTopic.publish(twist);      
  };

  function turnleft() {  
    twist.linear.x = 0.0;
    twist.angular.z = 1.570796;
    cmdVelTopic.publish(twist);      
  };

  function turnright() {  
    twist.linear.x = 0;
    twist.angular.z = -1.570796;
    cmdVelTopic.publish(twist);      
  };
~~~

Here is the code that detects whether the arrow keys are pressed, and controls the turtle through them.

~~~javascript
 /* This function:
  - detects when an arrow key is pressed down
  - calls the appropriate turn/move functions from above to move the turtle
  */
  document.onkeydown = function(e) {
    e = e || window.event;
    switch(e.which || e.keyCode) {
        case 37: turnleft()// left
        break;

        case 38: moveup()// up
        break;

        case 39: turnright()// right
        break;

        case 40: movedown()// down
        break;

        default: return; // exit this handler for other keys
    }
    e.preventDefault(); // prevent the default action (scroll / move caret)
  };
~~~


As we mentioned above, we wanted to allow control of the turtle through keyboard arrow keys or arrow key graphics on the webpage. Here we create the styles that controlled the size and position of the arrow key buttons.

~~~html
 	<style>
	 button.pos_left {
	   position: absolute;
	   left: 20px;
	   top: 350px;
	 }
	 button.pos_right{
	   position: absolute;
	   left: 75px;
	   top: 350px;
	 }
	 button.pos_up{
	   position: absolute;
	   left: 50px;
	   top: 325px;
	 }
	 button.pos_down{
	   position: absolute;
	   left: 50px;
	   top: 350px;
	 }
	
	</style>
~~~

The actual keys that use this functionality are coded by the following block of code. These can be placed anywhere in the HTML depending on where you want the buttons to be.

~~~html
	<p>
	  <button id="up" type="button" class = "pos_up" onclick="moveup()">&#8593;</button>
	  <button id="left" type="button" class = "pos_left" onclick="turnleft()">&#8592;</button>
	  <button id="down" type="button" class = "pos_down" onclick="movedown()">&#8595;</button>
	  <button id="right" type="button" class = "pos_right" onclick="turnright()">&#8594;</button>
	</p>
~~~


Finally, the following piece of code publishes a message to the webpage allowing you to know if the html file has properly connected to the Websocket server.

~~~html

<div id="feedback" class="Websocket"></div>

~~~

*8. Launch file*

Because this process needs several nodes running, to execute them easily, we created a .launch file which would automatically start `roscore`, WebSocket, `turtlesim_node`, and `turtle_teleop_key`. A user would need to run this launch file after opening the HTML webpage (or refresh the HTML page, if that was opened _before_ running the .launch file). The launch file is included in our repository.

*9. Extension - Remote Control of Turtlesim*

Our first task was to control `turtlesim` locally from an HTML page. For our second task, we wanted to give control of the 'turtlesim' on our local computer to anyone online. To test our idea, we put the HTML code on a free site builder, [rwebtools.weebly.com](rwebtools.weebly.com), which allowed us to control our running `turtlesim_node` (with WebSocket, `roscore` running) with our own computer.


Next, we tried running `turtlesim` over each other's computer via Ethernet. But we ran into some problems and this did not work (see **Difficulties We Faced** section).


Finally, we were able to give control of our local turtle to an online webpage by changing `localhost` from the line `url : 'ws://localhost:9090'` to our IP address. This seemed to work on a local network, although we ran into some strange behavior with Northwestern's network, which is also noticed in the **Difficulties We Faced** section. Both online control of the turtle on your local computer (assuming the appropriate nodes are running, e.g. from running the launch file) as well as control of Athulya's turtle from any computer with local network internet connection are on our website, [rwebtools.weebly.com](rwebtools.weebly.com).

####5. Difficulties We Faced

We tried to run `turtlesim` over each other's computer via Ethernet cable. However, this didn't work, and we ran into the following problems:

* Jackie's computer stopped being able to open `turtlesim_node` for some reason, giving the error 
> Couldn't find executable named turtlesim_node below /opt/ros/indigo/share/turtlesim

* Another classmate's laptop didn't work, giving an error that something was not defined in EventEmitter, which was the package that emitted "events" from the HTML back through WebSocket and `rosbridge`. This is probably regarding Ethernet connection.


Another problem we had was when we tried to control the turtle online remotely. In the code in the HTML, we changed `localhost` to our IP address. The result was that if we used our IP address over the Northwestern University network you could still connect to our computer even if you weren't on Northwestern's local network. But if you used the IP address connected to a local router then only computers on that local network were able to remotely control the turtle.  

####6. Mistakes We Found in Existing Code
None, though `rosbridge` was sparsely documented and had few tutorials. 

####7. Useful Resources
* [`rosbridge`](http://wiki.ros.org/rosbridge_suite) page on ROS.org
* [Iguanatronics turtlesim tutorial](http://iguanatronics.com/igtron/?p=313)
* [`roslibjs`](http://wiki.ros.org/roslibjs) page on ROS.org - has tutorials, including for URDF parser
* [Robot Web Tools](http://robotwebtools.org/)
* [Robot Management System](http://wiki.ros.org/rms) - we didn't use this for our project but it is something to explore in the future
* Apache is also something we would explore in the future
