// Common utilities and ROS helpers used by login and dashboard pages
// Safe helpers to get elements if present
function $(id) { return document.getElementById(id); }

// Global ROS/joystick-related variables
var rosbridgeServer = null;
var velocityCmdTopic = null;
var rosMasterIp = null;

// Create a message that conforms to the `Twist` structure defined in ROS.
var twist = null;
try {
    twist = new ROSLIB.Message({ linear: { x: 0.0, y: 0.0, z: 0.0 }, angular: { x: 0.0, y: 0.0, z: 0.0 } });
} catch (e) {
    // roslib may not be loaded yet; twist will be created later when needed
    twist = null;
}

function updatenavbar() {
    var navbar = $('navbar');
    if (!navbar) return;
    var username = localStorage.getItem('racecar_username') || '';
    var ip = localStorage.getItem('racecar_ip') || '';
    if (username || ip) {
        navbar.innerText = username + (username && ip ? ' | ' : '') + ip;
    }
}

function connectROS() {
    if (!rosMasterIp) {
        rosMasterIp = localStorage.getItem('racecar_ip');
        if (!rosMasterIp) return console.warn('No ros IP available to connect');
    }

    try {
        rosbridgeServer = new ROSLIB.Ros({ url: 'ws://' + rosMasterIp + ':9090' });
    } catch (e) {
        console.error('Failed to create ROSLIB.Ros', e);
        return;
    }

    rosbridgeServer.on('connection', function() {
        console.log('Connected to WebSocket server.');
        // Create a topic object to send propulsion commands to the racecar
        velocityCmdTopic = new ROSLIB.Topic({ ros: rosbridgeServer, name: '/prop_cmd', messageType: 'geometry_msgs/Twist' });
    });

    rosbridgeServer.on('error', function(error) { console.log('Error connecting to WebSocket server: ', error); });
    rosbridgeServer.on('close', function() { console.log('Closed connection to WebSocket server.'); });

    // Ensure twist exists
    if (!twist) {
        try {
            twist = new ROSLIB.Message({ linear: { x: 0.0, y: 0.0, z: 0.0 }, angular: { x: 0.0, y: 0.0, z: 0.0 } });
        } catch (e) { twist = null; }
    }

    // Start publisher loop if ROS topic available
    if (twist && !window._racecar_publisher_started) {
        window._racecar_publisher_started = true;
        setInterval(function() {
            if (velocityCmdTopic != null && twist != null) {
                velocityCmdTopic.publish(twist);
            }
        }, 200);
    }
}

// Expose connectROS/updatenavbar to other scripts
window.racecar = window.racecar || {};
window.racecar.connectROS = connectROS;
window.racecar.updatenavbar = updatenavbar;
