// Define some global variables
var rosbridgeServer = null;
var velocityCmdTopic = null;

// Safe helpers to get elements if present
function $(id) { return document.getElementById(id); }

// Calls added inside the anynonymous function are triggered after the page is loaded
document.addEventListener('DOMContentLoaded', function() {
    // Setup login page behavior if elements exist
    var btnsubmit = $('submit-button');
    var statusbox = $('status-block');
    var ipInput = $('ip-adress');
    var usernameInput = $('username');

    if (btnsubmit && ipInput && usernameInput) {
        btnsubmit.addEventListener('click', function(event) {
            event.preventDefault();
            var ipadress = ipInput.value.trim();
            var username = usernameInput.value.trim();

            const ipRegex = /^(25[0-5]|2[0-4]\d|1\d{2}|[1-9]?\d)(\.(25[0-5]|2[0-4]\d|1\d{2}|[1-9]?\d)){3}$/;

            if (!ipRegex.test(ipadress)) {
                if (statusbox) statusbox.innerText = "Adresse IP invalide — format attendu: x.x.x.x (0-255)\n";
                return;
            }

            // Persist login info locally so other pages can read it
            try {
                localStorage.setItem('racecar_username', username);
                localStorage.setItem('racecar_ip', ipadress);
            } catch (e) {
                console.warn('localStorage unavailable', e);
            }

            // Save IP for connectROS() and attempt connection
            rosMasterIp = ipadress;
            if (statusbox) statusbox.innerText = "Connexion à " + ipadress + "...\n";
            connectROS();
        });
    }

    updatenavbar();
});

function updatenavbar() {
    var navbar = $('navbar');
    if (!navbar) return;
    var username = localStorage.getItem('racecar_username') || '';
    var ip = localStorage.getItem('racecar_ip') || '';
    if (username || ip) {
        navbar.innerText = "Welcome: " + username + (username && ip ? '@' : '') + ip;
    }
}

// rosbridge / roslibjs function to connect to ROS
function connectROS() {
    // Connect to the rosbridge server running on localhost, on port 9090
    // HINT: The rosbridge server SHOULD be closed when disconnecting from ROS
    rosbridgeServer = new ROSLIB.Ros({ url: "ws://" + rosMasterIp + ":9090" });

    rosbridgeServer.on("connection", () => {
        console.log("Connected to WebSocket server.");

    window.location.href= "dashboard.html";

        // Create a topic object to send propulsion commands to the racecar
        velocityCmdTopic = new ROSLIB.Topic(
            { ros: rosbridgeServer, name: "/prop_cmd", messageType: "geometry_msgs/Twist" });
    });

    rosbridgeServer.on(
        "error", (error) => { console.log("Error connecting to WebSocket server: ", error); });

    rosbridgeServer.on("close", () => { console.log("Closed connection to WebSocket server."); });
}

// Create a message that conforms to the `Twist` structure defined in ROS.
var twist = new ROSLIB.Message(
    { linear: { x: 0.0, y: 0.0, z: 2.0 }, angular: { x: 0.0, y: 0.0, z: 0.0 } });

// Add timer callbacks here
setInterval(() => {
    if (velocityCmdTopic != null) {
        // HINT: The `twist` object's values SHOULD be updated BEFORE publishing the message.
        // Publish the message
        velocityCmdTopic.publish(twist);
    }
}, 200);

