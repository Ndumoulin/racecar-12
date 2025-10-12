// Define some global variables
var rosbridgeServer = null;
var velocityCmdTopic = null;


$(document).ready(() => {
    const form = document.getElementById("loginForm");
    const statusMessage = document.getElementById("statusMessage");

    if (!form) {
        console.error("Form not found in DOM.");
        return;
    }

    form.addEventListener("submit", function (event) {
        event.preventDefault(); // Prevent page refresh

        const username = document.getElementById("username").value.trim();
        const ipAddress = document.getElementById("ipAddress").value.trim();

        // IPv4 validation regex (0–255.0–255.0–255.0–255)
        const ipRegex = /^(25[0-5]|2[0-4]\d|[0-1]?\d?\d)(\.(25[0-5]|2[0-4]\d|[0-1]?\d?\d)){3}$/;

        if (!ipRegex.test(ipAddress)) {
            statusMessage.innerHTML = `<span class="text-danger">Adresse IP invalide.</span>`;
            return;
        }

            statusMessage.innerHTML = `<span class="text-warning">Trying to connect to server ... </span>`;

        // Connect to ROS bridge
        connectROS(username, ipAddress , statusMessage);
    });
});

// rosbridge / roslibjs function to connect to ROS
function connectROS(username, ipAddress, statusMessage) {
    // Connect to the rosbridge server running on localhost, on port 9090
    // HINT: The rosbridge server SHOULD be closed when disconnecting from ROS

    rosbridgeServer = new ROSLIB.Ros({ url: "ws://" + ipAddress + ":9090" });

    rosbridgeServer.on("connection", () => {
        console.log("Connected to WebSocket server.");

        sessionStorage.setItem("ros_ip", ipAddress);
        sessionStorage.setItem("ros_username", username);

        window.location.href = 'dashboard.html';

        // Create a topic object to send propulsion commands to the racecar
        velocityCmdTopic = new ROSLIB.Topic(
            { ros: rosbridgeServer, name: "/prop_cmd", messageType: "geometry_msgs/Twist" });
    });

    rosbridgeServer.on(
        "error", (error) => {
            console.log("Error connecting to WebSocket server: ", error);
            statusMessage.innerHTML = `<span class="text-danger">Impossible to connect to server </span>`;

        });

    rosbridgeServer.on("close", () => {
        console.log("Closed connection to WebSocket server.");
    });
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
