document.addEventListener("DOMContentLoaded", () => {
    const connectButton = document.getElementById("connectButton");
    connectButton.addEventListener("click", connectToRos);
});

function connectToRos() {
    const ip = document.getElementById("ipAddressInput").value.trim();
    const statusMessage = document.getElementById("statusMessage");

    // IPv4 pattern (each octet 0-255)
    const ipPattern = /^(25[0-5]|2[0-4]\d|1\d\d|[1-9]?\d)\.(25[0-5]|2[0-4]\d|1\d\d|[1-9]?\d)\.(25[0-5]|2[0-4]\d|1\d\d|[1-9]?\d)\.(25[0-5]|2[0-4]\d|1\d\d|[1-9]?\d)$/;

    if (!ip) {
        statusMessage.textContent = "Please enter an IP address.";
        statusMessage.className = "status error";
        return;
    }

    if (!ipPattern.test(ip)) {
        statusMessage.textContent = "Please enter a valid IP address.";
        statusMessage.className = "status error";
        return;
    }

    // If valid, proceed to connect
    statusMessage.textContent = "Connecting...";
    statusMessage.className = "status";

    const ros = new ROSLIB.Ros({ url: "ws://" + ip + ":9090" });


    ros.on("connection", function() {
        statusMessage.textContent = "Connected to ROS";
        statusMessage.className = "status success";

        window.location.href = "index.html"; 

    });

    ros.on("error", function(error) {
        statusMessage.textContent = "Connection error. Check the IP.";
        statusMessage.className = "status error";
    });

    ros.on("close", function() {
        statusMessage.textContent = "Connection failed.";
        statusMessage.className = "status error";
    });
}

