$(document).ready(() => {
    const ip = sessionStorage.getItem("ros_ip");
    const username = sessionStorage.getItem("ros_username");

    const userInfoSpan = document.getElementById("userInfo");
    const statusBadge = document.getElementById("connectionStatus");

    if (!ip || !username) {
        alert("Informations de connexion manquantes. Retour à la page login.");
        window.location.href = "login.html";
        return;
    }

    userInfoSpan.textContent = `Utilisateur: ${username} | IP: ${ip}`;

    // Connect to ROS
    const rosbridgeServer = new ROSLIB.Ros({ url: "ws://" + ip + ":9090" });

    rosbridgeServer.on("connection", () => {
        console.log("Connected to ROS");
        statusBadge.textContent = "Connecté";
        statusBadge.className = "badge bg-success"; // green
    });

    rosbridgeServer.on("error", (error) => {
        console.error("Error connecting to ROS:", error);
        statusBadge.textContent = "Erreur";
        statusBadge.className = "badge bg-danger"; // red
    });

    rosbridgeServer.on("close", () => {
        console.warn("Connection closed.");
        statusBadge.textContent = "Déconnecté";
        statusBadge.className = "badge bg-secondary"; // gray

        // Show alert and redirect to login page
        alert("La connexion au serveur ROS a été perdue. Vous serez redirigé vers la page de connexion.");
        window.location.href = "login.html";
    });
});
