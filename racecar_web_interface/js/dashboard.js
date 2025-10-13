$(document).ready(() => {
    const ip = sessionStorage.getItem("ros_ip");
    const username = sessionStorage.getItem("ros_username");

    const userInfoSpan = document.getElementById("userInfo");
    const statusBadge = document.getElementById("connectionStatus");

    // Vérifiez si les informations de connexion sont disponibles
    if (!ip || !username) {
        alert("Informations de connexion manquantes. Retour à la page login.");
        window.location.href = "login.html";
        return;
    }

    userInfoSpan.textContent = `Utilisateur: ${username} | IP: ${ip}`;

    // Connecter au serveur ROS
    const rosbridgeServer = new ROSLIB.Ros({ url: "ws://" + ip + ":9090" });

    rosbridgeServer.on("connection", () => {
        console.log("Connected to ROS");
        statusBadge.textContent = "Connecté";
        statusBadge.className = "badge bg-success"; // Vert
    });

    rosbridgeServer.on("error", (error) => {
        console.error("Error connecting to ROS:", error);
        statusBadge.textContent = "Erreur";
        statusBadge.className = "badge bg-danger"; // Rouge
    });

    rosbridgeServer.on("close", () => {
        console.warn("Connection closed.");
        statusBadge.textContent = "Déconnecté";
        statusBadge.className = "badge bg-secondary"; // Gris

        // Rediriger vers la page de connexion
        alert("La connexion au serveur ROS a été perdue. Vous serez redirigé vers la page de connexion.");
        window.location.href = "login.html";
    });

    // Initialiser le topic pour les commandes de vitesse
    const velocityCmdTopic = new ROSLIB.Topic({
        ros: rosbridgeServer,
        name: '/cmd_vel', // Nom du topic ROS
        messageType: 'geometry_msgs/Twist' // Type de message
    });

    // Initialiser le joystick
    let joystick = null;
    const joystickContainer = document.getElementById('joystick');
    if (joystickContainer) {
        joystick = new VirtualJoystick({
            container: joystickContainer,
            mouseSupport: true, // Permet d'utiliser la souris pour tester
            stationaryBase: true, // Le joystick reste fixe
            baseX: 150, // Position X de la base
            baseY: 150, // Position Y de la base
            limitStickTravel: true, // Limite le déplacement du joystick
            stickRadius: 100 // Rayon maximal du joystick
        });
        console.log("Joystick initialized.");
    } else {
        console.error("Joystick container not found.");
    }

    // Créer un message Twist
    const twist = new ROSLIB.Message({
        linear: { x: 0.0, y: 0.0, z: 0.0 },
        angular: { x: 0.0, y: 0.0, z: 0.0 }
    });

    // Écouter les mouvements du joystick et publier les commandes
    setInterval(() => {
        if (joystick && velocityCmdTopic != null) {
            const deltaX = joystick.deltaX(); // Déplacement horizontal
            const deltaY = joystick.deltaY(); // Déplacement vertical

            // Mettre à jour les valeurs du message Twist
            twist.linear.x = deltaY / 100; // Normaliser deltaY
            twist.angular.z = -deltaX / 100; // Normaliser deltaX

            // Publier le message
            velocityCmdTopic.publish(twist);
            console.log(`Command sent: linear.x=${twist.linear.x}, angular.z=${twist.angular.z}`);
        }
    }, 200); // Publier toutes les 200 ms
});

