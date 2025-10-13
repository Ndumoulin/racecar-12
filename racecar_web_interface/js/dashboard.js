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

    alert(
      "La connexion au serveur ROS a été perdue. Vous serez redirigé vers la page de connexion."
    );
    window.location.href = "login.html";
  });

  // Initialiser le topic pour les commandes de vitesse
  const velocityCmdTopic = new ROSLIB.Topic({
    ros: rosbridgeServer,
    name: "/ctl_ref",
    messageType: "geometry_msgs/Twist",
  });

  // Créer un message Twist
  const twist = new ROSLIB.Message({
    linear: { x: 0.0, y: 0.0, z: 7.0 },
    angular: { x: 0.0, y: 0.0, z: 0.0 },
  });

  // Variable pour stocker la vitesse actuelle
  let speedValue = 0.5; // Valeur par défaut (50%)

  // Récupérer l'élément du slider
  const speedRange = document.getElementById("speed_range");

  // Mettre à jour la vitesse lorsque le slider change
  speedRange.addEventListener("input", (event) => {
    speedValue = (parseFloat(event.target.value) / 100*2)+2; // Normaliser entre 0 et 1
    console.log(`Vitesse actuelle : ${speedValue}`);
  });

  //LE STOP
  function stopandlogout(){
    twist.linear.x = 0.0;
    twist.angular.z = 0.0;
    velocityCmdTopic.publish(twist);
    alert("Le robot a été arrêté. Vous allez être déconnecté.");
    rosbridgeServer.close();
    sessionStorage.removeItem("ros_ip");
    sessionStorage.removeItem("ros_username");
    window.location.href = "login.html";
  }

  const stopButton = document.getElementById("stop-button");
  stopButton.addEventListener("click", stopandlogout);

  // Fonction pour faire avancer le robot
  function avancer() {
    twist.linear.x = speedValue;
    twist.angular.z = 0.0;
  }

  function avancer_droite() {
    twist.linear.x = speedValue;
    twist.angular.z = -0.6;
  }

  function avancer_gauche() {
    twist.linear.x = speedValue;
    twist.angular.z = 0.6;
  }

  function reculer() {
    twist.linear.x = -speedValue;
    twist.angular.z = 0.0;
  }

  function reculer_droite() {
    twist.linear.x = -speedValue;
    twist.angular.z = -0.6;
  }

  function reculer_gauche() {
    twist.linear.x = -speedValue;
    twist.angular.z = 0.6;
  }

  // Fonction pour arrêter le robot
  function arreter() {
    twist.linear.x = 0.0;
    twist.angular.z = 0.0;
  }

  // Publication périodique à 5 Hz (200 ms)
  setInterval(() => {
    if (velocityCmdTopic != null) {
      velocityCmdTopic.publish(twist);
      console.log(`Publishing: linear.x=${twist.linear.x}, angular.z=${twist.angular.z}`);
    }
  }, 200);

  // Contrôle des boutons
  const arrowUpButton = document.getElementById("arrow-up");
  arrowUpButton.addEventListener("mousedown", avancer);
  arrowUpButton.addEventListener("mouseup", arreter);
  arrowUpButton.addEventListener("touchstart", avancer);
  arrowUpButton.addEventListener("touchend", arreter);

  const arrowRightButton = document.getElementById("arrow-up-right");
  arrowRightButton.addEventListener("mousedown", avancer_droite);
  arrowRightButton.addEventListener("mouseup", arreter);
  arrowRightButton.addEventListener("touchstart", avancer_droite);
  arrowRightButton.addEventListener("touchend", arreter);

  const arrowLeftButton = document.getElementById("arrow-up-left");
  arrowLeftButton.addEventListener("mousedown", avancer_gauche);
  arrowLeftButton.addEventListener("mouseup", arreter);
  arrowLeftButton.addEventListener("touchstart", avancer_gauche);
  arrowLeftButton.addEventListener("touchend", arreter);

  const arrowDownButton = document.getElementById("arrow-down");
  arrowDownButton.addEventListener("mousedown", reculer);
  arrowDownButton.addEventListener("mouseup", arreter);
  arrowDownButton.addEventListener("touchstart", reculer);
  arrowDownButton.addEventListener("touchend", arreter);

  const arrowDownRightButton = document.getElementById("arrow-down-right");
  arrowDownRightButton.addEventListener("mousedown", reculer_droite);
  arrowDownRightButton.addEventListener("mouseup", arreter);
  arrowDownRightButton.addEventListener("touchstart", reculer_droite);
  arrowDownRightButton.addEventListener("touchend", arreter);

  const arrowDownLeftButton = document.getElementById("arrow-down-left");
  arrowDownLeftButton.addEventListener("mousedown", reculer_gauche);
  arrowDownLeftButton.addEventListener("mouseup", arreter);
  arrowDownLeftButton.addEventListener("touchstart", reculer_gauche);
  arrowDownLeftButton.addEventListener("touchend", arreter);
});
