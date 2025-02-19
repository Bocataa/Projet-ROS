async function toggleButton() {
    // Envoie la requête pour inverser l'angle du servo
    // On doit utiliser une methode, ici on utilise POST
    const request = {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json'
        }
    };
    await fetch("http://127.0.0.1:5000/control_button", request);  // Envoie au serveur Flask pour appeler ROS2
    fetchServoAngle();  // Mise à jour immédiate de l'angle du servo car il change lors de l'appui
}

async function fetchTemp() {
    const response = await fetch("http://127.0.0.1:5000/temp"); // adresse local pour le moment à modifier plus tard
    const data = await response.json();
    const temp = data.temperature;
    if (temp >= 23){
        etat_led = "ON";
    }
    else{
        etat_led = "OFF";
    }
    document.getElementById("temp-value").innerHTML = "Température : " + temp + " °C";
    document.getElementById("led-state").innerText = "Etat led : " + etat_led;
}

async function fetchServoAngle() {
    const response = await fetch("http://127.0.0.1:5000/servo");
    const data = await response.json();
    const angle = data.servo_angle;
    document.getElementById("servo-value").innerHTML = "Angle actuel : " + angle + " °";
}

// Récupère les données toutes les secondes
setInterval(fetchTemp, 1000);
setInterval(fetchServoAngle, 1000);

// Event listener pour le bouton
document.getElementById("bp-button").addEventListener("click", toggleButton);
