async function fetchTemp() {
    const response = await fetch("http://127.0.0.1:5000/temp");
    const data = await response.json();
    const temp = data.temperature;
    document.getElementById("temp-value").innerHTML = "Température : " + temp + " °C";
}

async function fetchServoAngle() {
    const response = await fetch("http://127.0.0.1:5000/servoAngle");
    const data = await response.json();
    const angle = data.servoAngle;
    document.getElementById("servo-value").innerHTML = "Angle actuel : " + angle + " °";
}


async function toggleButton() {
    const response = await fetch("http://127.0.0.1:5000/get_button_state");
    const data = await response.json();
    const newState = !data.button_state;

    const request = {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json'
        },
        body: JSON.stringify({ state: newState })
    };

    await fetch("/control_button", request);
    fetchButtonState();  // Mise à jour de l'état du bouton
}

async function toggleLED() {
    const ledState = document.getElementById("led-toggle").checked;

    const request = {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json'
        },
        body: JSON.stringify({ state: ledState })
    };

    await fetch("/control_led", request);
}

async function controlServo() {
    const angle = document.getElementById("servo-angle").value;

    const request = {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json'
        },
        body: JSON.stringify({ angle: angle })
    };

    await fetch("/control_servo", request);
}

// fetch les données toutes les secondes
setInterval(fetchTemp, 1000);
setInterval(fetchServoAngle, 1000);

// Event listeners
document.getElementById("led-toggle").addEventListener("change", toggleLED);
document.getElementById("servo-submit").addEventListener("click", controlServo);

