async function fetchTemp() {
    const response = await fetch("/get_temp");
    const data = await response.json();
    const temp = data.temperature;
    document.getElementById("temp-value").innerHTML = "Température : " + temp + " °C";
}

async function fetchHumidity() {
    const response = await fetch("/get_humidity");
    const data = await response.json();
    const humidity = data.humidity;
    document.getElementById("humidity-value").innerHTML = "Humidité : " + humidity + " %";
}

async function fetchLight() {
    const response = await fetch("/get_light");
    const data = await response.json();
    const light = data.light_level;
    document.getElementById("light-value").innerHTML = "Luminosité : " + light + " lux";
}

async function fetchButtonState() {
    const response = await fetch("/get_button_state");
    const data = await response.json();
    const buttonState = data.button_state ? "Actif" : "Inactif";
    document.getElementById("bp-status").innerHTML = "État du bouton : " + buttonState;
}

async function toggleButton() {
    const response = await fetch("/get_button_state");
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
setInterval(fetchHumidity, 1000);
setInterval(fetchLight, 1000);
setInterval(fetchButtonState, 1000);

// Event listeners
document.getElementById("bp-button").addEventListener("click", toggleButton);
document.getElementById("led-toggle").addEventListener("change", toggleLED);
document.getElementById("servo-submit").addEventListener("click", controlServo);

