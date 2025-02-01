async function fetchData() {
    // Effectuer une requête GET à l'API pour récupérer les données des capteurs
    const reponse = await fetch("http://192.168.1.25:3000/"); // L'adresse de ton serveur Flask
    const valeur_JSON = await reponse.json();

    // Extraire les données
    let temperature = valeur_JSON[0].temperature;
    let humidite = valeur_JSON[0].humidity;
    let luminosite = valeur_JSON[0].light_level;

    console.log(valeur_JSON); // DEBUG

    // Mise à jour des éléments dans le DOM
    document.getElementById('temp-value').innerHTML = "Température : " + temperature + " °C";
    document.getElementById('humidity-value').innerHTML = "Humidité : " + humidite + " %";
    document.getElementById('light-value').innerHTML = "Luminosité : " + luminosite + " lux";
}

// Récupérer les données toutes les 5 secondes
setInterval(fetchData, 5000);

// Contrôle LED
document.getElementById('led-toggle').addEventListener('change', function() {
    const ledState = this.checked;
    fetch('http://192.168.1.25:3000/led', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json'
        },
        body: JSON.stringify({ ledState: ledState })
    });
});

// Contrôle du servo
document.getElementById('servo-submit').addEventListener('click', function() {
    const angle = document.getElementById('servo-angle').value;
    fetch('http://192.168.1.25:3000/servo', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json'
        },
        body: JSON.stringify({ angle: angle })
    });
});
