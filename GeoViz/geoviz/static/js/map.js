// Wait for the DOM to be fully loaded before initializing the map
document.addEventListener('DOMContentLoaded', function() {
    // Initialize map with center coordinates and appropriate zoom level
    var map = L.map('map').setView([39.941326, -75.199492], 16);

    // Try different tile providers - one of these should work
    // 1. Mapbox Satellite
    var mapboxDetailed = L.tileLayer('https://api.mapbox.com/styles/v1/{id}/tiles/{z}/{x}/{y}?access_token={accessToken}', {
        attribution: '© <a href="https://www.mapbox.com/about/maps/">Mapbox</a>',
        maxZoom: 30,
        id: 'mapbox/satellite-v9',
        accessToken: window.MAPBOX_TOKEN 
    }).addTo(map);
    
    // 2. OpenStreetMap as fallback
    var osm = L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
        maxZoom: 19,
        attribution: '© OpenStreetMap contributors'
    });

    var baseLayers = {
        "MapBox Satellite": mapboxDetailed,
        "OpenStreetMap": osm
    };

    L.control.layers(baseLayers).addTo(map);

    // Add a console log to check if the map initialized
    console.log("Map initialized with center:", map.getCenter(), "zoom:", map.getZoom());

    // Create layers for your data
    var pointsLayer = L.layerGroup().addTo(map);
    var pathLayer = L.layerGroup().addTo(map);
    var objectsLayer = L.layerGroup().addTo(map);
    var connectionsLayer = L.layerGroup().addTo(map);

    // Robot tracking variables
    var jackalMarkers = {}; // Store current jackal markers
    var droneMarker = null; // Store current drone marker
    var casualtyMarkers = {}; // Store casualty markers (persistent)
    var robotHealthData = {}; // Store health data for each robot
    
    var jackalColors = {
        'phobos': '#28B463',    // Green for Phobos
        'deimos': '#2980B9',     // Blue for Deimos
        'oberon': '#263238',
        'titania': '#FDD835',
    };
    var droneColor = '#FF692A'; // Orange for Dione drone
    var casualtyColor = '#FF3333'; // Red for casualties

    // Create custom marker icon for jackals
    function createJackalIcon(robotName) {
        var color = jackalColors[robotName.toLowerCase()];
        var letter = robotName.charAt(0).toUpperCase();
        
        return L.divIcon({
            className: 'jackal-marker',
            html: `<div style="
                background-color: ${color};
                width: 20px;
                height: 20px;
                border-radius: 50%;
                border: 3px solid white;
                box-shadow: 0 0 10px rgba(0,0,0,0.5);
                display: flex;
                align-items: center;
                justify-content: center;
                font-weight: bold;
                font-size: 10px;
                color: white;
            ">${letter}</div>`,
            iconSize: [26, 26],
            iconAnchor: [13, 13],
            popupAnchor: [0, -13]
        });
    }

    // Create custom marker icon for drone (different shape)
    function createDroneIcon() {
        return L.divIcon({
            className: 'drone-marker',
            html: `<div style="
                background-color: ${droneColor};
                width: 20px;
                height: 20px;
                border-radius: 3px;
                border: 3px solid white;
                box-shadow: 0 0 10px rgba(0,0,0,0.5);
                display: flex;
                align-items: center;
                justify-content: center;
                font-weight: bold;
                font-size: 10px;
                color: white;
                transform: rotate(45deg);
            "><div style="transform: rotate(-45deg);">D</div></div>`,
            iconSize: [26, 26],
            iconAnchor: [13, 13],
            popupAnchor: [0, -13]
        });
    }

    // Create custom marker icon for casualties (triangle/warning shape)
    function createCasualtyIcon() {
        return L.divIcon({
            className: 'casualty-marker',
            html: `<div style="
                width: 0;
                height: 0;
                border-left: 12px solid transparent;
                border-right: 12px solid transparent;
                border-bottom: 20px solid ${casualtyColor};
                filter: drop-shadow(0 0 5px rgba(0,0,0,0.5));
                position: relative;
            "><div style="
                position: absolute;
                top: 6px;
                left: -6px;
                width: 12px;
                height: 12px;
                color: white;
                font-weight: bold;
                font-size: 12px;
                text-align: center;
                line-height: 12px;
            ">!</div></div>`,
            iconSize: [24, 24],
            iconAnchor: [12, 24],
            popupAnchor: [0, -24]
        });
    }

    // Function to update jackal position
    function updateJackalPosition(robotName, lat, lon, popup) {
        // Remove existing marker if it exists
        if (jackalMarkers[robotName]) {
            objectsLayer.removeLayer(jackalMarkers[robotName]);
        }

        // Create new marker
        var icon = createJackalIcon(robotName);
        jackalMarkers[robotName] = L.marker([lat, lon], {icon: icon})
            .bindPopup(`<b>${popup}</b><br>Lat: ${lat.toFixed(6)}<br>Lon: ${lon.toFixed(6)}<br>Last Update: ${new Date().toLocaleTimeString()}`)
            .addTo(objectsLayer);

        console.log(`Updated ${robotName} position: ${lat}, ${lon}`);
    }

    // Function to update drone position
    function updateDronePosition(lat, lon, popup) {
        // Remove existing marker if it exists
        if (droneMarker) {
            objectsLayer.removeLayer(droneMarker);
        }

        // Create new marker
        var icon = createDroneIcon();
        droneMarker = L.marker([lat, lon], {icon: icon})
            .bindPopup(`<b>${popup}</b><br>Lat: ${lat.toFixed(6)}<br>Lon: ${lon.toFixed(6)}<br>Last Update: ${new Date().toLocaleTimeString()}`)
            .addTo(objectsLayer);

        console.log(`Updated Dione drone position: ${lat}, ${lon}`);
    }

    // Function to add casualty (persistent marker)
    function addCasualty(casualtyId, lat, lon, popup) {
        // Check if casualty already exists
        if (casualtyMarkers[casualtyId]) {
            console.log(`Casualty ${casualtyId} already exists on map`);
            return;
        }

        // Create new casualty marker
        var icon = createCasualtyIcon();
        casualtyMarkers[casualtyId] = L.marker([lat, lon], {icon: icon})
            .bindPopup(`<b>${popup}</b><br>Casualty ID: ${casualtyId}<br>Lat: ${lat.toFixed(6)}<br>Lon: ${lon.toFixed(6)}<br>Detected: ${new Date().toLocaleTimeString()}`)
            .addTo(objectsLayer);

        console.log(`Added casualty ${casualtyId} at position: ${lat}, ${lon}`);
    }

    // Function to create or update robot health status card
    function updateRobotHealth(healthData) {
        var robotName = healthData.robot_name.toLowerCase();
        var container = document.getElementById('health-status-container');
        var existingCard = document.getElementById('health-card-' + robotName);
        
        // Store the health data
        robotHealthData[robotName] = healthData;
        
        if (!existingCard) {
            // Create new health card
            var cardHtml = `
                <div class="robot-health-card" id="health-card-${robotName}">
                    <div class="robot-name">${robotName}</div>
                    <div class="sensor-grid" id="sensor-grid-${robotName}">
                        <!-- Sensors will be added here -->
                    </div>
                    <div class="last-update" id="last-update-${robotName}">
                        Last Update: ${new Date().toLocaleTimeString()}
                    </div>
                </div>
            `;
            container.insertAdjacentHTML('beforeend', cardHtml);
            existingCard = document.getElementById('health-card-' + robotName);
        }
        
        // Update sensor grid
        var sensorGrid = document.getElementById('sensor-grid-' + robotName);
        var sensorHtml = '';
        
        // Iterate through all sensors in the health data (excluding robot_name)
        Object.keys(healthData).forEach(function(key) {
            if (key !== 'robot_name') {
                var value = healthData[key];
                var statusClass = value ? 'healthy' : 'unhealthy';
                var statusText = value ? 'TRUE' : 'FALSE';
                
                sensorHtml += `
                    <div class="sensor-status">
                        <span class="sensor-name">${key}</span>
                        <span class="sensor-value ${statusClass}">${statusText}</span>
                    </div>
                `;
            }
        });
        
        sensorGrid.innerHTML = sensorHtml;
        
        // Update timestamp
        document.getElementById('last-update-' + robotName).textContent = 
            'Last Update: ' + new Date().toLocaleTimeString();
        
        console.log(`Updated health status for ${robotName}:`, healthData);
    }

    // Create a legend
    var legend = L.control({position: 'bottomright'});

    legend.onAdd = function (map) {
        var div = L.DomUtil.create('div', 'info legend');
        div.style.backgroundColor = 'rgba(255, 255, 255, 0.9)';
        div.style.padding = '10px';
        div.style.borderRadius = '5px';
        div.style.boxShadow = '0 0 15px rgba(0,0,0,0.2)';
        div.style.fontSize = '12px';
        
        // Add title
        div.innerHTML = '<h4 style="margin-top: 0; margin-bottom: 10px; text-align: center;">Robot Tracker</h4>';
        
        // Define legend items - for Jackals, Drone, and Casualties
        var items = [
            [jackalColors.phobos, 'Phobos'],
            [jackalColors.deimos, 'Deimos'],
            [jackalColors.oberon, 'Oberon'],
            [jackalColors.titania, 'Titania'],
            [droneColor, 'Dione'],
            [casualtyColor, 'Casualty']
        ];
        
        // Add each legend item
        for (var i = 0; i < items.length; i++) {
            div.innerHTML += 
                '<div style="display: flex; align-items: center; margin-bottom: 5px;">' +
                    '<span style="background:' + items[i][0] + '; width: 15px; height: 15px; border-radius: 50%; display: inline-block; margin-right: 8px; border: 2px solid #FFF; box-shadow: 0 0 3px rgba(0,0,0,0.3);"></span> ' +
                    '<span>' + items[i][1] + '</span>' +
                '</div>';
        }

        // Add controls section
        div.innerHTML += '<hr style="margin: 10px 0; border: none; border-top: 1px solid #ccc;">';
        div.innerHTML += '<div style="font-size: 10px; color: #666;">Click markers for details</div>';
        
        return div;
    };

    // Add legend to map
    legend.addTo(map);

    // Connect to WebSocket
    var socket = io();

    // Handle connection events
    socket.on('connect', function() {
        console.log('Connected to server via WebSocket');
    });

    socket.on('disconnect', function() {
        console.log('Disconnected from server');
    });

    // Handle jackal updates
    socket.on('jackal_update', function(data) {
        console.log('Received jackal update:', data);
        
        // Handle the data structure from your Python code
        // The data comes as a set with one dictionary
        if (data && typeof data === 'object') {
            // Convert set to array if needed, or access the first item
            var pointData;
            if (Array.isArray(data)) {
                pointData = data[0];
            } else if (data.lat !== undefined) {
                pointData = data;
            } else {
                // Handle case where data is a set-like object
                var values = Object.values(data);
                pointData = values[0];
            }
            
            if (pointData && pointData.lat && pointData.lon) {
                // Default to 'phobos' if no popup specified, or extract robot name from popup
                var robotName = 'phobos'; // default
                if (pointData.popup) {
                    var popup = pointData.popup.toLowerCase();
                    if (popup.includes('deimos')) {
                        robotName = 'deimos';
                    } else if (popup.includes('oberon')) {
                        robotName = 'oberon';
                    } else if (popup.includes('titania')) {
                        robotName = 'titania';
                    }
                }
                updateJackalPosition(robotName, pointData.lat, pointData.lon, pointData.popup || 'Phobos');
            }
        }
    });

    // Handle drone updates
    socket.on('drone_update', function(data) {
        console.log('Received drone update:', data);
        
        if (data && typeof data === 'object') {
            var pointData;
            if (Array.isArray(data)) {
                pointData = data[0];
            } else if (data.lat !== undefined) {
                pointData = data;
            } else {
                var values = Object.values(data);
                pointData = values[0];
            }
            
            if (pointData && pointData.lat && pointData.lon) {
                updateDronePosition(pointData.lat, pointData.lon, pointData.popup || 'Dione');
            }
        }
    });

    // Handle casualty updates
    socket.on('casualty_update', function(data) {
        console.log('Received casualty update:', data);
        
        if (data && typeof data === 'object') {
            var pointData;
            if (Array.isArray(data)) {
                pointData = data[0];
            } else if (data.lat !== undefined) {
                pointData = data;
            } else {
                var values = Object.values(data);
                pointData = values[0];
            }
            
            if (pointData && pointData.lat && pointData.lon) {
                // Extract casualty ID from popup or use a default
                var casualtyId = pointData.casualty_id || 'unknown';
                if (pointData.popup && pointData.popup.includes('Casualty:')) {
                    // Extract ID from popup like "Casualty: 001"
                    var match = pointData.popup.match(/Casualty:\s*(\w+)/);
                    if (match) {
                        casualtyId = match[1];
                    }
                }
                
                addCasualty(casualtyId, pointData.lat, pointData.lon, pointData.popup || 'Casualty');
            }
        }
    });

    // Handle health status updates
    socket.on('health_update', function(data) {
        console.log('Received health update:', data);
        
        if (data && typeof data === 'object' && data.robot_name) {
            updateRobotHealth(data);
        } else {
            console.error('Invalid health update data:', data);
        }
    });

    // Add layer control for toggling markers only
    var overlayLayers = {
        "Robot Markers": objectsLayer,
        "Additional Points": pointsLayer
    };

    // Update the layer control to include overlays
    L.control.layers(baseLayers, overlayLayers).addTo(map);

    // Force a map refresh
    setTimeout(function() {
        map.invalidateSize();
    }, 100);

    // Add utility functions for testing
    window.addTestJackal = function(robotName, lat, lon) {
        robotName = robotName || 'phobos';
        lat = lat || 39.941326;
        lon = lon || -75.199492;
        updateJackalPosition(robotName, lat, lon, 'Test ' + robotName.charAt(0).toUpperCase() + robotName.slice(1));
    };

    window.addTestDrone = function(lat, lon) {
        lat = lat || 39.941326;
        lon = lon || -75.199492;
        updateDronePosition(lat, lon, 'Test Dione');
    };

    window.addTestCasualty = function(casualtyId, lat, lon) {
        casualtyId = casualtyId || 'TEST-' + Math.floor(Math.random() * 1000);
        lat = lat || 39.941326;
        lon = lon || -75.199492;
        addCasualty(casualtyId, lat, lon, 'Test Casualty: ' + casualtyId);
    };

    window.addTestHealth = function(robotName, sensors) {
        robotName = robotName || 'phobos';
        sensors = sensors || {rgb: true, thermal: false, gps: true, rtk: false};
        var healthData = Object.assign({robot_name: robotName}, sensors);
        updateRobotHealth(healthData);
    };

    window.clearAllCasualties = function() {
        Object.keys(casualtyMarkers).forEach(function(casualtyId) {
            objectsLayer.removeLayer(casualtyMarkers[casualtyId]);
        });
        casualtyMarkers = {};
        console.log('Cleared all casualties');
    };

    console.log('Map.js loaded successfully. Use addTestJackal(), addTestDrone(), addTestCasualty(), or addTestHealth() to test.');
});
