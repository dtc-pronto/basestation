// Wait for the DOM to be fully loaded before initializing the map
document.addEventListener('DOMContentLoaded', function() {
    // Initialize map with center coordinates and appropriate zoom level
    var map = L.map('map').setView([32.502656, -83.752355], 16);

    // Simple: if OFFLINE_MODE is true, use local tiles. Otherwise use online tiles.
    var tileLayer;
    
    //if (window.OFFLINE_MODE) {
    // Use local tiles served by Flask
    tileLayer = L.tileLayer('/tiles/{z}/{x}/{y}.png', {
        attribution: 'Offline Satellite Tiles',
        maxZoom: 20,
        minZoom: 10
    }).addTo(map);
    console.log("Using offline tiles");
    //} else {
    //    // Use online Mapbox tiles
    //    tileLayer = L.tileLayer('https://api.mapbox.com/styles/v1/{id}/tiles/{z}/{x}/{y}?access_token={accessToken}', {
    //        attribution: '© <a href="https://www.mapbox.com/about/maps/">Mapbox</a>',
    //        maxZoom: 30,
    //        id: 'mapbox/satellite-v9',
    //        accessToken: window.MAPBOX_TOKEN 
    //    }).addTo(map);
    //    console.log("Using online Mapbox tiles");
    //}

    // Fallback OpenStreetMap
    var osm = L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
        maxZoom: 19,
        attribution: '© OpenStreetMap contributors'
    });

    // Create base layers object
    var baseLayers = {};
    //if (window.OFFLINE_MODE) {
    baseLayers["Offline Tiles"] = tileLayer;
    baseLayers["OpenStreetMap"] = osm;
    //} else {
    //    baseLayers["Mapbox Satellite"] = tileLayer;
    //    baseLayers["OpenStreetMap"] = osm;
    //}

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
    var robotRSSIData = {}; // Store RSSI data for each robot
    var serverReports = []; // Store server reports
    
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

    // Function to get RSSI color based on signal strength
    function getRSSIColor(rssi) {
        if (rssi > 34) return '#27ae60'; // Green - Excellent
        if (rssi > 25) return '#2ecc71'; // Light Green - Good  
        if (rssi > 15) return '#f39c12'; // Orange - Fair
        if (rssi > 0) return '#e67e22'; // Dark Orange - Poor
        return '#e74c3c'; // Red - Very Poor
    }

    // Function to get RSSI description
    function getRSSIDescription(rssi) {
        if (rssi > 34) return 'Excellent';
        if (rssi > 25) return 'Good';
        if (rssi > 15) return 'Fair';
        if (rssi > 0) return 'Poor';
        return 'Very Poor';
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
                    <div class="robot-header" style="display: flex; justify-content: space-between; align-items: center; margin-bottom: 12px;">
                        <div class="robot-name">${robotName}</div>
                        <div class="rssi-indicator" id="rssi-${robotName}" style="
                            padding: 3px 8px;
                            border-radius: 4px;
                            font-size: 10px;
                            font-weight: bold;
                            color: white;
                            background-color: #95a5a6;
                            min-width: 50px;
                            text-align: center;
                        ">NO RSSI</div>
                    </div>
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

    // Function to handle RSSI updates
    function updateRSSI(rssiData) {
        var robotName = rssiData.robot_name.toLowerCase();
        var rssi = rssiData.rssi;
        
        // Store RSSI data
        robotRSSIData[robotName] = rssiData;
        
        // Update RSSI indicator if health card exists
        var rssiIndicator = document.getElementById('rssi-' + robotName);
        if (rssiIndicator) {
            var color = getRSSIColor(rssi);
            var description = getRSSIDescription(rssi);
            
            rssiIndicator.style.backgroundColor = color;
            rssiIndicator.innerHTML = `${rssi}<br><span style="font-size: 8px;">${description}</span>`;
            
            console.log(`Updated RSSI for ${robotName}: ${rssi} (${description})`);
        } else {
            console.log(`RSSI indicator not found for ${robotName}, health card may not exist yet`);
        }
    }

    // Function to handle server report updates
    function handleServerReport(reportData) {
        console.log('Received server report:', reportData);
        
        // Add timestamp to the report
        reportData.timestamp = new Date().toLocaleTimeString();
        
        // Add to reports array (keep last 50 reports)
        serverReports.unshift(reportData);
        if (serverReports.length > 50) {
            serverReports = serverReports.slice(0, 50);
        }
        
        // Update the server reports display
        updateServerReportsDisplay();
        
        // Show notification popup if serious error code
        if (reportData.code && (reportData.code >= 400 || reportData.code < 200)) {
            showServerReportNotification(reportData);
        }
    }

    // Function to update server reports display
    function updateServerReportsDisplay() {
        var container = document.getElementById('health-status-container');
        var reportsSection = document.getElementById('server-reports-section');
        
        if (!reportsSection) {
            // Create server reports section
            var reportsSectionHtml = `
                <div id="server-reports-section" style="margin-top: 20px;">
                    <div class="panel-header" style="font-size: 14px; margin-bottom: 10px;">
                        Server Reports
                        <button id="toggle-reports" style="float: right; background: #34495e; color: white; border: none; padding: 2px 8px; border-radius: 3px; font-size: 10px; cursor: pointer;">Hide</button>
                    </div>
                    <div id="server-reports-container" style="max-height: 200px; overflow-y: auto;">
                        <!-- Reports will be added here -->
                    </div>
                </div>
            `;
            container.insertAdjacentHTML('beforeend', reportsSectionHtml);
            
            // Add toggle functionality
            document.getElementById('toggle-reports').addEventListener('click', function() {
                var reportsContainer = document.getElementById('server-reports-container');
                var toggleBtn = document.getElementById('toggle-reports');
                if (reportsContainer.style.display === 'none') {
                    reportsContainer.style.display = 'block';
                    toggleBtn.textContent = 'Hide';
                } else {
                    reportsContainer.style.display = 'none';
                    toggleBtn.textContent = 'Show';
                }
            });
        }
        
        // Update reports container
        var reportsContainer = document.getElementById('server-reports-container');
        var reportsHtml = '';
        
        serverReports.slice(0, 10).forEach(function(report) { // Show last 10 reports
            var statusColor = getReportStatusColor(report.code);
            var robotColor = jackalColors[report.robot_name?.toLowerCase()] || droneColor;
            
            reportsHtml += `
                <div class="server-report-item" style="
                    background-color: white;
                    border-left: 4px solid ${statusColor};
                    margin-bottom: 8px;
                    padding: 8px;
                    border-radius: 4px;
                    font-size: 11px;
                    box-shadow: 0 1px 3px rgba(0,0,0,0.1);
                ">
                    <div style="display: flex; justify-content: space-between; align-items: center; margin-bottom: 4px;">
                        <span style="
                            background-color: ${robotColor};
                            color: white;
                            padding: 2px 6px;
                            border-radius: 3px;
                            font-weight: bold;
                            font-size: 9px;
                        ">${report.robot_name?.toUpperCase() || 'UNKNOWN'}</span>
                        <span style="
                            background-color: ${statusColor};
                            color: white;
                            padding: 2px 6px;
                            border-radius: 3px;
                            font-weight: bold;
                            font-size: 9px;
                        ">CODE ${report.code || 'N/A'}</span>
                    </div>
                    <div style="color: #2c3e50; margin-bottom: 2px; font-weight: bold;">
                        ${report.message || 'No message'}
                    </div>
                    <div style="color: #7f8c8d; font-size: 9px;">
                        ${report.timestamp}
                    </div>
                </div>
            `;
        });
        
        if (reportsHtml === '') {
            reportsHtml = '<div style="text-align: center; color: #7f8c8d; font-style: italic; padding: 20px;">No server reports yet</div>';
        }
        
        reportsContainer.innerHTML = reportsHtml;
    }

    // Function to get status color based on code
    function getReportStatusColor(code) {
        if (!code) return '#95a5a6'; // Gray for unknown
        
        if (code >= 200 && code < 300) return '#27ae60'; // Green for success
        if (code >= 300 && code < 400) return '#f39c12'; // Orange for redirect
        if (code >= 400 && code < 500) return '#e74c3c'; // Red for client error
        if (code >= 500) return '#8e44ad'; // Purple for server error
        
        return '#3498db'; // Blue for info
    }

    // Function to show notification for important server reports
    function showServerReportNotification(reportData) {
        // Create or update notification
        var notification = document.getElementById('server-report-notification');
        if (!notification) {
            notification = document.createElement('div');
            notification.id = 'server-report-notification';
            notification.style.cssText = `
                position: fixed;
                top: 20px;
                right: 20px;
                background-color: #e74c3c;
                color: white;
                padding: 15px;
                border-radius: 5px;
                box-shadow: 0 4px 8px rgba(0,0,0,0.3);
                z-index: 1000;
                max-width: 300px;
                font-family: 'Courier New', monospace;
                font-size: 12px;
                transition: all 0.3s ease;
            `;
            document.body.appendChild(notification);
        }
        
        notification.innerHTML = `
            <div style="font-weight: bold; margin-bottom: 5px;">
                ${reportData.robot_name?.toUpperCase() || 'ROBOT'} ERROR (${reportData.code})
            </div>
            <div>${reportData.message || 'No message'}</div>
            <div style="margin-top: 8px; text-align: right;">
                <button onclick="document.getElementById('server-report-notification').remove()" 
                        style="background: rgba(255,255,255,0.2); color: white; border: 1px solid white; padding: 3px 8px; border-radius: 3px; cursor: pointer;">
                    Dismiss
                </button>
            </div>
        `;
        
        // Auto-remove after 10 seconds
        setTimeout(function() {
            if (notification && notification.parentNode) {
                notification.remove();
            }
        }, 10000);
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

    // Handle server report updates
    socket.on('server_report', function(data) {
        console.log('Received server report:', data);
        
        if (data && typeof data === 'object') {
            handleServerReport(data);
        } else {
            console.error('Invalid server report data:', data);
        }
    });

    // Handle RSSI updates
    socket.on('rssi_report', function(data) {
        console.log('Received RSSI update:', data);
        
        if (data && typeof data === 'object' && data.robot_name) {
            updateRSSI(data);
        } else {
            console.error('Invalid RSSI data:', data);
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

    window.addTestServerReport = function(robotName, code, message) {
        robotName = robotName || 'phobos';
        code = code || 200;
        message = message || 'Test server report message';
        var reportData = {robot_name: robotName, code: code, message: message};
        handleServerReport(reportData);
    };

    window.addTestRSSI = function(robotName, rssi) {
        robotName = robotName || 'phobos';
        rssi = rssi || -65; // Default to fair signal
        var rssiData = {robot_name: robotName, rssi: rssi};
        updateRSSI(rssiData);
    };

    window.clearAllCasualties = function() {
        Object.keys(casualtyMarkers).forEach(function(casualtyId) {
            objectsLayer.removeLayer(casualtyMarkers[casualtyId]);
        });
        casualtyMarkers = {};
        console.log('Cleared all casualties');
    };

    window.clearServerReports = function() {
        serverReports = [];
        updateServerReportsDisplay();
        console.log('Cleared all server reports');
    };

    console.log('Map.js loaded successfully. Use addTestJackal(), addTestDrone(), addTestCasualty(), addTestHealth(), or addTestServerReport() to test.');
});
