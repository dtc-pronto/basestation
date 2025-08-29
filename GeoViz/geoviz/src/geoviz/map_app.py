"""
    Jason Hughes
    March 2025

    Plot things on a map
"""
import os

import folium
from flask import Flask, render_template_string, render_template
from flask_socketio import SocketIO
from threading import Thread
from dotenv import load_dotenv

from typing import Tuple, Dict

class MapApp:

    def __init__(self, path : str, ip : str = '127.0.0.1', port : int = 5000, starting_ll: Tuple[float] = (39.94136, -75.199492)) -> None:
        print("[VISUALIZER] Starting viz on ip: %s:%i" %(ip, port))
        self.ip_ = ip
        self.port_ = port 

        self.app_ = Flask(__name__, template_folder=path+'/templates', static_folder=path+'/static')
        self.socketio_ = SocketIO(self.app_, cors_allowed_origins="*")
        
        self.map_ = folium.Map(location=starting_ll,
                               zoom_start=24,
                               tiles='https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
                               attr='Esri',
                               name='Esri Satellite')

        self.setup_routes()
        self.gps_points = list()
        self.odom_points = list()
        self.glins_points = list()

        env_file_path = os.path.join(path, "env", "mapbox_token.env")
        load_dotenv(env_file_path)

    def setup_routes(self) -> None:
        @self.app_.route('/')
        def index():
            return render_template('index.html', mapbox_token=os.getenv('MAPBOX_ACCESS_TOKEN'))

    def update_jackal(self, lat: float, lon: float, popup: str = "phobos"):
        """Update map with gps location of jackal,
            make sure to set the robot name as a the popup
        """
        point_data = {
            "lat": lat,
            "lon": lon,
            "popup": popup}
             
        # Send all points to frontend
        self.socketio_.emit('jackal_update', point_data)

    def update_falcon(self, lat: float, lon: float, popup: str = "dione"): 
        point_data = {
            "lat": lat,
            "lon": lon,
            "popup": popup}

        self.socketio_.emit("drone_update", point_data)

    def update_casualty(self, lat: float, lon: float, casualty_id : int, popup : str = None):
        if popup is None:
            popup = f"Casualty: {casualty_id}"

        point_data = {"lat": lat, 
                      "lon": lon, 
                      "casualty_id": casualty_id, 
                      "popup": popup}
        
        self.socketio_.emit('casualty_update', point_data)

    def update_status(self, health_data : Dict) -> None:
        self.socketio_.emit('health_update', health_data)

    def run_in_thread(self) -> None:
        thread = Thread(target=self.run)
        thread.daemon = True
        thread.start()

    def run(self) -> None:
        self.app_.run(debug=False, host=self.ip_, port=self.port_)

