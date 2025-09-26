import requests
import os
import math
import time
from urllib.parse import urlparse

class TileDownloader:
    def __init__(self, mapbox_token, output_dir="tiles"):
        self.mapbox_token = mapbox_token
        self.output_dir = output_dir
        self.base_url = "https://api.mapbox.com/styles/v1/mapbox/satellite-v9/tiles"
        
        # Create output directory
        os.makedirs(output_dir, exist_ok=True)
    
    def deg2num(self, lat_deg, lon_deg, zoom):
        """Convert lat/lon to tile numbers"""
        lat_rad = math.radians(lat_deg)
        n = 2.0 ** zoom
        xtile = int((lon_deg + 180.0) / 360.0 * n)
        ytile = int((1.0 - math.asinh(math.tan(lat_rad)) / math.pi) / 2.0 * n)
        return (xtile, ytile)
    
    def download_tile(self, x, y, z):
        """Download a single tile"""
        url = f"{self.base_url}/{z}/{x}/{y}?access_token={self.mapbox_token}"
        
        # Create directory structure
        tile_dir = os.path.join(self.output_dir, str(z), str(x))
        os.makedirs(tile_dir, exist_ok=True)
        
        tile_path = os.path.join(tile_dir, f"{y}.png")
        
        # Skip if already downloaded
        if os.path.exists(tile_path):
            print(f"Tile {z}/{x}/{y} already exists")
            return True
        
        try:
            response = requests.get(url, timeout=30)
            response.raise_for_status()
            
            with open(tile_path, 'wb') as f:
                f.write(response.content)
            
            print(f"Downloaded tile {z}/{x}/{y}")
            return True
            
        except Exception as e:
            print(f"Failed to download tile {z}/{x}/{y}: {e}")
            return False
    
    def download_area(self, north, south, east, west, min_zoom=10, max_zoom=18):
        """Download tiles for a bounding box area"""
        print(f"Downloading tiles for area: N{north}, S{south}, E{east}, W{west}")
        print(f"Zoom levels: {min_zoom} to {max_zoom}")
        
        total_tiles = 0
        downloaded_tiles = 0
        
        for zoom in range(min_zoom, max_zoom + 1):
            # Get tile bounds for this zoom level
            x_min, y_max = self.deg2num(north, west, zoom)
            x_max, y_min = self.deg2num(south, east, zoom)
            
            # Ensure proper bounds
            x_min, x_max = min(x_min, x_max), max(x_min, x_max)
            y_min, y_max = min(y_min, y_max), max(y_min, y_max)
            
            zoom_tiles = (x_max - x_min + 1) * (y_max - y_min + 1)
            total_tiles += zoom_tiles
            
            print(f"Zoom {zoom}: {zoom_tiles} tiles")
            
            for x in range(x_min, x_max + 1):
                for y in range(y_min, y_max + 1):
                    if self.download_tile(x, y, zoom):
                        downloaded_tiles += 1
                    
                    # Rate limiting - be nice to Mapbox
                    time.sleep(0.1)
        
        print(f"Downloaded {downloaded_tiles}/{total_tiles} tiles")


if __name__ == "__main__":
    # Your area coordinates (adjust as needed)
    NORTH = 39.943
    SOUTH = 39.939  
    EAST = -75.198
    WEST = -75.200
    
    # Your Mapbox token
    MAPBOX_TOKEN = "pk.eyJ1IjoiamFzb25haCIsImEiOiJjbThxOWt5ZnMwa3NxMmtwdTEwYjJ1ajZ3In0.LGKzoIXaNi9j7lXhypfBPQ"
    
    downloader = TileDownloader(MAPBOX_TOKEN)
    
    # Download tiles for zoom levels 14-20 (adjust based on your needs)
    # Higher zoom = more detail but exponentially more tiles
    downloader.download_area(NORTH, SOUTH, EAST, WEST, min_zoom=14, max_zoom=20)
