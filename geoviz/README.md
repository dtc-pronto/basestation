# DTC Basestation Visualizer

This tool always you to interface position topics with a MapBox satellite map. It is a ros package called `geoviz`. It subscirbes to the robot locations and casualties and plots them on an interactive satillite map. 

### Running
Run this with `roslaunch geoviz viz.launch`, this will start the map server on your local machine. To view the map enter `127.0.0.1:5000` into your browser. 

You can also run this on a remote server by setting the ip and port 
```
roslaunch geoviz viz.launch ip:=<ADDRESS> port:=<PORT>
```
You can configure what robots are in the swarm in `config/map.yaml` under the `robots` parameter. 


### Key Setup
You'll need to setup a mapbox public api key, directions are [here](https://docs.mapbox.com/help/dive-deeper/access-tokens/) and add the key to `env/mapbox_token.env`.

### Authors
 - Jason Hughes - `jasonah.at.seas.upenn.edu`

