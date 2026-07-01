ros2 topic pub -r 1 /dione/glider/casualty/fix dtc_msgs/msg/CasualtyFixArray "{
  casualties: [
    {
      casualty_id: 0,
      location: {latitude: 30.634778, longitude: -96.328833}
    },
    {
      casualty_id: 1,
      location: {latitude: 30.634788, longitude: -96.328924}
    },
    {
      casualty_id: 2,
      location: {latitude: 30.634763, longitude: -96.328844}
    },
    {
      casualty_id: 3,
      location: {latitude: 30.634798, longitude: -96.328714}
    },
    {
      casualty_id: 4,
      location: {latitude: 30.634753, longitude: -96.328754}
    }
  ]
}"