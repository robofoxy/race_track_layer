**race_track_layer**
==================

## Attaching Plugin

* First, copy the race\_track\_layer to the *catkin_ws/src*. Then, do *catkin_make*.

*  In order to attach *race\_track\_layer* plugin, insert

```
- {name: race_track_layer,       type: "costmap_2d::RaceTrackLayer"}
```

to array of plugins of local costmap and global costmap just below the *obstacle_layer*.

## Explanation

* *race\_track\_layer* detects all footprints that are published by robots whatever their names are.

* *race\_track\_layer* customizes footprints for every robot. For example, own footprint of a robot cannot be an obstacle on own costmap.

* *race\_track\_layer* fills polygons that are published by robots, with *LETHAL_OBSTACLE*.

* *race\_track\_layer* clears old footprint locations and creates obstacles new footprint areas.

* *race\_track\_layer* detects other robot's speeds and directions, then sets inflation cost between 253 and 40 to their race tracks.

* Race tracks are triangles that start from in the line of frontmost edge of footprints of robots.

* Length of a race track depends on velocity(speed and direction) of robot. If speed is under 1 m/s, x and y coordinates of pioneer edge of the triangle will be:
```
c.x = mid.x + cos(angle) * 10;
c.y = mid.y + sin(angle) * 10;
```
Else:
```
c.x = mid.x + cos(angle) * speed * 10;
c.y = mid.y + sin(angle) * speed * 10;
```


* Width of race tracks is equal to length of vertical projection of footprint polygons of robots.

* After position of race track triangle is changed, *race\_track\_layer* clears cells whose costs are under LETHAL_OBSTACLE. Therefore, *race\_track\_layer* does not damage effects of other layers.

* In order to prevent from planner failure, race track triangle is filled with cost of 253 to 40.

| Range        | Collision Status   |
| ------------- | ------------- |
| 253 - 254 | definitely in collision |
| 253 - 128 | possibly in collision |
| 127 - 1 | definitely not in collision |
| 0 | free space |


* Race tracks will be cut off if they encounter with a LETHAL_OBSTACLE cell.

* The robot can make plans passing through other robot's race tracks. However, costs will be bigger near the robot because of the triangular structure of the race tracks.

* *race\_track\_layer* detects maximum distance between edges of footprint of host of costmap.

* *race\_track\_layer* creates an abstract square around footprint of host of costmap with respect to maximum distance between edges of the footprint of the host.

* Race tracks will be cut off if they encounter with a cell in the abstract square of the host.

* Costmap cells are sorted with *bubble sort* algorithm with respect to their distances between midpoint of the footprint of the host.

* Costs of sorted costmap cells decrease from 253 to 40 in order after 5/8 of the cells are filled with *inscribed cost*(253).

## Screenshots (evolution of the code)

Attention! Links may contain multiple images and last link is the most current.

* http://imgur.com/a/OZkh0

* http://imgur.com/a/E8Mf7

* http://imgur.com/a/69WkQ

* http://imgur.com/a/MKntZ

* http://imgur.com/a/yxSNE
