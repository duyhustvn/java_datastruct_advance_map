This repo hosts code following the course [Advanced Data Structures in Java](https://www.coursera.org/learn/advanced-data-structures) 

| ![](https://raw.githubusercontent.com/duyhustvn/java_datastruct_advance_map/master/images/map.gif) |
|:--:| 
| *Demo gif* |

## REQUIREMENT 
1. Oracle jdk 8 
2. An google map API key. Get one at this [link](https://developers.google.com/maps/documentation/javascript/) and replace key in file src/html/index.html

## HOW TO RUN
### Using command line
1. Go to the root of project
2. Compile the application

``` sh
javac -cp src:libs/javax.json-1.0.4.jar src/application/MapApp.java
```
3. Run 

``` sh
java -cp src:libs/javax.json-1.0.4.jar application.MapApp
```

### Using eclipse
1. Create a new Java Project in your workspace
2. Import the starter files:
	  File -> Import -> Select "File System" -> Next -> Browse and set 
	  root directory to folder contents of zip were extracted to -> Finish


## FILES BY WEEK 

Below are the files introduced in each week and used in each week
of the course. See file for description...

Week 1 : Introduction to the course and graphs
==============================================
basicgraph.Graph.java
basicgraph.GraphAdjList.java
basicgraph.GraphAdjMatrix.java

Week 2 : Class design and simple graph search
==================================================
roadgraph.MapGraph.java
week2example.Maze.java
week2example.MazeLoader.java
week2example.MazeNode.java

Utility files
=============
geography.GeographicPoint.java
geography.RoadSegment.java
util.GraphLoader.java
