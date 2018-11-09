package roadgraph;

import geography.GeographicPoint;

import java.util.ArrayList;
import java.util.List;

public class MapNode {
    private GeographicPoint location;
    private List<MapEdge> neighbors;

    public MapNode(GeographicPoint location) {
        this.location = location;
    };

    public MapNode(GeographicPoint location, List<MapEdge> neighbors) {
        this.location = location;
        this.neighbors = neighbors;
    }

    public GeographicPoint getLocation() {
        return location;
    }

    public void setLocation(GeographicPoint location) {
        this.location = location;
    }

    public List<MapNode> getNeighbors() {
        List<MapNode> neighborsNode = new ArrayList<MapNode>();
        for (MapEdge edge: neighbors) {
            neighborsNode.add(edge.getEnd());
        }
        return neighborsNode;
    }

    public void addEdge(MapEdge edge) {
        this.neighbors.add(edge);
    }

    public boolean isEqual(MapNode node) {
        return  this.getLocation().getX() == node.getLocation().getX() &&
        this.getLocation().getY() == node.getLocation().getY();
    }
}
