package roadgraph;


import java.util.Map;

public class MapEdge {
    private MapNode start;
    private MapNode end;
    private String streetName;
    private double distance;

    public MapEdge() {
    }

    public MapEdge(MapNode start, MapNode end, String streetName) {
        this.start = start;
        this.end = end;
        this.streetName = streetName;
    }

    public MapNode getStart() {
        return start;
    }

    public void setStart(MapNode start) {
        this.start = start;
    }

    public MapNode getEnd() {
        return end;
    }

    public void setEnd(MapNode end) {
        this.end = end;
    }

    public String getStreetName() {
        return streetName;
    }

    public void setStreetName(String streetName) {
        this.streetName = streetName;
    }

    public double getDistance(MapNode node1, MapNode node2) {
        return node1.getLocation().distance(node2.getLocation());
    }

}
