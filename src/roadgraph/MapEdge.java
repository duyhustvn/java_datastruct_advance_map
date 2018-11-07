package roadgraph;


import java.util.Map;

public class MapEdge {
    private MapNode start;
    private MapNode end;
    private String streetName;
    private String roadType;
    private double distance;

    public MapEdge() {
    }

    public MapEdge(MapNode start, MapNode end, String streetName, String roadType) {
        this.start = start;
        this.end = end;
        this.streetName = streetName;
        this.roadType = roadType;
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

    public String getRoadType() {
        return roadType;
    }

    public void setRoadType(String roadType) {
        this.roadType = roadType;
    }

    public double getDistance(MapNode node1, MapNode node2) {
        return node1.getLocation().distance(node2.getLocation());
    }

    public boolean isEqual(MapNode start, MapNode end) {
        return  (this.start.isEqual(start) && this.end.isEqual(end));
    }
}
