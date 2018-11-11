package roadgraph;

public class NodeWithDistance implements Comparable {
    private MapNode node;
    private double distance;

    public NodeWithDistance() {
    }

    public NodeWithDistance(MapNode node, double distance) {
        this.node = node;
        this.distance = distance;
    }

    public NodeWithDistance(MapNode node) {
        this.node = node;
        this.distance = 0;
    }

    public MapNode getNode() {
        return node;
    }

    public void setNode(MapNode node) {
        this.node = node;
    }

    public double getDistance() {
        return distance;
    }

    public void setDistance(double distance) {
        this.distance = distance;
    }

    @Override
    public int compareTo(Object o) {
        if (this.getDistance() - ((NodeWithDistance) o).getDistance() > 0) {
            return 1;
        } else {
            return -1;
        }
    }

    public boolean isEqual(NodeWithDistance node) {
        boolean sameLocation = (this.getNode().getLocation().getX() == node.getNode().getLocation().getX() &&
                this.getNode().getLocation().getY() == node.getNode().getLocation().getY());
        boolean sameDistance = (this.getDistance() == node.getDistance());
        if (sameDistance && sameDistance) return true;
        return false;
    }
}
