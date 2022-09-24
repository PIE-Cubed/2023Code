public class Main {
    public static void main(String[] args) {
        AprilTag topRightCorner = new AprilTag(0, 10);
        AprilTag rightEdge = new AprilTag(-5, -5);

        double[] robot_coords = AprilTag.calculateRobotLocation(topRightCorner, 50, rightEdge, 50);

        // Length of 0 would mean error
        if (robot_coords.length == 2) {
            System.out.println(robot_coords[0] + " " + robot_coords[1]);
        }
    }
}