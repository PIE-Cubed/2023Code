public class AprilTag {

    private double x_coord;
    private double y_coord;

    public AprilTag(double x, double y) {
        x_coord = x;
        y_coord = y;
    }

    // To-do - make a function for 3+ tags where certainty would be factored in
    // Note - angles are based on trigonometry standards - 0 degrees is right, increases counter-clockwise
    public static double[] calculateRobotLocation(AprilTag tag1, double angle1, AprilTag tag2, double angle2) {
        double x1 = tag1.getX();
        double x2 = tag2.getX();
        double y1 = tag1.getY();
        double y2 = tag2.getY();

        double tan1 = 0;
        double tan2 = 0;

        // Dealing with edge cases with tan that will never occur and won't affect result by a significant amount (<10^-7)
        // Can do separate math if we know these points are straight ahead, but it makes code messier
        if (angle1 % 360 == 90) {
            tan1 = 999999999;
        }
        else if (angle1 % 360 == 270) {
            tan1 = -999999999;
        }
        else {
            tan1 = Math.tan(Math.toRadians(angle1));
        }

        if (angle2 % 360 == 90) {
            tan2 = 999999999;
        }
        else if (angle2 % 360 == 270) {
            tan2 = -999999999;
        }
        else {
            tan2 = Math.tan(Math.toRadians(angle2));
        }

        // Divide by zero error - both points are in a line with the robot, can't find coordinates
        if (tan1 == tan2) {
            System.out.println("Invalid coordinates, try again");
            return new double[0];
        }

        double robotX = ((y1 - x1*tan1) - (y2 - x2 * tan2)) / (tan2 - tan1); // difference in y-intercept / negative difference in slope
        double robotY = tan1 * robotX + (y1 - x1 * tan1); // y = mx+b
        double[] finalResults = {robotX, robotY};
        return finalResults; 
    }

    private double getX() {
        return x_coord;
    }

    private double getY() {
        return y_coord;
    }
}