// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.LimelightHelpers.PoseEstimate;

/** Add your docs here. */
public class ShooterCalculations {
    /**
     * Calculates the height based on distance, angle, and velocity.
     * @param distance Distance to calculate for.
     * @param angle Angle of the launch.
     * @param velocity Velocity of the launch.
     * @return Height that the object reaches at the distance specified.
     */
    private static double trajectory(double distance, double angle, double velocity){
        return (Constants.ShooterConstants.SHOOTER_HEIGHT + (distance * Math.tan(angle)) - ((Constants.FieldConstants.GRAVITY * distance * distance) / (2 * velocity * velocity * Math.cos(angle) * Math.cos(angle))));
    }

    /**
     * Calculates the derivative with respect to the angle.
     * @param distance Distance to calculate for.
     * @param angle Angle of the launch.
     * @param velocity Velocity of the launch.
     * @return 
     */
    private static double derivative_angle(double distance, double angle, double velocity){
        return (trajectory(distance, angle + Constants.ShooterConstants.H, velocity) - trajectory(distance, angle - Constants.ShooterConstants.H, velocity)) / (2 * Constants.ShooterConstants.H);
    }

    /**
     * Calculates the derivative with respect to the velocity.
     * @param distance Distance to calculate for.
     * @param angle Angle of the launch.
     * @param velocity Velocity of the launch.
     * @return 
     */
    private static double derivative_velocity(double distance, double angle, double velocity){
        return (trajectory(distance, angle, velocity + Constants.ShooterConstants.H) - trajectory(distance, angle, velocity - Constants.ShooterConstants.H)) / (2 *Constants.ShooterConstants. H);
    }

    /**
     * Finds the best angle and velocity to reach the distance with an iterative method.
     * @param distance Distance to the hub.
     * @param initial_angle Starting angle to calculate from.
     * @param initial_velocity Starting velocity to calculate from.
     * @return Returns [angle, velocity] in an array. Will return [0, 0] if it cannot find a solution.
     */
    private static double[] optimize(double distance, double initial_angle, double initial_velocity){
        double[] result = new double[] {0, 0}; // The return value.
        double new_angle = initial_angle; 
        double new_velocity = initial_velocity;
        double height_low, height_high; // The height before and after the hub.

        // This will repeat until DIETIME, so that an infinite loop does not occur.
        // If a loop occurs, the result will be 0, 0.
        for (int i = 0; i < Constants.ShooterConstants.SHOOTER_DIE_TIME; i++){
            height_low = trajectory(distance - Constants.FieldConstants.HUB_HALF_LENGTH, new_angle, new_velocity); // Calculate the height before the hub.
            height_high = trajectory(distance + Constants.FieldConstants.HUB_HALF_LENGTH, new_angle, new_velocity); // Calculate the height after the hub.
            if ((height_low > Constants.FieldConstants.HUB_HEIGHT) && (height_high < Constants.FieldConstants.HUB_HEIGHT)){ // If we have cleared the hub and then hit the back wall...
                result[0] = new_angle; // ...set the angle,
                result[1] = new_velocity; // and the velocity.
                break;
            }

            // Otherwise...
            new_velocity += Math.signum(derivative_velocity(distance, new_angle, new_velocity)) * Constants.ShooterConstants.SHOOTER_STEP; // ...step the velocity in a direction the derivative describes (normalized to STEP)
            if (new_velocity > Constants.ShooterConstants.MAX_FLYWHEEL_VELOCITY){ // If we have reached maximum velocity...
                new_angle +=  Math.signum(derivative_angle(distance, new_angle, new_velocity)) * Constants.ShooterConstants.SHOOTER_STEP; // ...step the angle,
                new_velocity = Constants.ShooterConstants.MIN_FLYWHEEL_VELOCITY; // and reset the velocity.
            }
        }

        return result;
    }

    public static double distanceToHub(){
        /*PoseEstimate robotPose;
        double[] hubXY = new double[2];
        if (DriverStation.getAlliance().get() == Alliance.Blue){
            robotPose = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-left");
            hubXY[0] = Units.metersToFeet(Constants.FieldConstants.BLUE_HUB.getX());
            hubXY[1] = Units.metersToFeet(Constants.FieldConstants.BLUE_HUB.getY());
        } else if (DriverStation.getAlliance().get() == Alliance.Red) { 
            robotPose = LimelightHelpers.getBotPoseEstimate_wpiRed("limelight-left");
            hubXY[0] = Units.metersToFeet(Constants.FieldConstants.RED_HUB.getX());
            hubXY[1] = Units.metersToFeet(Constants.FieldConstants.RED_HUB.getY());
        } else {
            throw new RuntimeException("Driver station alliance not found!");
        }

        // TODO: Does Pose2d.getX() return metres?
        double[] robotXY = {Units.metersToFeet(robotPose.pose.getX()), Units.metersToFeet(robotPose.pose.getY())};
        double distance = Math.sqrt(Math.pow((robotXY[0] - hubXY[0]), 2) + Math.pow((robotXY[1] - hubXY[1]), 2)); // Pythagorean theorem to get distance
        System.out.println("DISTANCE TO HUB: " + distance);
        //double distance = robotPose.avgTagDist + Constants.FieldConstants.HUB_HALF_LENGTH; // Get the distance to the tag plus the hub's length
        return distance;*/

        // double distance = 0;
        // if (!LimelightHelpers.getTV("limelight-left")){
        //     return distance;
        // }
        // double offsetAngleVertical = LimelightHelpers.getTY("limelight-left");
        // double angleToGoal = Math.toRadians(Constants.ShooterConstants.LIMELIGHT_ANGLE + offsetAngleVertical);

        // distance = (Constants.FieldConstants.HUB_APRILTAG_HEIGHT - Constants.ShooterConstants.LIMELIGHT_HEIGHT) / Math.tan(angleToGoal);
        
        // return distance;
        double distance = 0;
        if (!LimelightHelpers.getTV("limelight-left")) {
        return distance; // Return the last calculated distance if target is lost
    }

        //Get the vertical offset (ty) from LimelightHelpers
        double targetOffsetAngle_Vertical = LimelightHelpers.getTY("limelight-left");

        //Get constants from the Constants file
        double limelightMountAngleDegrees = Constants.ShooterConstants.LIMELIGHT_ANGLE; 
        double limelightLensHeight = Constants.ShooterConstants.LIMELIGHT_HEIGHT; 
        double goalHeight = Constants.FieldConstants.HUB_HEIGHT; 

        //Calculate the total angle to the target
        double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
        double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);

        //Calculate horizontal distance (X)
        // Formula: (TargetHeight - CameraHeight) / tan(TotalAngle)
        distance = (goalHeight - limelightLensHeight) / Math.tan(angleToGoalRadians);
        
        return distance;
    }

    /**
     * Calculates the best angle and velocity to reach the distance--wrapper for optimize().
     * @param hub_distance Distance to the hub.
     * @return Returns [angle, velocity] in an array. Will return [-1, -1] if it cannot find a solution.
     */
    public static double[] calculateShooterTrajectory(double hub_distance){
        // Set a reasonable initial value for the angle and velocity.
        // TODO: Check the robot's current velocity and factor that in
        double initial_angle = Constants.ShooterConstants.MIN_SHOOTER_ANGLE;
        double initial_velocity = Constants.ShooterConstants.MIN_FLYWHEEL_VELOCITY;

        double[] result = optimize(hub_distance, initial_angle, initial_velocity); // Get the result.

        if ((result[0] < Constants.ShooterConstants.MIN_SHOOTER_ANGLE) || (result[0] > Constants.ShooterConstants.MAX_SHOOTER_ANGLE)){ // If the angle is invalid...
            result[0] = 0; // ...set the angle to 0.
            throw new IndexOutOfBoundsException("Angle out of bounds");
        }
        if ((result[1] < Constants.ShooterConstants.MIN_FLYWHEEL_VELOCITY) || (result[1] > Constants.ShooterConstants.MAX_FLYWHEEL_VELOCITY)){ // If the velocity is invalid...
            result[1] = 0; // ..set the velocity to 0.
            throw new IndexOutOfBoundsException("Velocity out of bounds");
        }

        return result;
    }

}
