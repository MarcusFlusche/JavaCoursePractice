package frc.robot.utils;

import java.lang.reflect.Method;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N7;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;

public class DrivetrainSim extends DifferentialDrivetrainSim {
    public static final double visualWidth = 0.686;
    public static final double visualLength = 0.82;
    public static final double halfWidth = visualWidth / 2.0;
    public static final double halfLength = visualLength / 2.0;
    public static final double fieldX = 16.542;
    public static final double fieldY = 8.211;

    Method getStateMethod;

    public DrivetrainSim(DCMotor driveMotor, double gearing, double jKgMetersSquared, double massKg,
            double wheelRadiusMeters, double trackWidthMeters, Matrix<N7, N1> measurementStdDevs) {
        super(driveMotor, gearing, jKgMetersSquared, massKg, wheelRadiusMeters, trackWidthMeters, measurementStdDevs);
        try {
            getStateMethod = DifferentialDrivetrainSim.class.getDeclaredMethod("getState");
            getStateMethod.setAccessible(true);
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
    }

    public DrivetrainSim(LinearSystem<N2, N2, N2> plant, DCMotor driveMotor, double gearing, double trackWidthMeters,
            double wheelRadiusMeters, Matrix<N7, N1> measurementStdDevs) {
        super(plant, driveMotor, gearing, trackWidthMeters, wheelRadiusMeters, measurementStdDevs);
        try {
            getStateMethod = DifferentialDrivetrainSim.class.getDeclaredMethod("getState");
            getStateMethod.setAccessible(true);
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
    }

    @SuppressWarnings("unchecked")
    public Matrix<N7, N1> getCurrentState() {
        try {
            return (Matrix<N7, N1>) getStateMethod.invoke(this);
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
    }

    public Translation2d[] getRobotCorners() {
        Translation2d pose = getPose().getTranslation();
        Rotation2d rot = getPose().getRotation();
        Translation2d[] robot = new Translation2d[4];
        robot[0] = new Translation2d(halfLength, halfWidth).rotateBy(rot).plus(pose);
        robot[1] = new Translation2d(halfLength, -halfWidth).rotateBy(rot).plus(pose);
        robot[2] = new Translation2d(-halfLength, -halfWidth).rotateBy(rot).plus(pose);
        robot[3] = new Translation2d(-halfLength, halfWidth).rotateBy(rot).plus(pose);
        return robot;
    }

    public boolean isOutsideField() {
        Translation2d[] robot = getRobotCorners();
        for (Translation2d corner : robot) {
            if (corner.getX() < 0 || corner.getX() > fieldX || corner.getY() < 0 || corner.getY() > fieldY) {
                return true;
            }
        }
        return false;
    }
}
