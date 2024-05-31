package frc.robot.utils;

import java.lang.reflect.Method;
import java.util.ArrayList;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N7;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import me.nabdev.pathfinding.Pathfinder;
import me.nabdev.pathfinding.PathfinderBuilder;
import me.nabdev.pathfinding.structures.Edge;
import me.nabdev.pathfinding.structures.Obstacle;
import me.nabdev.pathfinding.structures.Vector;
import me.nabdev.pathfinding.structures.Vertex;
import me.nabdev.pathfinding.utilities.FieldLoader.Field;

public class DrivetrainSim extends DifferentialDrivetrainSim {
    public enum CollisionMode {
        BOUNDARIES,
        COLLISIONS,
        CORRECT_COLLISIONS,
    }

    public static CollisionMode collisionMode = CollisionMode.CORRECT_COLLISIONS;

    public static final double visualWidth = 0.686;
    public static final double visualLength = 0.82;
    public static final double halfWidth = visualWidth / 2.0;
    public static final double halfLength = visualLength / 2.0;
    public static final double fieldX = 16.542;
    public static final double fieldY = 8.211;

    private Pathfinder pathfinder = new PathfinderBuilder(Field.CRESCENDO_2024).setRobotLength(0.001)
            .setRobotWidth(0.001)
            .build();

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

    public boolean edgesIntersectWithObstacles() {
        ArrayList<Vertex> vertices = pathfinder.map.getPathVerticesStatic();
        for (Edge edge : pathfinder.map.getValidObstacleEdges()) {
            if (robotIntersectsWithEdge(edge.getVertexOne(vertices), edge.getVertexTwo(vertices))) {
                return true;
            }
        }
        return false;
    }

    public boolean robotIntersectsWithEdge(Vertex a, Vertex b) {
        Translation2d[] robot = getRobotCorners();
        for (int i = 0; i < 4; i++) {
            Translation2d corner = robot[i];
            Translation2d nextCorner = robot[(i + 1) % 4];
            if (Vector.dotIntersectFast(a, b, new Vertex(corner), new Vertex(nextCorner))) {
                return true;
            }
        }
        return false;
    }

    public boolean isOutsideField() {
        Translation2d[] robot = getRobotCorners();
        for (Translation2d corner : robot) {
            if (corner.getX() < 0 || corner.getX() > fieldX || corner.getY() < 0 || corner.getY() > fieldY
                    || Obstacle.isRobotInObstacle(pathfinder.map.getObstacles(), new Vertex(corner)).size() > 0) {
                return true;
            }
        }
        return false;
    }

    public boolean isOkToMove() {
        Translation2d[] robot = getRobotCorners();
        for (Translation2d corner : robot) {
            if (collisionMode == CollisionMode.BOUNDARIES
                    && (corner.getX() < 0 || corner.getX() > fieldX || corner.getY() < 0 || corner.getY() > fieldY)) {
                return false;
            }
            if ((collisionMode == CollisionMode.COLLISIONS && Obstacle
                    .isRobotInObstacle(pathfinder.map.getObstacles(), new Vertex(corner)).size() > 0)) {
                return false;
            }
            if (collisionMode == CollisionMode.CORRECT_COLLISIONS && edgesIntersectWithObstacles()) {
                return false;
            }
        }
        return true;
    }

}
