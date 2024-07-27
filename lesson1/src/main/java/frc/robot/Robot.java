/*
 * <! ATTENTION JAVA COURSE STUDENTS! !>
 * This is the file where you will do most of your work. In this lesson, it is the only file you will touch.
 */

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// This package statement helps java and wpilib find the right file to run.
package frc.robot;

// These are the imports used in this file. They allow us to use classes from other files.
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

public class Robot extends TimedRobot {
    // The XboxController class interfaces with an Xbox-style controller connected
    // to the Driver Station (or a simulated one, controlled via keyboard)
    private final XboxController m_controller = new XboxController(0);

    // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
    private final SlewRateLimiter m_speedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

    // The robot's subsystems and commands are defined here...

    // This is the drive subsystem, which includes code for the robot's drivetrain.
    private final Drivetrain m_drive = new Drivetrain();

    /**
     * This method runs once when the robot is turned on.
     */
    @Override
    public void robotInit() {
        // Controlls the initial pose for the robot
        m_drive.resetOdometry(new Pose2d(2, 2, new Rotation2d()));
    }

    /**
     * Robot periodic runs once every 20ms when the robot is turned on.
     */
    @Override
    public void robotPeriodic() {
        // This method is used to handle updating our robot's position and drawing it to
        // the screen.
        m_drive.periodic();
    }

    /**
     * autonomousInit runs once when the robot is put into autonomous mode.
     */
    @Override
    public void autonomousInit() {
    }

    /**
     * autonomousPeriodic runs once every 20ms when the robot is put into
     * autonomous.
     */
    @Override
    public void autonomousPeriodic() {
        /*
         * <! ATTENTION JAVA COURSE STUDENTS! !>
         * This is where you write code for lesson 1!
         * Your task: make the robot drive in a circle.
         * 
         * To do this, think about what the robot needs to be doing at any time: Is it
         * turning? Is it driving straight?
         * 
         * Take a look at teleopPeriodic() to see how the drivetrain is controlled.
         * 
         * If the robot keeps crashing into things, think about the adjustments you need
         * to make to change the size of the circle.
         */
    }

    /**
     * teleopPeriodic runs once every 20ms when the robot is put into teleop mode.
     */
    @Override
    public void teleopPeriodic() {
        // Get the x speed. We are inverting this because Xbox controllers return
        // negative values when we push forward.
        // The deadband is used to prevent stick drift.
        double joystickLeftY = MathUtil.applyDeadband(m_controller.getLeftY(), 0.05);
        double xSpeed = -m_speedLimiter.calculate(joystickLeftY) * Drivetrain.kMaxSpeed;

        // Get the rate of angular rotation. We are inverting this because we want a
        // positive value when we pull to the left (remember, counterclockwise is
        // positive in mathematics). Xbox controllers return positive values when you
        // pull to the right by default.
        double joystickRightX = MathUtil.applyDeadband(m_controller.getRightX(), 0.05);
        double rot = -m_rotLimiter.calculate(joystickRightX) * Drivetrain.kMaxAngularSpeed;

        // Provide the speeds to the drive subsystem.
        m_drive.drive(xSpeed, rot);
    }

    /**
     * simulationPeriodic runs once every 20ms when the robot is being simulated. On
     * a real robot, this method is never called.
     */
    @Override
    public void simulationPeriodic() {
        m_drive.simulationPeriodic();
    }
}