# Java Course Practice

Collection of robot exercises for the 3044 Java Course!

## Instructions

Welcome to home-base for your robot projects!

If you haven't already, you should create a fork.

1. Click on "Fork" in the top right corner
2. Click "Create Fork" in the bottom right

**IMPORTANT - Enable issues!**

Issues are where we will give you feedback, suggestions, or where you can leave notes for yourself or us. By default, they are turned off in forks. However, it's very easy to turn back on:

1. On your fork's page, click "Settings" on the top bar.
2. Scroll down until you see "Features"
3. Click the checkbox next to issues

Congratulations! You have now setup your fork.

### Lesson one - Drive in a circle

Your job is simple: Make the robot drive in a circle in autonomous mode.

You are given a robot that can drive around in teleop, but does nothing in autonomous. You can look at the `teleopPeriodic` method to get an idea of how to control the robot.

**Working files**

These are the files that you will be working in for lesson 1:

- `src/main/java/frc/robot/Robot.java`

### Lesson two - Drive in a line, and stop

Using the skills you learned from lesson one, it should be easy to make the robot drive forward. But now, you have to make it stop after 3 seconds!

You are again given a robot that can drive around in teleop, but does nothing in autonomous. You can look at the `teleopPeriodic` method to get an idea of how to control the robot if you forgot.

The trick now is the `Timer` class! This class is provided by WPILIB, and it does just what you'd expect - it times!

Here are some examples of how to use it:

```java
Timer timer = new Timer();

// Call once, to start the timer
timer.start();

// Call whenever you want, to get the value of the timer, in seconds
timer.get()

// Call once, to reset the timer
timer.reset()
```

You're going to have to create your own Timer object in the Robot class, then use the autonomous methods to start and stop the robot! Remeber, if your not sure what to do, just try things! Trial and error is a great way to learn.

**Working files**

These are the files that you will be working in:

- `src/main/java/frc/robot/Robot.java`
