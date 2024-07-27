# Lesson two - Drive in a line, and stop

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

### Working files

These are the files that you will be working in:

- `src/main/java/frc/robot/Robot.java`
