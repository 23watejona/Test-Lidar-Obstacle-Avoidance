package frc.robot.commands.drivecommands;

import frc.robot.subsystems.ArduinoSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class PixyGalacticCommand extends SequentialCommandGroup {

    private final ArduinoSubsystem m_arduinoSubsystem;
    private final DriveSubsystem m_driveSubsystem;
    private final Trajectory m_trajectory;

    // TODO- change values here
    double aredx = 133;
    double abluex = 200;
    double bredx = 100;
    double bbluex = 180;
    double threshhold = 20;

    public PixyGalacticCommand(ArduinoSubsystem arduinoSubsystem, DriveSubsystem driveSubsystem, Trajectory trajectoryBlue, Trajectory trajectoryRed) {
      
        m_arduinoSubsystem = arduinoSubsystem;
        m_driveSubsystem = driveSubsystem;

        m_arduinoSubsystem.update();
        // System.out.println("the x value is: " + m_arduinoSubsystem.getXValue());
        // if (m_arduinoSubsystem.getXValue() >= aredx - threshhold
        //         && m_arduinoSubsystem.getXValue() <= aredx + threshhold) { //the target is in the acceptable range --> run red path
        //     m_trajectory = trajectoryRed;
        // } else { //run the blue path
        //     m_trajectory = trajectoryRed;
        // }

        if (m_arduinoSubsystem.getTargetInView()) { //the target is in the acceptable range --> run red path
            m_trajectory = trajectoryRed;
        } else { //run the blue path
            m_trajectory = trajectoryBlue;
        }
        
        addRequirements(m_driveSubsystem, m_arduinoSubsystem);
        addCommands(new TrajectoryFollowCommand(m_driveSubsystem, m_trajectory));
        
    }

    public void end() { // TODO this might cause problems so might need to take it out later
        m_driveSubsystem.tankDrive(0, 0);
    }

}