package frc.robot.commands.drivecommands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class TrajectoryFollowCommand extends RamseteCommand {

    private final DriveSubsystem m_driveSubsystem;

    /**
     * Follow a trajectory, either created through Pathweaver or manually
     * 
     * @param driveSubsystem The subsystem to be used
     * @param trajectory     The trajectory that the ramsete controller seeks to
     *                       follow
     */
    public TrajectoryFollowCommand(DriveSubsystem driveSubsystem, Trajectory trajectory) {
        // This class serves to abstract away creating the RamseteCommand by just taking
        // in a trajectory and handing the rest

        // so this is what they did last year, having the "set wheel speeds" method
        // handle the PID
        // super(trajectory, driveSubsystem::getPose, new RamseteController(),
        // DriveConstants.kDriveKinematics,
        // (left, right) -> driveSubsystem.tankDriveVolts(left, right),
        // driveSubsystem);
        //m_driveSubsystem = driveSubsystem;

        // this includes the pids as part of the ramsete stuff, so it can pass voltages
        // directly to the drive subsytem through tankDriveVolts
        super(trajectory, driveSubsystem::getPose, new RamseteController(), DriveConstants.kFeedForward,
                DriveConstants.kDriveKinematics, driveSubsystem::getWheelSpeeds,
                new PIDController(DriveConstants.kPDriveVel, 0, 0), new PIDController(DriveConstants.kPDriveVel, 0, 0),
                driveSubsystem::tankDriveVolts, driveSubsystem);

       m_driveSubsystem = driveSubsystem;
      // m_trajectory = trajectory;
     //  m_driveSubsystem.resetOdometry(trajectory.getInitialPose());
       //System.out.println("trajectory initial pose: " + trajectory.getInitialPose().toString());
       System.out.println("robot pose" + m_driveSubsystem.getPose().toString());
        // m_driveSubsystem.resetOdometry(new Pose2d(0, 0, new Rotation2d()));
        // System.out.println("here");
        // m_driveSubsystem.resetOdometry(pose);
    }

    /**
     * Stop the drivetrain at the end of the command
     */
    public void end(boolean interrupted) {
        m_driveSubsystem.tankDriveVolts(0, 0);
    }
}
