package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.ShuffleboardLogging;

public class DriveSubsystem extends SubsystemBase implements ShuffleboardLogging {

    private final WPI_TalonSRX m_masterLeft = new WPI_TalonSRX(DriveConstants.kMasterLeftPort);
    private final CANSparkMax m_followerLeft = new
    CANSparkMax(DriveConstants.kFollowerLeftPort, MotorType.kBrushless);

    private final WPI_TalonSRX m_masterRight = new WPI_TalonSRX(DriveConstants.kMasterRightPort);
    private final CANSparkMax m_followerRight = new
    CANSparkMax(DriveConstants.kFollowerRightPort,
    MotorType.kBrushless);

    private final AHRS m_gyro = new AHRS(DriveConstants.kGyroPort);
    private final DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(
            Rotation2d.fromDegrees(getHeading()));
    private Pose2d m_Pose;
    protected NetworkTableInstance m_table;
    protected NetworkTable m_odometryTable;
    protected NetworkTableEntry m_wheelOdomEntry;
    protected NetworkTableEntry m_fusedOdomEntry;
    /**
     * Initializes a new instance of the {@link DriveSubsystem} class.
     */
    public DriveSubsystem() {

        // front left Talon

        m_masterLeft.configFactoryDefault();
        m_masterLeft.setInverted(DriveConstants.kMasterLeftInvert);
        m_masterLeft.setNeutralMode(NeutralMode.Brake);
        m_masterLeft.enableVoltageCompensation(DriveConstants.kEnableVoltageComp);
       // m_masterLeft.configVoltageCompSaturation(DriveConstants.kVoltageComp);
        m_masterLeft.enableCurrentLimit(true);
        m_masterLeft.configPeakCurrentLimit((int) DriveConstants.kPeakCurrentLimit);
        // m_masterLeft.setSecondaryCurrentLimit(DriveConstants.kPeakCurrentLimit,
        // DriveConstants.kPeakCurrentDurationMillis); //is this secondary limit
        // necessary?
        m_masterLeft.configOpenloopRamp(DriveConstants.kRampRate);
        m_masterLeft.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, DriveConstants.kPIDLoopIdx,
                10);

        // back left Spark Max
        m_followerLeft.restoreFactoryDefaults();
        m_followerLeft.setInverted(DriveConstants.kFollowerLeftOppose);
        m_followerLeft.setIdleMode(IdleMode.kCoast);
        m_followerLeft.enableVoltageCompensation(DriveConstants.kVoltageComp);
        m_followerLeft.setSmartCurrentLimit(DriveConstants.kSmartCurrentLimit);
        m_followerLeft.setSecondaryCurrentLimit(DriveConstants.kPeakCurrentLimit,
        DriveConstants.kPeakCurrentDurationMillis);
        m_followerLeft.setOpenLoopRampRate(DriveConstants.kRampRate);

        // front right Talon
        m_masterRight.configFactoryDefault();
        m_masterRight.setInverted(DriveConstants.kMasterRightInvert);
        m_masterRight.setNeutralMode(NeutralMode.Brake);
        m_masterRight.enableVoltageCompensation(DriveConstants.kEnableVoltageComp);
      //  m_masterRight.configVoltageCompSaturation(DriveConstants.kVoltageComp);
        m_masterRight.enableCurrentLimit(true);
        m_masterRight.configPeakCurrentLimit((int) DriveConstants.kPeakCurrentLimit);
        // m_masterRight.setSecondaryCurrentLimit(DriveConstants.kPeakCurrentLimit,
        // //again, is this secondary limit necessary?
        // DriveConstants.kPeakCurrentDurationMillis);
        m_masterRight.configOpenloopRamp(DriveConstants.kRampRate);
        m_masterRight.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, DriveConstants.kPIDLoopIdx2,
                10);

        // // back right Spark Max
        m_followerRight.restoreFactoryDefaults();
        m_followerRight.setInverted(DriveConstants.kFollowerRightOppose);
        m_followerRight.setIdleMode(IdleMode.kCoast);
        m_followerRight.enableVoltageCompensation(DriveConstants.kVoltageComp);
        m_followerRight.setSmartCurrentLimit(DriveConstants.kSmartCurrentLimit);
        m_followerRight.setSecondaryCurrentLimit(DriveConstants.kPeakCurrentLimit,
        DriveConstants.kPeakCurrentDurationMillis);
        m_followerRight.setOpenLoopRampRate(DriveConstants.kRampRate);

        // // Potentially needed for PID control - not sure though, yeah might need to turn that feature off...
        // m_masterLeft.configNominalOutputForward(0);
        // m_masterLeft.configNominalOutputReverse(0);
        // m_masterLeft.configPeakOutputForward(1);
        // m_masterLeft.configPeakOutputReverse(-1);

        // m_masterRight.configNominalOutputForward(0);
        // m_masterRight.configNominalOutputReverse(0);
        // m_masterRight.configPeakOutputForward(1);
        // m_masterRight.configPeakOutputReverse(-1);

        // Sets up the PID controller within the Talons
        //update 2/9: might need to get rid of this pid stuff - potentially being handled by ramsete
        // m_masterLeft.config_kP(DriveConstants.kPIDLoopIdx, DriveConstants.kP);
        // m_masterLeft.config_kI(DriveConstants.kPIDLoopIdx, DriveConstants.kI);
        // m_masterLeft.config_IntegralZone(DriveConstants.kPIDLoopIdx, (int) DriveConstants.kIz);
        // m_masterLeft.config_kD(DriveConstants.kPIDLoopIdx, DriveConstants.kD);
        // m_masterLeft.config_kF(DriveConstants.kPIDLoopIdx, DriveConstants.kFF);

        // m_masterRight.config_kP(DriveConstants.kPIDLoopIdx2, DriveConstants.kP);
        // m_masterRight.config_kI(DriveConstants.kPIDLoopIdx2, DriveConstants.kI);
        // m_masterRight.config_IntegralZone(DriveConstants.kPIDLoopIdx2, (int) DriveConstants.kIz);
        // m_masterRight.config_kD(DriveConstants.kPIDLoopIdx2, DriveConstants.kD);
        // m_masterRight.config_kF(DriveConstants.kPIDLoopIdx2, DriveConstants.kFF);

        // setting up the encoders
        m_masterRight.setSelectedSensorPosition(0);
        m_masterLeft.setSelectedSensorPosition(0);

        m_masterLeft.setSensorPhase(DriveConstants.kLeftSensorPhase); // The sensor phase needs to be checked in the
                                                                      // phoenix tuner once we've connected to the robot
        m_masterRight.setSensorPhase(DriveConstants.kRightSensorPhase); // then it can be changed here accordingly
        //**update as of 2/9: when last checked, the sensors were in phase so they are good

        //  m_gyro.calibrate(); //TODO this might become an issue - maybe make an accessor method to be called from the robot container on startup?
        m_table = NetworkTableInstance.getDefault();
        //m_wheelOdomEntry = m_odometryTable.getEntry("wheelOdom");
        //m_fusedOdomEntry = m_odometryTable.getEntry("fusedOdom");
        resetOdometry(new Pose2d(0, 0, new Rotation2d()));
        getPose();
    }

    /**
     * Update odometry
     */
    public void periodic() {
        SmartDashboard.putNumber("Left wheel", getLeftEncoderPosition());
        SmartDashboard.putNumber("Right wheel", getRightEncoderPosition());
        SmartDashboard.putNumber("Heading", m_odometry.getPoseMeters().getRotation().getDegrees());
        m_Pose = m_odometry.update(Rotation2d.fromDegrees(getHeading()), getLeftEncoderPosition(), getRightEncoderPosition());
        //   NetworkTable table = Robot.table;
        m_odometry.update(Rotation2d.fromDegrees(getHeading()), getLeftEncoderPosition(), getRightEncoderPosition());
        /*m_wheelOdomEntry.setDoubleArray(new double[] {m_odometry.getPoseMeters().getTranslation().getX(),
                                                        m_odometry.getPoseMeters().getTranslation().getY(),
                                                        m_odometry.getPoseMeters().getRotation().getRadians(),
                                                        m_gyro.getVelocityX(),
                                                        m_gyro.getVelocityY(),
                                                        getTurnRate()});*/
    }

    /**
     * @return The left encoder position (meters)
     */
    public double getLeftEncoderPosition() {
        return -m_masterLeft.getSelectedSensorPosition() * DriveConstants.kEncoderPositionConversionFactor;
    }

    /**
     * @return The right encoder position (meters)
     */
    public double getRightEncoderPosition() {
        return m_masterRight.getSelectedSensorPosition() * DriveConstants.kEncoderPositionConversionFactor; //TODO either this one or the left might need to be negated!!!
    }

    /**
     * @return The average encoder distance of both encoders (meters)
     */
    public double getAverageEncoderDistance() {
        return (getLeftEncoderPosition() + getRightEncoderPosition()) / 2.0;
    }

    /**
     * @return The velocity of the left encoder (meters/s)
     */
    public double getLeftEncoderVelocity() {
        return -m_masterLeft.getSelectedSensorVelocity() * DriveConstants.kEncoderVelocityConversionFactor;
    }

    /**
     * @return The velocity of the right encoder (meters/s)
     */
    public double getRightEncoderVelocity() {
       return m_masterRight.getSelectedSensorVelocity() * DriveConstants.kEncoderVelocityConversionFactor;

    }

    /**
     * @return Pose of the robot
     */
    public Pose2d getPose() {
        /*double[] lidarOdom = m_fusedOdomEntry.getDoubleArray(new double[] {});
        if(lidarOdom.length == 0){
            return m_odometry.getPoseMeters();
        }
        double x = lidarOdom[0];
        double y = lidarOdom[1];
        double th = lidarOdom[2];
        SmartDashboard.putNumber("X", x);
        SmartDashboard.putNumber("y", y);
        SmartDashboard.putNumber("th", th);*/
        return m_odometry.getPoseMeters();
    }

    /**
     * @return Wheel speeds of the robot
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(getLeftEncoderVelocity(), getRightEncoderVelocity());
    }

    /**
     * @return The heading of the gyro (degrees)
     */
    public double getHeading() {
        return m_gyro.getYaw() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    }

    /**
     * @return The rate of the gyro turn (deg/s)
     */
    public double getTurnRate() {
        return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    }

    /**
     * Sets both encoders to 0
     */
    public void resetEncoders() {
        m_masterLeft.setSelectedSensorPosition(0);
        m_masterRight.setSelectedSensorPosition(0);
    }

    /**
     * @param pose Pose to set the robot to 
     */
    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
    }

    /**
     * @param straight Straight percent output
     * @param left     Left percent output
     * @param right    Right percent output
     */
    public void arcadeDrive(double straight, double left, double right) {
        tankDrive(DriveConstants.kSpeedLimitFactor * (straight - left + right),
                DriveConstants.kSpeedLimitFactor * (straight + left - right));
    }

    /**
     * @param leftSpeed  Left motors percent output
     * @param rightSpeed Right motors percent output
     */
    public void tankDrive(double leftSpeed, double rightSpeed) {
        m_masterLeft.set(ControlMode.PercentOutput, leftSpeed);
        m_masterRight.set(ControlMode.PercentOutput, rightSpeed);
        m_followerLeft.set(m_masterLeft.getMotorOutputPercent());
        m_followerRight.set(m_masterRight.getMotorOutputPercent());

        // System.out.println("angle of gyro: " + getHeading());
        // System.out.println("reported velocity LEFT: " + getLeftEncoderVelocity());
        // System.out.println("reported velocity RIGHT: " + getRightEncoderVelocity());
        // System.out.println("the pose of the robot: " + getPose().toString());
    }


    /**
     * @param leftVolts  Left motors percent output
     * @param rightVolts Right motors percent output
     */
    public void tankDriveVolts(double leftVolts, double rightVolts) {
        m_masterLeft.setVoltage(leftVolts);
        m_masterRight.setVoltage(rightVolts);
      //  m_followerLeft.setVoltage(leftVolts);
      //  m_followerRight.setVoltage(rightVolts);
        m_masterLeft.feed();
        m_masterRight.feed();
    }

    /**
     * 
     * @param wheelSpeeds Left and right wheel speeds to be driven (contained in one variable)
     */

    public void setWheelSpeeds(DifferentialDriveWheelSpeeds wheelSpeeds) {
        // Unsure whether this will use the PID within the Talons to set a target
        // velocity

        double targetVelocity_UnitsPer100msLEFT = wheelSpeeds.leftMetersPerSecond * (1
                / DriveConstants.kEncoderVelocityConversionFactor);
        double targetVelocity_UnitsPer100msRIGHT = wheelSpeeds.rightMetersPerSecond * (1
                / DriveConstants.kEncoderVelocityConversionFactor);

        m_masterLeft.set(ControlMode.Velocity, targetVelocity_UnitsPer100msLEFT);
        m_masterRight.set(ControlMode.Velocity, targetVelocity_UnitsPer100msRIGHT);

        m_masterLeft.feed(); //potentially comment these out again
        m_masterRight.feed();
    }
    
    
    public void configureShuffleboard() {
        ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Drive");
        shuffleboardTab.addNumber("Left speed", () -> getWheelSpeeds().leftMetersPerSecond).withSize(4, 2)
                .withPosition(0, 0).withWidget(BuiltInWidgets.kGraph);
        shuffleboardTab.addNumber("Right speed", () -> getWheelSpeeds().rightMetersPerSecond).withSize(4, 2)
                .withPosition(4, 0).withWidget(BuiltInWidgets.kGraph);
        shuffleboardTab.addNumber("Left motor speed", () -> getLeftEncoderPosition()).withSize(1, 1).withPosition(0, 2)
                .withWidget(BuiltInWidgets.kTextView);
        shuffleboardTab.addNumber("Right motor speed", () -> getRightEncoderPosition()).withSize(1, 1)
                .withPosition(1, 2).withWidget(BuiltInWidgets.kTextView);
        shuffleboardTab.addNumber("Heading", () -> getHeading()).withSize(1, 1).withPosition(2, 2)
                .withWidget(BuiltInWidgets.kTextView);

    }
}