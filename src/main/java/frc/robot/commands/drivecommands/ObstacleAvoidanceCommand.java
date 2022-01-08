package frc.robot.commands.drivecommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants.ObstacleAvoidanceConstants;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.vision.VisionThread;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;

import java.util.ArrayList;
import java.util.Arrays;
public class ObstacleAvoidanceCommand extends CommandBase {

    private DriveSubsystem m_driveSubsystem;
    private NetworkTableInstance m_table;
    private NetworkTable m_lidarTable;
    private NetworkTableEntry m_occGridEntry;
    private DifferentialDriveWheelSpeeds m_wheelSpeeds;
    private double[] m_lastSet = {-0.5,-0.25};
    public ObstacleAvoidanceCommand(DriveSubsystem driveSubsystem) {
        m_driveSubsystem = driveSubsystem;
        m_table = NetworkTableInstance.getDefault();

        addRequirements(m_driveSubsystem);
    }

    
    public void initialize() {
        m_lidarTable = m_table.getTable("ODOMETRY");
        m_occGridEntry = m_lidarTable.getEntry("grid");
        m_wheelSpeeds = new DifferentialDriveWheelSpeeds(-0.5, -0.25);
    }


    public void execute() {
        System.out.println("running");
        double leftVoltage = DriveConstants.kFeedForward.calculate(m_wheelSpeeds.leftMetersPerSecond*2);
        double rightVoltage = DriveConstants.kFeedForward.calculate(m_wheelSpeeds.rightMetersPerSecond*2);
        m_driveSubsystem.tankDriveVolts(leftVoltage, rightVoltage);
        while(!DriverStation.getInstance().isDisabled()){
            //System.out.println("running");
            //m_wheelSpeeds = new DifferentialDriveWheelSpeeds(-1, -0.5);
            //System.out.println("R " + m_driveSubsystem.getLeftEncoderVelocity() + ", L " +  m_driveSubsystem.getRightEncoderVelocity());
            //System.out.println("mR " + m_lastSet[0] + ", mL " +  m_lastSet[1]);
            double[] speedVals = avoidObstacles(m_wheelSpeeds.leftMetersPerSecond, m_wheelSpeeds.rightMetersPerSecond);
            m_wheelSpeeds.leftMetersPerSecond = speedVals[0];
            m_wheelSpeeds.rightMetersPerSecond = speedVals[1];
            leftVoltage = DriveConstants.kFeedForward.calculate(m_wheelSpeeds.leftMetersPerSecond);
            rightVoltage = DriveConstants.kFeedForward.calculate(m_wheelSpeeds.rightMetersPerSecond);
            m_driveSubsystem.tankDriveVolts(leftVoltage, rightVoltage);
            try{
                Thread.sleep(50);
            }catch(Exception e){

            }
        }
    }


    public void end(boolean interrupted) {
        m_driveSubsystem.tankDriveVolts(0, 0);
    }

    public Double[][] getLidarOccupancyMap(){
        boolean[] grid = m_occGridEntry.getBooleanArray(new boolean[256]);
        ArrayList<Double[]> ret = new ArrayList<Double[]>();
        boolean[][] grid2 = new boolean[16][16];
        for(int i = 0; i<255; ++i){
            grid2[(int)Math.floor(i/16)][(i%16)] = grid[i];
        }
        for(int i = 0; i<grid.length; ++i){
            if(grid[i]){
                Double[] temp = new Double[2];
                temp[1] = (Double)(Math.floor(i/16))*-6;
                temp[0] = (Double)((double)(i%16)-7)*-6;
                ret.add(temp);
            }
        }
        return ret.toArray(new Double[ret.size()][2]);
    }

    public int avoidObstaclesNew(){
        Double[][] obList = getLidarOccupancyMap();
        double cutoff = 2.0;
        double velL = m_driveSubsystem.getLeftEncoderVelocity();
        double velR = m_driveSubsystem.getRightEncoderVelocity();
        double vCenter = (velL + velR) / 2;
        double vTh = (velR - velL) / Constants.ObstacleAvoidanceConstants.kTrackWidth;
        double cumTime = 0;
        for(Double[] obCoord : obList){
            double timeToHit = checkPath(vCenter, vTh, obCoord[0], obCoord[1], cutoff);
            cumTime += timeToHit;
        }
    }


    /**
     * check a given robot state for collisions with an obstacle
     * @param vCenter center velocity of robot
     * @param vTheta angular velocity of robot
     * @param oX obstacle x position(relative to robot)
     * @param oY obstacle y position(relative to robot)
     * @param cutoffSec max time to trace path **not real time** (seconds)
     * @return time until collision *if no collision, will return cutoffSec*
     */
    public double checkPath(double vCenter, double vTheta, Double[][] obPositions, double cutoffSec){
        
        double elapsed = 0;
        double dT = .1; //100 ms
        double x = 0;
        double y = 0;
        double theta = 0;
        double largestDSq = vCenter * cutoffSec * vCenter * cutoffSec; 
        ArrayList<Double[]> obstaclesToCheck = new ArrayList<Double[]>();
        for(Double[] i : obPositions){
            if(largestDSq >= i[0]*i[0] + i[1]*i[1]){
                obstaclesToCheck.add(new Double[]{i[0],i[1]});
            }
        }
        Double[][] obstaclesToCheckArr = (Double[][])obstaclesToCheck.toArray();
        //check our expected position every dT seconds for a collision
        double radCheck = (Constants.ObstacleAvoidanceConstants.kTrackWidth/2) * (Constants.ObstacleAvoidanceConstants.kTrackWidth/2);
        while(elapsed < cutoffSec){

            //update x,y,theta given velocities
            x += vCenter * Math.cos(theta) * dT;
            y += vCenter * Math.sin(theta) * dT;
            theta += vTheta*dT;

            //increment time
            elapsed += dT;


            //distance to obstacle
            double d = (oX - x) * (oX - x) + (oY - y) * (oY - y);

            //if obstacle is within robot perimeter, return the time
            if(d < ){
                return elapsed;
            }
        }        
        return cutoffSec;
    }

    public double[] avoidObstacles(double l, double r){

        Double[][] occMap = getLidarOccupancyMap();
        double velL = m_driveSubsystem.getLeftEncoderVelocity() ;
        double velR = m_driveSubsystem.getRightEncoderVelocity();
        double[] ret = {l,r};
        double width = ObstacleAvoidanceConstants.kTrackWidth;
        if((velL < (m_lastSet[0] + 0.1) && velL > (m_lastSet[0] - 0.1) && velR < (m_lastSet[1] + 0.1) && velR > (m_lastSet[1] - 0.1))){
            if(velL != velR){
                double d = ((velR * width) / (velL - velR)) + (width/2);
                //double velC = (velL + velR)/2;
                double x1 = 0-d;
                double x2 = 1-d;
                double y1 = 0;
                double y2 = 0;
                double dy = 0;
                double p = 0;
                double q = 0;
                double dr = 0;
                double n = 0;
                double D = 0;
                double s = 0;
                double dp = 0;
                double dn = 0;
                double kSpace = -3500;//-1076.24; 
                double kCenterLoc = 8.75;//26.25; //17.5; 
                boolean collDetected = false;
                for(Double[] i : occMap){
                    p = i[1];
                    q = i[0];
                    y1 = ((kSpace + (q*q) + (p*p))/((2*q)+(2*kCenterLoc)))+kCenterLoc;
                    y2 = (((2*(d-p)) + kSpace + (q*q) + (p*p))/((2*q)+(2*kCenterLoc)))+kCenterLoc;
                    dy = y2-y1;
                    dr = Math.sqrt(1+(dy*dy));
                    D=(x1*y2)-(x2*y1);
                    n = (2*q)+(2*kCenterLoc);
                    s = (d*d*dr*dr)-(D*D);
                    if(s>=0){
                        System.out.println(collDetected);
                        collDetected = true;
                        break;
                    }
                }
                if(collDetected){
                    double radical = Math.sqrt((-1 * (y1 * y1)) * ((4 * y1 * n) - (n * n) - (4 * (p * p))));
                    dp = ((2*y1*p)+radical)/((4*y1)-n);
                    dn = ((2*y1*p)-radical)/((4*y1)-n);
                    double dCh = leastDiff(dp, dn, d);
                    System.out.println(dCh == dp);
                    if(dCh-d<0){
                        //left wheel needs to go faster
                        //or right wheel needs to go slower
                        velL = m_lastSet[1] + ((29*m_lastSet[1])/(dCh-14.5));
                        ret[0] = velL;
                        ret[1] = m_lastSet[1];
                        m_lastSet = ret;
                        System.out.println("1  " +Arrays.toString(m_lastSet));
                        //m_lastSet[0] *= -1;
                        //m_lastSet[1] *= -1;
                    }else if(dCh-d>0){
                        //right wheel needs to go faster
                        //or left wheel needs to go slower
                        velR = ((dCh - 14.5) * (m_lastSet[0])) / (29+dCh-14.5);
                        ret[0] = m_lastSet[0];
                        ret[1] = velR;
                        m_lastSet = ret;
                        System.out.println("2  " +Arrays.toString(m_lastSet));
                        //m_lastSet[0] *= -1;
                        //m_lastSet[1] *= -1;
                    }
                }
            }
        } else{
            //System.out.println("adjusting");
            ret[0] += ((m_lastSet[0] - velL) * 0.1);
            //System.out.println("Left adj:" + ((m_lastSet[0] - velL) *0.1));
            ret[1] += ((m_lastSet[1] - velR) * 0.1);
            //System.out.println("Right adj:" + ((m_lastSet[1] - velL)*0.1));
//            ret[0] *= -1;
//            ret[1] *= -1;
        }
        return ret;
    }
    private double leastDiff(double d1, double d2, double dCurr){
        if(Math.abs(d1-dCurr) <= Math.abs(d2-dCurr)){
            return d1;
        }else{
            return d2;
        }
    }
}