package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.loops.ILooper;
import frc.lib.loops.Loop;
import frc.lib.util.DriveSignal;
import frc.lib.util.HIDHelper;
import frc.lib.util.HIDHelper.HIDConstants;
import frc.robot.Constants;
import frc.robot.RoboMAP;

public class Drive extends Subsystem {

	// Left mortors/talons
    public TalonSRX leftMaster;
    public TalonSRX leftSlave;

    // Right mortors/talons
    public TalonSRX rightMaster;
    public TalonSRX rightSlave;

    // The joystick
     public Joystick stick;
     //public HIDHelper HIDStick;

     
     private static Drive mDriveInstance = new Drive();
     
     public DriveIO periodic = new DriveIO();

    public static  Drive getInstance() {
        return mDriveInstance;
    }

    private Drive() {
        //Left mortrs/talons ports
        leftMaster = new TalonSRX(Constants.DRIVE_FRONT_LEFT_ID);
        leftSlave = new TalonSRX(Constants.DRIVE_BACK_LEFT_ID);

        //Right mortrs/talons ports
        rightMaster = new TalonSRX(Constants.DRIVE_FRONT_RIGHT_ID);
        rightSlave = new TalonSRX(Constants.DRIVE_BACK_RIGHT_ID);

        //get the slaves to follow the masters
        leftSlave.follow(leftMaster);  
        rightSlave.follow(rightMaster);

        rightMaster.setNeutralMode(NeutralMode.Brake);
        rightSlave.setNeutralMode(NeutralMode.Brake);

        leftSlave.setNeutralMode(NeutralMode.Brake);
        leftMaster.setNeutralMode(NeutralMode.Brake);

        rightMaster.setInverted(true);
        rightSlave.setInverted(true);
        //joystick port
        stick = new Joystick(RoboMAP.JOYSTICK_ID);
    }


    /*
    * Updates all periodic variables and sensors
    * In this case, reading the x and y of the joystic gyro
    */

    public void readPeriodicInputs() {
        //double xSpeed = stick.getX(Hand.kRight);
        //double zRotation = stick.getY(Hand.kRight);
        periodic.xSpeed = HIDHelper.getAdjStick(Constants.MASTER_STICK)[1];
        periodic.zRotation = HIDHelper.getAdjStick(Constants.MASTER_STICK)[2];
    }
     

    /**
     * Required for the subsystem's looper to be registered to the state machine
     * not required for subsystems that do not use looper
     *
     * @param enabledLooper the subsystem's Looper
     */
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
                public void onStart(double timestamp) {
 
                }

                public void onStop(double timestamp) {

                }

                public void onLoop(double timestamp) {
                    DriveSignal driveOutput = arcadeDrive(periodic.xSpeed, periodic.zRotation);
                    periodic.rightMotorOutput = driveOutput.getRight();
                    periodic.leftMotorOutput = driveOutput.getLeft();
                }
        }
        );
    }

    /**
     * Writes the periodic outputs to actuators (motors and ect...)
     * in this case, writing the voltage output for the mortors
     */
    public void writePeriodicOutputs() {
        rightMaster.set(ControlMode.PercentOutput, periodic.rightMotorOutput);
        leftMaster.set(ControlMode.PercentOutput, periodic.leftMotorOutput);
    }
    
    /**
     * Outputs all logging information to the SmartDashboard
     */
    public void outputTelemetry() {
        SmartDashboard.putNumber("RightMotor", periodic.rightMotorOutput);
        SmartDashboard.putNumber("LeftMotor", periodic.leftMotorOutput);
    }

    /**
     * Called to reset and configure the subsystem
     */
    public void reset() {

    }

    public LogData getLogger(){
        return null;
    }

    /**
     * Inheritable data class for capuring log data from the subsystems
     */
    public class DriveIO extends PeriodicIO {
        public double leftMotorOutput;
        public double rightMotorOutput;

        public double xSpeed = 0;
        public double zRotation = 0;
    }

    /**
     * Arcade drive method for calculating drivetrain output.
     * <p>
     * defined as positive forward on both outputs and turning right yields positive
     * left output and negative right output
     * 
     * @param xSpeed    desired travel velocity
     * @param zRotation desired rotational velocity
     * @return a drivesignal for open loop use
     */
    private DriveSignal arcadeDrive(double xSpeed, double zRotation) {
        final double maxInput = Math.max(Math.max(Math.abs(xSpeed - zRotation), Math.abs(xSpeed + zRotation)), 1);

        final double rightMotorOutput = (xSpeed + zRotation) / maxInput;
        final double leftMotorOutput = (xSpeed - zRotation) / maxInput;

        return new DriveSignal(rightMotorOutput, leftMotorOutput);
    
    }
    
    
}