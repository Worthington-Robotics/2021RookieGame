package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Joystick.Axis;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.lib.loops.ILooper;
import frc.lib.loops.Loop;
import frc.lib.util.DriveSignal;
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
    public Drive() {
        leftMaster = new TalonSRX(RoboMAP.leftMasterPort);
        leftSlave = new TalonSRX(RoboMAP.leftSlavePort);

        rightMaster = new TalonSRX(RoboMAP.rightMasterPort);
        rightSlave = new TalonSRX(RoboMAP.rightSlavePort);

        leftSlave.follow(leftMaster);
        rightSlave.follow(rightMaster);

        stick = new Joystick(RoboMAP.joystickPort);
    }



  
    /*
    * Updates all periodic variables and sensors
    */

    public void readPeriodicInputs() {
        double xSpeed = stick.getX(Hand.kRight);
        double zRotation = stick.getY(Hand.kRight);
      //  ((Drive) stick).readPeriodicInputs();
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

                }
        }
        );
    }

    /**
     * Writes the periodic outputs to actuators (motors and ect...)
     */
    public void writePeriodicOutputs() {
        double rightMotorOutput = rightMaster.getMotorOutputVoltage(); 
        double leftMotorOutput = leftMaster.getMotorOutputVoltage();
    }

    /**
     * Outputs all logging information to the SmartDashboard
     */
    public void outputTelemetry() {

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