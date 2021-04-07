package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.drivers.SimTimeOfFlight;
import frc.lib.loops.ILooper;
import frc.lib.loops.Loop;
import frc.lib.util.HIDHelper;
import frc.lib.util.Logable;
import frc.robot.Constants;

import javax.naming.ldap.Control;

public class Shooter extends Subsystem {
    PeriodicIO periodicIO;
    TalonSRX ballGate;
   // DoubleSolenoid intakeExtension;
    TalonSRX intakeMotor;
    TalonSRX shooterMotorLeft;
    TalonSRX shooterMotorRight;
    static final Shooter shooterInstance = new Shooter();


    private Shooter() {
       // intakeExtension = new DoubleSolenoid(Constants.SHOOTER_HIGH_ID, Constants.SHOOTER_LOW_ID);
        intakeMotor = new TalonSRX(Constants.ID_SUPER_INTAKE);
        shooterMotorLeft = new TalonSRX(Constants.SHOOTER_FLYWHEEL_LEFT);
        shooterMotorRight = new TalonSRX(Constants.SHOOTER_FLYWHEEL_RIGHT);
       // ballGate = new TalonSRX(Constants.SHOOTER_LOW_ID);
        reset();
    }

    public static Shooter getInstance() {
        return shooterInstance;
    }

    /**
     * Updates all periodic variables and sensors
     */


    public void readPeriodicInputs() {
        periodicIO.operatorInput = HIDHelper.getAdjStick(Constants.MASTER_STICK);
    }


    /**
     * Required for the subsystem's looper to be registered to the state machine
     * not required for subsystems that do not use looper
     *
     * @param enabledLooper the subsystem's Looper
     */
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onLoop(double timestamp) {
                switch (periodicIO.state) {
                    //case intake state: a ball will be intake
                    case INTAKE_STATE:
                        if (!periodicIO.wantIntake) {
                            periodicIO.state = IndexerState.IDLE_STATE;
                        }
                        //case idle state: nothing will happen and the double solenoid is forward so the ball won't enter the shooter
                    case IDLE_STATE:
                        if (periodicIO.shooterWantBall) {
                            periodicIO.state = IndexerState.SHOOT_STATE;
                        } else if (periodicIO.wantIntake) {
                            periodicIO.state = IndexerState.INTAKE_STATE;
                        }
                        //case shoot state: the double solenoid is reverse so a ball can be entered into the shooter
                    case SHOOT_STATE:
                        if (!periodicIO.shooterWantBall) {
                            periodicIO.state = IndexerState.IDLE_STATE;
                        }
                        break;
                }
            }

            //the double solenoid is either forward or reverse
            //5 outputs: double solenoid for intake, double solenoid for ball gate, motor for intake, and two motors for the shooter
            @Override
            public void onStart(double timestamp) {

            }

            @Override
            public void onStop(double timestamp) {

            }
        });
    }

    /**
     * Writes the periodic outputs to actuators (motors and ect...)
     */
    public void writePeriodicOutputs() {
        //ballGate.set(periodicIO.extendedBallGate);
        //intakeExtension.set(periodicIO.extendedIntake);
        intakeMotor.set(ControlMode.PercentOutput, periodicIO.intakeMotorPower);
        shooterMotorLeft.set(ControlMode.PercentOutput, periodicIO.shooterMotorLeftPower);
        shooterMotorRight.set(ControlMode.PercentOutput, periodicIO.shooterMotorRightPower);
    }

    /**
     * Outputs all logging information to the SmartDashboard
     */
    public void outputTelemetry() {
        SmartDashboard.putBoolean("Shooter Wants Ball: ", periodicIO.shooterWantBall);
        SmartDashboard.putBoolean("Robot Wants Intake: ", periodicIO.wantIntake);
        SmartDashboard.putNumber("Intake Motor Power: ", periodicIO.intakeMotorPower);
        SmartDashboard.putNumber("Shooter's Left Motor Power", periodicIO.shooterMotorLeftPower);
        SmartDashboard.putNumber("Shooter's Right Motor Power", periodicIO.shooterMotorRightPower);
    }

    /**
     * Called to reset and configure the subsystem
     */
    public void reset() {
        periodicIO = new PeriodicIO();
    }

    /**
     * Called to stop the autonomous functions of the subsystem and place it in open loop
     */
    public void onStop() {
    }

    public LogData getLogger() {
        return null;
    }

    public void setWantBall(boolean wantBall){
        periodicIO.shooterWantBall = wantBall;
    }

    public void setWantIntake(boolean wantIntake){
       periodicIO.wantIntake = wantIntake;
    }

    public void setShooterPower(double motorDemand){ periodicIO.shooterMotorLeftPower = motorDemand;}

    public void setIntakePower(double intakeDemand){ periodicIO.intakeMotorPower = intakeDemand;}


    public enum IndexerState {
        DISABLED, INTAKE_STATE, IDLE_STATE, SHOOT_STATE;

        public String toString() {
            return name().charAt(0) + name().substring(1).toLowerCase();
        }
    }

    /**
     * Inheritable data class for capturing log data from the subsystems
     */
    public class PeriodicIO extends Logable.LogData {
        public double[] operatorInput = {0, 0, 0};

       // DoubleSolenoid.Value extendedBallGate = TalonSRXControlMode;
        //DoubleSolenoid.Value extendedIntake = DoubleSolenoid.Value.kReverse;
        public double intakeMotorPower;
        public double shooterMotorLeftPower;
        public double shooterMotorRightPower;

        public IndexerState state = IndexerState.DISABLED;
        public boolean shooterWantBall;
        public boolean wantIntake;
    }

}
