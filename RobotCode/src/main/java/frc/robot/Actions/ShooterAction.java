package frc.robot.Actions;

import frc.lib.statemachine.Action;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class ShooterAction extends Action {

    @Override
    public void onStart() {
        Shooter.getInstance().setShooterPower(Constants.SHOOTER_FLYWHEEL_LEFT);
    }

    @Override
    public void onLoop() {
        Shooter.getInstance().setShooterPower(Constants.SHOOTER_FLYWHEEL_LEFT);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void onStop() {
        Shooter.getInstance().setShooterPower(0);
    }
}
