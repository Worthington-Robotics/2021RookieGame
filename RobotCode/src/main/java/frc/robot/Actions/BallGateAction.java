package frc.robot.Actions;

import frc.lib.statemachine.Action;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class BallGateAction extends Action {

    @Override
    public void onStart() {
        Shooter.getInstance().setWantBall(true);
        Shooter.getInstance().setBallGatePower(Constants.ID_SUPER_DELIVERY_WHEEL);
        Shooter.getInstance().setShooterPower(Constants.SHOOTER_FLYWHEEL_LEFT);
    }

    @Override
    public void onLoop() {
        Shooter.getInstance().setWantBall(true);
        Shooter.getInstance().setBallGatePower(Constants.ID_SUPER_DELIVERY_WHEEL);
        Shooter.getInstance().setShooterPower(Constants.SHOOTER_FLYWHEEL_LEFT);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void onStop() {
        Shooter.getInstance().setWantBall(false);
        Shooter.getInstance().setBallGatePower(0);
        Shooter.getInstance().setShooterPower(0);
    }
}
