package frc.robot.Actions;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.Shooter;

public class BallGateAction extends Action {

    @Override
    public void onStart() {
        Shooter.getInstance().setWantBall(true);
    }

    @Override
    public void onLoop() {
        Shooter.getInstance().setWantBall(true);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void onStop() {
        Shooter.getInstance().setWantBall(false);
    }
}
