package frc.robot.Actions;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.Shooter;

public class IntakeExtensionAction extends Action {
    @Override
    public void onStart() {
        Shooter.getInstance().setWantIntake(true);
    }

    @Override
    public void onLoop() {
        Shooter.getInstance().setWantIntake(true);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void onStop() {
        Shooter.getInstance().setWantIntake(false);
    }
}
