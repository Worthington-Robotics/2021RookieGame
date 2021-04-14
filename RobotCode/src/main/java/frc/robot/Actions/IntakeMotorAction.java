package frc.robot.Actions;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.Shooter;
import frc.robot.Constants;


public class IntakeMotorAction extends Action {

    @Override
    public void onStart() {
        Shooter.getInstance().setIntakePower(Constants.ID_SUPER_INTAKE);
        Shooter.getInstance().setWantIntake(true);
    }

    @Override
    public void onLoop() {
        Shooter.getInstance().setIntakePower(Constants.ID_SUPER_INTAKE);
        Shooter.getInstance().setWantIntake(true);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void onStop() {
        Shooter.getInstance().setWantIntake(false);
        Shooter.getInstance().setIntakePower(0);
    }
}
