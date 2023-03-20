package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

public class BoolSupplierDriveUntilAngle implements BooleanSupplier{
    boolean Returned;
    /**Changes Boolean returned */
    public void SetValue(boolean bool) {
        Returned = bool;
    }
    @Override
    public boolean getAsBoolean() {
        // TODO Auto-generated method stub
        return Returned;
    }
    
}
