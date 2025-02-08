package frc.robot;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.GenericHID;

public class ButtonBoard extends GenericHID implements Sendable{
    /** 
     * All the button enums. 
    */
    public enum Button{
        reefHeight1Button(1),
        reefHeight2Button(2),
        reefHeight3Button(3),
        reefHeight4Button(4),
        coralIntakeHeightButton(5),
        bargeHeightButton(6),
        leftReefLineupButton(7),
        rightReefLineupButton(8),
        climbCommandButton(9),
        goForAlgaeButton(10),
        forceEjectCoralButton(11),
        forceElevatorButton(12);

        public final int value;
        
        Button(int value){
            this.value=value;
        }
    }
    public ButtonBoard(final int port){
        super(port);
    }
    public boolean getReefHeight1Button(){
        return getRawButton(Button.reefHeight1Button.value);
    }
    public boolean getReefHeight2Button(){
        return getRawButton(Button.reefHeight2Button.value);
    }
    public boolean getReefHeight3Button(){
        return getRawButton(Button.reefHeight3Button.value);
    }
    public boolean getReefHeight4Button(){
        return getRawButton(Button.reefHeight4Button.value);
    }
    public boolean getCoralIntakeHeightButton(){
        return getRawButton(Button.coralIntakeHeightButton.value);
    }
    public boolean getBargeHeightButton(){
        return getRawButton(Button.bargeHeightButton.value);
    }
    public boolean getLeftReefLineupButton(){
        return getRawButton(Button.leftReefLineupButton.value);
    }
    public boolean getRightReefLineupButton(){
        return getRawButton(Button.rightReefLineupButton.value);
    }
    public void initSendable(SendableBuilder builder){
        builder.setSmartDashboardType("HID");
        builder.addBooleanProperty("Reef Height 1 Button", this::getReefHeight1Button, null);
        builder.addBooleanProperty("Reef Height 2 Button", this::getReefHeight2Button, null);
        builder.addBooleanProperty("Reef Height 3 Button", this::getReefHeight3Button, null);
        builder.addBooleanProperty("Reef Height 4 Button", this::getReefHeight4Button, null);
        builder.addBooleanProperty("Coral Intake Height Button", this::getCoralIntakeHeightButton, null);
        builder.addBooleanProperty("Barge Height Button", this::getBargeHeightButton, null);
    }
}
