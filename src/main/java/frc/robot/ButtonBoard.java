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
        processorHeightButton(5),
        bargeHeightButton(6),
        forceEjectCoralButton(7),
        leftReefLineupButton(8),
        rightReefLineupButton(9),
        goForAlgaeButton(10),
        climbCommandButton(11),
        climbModeAuthorizer(12);

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
    public boolean getProcessorHeightButton(){
        return getRawButton(Button.processorHeightButton.value);
    }
    public boolean getBargeHeightButton(){
        return getRawButton(Button.bargeHeightButton.value);
    }
    public boolean getForceEjectCoralButton() {
        return getRawButton(Button.forceEjectCoralButton.value);
    }
    public boolean getLeftReefLineupButton(){
        return getRawButton(Button.leftReefLineupButton.value);
    }
    public boolean getRightReefLineupButton(){
        return getRawButton(Button.rightReefLineupButton.value);
    }
    public boolean getGoForAlgaeButton() {
        return getRawButton(Button.goForAlgaeButton.value);
    }
    public boolean getclimbCommandButton(){
        return getRawButton(Button.climbCommandButton.value);
    }
    public boolean getClimbModeAuthorizer() {
        return getRawButton(Button.climbModeAuthorizer.value);
    }
    public void initSendable(SendableBuilder builder){
        builder.setSmartDashboardType("HID");
        builder.addBooleanProperty("Reef Height 1 Button", this::getReefHeight1Button, null);
        builder.addBooleanProperty("Reef Height 2 Button", this::getReefHeight2Button, null);
        builder.addBooleanProperty("Reef Height 3 Button", this::getReefHeight3Button, null);
        builder.addBooleanProperty("Reef Height 4 Button", this::getReefHeight4Button, null);
        builder.addBooleanProperty("Processor Height Button", this::getProcessorHeightButton, null);
        builder.addBooleanProperty("Barge Height Button", this::getBargeHeightButton, null);
        builder.addBooleanProperty("Force Eject Coral Button", this::getForceEjectCoralButton, null);
        builder.addBooleanProperty("Left Reef Lineup Button", this::getLeftReefLineupButton, null);
        builder.addBooleanProperty("Right Reef Lineup Button", this::getRightReefLineupButton, null);
        builder.addBooleanProperty("Go For Algae Button", this::getGoForAlgaeButton, null);
        builder.addBooleanProperty("Climb Command Button", this::getclimbCommandButton, null);
        builder.addBooleanProperty("Climb Mode Authorizer", this::getClimbModeAuthorizer, null);

    }
}
