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
        climbModeAuthorizer(12),
        climberResetButton(13),
        reefPoleAButton(14),
        reefPoleBButton(15),
        reefPoleCButton(16),
        reefPoleDButton(17),
        reefPoleEButton(18),
        reefPoleFButton(19),
        reefPoleGButton(20),
        reefPoleHButton(21),
        reefPoleIButton(22),
        reefPoleJButton(23),
        reefPoleKButton(24),
        reefPoleLButton(25);
        
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
    public boolean getClimberResetButton(){
        return getRawButton(Button.climberResetButton.value);
    }
    public boolean getReefPoleAButton(){
        return getRawButton(Button.reefPoleAButton.value);
    }
    public boolean getReefPoleBButton(){
        return getRawButton(Button.reefPoleBButton.value);
    }
    public boolean getReefPoleCButton(){
        return getRawButton(Button.reefPoleCButton.value);
    }
    public boolean getReefPoleDButton(){
        return getRawButton(Button.reefPoleDButton.value);
    }
    public boolean getReefPoleEButton(){
        return getRawButton(Button.reefPoleEButton.value);
    }
    public boolean getReefPoleFButton(){
        return getRawButton(Button.reefPoleFButton.value);
    }
    public boolean getReefPoleGButton(){
        return getRawButton(Button.reefPoleGButton.value);
    }
    public boolean getReefPoleHButton(){
        return getRawButton(Button.reefPoleHButton.value);
    }
    public boolean getReefPoleIButton(){
        return getRawButton(Button.reefPoleIButton.value);
    }
    public boolean getReefPoleJButton(){
        return getRawButton(Button.reefPoleJButton.value);
    }
    public boolean getReefPoleKButton(){
        return getRawButton(Button.reefPoleKButton.value);
    }
    public boolean getReefPoleLButton(){
        return getRawButton(Button.reefPoleLButton.value);
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
        builder.addBooleanProperty("Climber Reset Button", this::getClimberResetButton, null);
        builder.addBooleanProperty("Reef Pole A Button", this::getReefPoleAButton, null);
        builder.addBooleanProperty("Reef Pole B Button", this::getReefPoleBButton, null);
        builder.addBooleanProperty("Reef Pole C Button", this::getReefPoleCButton, null);
        builder.addBooleanProperty("Reef Pole D Button", this::getReefPoleDButton, null);
        builder.addBooleanProperty("Reef Pole E Button", this::getReefPoleEButton, null);
        builder.addBooleanProperty("Reef Pole F Button", this::getReefPoleFButton, null);
        builder.addBooleanProperty("Reef Pole G Button", this::getReefPoleGButton, null);
        builder.addBooleanProperty("Reef Pole H Button", this::getReefPoleHButton, null);
        builder.addBooleanProperty("Reef Pole I Button", this::getReefPoleIButton, null);
        builder.addBooleanProperty("Reef Pole J Button", this::getReefPoleJButton, null);
        builder.addBooleanProperty("Reef Pole K Button", this::getReefPoleKButton, null);
        builder.addBooleanProperty("Reef Pole L Button", this::getReefPoleLButton, null);
    }
}
