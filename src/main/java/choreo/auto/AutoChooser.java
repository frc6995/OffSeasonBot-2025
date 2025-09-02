package choreo.auto;

import static edu.wpi.first.wpilibj.Alert.AlertType.kError;


import java.util.HashMap;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import java.util.function.Supplier;

import choreo.util.ChoreoAlert;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;


public class AutoChooser implements Sendable {
    static final String NONE_NAME = "Default";
    private static final Alert selectedNonexistentAuto =
        ChoreoAlert.alert("Selected an auto that isn't an option", kError);
  
    private final HashMap<String, Supplier<Command>> autoRoutines =
        new HashMap<>(Map.of(NONE_NAME, Commands::none));
  
    private String selected = NONE_NAME;
    private String[] options = new String[] {NONE_NAME};
  
    private Optional<Alliance> allianceAtGeneration = Optional.empty();
    private String nameAtGeneration = NONE_NAME;
    private Command generatedCommand = Commands.none();

    public AutoChooser() {}

    public String select(String selectStr) {
        return select(selectStr, false);
      }
    
      private String select(String selectStr, boolean force) {
        selected = selectStr;
        if (selected.equals(nameAtGeneration)
            && allianceAtGeneration.equals(DriverStation.getAlliance())) {
          // early return if the selected auto matches the active auto
          return nameAtGeneration;
        }
        boolean dsValid = DriverStation.isDisabled() && DriverStation.getAlliance().isPresent();
        if (dsValid || force) {
          if (!autoRoutines.containsKey(selected) && !selected.equals(NONE_NAME)) {
            selected = NONE_NAME;
            selectedNonexistentAuto.set(true);
          } else {
            selectedNonexistentAuto.set(false);
          }
          allianceAtGeneration = DriverStation.getAlliance();
          nameAtGeneration = selected;
          generatedCommand = autoRoutines.get(nameAtGeneration).get().withName(nameAtGeneration);
        } else {
          allianceAtGeneration = Optional.empty();
          nameAtGeneration = NONE_NAME;
          generatedCommand = Commands.none();
        }
        return nameAtGeneration;
      }

  /**
   * Add an AutoRoutine to the chooser.
   *
   * <p>This is done to load AutoRoutines when and only when they are selected, in order to save
   * memory and file loading time for unused AutoRoutines.
   *
   * <p>The generators are only run when the DriverStation is disabled and the alliance is known.
   *
   * <p>One way to keep this clean is to make an `Autos` class that all of your subsystems/resources
   * are <a href="https://en.wikipedia.org/wiki/Dependency_injection">dependency injected</a> into.
   * Then create methods inside that class that take an {@link AutoFactory} and return an {@link
   * AutoRoutine}.
   *
   * <h3>Example:</h3>
   *
   * <pre><code>
   * AutoChooser chooser;
   * Autos autos = new Autos(swerve, shooter, intake, feeder);
   * public Robot() {
   *   chooser = new AutoChooser("/Choosers");
   *   SmartDashboard.putData(chooser);
   *   // fourPieceRight is a method that accepts an AutoFactory and returns an AutoRoutine.
   *   chooser.addRoutine("4 Piece right", autos::fourPieceRight);
   *   chooser.addRoutine("4 Piece Left", autos::fourPieceLeft);
   *   chooser.addRoutine("3 Piece Close", autos::threePieceClose);
   * }
   * </code></pre>
   *
   * @param name The name of the auto routine.
   * @param generator The function that generates the auto routine.
   */
  public void addRoutine(String name, Supplier<AutoRoutine> generator) {
    autoRoutines.put(name, () -> generator.get().cmd());
    options = autoRoutines.keySet().toArray(new String[0]);
  }

  /**
   * Adds a Command to the auto chooser.
   *
   * <p>This is done to load autonomous commands when and only when they are selected, in order to
   * save memory and file loading time for unused autonomous commands.
   *
   * <p>The generators are only run when the DriverStation is disabled and the alliance is known.
   *
   * <h3>Example:</h3>
   *
   * <pre><code>
   * AutoChooser chooser;
   * Autos autos = new Autos(swerve, shooter, intake, feeder);
   * public Robot() {
   *   chooser = new AutoChooser("/Choosers");
   *   SmartDashboard.putData(chooser);
   *   // fourPieceLeft is a method that accepts an AutoFactory and returns a command.
   *   chooser.addCmd("4 Piece left", autos::fourPieceLeft);
   *   chooser.addCmd("Just Shoot", shooter::shoot);
   * }
   * </code></pre>
   *
   * @param name The name of the autonomous command.
   * @param generator The function that generates an autonomous command.
   * @see AutoChooser#addRoutine
   */
  public void addCmd(String name, Supplier<Command> generator) {
    autoRoutines.put(name, generator);
    options = autoRoutines.keySet().toArray(new String[0]);
  }

  public void setDefaultCmd(Supplier<Command> generator) {
    autoRoutines.put(NONE_NAME, generator);
    options = autoRoutines.keySet().toArray(new String[0]);
  }

  /**
   * Gets a Command that schedules the selected auto routine. This Command shares the lifetime of
   * the scheduled Command. This Command can directly be bound to a trigger, like so:
   *
   * <pre><code>
   *     AutoChooser chooser = ...;
   *
   *     public Robot() {
   *         RobotModeTriggers.autonomous().whileTrue(chooser.selectedCommandScheduler());
   *     }
   * </code></pre>
   *
   * @return A command that runs the selected {@link AutoRoutine}
   */
  public Command selectedCommandScheduler() {
    return Commands.defer(() -> selectedCommand().asProxy(), Set.of());
  }

  /**
   * Returns the currently selected command.
   *
   * <p>If you plan on using this {@link Command} in a {@code Trigger} it is recommended to use
   * {@link #selectedCommandScheduler()} instead.
   *
   * @return The currently selected command.
   */
  public Command selectedCommand() {
    if (RobotBase.isSimulation() && nameAtGeneration == NONE_NAME) {
      select(selected, true);
    }
    return generatedCommand;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("String Chooser");
    builder.publishConstBoolean(".controllable", true);
    builder.publishConstString("default", NONE_NAME);
    builder.addStringArrayProperty("options", () -> options, null);
    builder.addStringProperty("selected", null, this::select);
    builder.addStringProperty("active", () -> select(selected), null);
  }

}