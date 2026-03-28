package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import static edu.wpi.first.units.Units.*;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Constants {

        public static class FieldConstants {
                public static final Distance FIELD_LENGTH = Inches.of(650.12);
                public static final Distance FIELD_WIDTH = Inches.of(316.64);

                public static final Distance ALLIANCE_ZONE = Inches.of(156.06);

                public static final Translation3d HUB_BLUE = new Translation3d(Inches.of(181.56), FIELD_WIDTH.div(2),
                                Inches.of(56.4));
                public static final Translation3d HUB_RED = new Translation3d(FIELD_LENGTH.minus(Inches.of(181.56)),
                                FIELD_WIDTH.div(2), Inches.of(56.4));
                public static final Distance FUNNEL_RADIUS = Inches.of(24);
                public static final Distance FUNNEL_HEIGHT = Inches.of(72 - 56.4);

                public static final Translation2d STOCKPILE_BLUE = new Translation2d(0.5, 0.5); // Bottom left corner
                public static final Translation2d STOCKPILE_RED = new Translation2d(FIELD_LENGTH.in(Meters) - 0.5, 0.5); // Bottom
                                                                                                                         // right
                                                                                                                         // corner
        }

        public static class VisionConstants {

                public static final Translation2d SHOOTER_OFFSET_2D = new Translation2d(0.18375715, 0.0);
                public static final Transform2d ROBOT_TO_LAUNCHER = new Transform2d(
                                SHOOTER_OFFSET_2D,
                                Rotation2d.fromDegrees(180));

                // How far forward the camera is from the center of the robot (meters)
                public static final double kCameraXOffset = 0.25;
                // How far left/right the camera is (0 if centered)
                public static final double kCameraYOffset = 0.0;

                public static final double SHOOTING_DISTANCE_METERS = 3.25;

                // ---------- FRONT (POSE) CAMERA ----------
                public static final String FRONT_CAMERA_NAME = "frontCamera";
                public static final Transform3d ROBOT_TO_FRONT_CAMERA = new Transform3d(
                                new Translation3d(0.3, 0.0, 0.2),
                                new Rotation3d(0, 0, 0));

                // ---------- SHOOTER (DISTANCE + POSE) CAMERA ----------
                public static final String SHOOTER_CAMERA_NAME = "shooterCamera";
                public static final Transform3d ROBOT_TO_SHOOTER_CAMERA = new Transform3d(
                                new Translation3d(0.26486640, 0.12246703, 0.41597024), // above shooter
                                new Rotation3d(0, Units.degreesToRadians(-15), Units.degreesToRadians(180)) // pitched
                                                                                                            // down
                );

                // ---------- SHOOTER DOCKING ----------
                public static final double SHOOTER_YAW_KP = 0.039;
                public static final double SHOOTER_YAW_KD = 0;

                public static final double SHOOTER_Strafe_KP = 0.8;
                public static final double SHOOTER_Strafe_KD = 0;
                public static final double SHOOTER_STRAFE_TOLERANCE_M = 1; // 5 cm

                public static final double SHOOTER_DISTANCE_KP = 2;
                public static final double SHOOTER_DISTANCE_KD = 0.04;

                public static final double SHOOTER_YAW_TOLERANCE_RAD = Units.degreesToRadians(3.5);

                public static final double SHOOTER_DISTANCE_TOLERANCE_M = 0.5; // 5 cm

                public static final double SHOOTER_MAX_TRANSLATION_SPEED = 3.5; // m/s
                public static final double SHOOTER_MAX_ANGULAR_SPEED = Math.PI; // rad/s

                // ---------- VISION UNCERTAINTY ----------
                public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(4, 4, 8);
                public static final Matrix<N3, N1> MULTI_TAG_STD_DEVS = VecBuilder.fill(0.5, 0.5, 1);

                // ---------- HUB TAGS ----------
                public static final int[] RED_HUB_TAGS = { 2, 3, 4, 5, 8, 9, 10, 11 };
                public static final int[] BLUE_HUB_TAGS = { 18, 19, 20, 21, 24, 25, 26, 27 };

                // ---------- TOWER TAGS ----------
                public static final int[] RED_TOWER_TAGS = { 15, 16 };
                public static final int[] BLUE_TOWER_TAGS = { 31, 32 };;

        }

        public final static class Lighting {
                public final static int lightingPort = 1;

                public enum Colors {
                        // HOTPINK ("Hot Pink", 0.57),
                        // DARKRED ("Dark Red", 0.59),
                        RED("Red", 0.61),
                        // REDORANGE ("Red Orange", 0.63),
                        ORANGE("Orange", 0.65),
                        // GOLD ("Gold", 0.67),
                        // YELLOW ("Yellow", 0.69),
                        // LAWNGREEN ("Lawn Green", 0.71),
                        LIME("Lime", 0.73),
                        // DARKGREEN ("Dark Green", 0.75),
                        GREEN("Green", 0.77),
                        // BLUEGREEN ("Blue Green", 0.79),
                        // AQUA ("Aqua", 0.81),
                        // SKYBLUE ("Sky Blue", 0.83),
                        // DARKBLUE ("Dark Blue", 0.85),
                        BLUE("Blue", 0.87),
                        BLUEVIOLET("Blue Violet", 0.89),
                        VIOLET("Violet", 0.91),
                        WHITE("White", 0.93),
                        GRAY("Gray", 0.95),
                        DARKGRAY("Dark Gray", 0.97),
                        RAINBOW("Rainbow", -0.99),
                        RAINBOWGLITTER("Rainbow- Glitter", -0.89),
                        BLINK_RED("Blink Red", -0.11), // Heartbeat Red (Good for "Motor Hot")
                        // RAINBOWSINELON ("Rainbow - Sinelon", -0.79),
                        // RAINBOWBEATS ("Rainbow - Beats Per Minute", -0.69),
                        // RAINBOWTWINKLES ("Rainbow - Tinkles", -0.55),
                        // RAINBOWWAVES ("Rainbow - Color Waves", -0.45),
                        // REDCHASE ("Light Chase - Red", -0.31),
                        // BLUECHASE ("Light Chase - Blue", -0.29),
                        OFF("Off", 0.99);

                        public final String colorName;
                        public final double colorValue;

                        Colors(String colorName, double colorValue) {
                                this.colorName = colorName;
                                this.colorValue = colorValue;
                        }

                        public String getColorName() {
                                return this.colorName;
                        }

                        public double getColorValue() {
                                return this.colorValue;
                        }
                }

                public final static Colors startingColor = Colors.OFF;
        }

}