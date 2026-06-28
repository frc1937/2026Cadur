package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.LEDStrip;
import frc.lib.util.flippable.Flippable;

import java.util.EnumMap;

import static frc.robot.utilities.PortsConstants.LEDSTRIP_PORT_PWM;

public class Leds extends SubsystemBase {
    private static final int FRONT_LENGTH = 42;
    private static final int BACK_LENGTH = 26;
    private static final int TOTAL_LENGTH = FRONT_LENGTH + BACK_LENGTH;

    private static final int[] RED_ALLIANCE_COLOURS_FLIPPED = {0x0000FF, 0xFF0000};//{0x8B0000, 0xFF0000, 0x8B0000, 0xFF8C00};
    private static final int[] RED_ALLIANCE_COLOURS = {0x8B0000, 0xFF0000, 0x8B0000, 0xFF8C00}; //for police: {0xFF0000, 0x0000FF};//
    private static final int[] BLUE_ALLIANCE_COLOURS = {0x87CEEB, 0x6495ED, 0x0000FF, 0xADD8E6, 0xC0C0C0};

    // Modes ordered lowest → highest priority. LAST active entry wins.
    public enum LEDMode {
        DEFAULT {
            @Override
            public void apply(Leds leds) {
                int[] c = Flippable.isRedAlliance() ? RED_ALLIANCE_COLOURS : BLUE_ALLIANCE_COLOURS;
//                leds.front.flashing(RED_ALLIANCE_COLOURS);
//                leds.back.flashing(RED_ALLIANCE_COLOURS_FLIPPED); //POLICIA!
                leds.front.interpolated(c);
                leds.back.interpolated(c);
            }
        },
        AUTO_START {
            @Override
            public void apply(Leds leds) {
                leds.front.interpolated(0xFFFFFF, 0x00FFFF);
                leds.back.interpolated(0xFFFFFF, 0x00FFFF);
            }
        },

        /**
         * Intake is deployed and rolling — actively collecting fuel.
         */
        INTAKE_DEPLOYED {
            @Override
            public void apply(Leds leds) {
                leds.front.pulse(0x3C0055, 3);
                leds.back.pulse(0x3C0055, 3);
            }
        },
        /**
         * Intake deployed but roller stopped (DEPLOYED_NO_ROLLER).
         */
        INTAKE_DEPLOYED_NO_ROLLER {
            @Override
            public void apply(Leds leds) {
                leds.front.breathing(0xB400FF, 0x3C0055);
                leds.back.breathing(0xB400FF, 0x3C0055);
            }
        },

        /**
         * Flywheel at speed AND turret locked — ready to fire immediately.
         */
        READY_TO_SHOOT {
            @Override
            public void apply(Leds leds) {
                leds.front.interpolated(0x0000FF, 0x00FFFF, 0xFFFFFF, 0x00FFFF, 0x0000FF, 0xB400FF);
                leds.back.interpolated(0x0000FF, 0x00FFFF, 0xFFFFFF, 0x00FFFF, 0x0000FF, 0xB400FF);
            }
        },
        /**
         * Actively shooting at hub.
         */
        SHOOTING_HUB {
            @Override
            public void apply(Leds leds) {
                leds.front.rainbow();
                leds.back.rainbow();
            }
        },
        /**
         * Passing mode active — lobbing to alliance partner.
         */
        PASSING {
            @Override
            public void apply(Leds leds) {
                leds.front.theatreChase(0xFFFF00, 4);
                leds.back.theatreChase(0xFFFF00, 4);
            }
        },

        // ── Hub / match state ─────────────────────────────────────────────
        /**
         * Hub is active but robot is not in the alliance zone yet.
         */
        HUB_ACTIVE_NOT_IN_ZONE {
            @Override
            public void apply(Leds leds) {
                leds.front.pulse(0xFF8C00, 8);
                leds.back.pulse(0xFF8C00, 8);
            }
        },
        /**
         * < 3 seconds remaining in active hub period.
         */
        SHIFT_ENDING {
            @Override
            public void apply(Leds leds) {
                leds.front.strobe(0xFF0000, 3);
                leds.back.strobe(0xFF0000, 3);
            }
        },
        END_OF_MATCH {
            @Override
            public void apply(Leds leds) {
                leds.front.outwardsPoints(0xFFD700);
                leds.back.strobe(0xFFD700, 4);
            }
        },

        // ── Operator alerts (require human action) ────────────────────────
        /**
         * Operator has toggled ignore-hub-state override — reminder it's on.
         */
        OVERRIDE_ACTIVE {
            @Override
            public void apply(Leds leds) {
                leds.front.breathing(0xFF00FF, 0x000000);
                leds.back.breathing(0xFF00FF, 0x000000);
            }
        },
        /**
         * FMS game data not received — operator must set auto winner manually.
         */
        NO_GAME_DATA {
            @Override
            public void apply(Leds leds) {
                leds.front.flashing(0xFF0000, 0xFFFFFF);
                leds.back.flashing(0xFF0000, 0xFFFFFF);
            }
        },

        // ── Critical (always visible) ─────────────────────────────────────
        BATTERY_LOW {
            @Override
            public void apply(Leds leds) {
                leds.front.pulse(0xFF00FF, 10);
                leds.back.pulse(0xFF00FF, 10);
            }
        };

        public abstract void apply(Leds leds);
    }

    // Cached to avoid per-periodic array allocation from LEDMode.values().
    private static final LEDMode[] MODES = LEDMode.values();

    private final AddressableLED ledstrip = new AddressableLED(LEDSTRIP_PORT_PWM);
    private final AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(TOTAL_LENGTH);

    final LEDStrip front = new LEDStrip(FRONT_LENGTH);
    final LEDStrip back = new LEDStrip(BACK_LENGTH);

    // boolean[] indexed by ordinal — avoids EnumMap boxing/unboxing overhead.
    private final boolean[] activeRequests = new boolean[MODES.length];

    public Leds() {
        ledstrip.setLength(TOTAL_LENGTH);
        ledstrip.setData(ledBuffer);
        ledstrip.start();
    }

    @Override
    public void periodic() {
        LEDMode toDisplay = LEDMode.DEFAULT;

        for (int i = MODES.length - 1; i >= 0; i--) {
            if (activeRequests[i]) {
                toDisplay = MODES[i];
                break;
            }
        }

        toDisplay.apply(this);

        front.writeToBuffer(ledBuffer, 0);
        back.writeToBuffer(ledBuffer, FRONT_LENGTH);

        ledstrip.setData(ledBuffer);
    }

    public Command showFor(LEDMode mode, double durationSeconds) {
        return show(mode).withTimeout(durationSeconds);
    }

    public Command show(LEDMode mode) {
        int ord = mode.ordinal();
        return Commands.startEnd(
                () -> activeRequests[ord] = true,
                () -> activeRequests[ord] = false,
                this
        );
    }
}