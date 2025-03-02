package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class AutoRobot {

    private Timer timer;
    
    public Slides slides;
    public IO io;

    public static double pivInit = 600, pivDown = 1700, pivUp = 0, pivSpec = 400, pivBack = -50;
    public static double extIn = 0, extMid = 600, extOut = 2475;

    public double extSpecI = 200, extSpecO = 1500;

    public boolean bool = false;
    
    public AutoRobot(HardwareMap map, Telemetry telemetry) {
        slides = new Slides(map, telemetry);
        io = new IO(map, telemetry);

        timer = new Timer();
    }
    
    public void update() {
        slides.update();
        io.update();
    }
    
    public double timer() {return timer.getElapsedTimeSeconds();}

    public class WaitSeconds implements Action {

        private double delay;

        public WaitSeconds(double delay) {
            this.delay = delay;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!bool) {
                timer.resetTimer();
                bool = true;
            }

            if (timer() > delay) {
                bool = false;
                return false;
            }

            return true;

        }
    }
    public Action waitSeconds(double delay) {return new WaitSeconds(delay);}

    public class Init implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            slides.setPivTarget(pivInit);
            slides.setExtTarget(extIn);

            io.init();
            io.clawClose();
            return false;
        }
    }
    public Action init() {return new Init();}

    public class Rest implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            slides.setPivTarget(pivUp);
            io.outtakeInit();
            return false;
        }
    }
    public Action rest() {return new Rest();}

    public class Straight implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            slides.setPivTarget(pivUp);
            io.straight();
            return false;
        }
    }
    public Action straight() {return new Straight();}

    public class PivDown implements Action {

        private double pos;

        public PivDown(double pos) {
            this.pos = pos;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            slides.setPivTarget(pos);
            slides.setExtTarget(extIn);
            io.specimenInit();
            return false;
        }
    }
    public Action pivDown(double pos) {return new PivDown(pos);}

    public class Update implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            slides.update();
            io.update();
            return true;
        }
    }
    public Action updateAction() {return new Update();}

    public class SamplePick implements Action {

        double delay;
        boolean rotate;

        public SamplePick(double delay, boolean rotate) {
            this.delay = delay;
            this.rotate = rotate;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!bool) {
                timer.resetTimer();
                slides.setExtTarget(extIn);
                slides.setPivTarget(pivDown);
                io.gbPos = io.gbSetter(slides.spools()[0].getCurrentPosition(), 0.025);
                io.pivPos = io.pivSetter(slides.spools()[0].getCurrentPosition(), 0.025);

                if (rotate) io.rotPos = 0.8;
                else io.rotPos = 0.5;

                bool = true;
            }

            if (timer() > delay - 0.5 && timer() < delay) {
                io.clawClose();
            }

            if (timer() > delay) {
                bool = false;
                return false;
            }
            return true;
        }
    }
    public Action samplePick(double delay, boolean rotate) {return new SamplePick(delay, rotate);}

    public class SampleScore implements Action {

        double delay;

        public SampleScore(double delay) {
            this.delay = delay;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!bool) {
                timer.resetTimer();
                slides.setPivTarget(pivBack);
                io.straight();

                bool = true;
            }

            if (timer() > 1 && timer() < delay - 1) {
                slides.setExtTarget(extOut);
            }

            if (timer() > delay - 1.5 && timer() < delay - 1) {
                io.outtakeInit();
                io.clawOpen();
            }

            if (timer() > delay - 1) {
                io.intakeInit();
                slides.setExtTarget(extIn);
            }

            if (timer() > delay) {
                bool = false;
                return false;
            }
            return true;
        }
    }
    public Action sampleScore(double delay) {return new SampleScore(delay);}
    
    public class SpecimenPickUp implements Action {
        
        double delay;
        double rotation;
        
        public SpecimenPickUp(double delay, double rotation) {
            this.delay = delay;
            this.rotation = rotation;
        }
        
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!bool) {
                timer.resetTimer();
                slides.setExtTarget(extIn);
                slides.setPivTarget(pivDown);
                io.gbPos = io.gbSetter(slides.spools()[0].getCurrentPosition(), 0.025);
                io.pivPos = io.pivSetter(slides.spools()[0].getCurrentPosition(), 0.025);
                
                io.rotPos = rotation;
                
                bool = true;
            }
            
            if (timer() > delay - 0.5 && timer() < delay) {
                io.clawClose();
            }
            
            if (timer() > delay) {
                bool = false;
                return false;
            }
            return true;
        }
    }
    public Action specimenPickUp(double delay, double rotation) {return new SpecimenPickUp(delay, rotation);}

    public class SpecimenPick implements Action {

        double delay;

        public SpecimenPick(double delay) {
            this.delay = delay;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!bool) {
                timer.resetTimer();
                slides.setPivTarget(pivBack);
                slides.setExtTarget(extSpecI);
                io.specimenInit();
                io.clawOpen();

                bool = true;
            }

            if (timer() > delay-0.5) {
                io.clawClose();
            }

            if (timer() > delay) {
                bool = false;
                return false;
            }
            return true;
        }
    }
    public Action specimenPick(double delay) {return new SpecimenPick(delay);}

    public class SpecimenScore implements Action {

        double delay;

        public SpecimenScore(double delay) {
            this.delay = delay;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!bool) {
                timer.resetTimer();
                slides.setPivTarget(pivBack);
                slides.setExtTarget(extSpecI);
                io.spec4auto();
                io.clawClose();

                bool = true;
            }

            if (timer() > delay-1 && timer() < delay) {
                slides.setExtTarget(extSpecO);
            }

            if (timer() > delay) {
                slides.setExtTarget(extSpecI);
                io.clawOpen();
                bool = false;
                return false;
            }
            return true;
        }
    }
    public Action specimenScore(double delay) {return new SpecimenScore(delay);}
    
    


    
}
