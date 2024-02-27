package org.firstinspires.ftc.teamcode.util;

public class robotConstants
{
    public static class outtakeSlide
    {
        public static double P = 0.0;
        public static double I = 0.0;
        public static double D = 0.0;

        public static int MOSTHIGHLEFT = 1585;
        public static int MOSTHIGHRIGHT = 1585;

        public static int HIGHLEFT = 1585;
        public static int HIGHRIGHT = 1585;

        public static int MEDIUMLEFT = 1300;
        public static int MEDIUMRIGHT = 1300;
        public static int LOWMEDLEFT = 1000;
        public static int LOWMEDRIGHT = 1000;

        public static int LOWLEFT = 675;
        public static int LOWRIGHT = 675;

        public static int AUTOLOWLEFT = 100;
        public static int AUTOLOWRIGHT = 100;

        public static int GROUNDLEFT = 0;
        public static int GROUNDRIGHT = 0;
    }

    public static class activeIntake
    {
        public static double
                active = -1;
        public static double reverseActive = 1;

        public static double autoActive = -.7;
        public static double autoReverseActive = .7;
    }

    public static class Wrist
    {
        public static double outtaking = .365;
        public static double intaking = .785;
        public static double init = .42;
    }
    public static class Lid
    {
        public static double close = 0;
        public static double open = 0.2;
    }

    public static class Arm
    {
        public static double intakingLeft = 0.006;

        public static double intakingRight = 0.043;


        public static double intakingInitLeft = .5;

        public static double intakingInitRight = .5;

        public static double outtakingLeft = .52;
        public static double outtakingRight = .53;

        public static double initLeft = .21;

        public static double initRight = .2;

        public static double outtakingDownLeft = .62;
        public static double outtakingDownRight = .08;

        public static double autoDropLeft = .57;
        public static double autoDropRight = .15;
    }

    public static class linkage
    {
        public static double supaRus = .77;
        public static double highPosition = .84;
        public static double mediumPosition = .875;
        public static double lowPosition = .935;

    }

    public static class drone{
        public static double droneLaunch = .4;
        public static double droneReset = .6;
    }

    public static class holderServo{
            public static double open = 0;
            public static double close = .50;
    }

}