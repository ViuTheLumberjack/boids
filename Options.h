//
// Created by viu on 28/01/2025.
//

#ifndef OPTIONS_H
#define OPTIONS_H

struct BoidOptions {
    static int constexpr BoidNum = 1000;

    static double constexpr TurnFactor = 0.5;
    static double constexpr VisualRange = 100;
    static double constexpr ProtectedRange = 15;
    static double constexpr AvoidFactor = 0.05;
    static double constexpr CenteringFactor = 0.0005;
    static double constexpr MatchingFactor = 0.05;

    static double constexpr MaxSpeed = 100;
    static double constexpr MinSpeed = 40;
};

struct ScreenOptions {
    static int constexpr WindowWidth = 1800;
    static int constexpr WindowHeight = 900;

    static int constexpr FrameRate = 60;
    static double constexpr LeftMargin = 300;
    static double constexpr RightMargin = 300;
    static double constexpr BottomMargin = 300;
    static double constexpr TopMargin = 300;
};

#endif //OPTIONS_H
