//
// Created by viu on 28/01/2025.
//

#ifndef OPTIONS_H
#define OPTIONS_H

struct BoidOptions {
    static int constexpr BoidNum = 50;
    static double constexpr TurnFactor = 0.2;
    static double constexpr VisualRange = 100;

    static double constexpr ProtectedRange = 10;
    static double constexpr AvoidFactor = 0.05;
    static double constexpr CenteringFactor = 0.0005;
    static double constexpr MatchingFactor = 0.05;

    static double constexpr MaxSpeed = 75;
    static double constexpr MinSpeed = 25;

    static double constexpr MaxBias = 0.01;
    static double constexpr BiasIncrement = 0.00004;
    static double constexpr BiasVal = 0.001;
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
