//
// Created by viu on 28/01/2025.
//

#ifndef OPTIONS_H
#define OPTIONS_H

struct BoidOptions {
    int BoidNum;
    int MaxV;
    float VisualRange;

    float TurnFactor = 0.5;
    float ProtectedRange = 15;
    float AvoidFactor = 0.05;
    float CenteringFactor = 0.0005;
    float MatchingFactor = 0.05;
    float MaxSpeed = 100;
    float MinSpeed = 40;
};

struct ScreenOptions {
    static int constexpr WindowWidth = 1800;
    static int constexpr WindowHeight = 900;

    static int constexpr FrameRate = 30;
    static float constexpr LeftMargin = 300;
    static float constexpr RightMargin = 300;
    static float constexpr BottomMargin = 300;
    static float constexpr TopMargin = 300;
};

#endif //OPTIONS_H
