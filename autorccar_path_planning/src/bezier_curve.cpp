#include "Glob_def.h"

Bezier::Bezier() {
    B0[X] = B0[Y] = 0;
    B1[X] = B1[Y] = 0;
    B2[X] = B2[Y] = 0;
    B3[X] = B3[Y] = 0;
    B4[X] = B4[Y] = 0;

    Gs = Vs = Cs = 0;
    Gg = Vg = Cg = 0;
}

Bezier::Bezier(double X0_, double Y0_, double X4_, double Y4_) {
    B0[X] = X0_;
    B0[Y] = Y0_;
    B4[X] = X4_;
    B4[Y] = Y4_;

    Gs = Vs = Cs = 0;
    Gg = Vg = Cg = 0;

    update_bezier_curve();
}

Bezier::Bezier(double* B0_, double* B4_) {
    memcpy(B0, B0_, sizeof(double) * 2);
    memcpy(B4, B4_, sizeof(double) * 2);

    Gs = Vs = Cs = 0;
    Gg = Vg = Cg = 0;

    update_bezier_curve();
}

Bezier::Bezier(double* B0_, double Gs_, double Vs_, double* B4_, double Gg_, double Vg_) {
    memcpy(B0, B0_, sizeof(double) * 2);
    memcpy(B4, B4_, sizeof(double) * 2);

    Gs = Gs_;
    Vs = Vs_;
    Cs = 0;
    Gg = Gg_;
    Vg = Vg_;
    Cg = 0;

    update_bezier_curve();
}

Bezier::Bezier(double* B0_, double Gs_, double Vs_, double Cs_, double* B4_, double Gg_, double Vg_, double Cg_) {
    memcpy(B0, B0_, sizeof(double) * 2);
    memcpy(B4, B4_, sizeof(double) * 2);

    Gs = Gs_;
    Vs = Vs_;
    Cs = Cs_;
    Gg = Gg_;
    Vg = Vg_;
    Cg = Cg_;

    update_bezier_curve();
}

Bezier::Bezier(double X0_, double Y0_, double Gs_, double Vs_, double Cs_, double X4_, double Y4_, double Gg_, double Vg_, double Cg_) {
    B0[X] = X0_;
    B0[Y] = Y0_;
    B4[X] = X4_;
    B4[Y] = Y4_;
    Gs = Gs_;
    Vs = Vs_;
    Cs = Cs_;
    Gg = Gg_;
    Vg = Vg_;
    Cg = Cg_;

    update_bezier_curve();
}

void Bezier::set_start_point(double X0_, double Y0_) {
    B0[X] = X0_;
    B0[Y] = Y0_;
}

void Bezier::set_start_velocity(double Gs_, double Vs_) {
    Gs = Gs_;
    Vs = Vs_;
}

void Bezier::set_start_curvature(double Cs_) {
    Cs = Cs_;
}

void Bezier::set_goal_point(double X4_, double Y4_) {
    B4[X] = X4_;
    B4[Y] = Y4_;
}

void Bezier::set_goal_velocity(double Gg_, double Vg_) {
    Gg = Gg_;
    Vg = Vg_;
}

void Bezier::set_goal_curvature(double Cg_) {
    Cg = Cg_;
}

void Bezier::update_bezier_curve() {

    double X0, X1, X2, X3, X4;
    double Y0, Y1, Y2, Y3, Y4;

    X0 = B0[X];
    Y0 = B0[Y];
    X4 = B4[X];
    Y4 = B4[Y];

    double d = sqrt(DEF_SQ(X4 - X0) + DEF_SQ(Y4 - Y0)) / 4.0;

    if (Cs >= 0) {
        X1 = X0 + d / sqrt(1 + DEF_SQ(Gs));
    }
    else {
        X1 = X0 - d / sqrt(1 + DEF_SQ(Gs));
    }

    if (Cg >= 0) {
        X3 = X4 + d / sqrt(1 + DEF_SQ(Gg));
    }
    else {
        X3 = X4 - d / sqrt(1 + DEF_SQ(Gg));
    }

    Y1 = Y0 + Gs*(X1 - X0);
    Y3 = Y4 + Gg*(X3 - X4);

    X2 = 2*X1 - X0 - (4*Gs*Cs*(DEF_SQ(X1-X0)+DEF_SQ(Y1-Y0)))/(3*(X1-X0+Gs*(Y1-Y0)));
    Y2 = 2*Y1 - Y0 + (2*X1-X2-X0)/(Gs);

    B1[X] = X1;
    B1[Y] = Y1;
    B2[X] = X2;
    B2[Y] = Y2;
    B3[X] = X3;
    B3[Y] = Y3;

}

Bezier::~Bezier() {
}