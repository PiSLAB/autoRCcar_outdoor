#ifndef __BEZIER_CURVE_H__
#define __BEZIER_CURVE_H__

#define B0_INIT_X   0
#define B0_INIT_Y   0
#define B0_INIT_G   0.1
#define B0_INIT_V   0
#define B0_INIT_C   0.01

#define B4_INIT_X   100
#define B4_INIT_Y   100
#define B4_INIT_G   0.1
#define B4_INIT_V   0
#define B4_INIT_C   -0.01

class Bezier {
   private:

   protected:
   public:

double B0[2], B1[2], B2[2], B3[2], B4[2];
    double Gs, Vs, Cs;
    double Gg, Vg, Cg;

    Bezier();  // constructor
    Bezier(double X0_, double Y0_, double X4_, double Y4_);
    Bezier(double* B0_, double* B4_);
    Bezier(double* B0_, double Gs_, double Vs_, double* B4_, double Gg_, double Vg_);
    Bezier(double* B0_, double Gs_, double Vs_, double Cs_, double* B4_, double Gg_, double Vg_, double Cg_);
    Bezier(double X0_, double Y0_, double Gs_, double Vs_, double Cs_, double X4_, double Y4_, double Gg_, double Vg_, double Cg_);

    void set_start_point(double X0, double Y0);
    void set_start_velocity(double Gs, double Vs);
    void set_start_curvature(double Cs);
    void set_goal_point(double X4, double Y4);
    void set_goal_velocity(double Gg, double Vg);
    void set_goal_curvature(double Cg);

    void update_bezier_curve();

    ~Bezier();  // destructor
};

#endif