#ifndef __CUBICSPLINE_H__
#define __CUBICSPLINE_H__

class cubicSpline {
    public:
        typedef enum BoundType_e {
            BOUNDTYPE_FIRST_DERIVATIVE,
            BOUNDTYPE_SECOND_DERIVATIVE,
        }BoundType;

        cubicSpline();
        ~cubicSpline();

        void initParam();
        void releaseMem();
        bool loadData(double *x_data, double *y_data, int count, double bound1, double bound2, BoundType type);
        bool getYbyX(double &x_in, double &y_out);
        
    protected:
        double *x_sample_, *y_sample_;
        double *M_;
        int sample_count_;
        double bound1_, bound2_;

        bool spline(BoundType type);
};
#endif
