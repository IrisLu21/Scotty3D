
#include "../geometry/spline.h"
#include "debug.h"

template<typename T>
T Spline<T>::cubic_unit_spline(float time, const T& position0, const T& position1,
                               const T& tangent0, const T& tangent1) {

    // TODO (Animation): Task 1a
    // Given time in [0,1] compute the cubic spline coefficients and use them to compute
    // the interpolated value at time 'time' based on the positions & tangents

    // Note that Spline is parameterized on type T, which allows us to create splines over
    // any type that supports the * and + operators.
    float t2 = (float) pow(time, 2);
    float t3 = (float) pow(time, 3);
    float h00 = 2*t3 - 3*t2 + 1;
    float h10 = t3 - 2*t2 + time;
    float h01 = -2*t3 + 3*t2; 
    float h11 = t3 - t2;

    return h00*position0 + h10*tangent0 + h01*position1 + h11*tangent1;
}

template<typename T> T Spline<T>::at(float time) const {

    // TODO (Animation): Task 1b

    // Given a time, find the nearest positions & tangent values
    // defined by the control point map.

    // Transform them for use with cubic_unit_spline

    // Be wary of edge cases! What if time is before the first knot,
    // before the second knot, etc...
    printf("enter\n");
    if (!any()) {
        printf("empty\n");
        return T();
    } else if (control_points.size() == 1) {
        printf("one\n");
        auto t = *(control_points.begin());
        printf("get begin\n");
        T result = t.second;
        printf("done\n");
        return result;
    }
    auto lastIt = std::prev(control_points.end());
    T first = (*control_points.begin()).second;
    T last = (*lastIt).second;
    float firstTime = (*control_points.begin()).first;
    float lastTime = (*lastIt).first;
    if (time <= firstTime) {
        printf("<= first\n");
        return first;
    }
    if (time >= lastTime) {
        printf(">= last\n");
        return last;
    }
    printf("2 knots\n");
    auto k2 = control_points.upper_bound(time); //iterator to pair of (float,T)
    auto k1 = std::prev(k2);
    T k1Val=(*k1).second;
    T k2Val = (*k2).second;
    T k0Val;
    T k3Val;

    float t1 = (*k1).first;
    float t2 = (*k2).first;
    float t0;
    float t3;
    if (k1 == control_points.begin()) {
        printf("k1 == first\n");
        //k1 is the first one
        k0Val = k1Val - (k2Val - k1Val);
        t0 = t1 - (t2 - t1);
    } else {
        k0Val = (*std::prev(k1)).second;
        t0 = (*std::prev(k1)).first;
    }
    if (k2 == lastIt) {
        printf("k2 == last\n");
        k3Val = k2Val + (k2Val - k1Val);
        t3 = t2 + (t2 - t1);
    } else {
        k3Val = (*std::next(k2)).second;
        t3 = (*std::next(k2)).first;
    }
    T m1 = (k2Val - k0Val) / (t2 - t0);
    T m2 = (k3Val - k1Val) / (t3 - t1);

    float t = (time - t1) / (t2 - t1);
    float scale = t2 - t1;

    return cubic_unit_spline(t, k1Val, k2Val, m1 * scale, m2 * scale);
}
