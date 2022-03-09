
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

    float t = time;
    float t_2 = t * t;
    float t_3 = t_2 * t;
    float h00 = 2.f * t_3 - 3.f * t_2 + 1.f;
    float h10 = t_3 - 2.f * t_2 + t;
    float h01 = -2.f * t_3 + 3.f * t_2;
    float h11 = t_3 - t_2;

    return position0 * h00 + tangent0 * h10 + position1 * h01 + tangent1 * h11;
}

template<typename T> T Spline<T>::at(float time) const {

    // TODO (Animation): Task 1b

    // Given a time, find the nearest positions & tangent values
    // defined by the control point map.

    // Transform them for use with cubic_unit_spline

    // Be wary of edge cases! What if time is before the first knot,
    // before the second knot, etc...
    
    if(control_points.empty()) {
        return T();
    }
    if(control_points.size() == 1) {
        return control_points.begin()->second;
    }
    auto first = control_points.begin();
    if(time <= first->first) {
        return first->second;
    }
    auto last = --control_points.end();
    if(time >= last->first) {
        return last->second;
    }

    auto itr2 = control_points.upper_bound(time);
    auto itr1 = itr2;
    --itr1;
    auto itr3 = itr2;
    if(itr3 != last) {
        ++itr3;
    }
    auto itr0 = itr1;
    if(itr0 != first) {
        --itr0;
    }
    float t0, t1, t2, t3;
    T p0, p1, p2, p3;
    t0 = itr0->first;
    p0 = itr0->second;
    t1 = itr1->first;
    p1 = itr1->second;
    t2 = itr2->first;
    p2 = itr2->second;
    t3 = itr3->first;
    p3 = itr3->second;
    if(itr0 == itr1) {
        t0 = t1 - (t2 - t1);
        p0 = p1 - (p2 - p1);
    }
    if(itr2 == itr3) {
        t3 = t2 + (t2 - t1);
        p3 = p2 + (p2 - p1);
    }
    T tangent1 = (p2 - p0) / (t2 - t0);
    T tangent2 = (p3 - p1) / (t3 - t1);

    return cubic_unit_spline((time - t1) / (t2 - t1), p1, p2, tangent1 * (t2 - t1),
                             tangent2 * (t2 - t1));
}
