#include <ros/ros.h>
#include <numeric>

struct FIRDiffFilter {
    std::vector<double> history;
    int step;
    double coll_sig, time_const;

    FIRDiffFilter(int history_size=15, double time_constant=0.01) : 
                                      history(history_size), step(0), coll_sig(0.0),
                                      time_const(time_constant) {
    }
    double updateState(double z_obs) {
        // find the average value over the past few steps
        double avg = std::accumulate(history.begin(), history.end(), 0.0) / history.size();
        // exponential zeroing filter
        // rapidly increases with every observation far from average
        coll_sig = time_const * (coll_sig + z_obs - avg);
        // add this observation to the history buffer
        history[step % history.size()] = z_obs;
        step++;
        // return current state
        return std::fabs(coll_sig);
    }
};
