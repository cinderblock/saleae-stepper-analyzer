#ifndef STEPPER_PWM_ESTIMATOR_H
#define STEPPER_PWM_ESTIMATOR_H

#include <cstdint>
#include <deque>
#include <vector>

namespace stepper
{
    // Detects the PWM (chopper) period from rising-edge intervals of the drive channels.
    //
    // Every rising edge yields an interval since the previous rising edge on the same channel.
    // Within a sliding time horizon, the intervals are considered PWM when enough of them are
    // short (below `max_period`) and cluster tightly around their median. Center-aligned PWM with
    // changing duty jitters each interval slightly, which the median and the tolerance absorb.
    class PwmEstimator
    {
      public:
        struct Config
        {
            double max_period = 0;      // samples; longer intervals are step timing, not PWM
            double horizon = 0;         // samples; intervals older than this before the query point are dropped
            int min_intervals = 8;      // minimum number of short intervals to call it PWM
            double tolerance = 0.05;    // fraction of the median an interval may deviate and still agree
            double min_agreement = 0.7; // fraction of the short intervals that must agree with the median
        };

        explicit PwmEstimator( const Config& config, int channel_count );

        void AddRisingEdge( int channel, uint64_t sample );

        // Estimated PWM period in samples around `sample`, or 0 when no PWM is detected.
        double PeriodAt( uint64_t sample );

      private:
        struct Interval
        {
            uint64_t sample;
            double length;
        };

        Config mConfig;
        std::vector<uint64_t> mLastRise;
        std::vector<bool> mHaveRise;
        std::deque<Interval> mIntervals;

        bool mDirty = true;
        double mCachedPeriod = 0;
        std::vector<double> mScratch;
    };
}

#endif // STEPPER_PWM_ESTIMATOR_H
