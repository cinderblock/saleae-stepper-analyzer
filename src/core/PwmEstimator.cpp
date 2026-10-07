#include "PwmEstimator.h"

#include <algorithm>
#include <cmath>

namespace stepper
{
    PwmEstimator::PwmEstimator( const Config& config, int channel_count )
        : mConfig( config ), mLastRise( channel_count, 0 ), mHaveRise( channel_count, false )
    {
    }

    void PwmEstimator::AddRisingEdge( int channel, uint64_t sample )
    {
        if( mHaveRise[ channel ] )
        {
            const double length = double( sample - mLastRise[ channel ] );
            if( length <= mConfig.max_period )
            {
                // Edges from different channels arrive out of order, so keep the deque sorted.
                Interval interval{ sample, length };
                auto it = mIntervals.end();
                while( it != mIntervals.begin() && ( it - 1 )->sample > sample )
                    --it;
                mIntervals.insert( it, interval );
                mDirty = true;
            }
        }
        mLastRise[ channel ] = sample;
        mHaveRise[ channel ] = true;
    }

    double PwmEstimator::PeriodAt( uint64_t sample )
    {
        const double oldest = double( sample ) - mConfig.horizon;
        while( !mIntervals.empty() && double( mIntervals.front().sample ) < oldest )
        {
            mIntervals.pop_front();
            mDirty = true;
        }

        if( !mDirty )
            return mCachedPeriod;
        mDirty = false;
        mCachedPeriod = 0;

        if( int( mIntervals.size() ) < mConfig.min_intervals )
            return mCachedPeriod;

        mScratch.clear();
        for( const Interval& interval : mIntervals )
            mScratch.push_back( interval.length );

        auto mid = mScratch.begin() + mScratch.size() / 2;
        std::nth_element( mScratch.begin(), mid, mScratch.end() );
        const double median = *mid;

        // Average the agreeing intervals for sub-sample precision.
        double sum = 0;
        int agree = 0;
        for( double length : mScratch )
        {
            if( std::fabs( length - median ) <= mConfig.tolerance * median )
            {
                sum += length;
                ++agree;
            }
        }

        if( agree >= mConfig.min_intervals && agree >= mConfig.min_agreement * double( mScratch.size() ) )
            mCachedPeriod = sum / agree;

        return mCachedPeriod;
    }
}
