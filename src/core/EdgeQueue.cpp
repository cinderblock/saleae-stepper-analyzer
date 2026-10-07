#include "EdgeQueue.h"

#include <limits>

namespace stepper
{
    void EdgeQueue::Reset( uint64_t start_sample, bool initial_state )
    {
        mEdges.clear();
        mCursor = start_sample;
        mState = initial_state;
    }

    void EdgeQueue::Push( uint64_t sample )
    {
        mEdges.push_back( sample );
    }

    uint64_t EdgeQueue::ConsumeHighSamples( uint64_t end )
    {
        uint64_t high = 0;

        while( !mEdges.empty() && mEdges.front() < end )
        {
            const uint64_t edge = mEdges.front();
            mEdges.pop_front();

            if( edge > mCursor )
            {
                if( mState )
                    high += edge - mCursor;
                mCursor = edge;
            }
            mState = !mState;
        }

        if( end > mCursor )
        {
            if( mState )
                high += end - mCursor;
            mCursor = end;
        }

        return high;
    }

    uint64_t EdgeQueue::NextEdge() const
    {
        return mEdges.empty() ? std::numeric_limits<uint64_t>::max() : mEdges.front();
    }
}
