#ifndef STEPPER_EDGE_QUEUE_H
#define STEPPER_EDGE_QUEUE_H

#include <cstdint>
#include <deque>

namespace stepper
{
    // Transitions of one digital channel that have been read ahead of the integration cursor.
    //
    // The analyzer reads edges from Logic a little ahead of where it is decoding (so the PWM
    // estimator can see what is coming); this queue holds them until the integrator consumes them.
    class EdgeQueue
    {
      public:
        void Reset( uint64_t start_sample, bool initial_state );

        // Record a transition at `sample`: the channel holds the new state from `sample` onward.
        // Samples must be pushed in increasing order and must not be before the cursor.
        void Push( uint64_t sample );

        // Number of samples in [cursor, end) during which the channel is high. Consumes the range.
        uint64_t ConsumeHighSamples( uint64_t end );

        // First buffered transition after the cursor, or UINT64_MAX when none is buffered.
        uint64_t NextEdge() const;

        bool State() const
        {
            return mState;
        }

        uint64_t Cursor() const
        {
            return mCursor;
        }

      private:
        std::deque<uint64_t> mEdges;
        uint64_t mCursor = 0;
        bool mState = false;
    };
}

#endif // STEPPER_EDGE_QUEUE_H
