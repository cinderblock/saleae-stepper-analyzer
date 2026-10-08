#ifndef STEPPER_MOVE_GROUPER_H
#define STEPPER_MOVE_GROUPER_H

#include "StepperDecoder.h"

#include <cstdint>
#include <vector>

namespace stepper
{
    // Collapses runs of short position segments into single moves.
    //
    // A position segment lasting at least `hold_samples` is a hold and passes through unchanged.
    // Consecutive shorter ones are a move: one Move segment from the position before the move to
    // the position after it. Zoomed out, a move then reads as "0 -> -142.25" instead of hundreds
    // of frames too narrow to label.
    class MoveGrouper
    {
      public:
        explicit MoveGrouper( uint64_t hold_samples );

        // Feed segments in order; finished segments are appended to `out`.
        void Add( const Segment& segment, std::vector<Segment>& out );

        // Emit whatever is still buffered (end of data).
        void Flush( std::vector<Segment>& out );

        // Decoding has reached `now`, and `open` (when `has_open`) is the segment the decoder is in
        // but has not emitted yet. A move in progress ends as soon as that is a position held for the
        // hold time, or the motor is off, rather than when the hold eventually ends.
        void Tick( uint64_t now, bool has_open, const Segment& open, std::vector<Segment>& out );

      private:
        void EndMove( double to, std::vector<Segment>& out );

        uint64_t mHoldSamples;

        bool mInMove = false;
        Segment mMove;
        double mMoveDriveSum = 0;
        double mMoveSamples = 0;
        double mLastPosition = 0; // position of the last segment of the move so far
        int mMoveCount = 0;       // segments in the move so far
        Segment mFirst;           // the move's first segment, as it arrived

        bool mHavePosition = false;
        double mPosition = 0; // last held position
    };
}

#endif // STEPPER_MOVE_GROUPER_H
