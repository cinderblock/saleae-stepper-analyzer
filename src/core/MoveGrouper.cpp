#include "MoveGrouper.h"

namespace stepper
{
    MoveGrouper::MoveGrouper( uint64_t hold_samples ) : mHoldSamples( hold_samples )
    {
    }

    void MoveGrouper::Add( const Segment& segment, std::vector<Segment>& out )
    {
        const bool short_position = segment.type == SegmentType::Position && segment.end - segment.start < mHoldSamples;

        if( !short_position )
        {
            // A hold, an off span or an ambiguous jump ends any move in progress. A hold is where the
            // move arrived; otherwise the move ends where its last segment was.
            if( mInMove )
                EndMove( segment.type == SegmentType::Position ? segment.position : mLastPosition, out );

            out.push_back( segment );
            if( segment.type == SegmentType::Position )
            {
                mPosition = segment.position;
                mHavePosition = true;
            }
            else
            {
                mHavePosition = false;
            }
            return;
        }

        if( !mInMove )
        {
            mInMove = true;
            mMove = segment;
            mMove.type = SegmentType::Move;
            mMove.from_position = mHavePosition ? mPosition : segment.position;
            mMoveDriveSum = 0;
            mMoveSamples = 0;
            mMoveCount = 0;
            mFirst = segment;
        }

        const double width = double( segment.end - segment.start );
        mMove.end = segment.end;
        mMoveDriveSum += segment.drive * width;
        mMoveSamples += width;
        mLastPosition = segment.position;
        ++mMoveCount;
    }

    void MoveGrouper::Flush( std::vector<Segment>& out )
    {
        if( mInMove )
            EndMove( mLastPosition, out );
    }

    void MoveGrouper::EndMove( double to, std::vector<Segment>& out )
    {
        mInMove = false;

        // A single short span that went nowhere is just a short position, not a move.
        if( mMoveCount == 1 && to == mMove.from_position )
        {
            out.push_back( mFirst );
            mPosition = to;
            mHavePosition = true;
            return;
        }

        Segment move = mMove;
        move.position = to;
        move.mean_position = to;
        move.delta = to - move.from_position;
        move.direction = move.delta > 0 ? 1 : ( move.delta < 0 ? -1 : 0 );
        move.drive = mMoveSamples > 0 ? mMoveDriveSum / mMoveSamples : 0;
        out.push_back( move );

        mPosition = to;
        mHavePosition = true;
    }
}
