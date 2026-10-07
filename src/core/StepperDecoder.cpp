#include "StepperDecoder.h"

#include <algorithm>
#include <cmath>

namespace stepper
{
    namespace
    {
        const double kPi = 3.14159265358979323846;

        // Wrap degrees into [-180, 180).
        double Wrap180( double degrees )
        {
            return degrees - 360.0 * std::floor( ( degrees + 180.0 ) / 360.0 );
        }
    }

    StepperDecoder::StepperDecoder( const DecoderConfig& config ) : mConfig( config )
    {
    }

    double StepperDecoder::AngleDegrees( double a, double b )
    {
        return std::atan2( b, a ) * 180.0 / kPi;
    }

    double StepperDecoder::Quantize( double position ) const
    {
        return std::round( position / mConfig.resolution ) * mConfig.resolution;
    }

    void StepperDecoder::AddWindow( uint64_t start, uint64_t end, const double duty[ TERMINAL_COUNT ], std::vector<Segment>& out )
    {
        Window window;
        window.start = start;
        window.end = end;
        window.a = duty[ A_POS ] - duty[ A_NEG ];
        window.b = duty[ B_POS ] - duty[ B_NEG ];
        window.drive = std::hypot( window.a, window.b );
        mLastEnd = end;

        if( window.drive < mConfig.energized_threshold )
        {
            if( mPendingSamples >= mConfig.min_energized_samples )
            {
                FlushPending( out );
            }
            else
            {
                // Energized windows that did not last long enough were switching glitches: they
                // become part of the de-energized span.
                for( const Window& pending : mPending )
                    AddOff( pending, out );
                mPending.clear();
                mPendingSamples = 0;
            }
            AddOff( window, out );
            return;
        }

        if( mHaveAngle )
        {
            AddEnergized( window, out );
            return;
        }

        // Coming out of de-energized: hold the windows back until the drive has lasted long enough
        // to be real. The very first time, also wait for the drive to settle before choosing the
        // zero reference.
        uint64_t needed = mConfig.min_energized_samples;
        if( !mEverEnergized && mConfig.start_at_zero )
            needed = std::max( needed, mConfig.zero_settle_samples );

        mPending.push_back( window );
        mPendingSamples += end - start;
        if( mPendingSamples >= needed )
            FlushPending( out );
    }

    void StepperDecoder::FlushPending( std::vector<Segment>& out )
    {
        if( !mEverEnergized && mConfig.start_at_zero && !mPending.empty() )
        {
            // Zero is where the motor sits once the drive has settled: the last held-back window.
            double previous = AngleDegrees( mPending.front().a, mPending.front().b );
            double unwrapped = previous / 90.0;
            for( const Window& pending : mPending )
            {
                const double angle = AngleDegrees( pending.a, pending.b );
                unwrapped += Wrap180( angle - previous ) / 90.0;
                previous = angle;
            }
            mOffset = -unwrapped;
            mOffsetChosen = true;
        }

        for( const Window& pending : mPending )
            AddEnergized( pending, out );
        mPending.clear();
        mPendingSamples = 0;
    }

    void StepperDecoder::AddOff( const Window& window, std::vector<Segment>& out )
    {
        mHaveAngle = false;
        if( !mHaveOpen || mOpen.type != SegmentType::Off )
        {
            if( mHaveOpen )
                Close( window.start, 0, out );
            Open( SegmentType::Off, window.start, 0 );
        }
        const double width = double( window.end - window.start );
        mSumDrive += window.drive * width;
        mSumSamples += width;
    }

    void StepperDecoder::AddEnergized( const Window& window, std::vector<Segment>& out )
    {
        const uint64_t start = window.start;
        const uint64_t end = window.end;
        const double drive = window.drive;
        const double width = double( end - start );
        const double angle = AngleDegrees( window.a, window.b );
        bool ambiguous = false;

        if( !mEverEnergized )
        {
            mUnwrapped = angle / 90.0;
            if( !mOffsetChosen )
                mOffset = mConfig.start_at_zero ? -mUnwrapped : 0;
            mEverEnergized = true;
        }
        else if( !mHaveAngle )
        {
            // Re-energized after being off: the rotor is assumed to have stayed within the nearest
            // electrical cycle (4 full steps) of where it was left.
            double candidate = angle / 90.0;
            candidate += 4.0 * std::round( ( mUnwrapped - candidate ) / 4.0 );
            ambiguous = std::fabs( candidate - mUnwrapped ) > 1.5;
            mUnwrapped = candidate;
        }
        else
        {
            const double delta = Wrap180( angle - mPreviousAngle );
            ambiguous = std::fabs( delta ) > mConfig.ambiguous_degrees;
            mUnwrapped += delta / 90.0;
        }

        mPreviousAngle = angle;
        mHaveAngle = true;

        const double position = ( mUnwrapped + mOffset ) * ( mConfig.invert ? -1.0 : 1.0 );

        if( ambiguous )
        {
            if( mHaveOpen )
                Close( start, 0, out );
            Open( SegmentType::Ambiguous, start, Quantize( position ) );
        }
        else if( !mHaveOpen || mOpen.type != SegmentType::Position )
        {
            if( mHaveOpen )
                Close( start, 0, out );
            Open( SegmentType::Position, start, Quantize( position ) );
        }
        else if( std::fabs( position - mOpen.position ) > mConfig.resolution * ( 0.5 + mConfig.hysteresis ) )
        {
            const double quantized = Quantize( position );
            Close( start, quantized - mOpen.position, out );
            Open( SegmentType::Position, start, quantized );
        }

        mSumPosition += position * width;
        mSumDrive += drive * width;
        mSumSamples += width;

        // An ambiguous span covers only the window that jumped.
        if( ambiguous )
            Close( end, 0, out );
    }

    void StepperDecoder::Flush( std::vector<Segment>& out )
    {
        FlushPending( out );
        if( mHaveOpen )
            Close( mLastEnd, 0, out );
    }

    void StepperDecoder::Open( SegmentType type, uint64_t start, double position )
    {
        mOpen = Segment();
        mOpen.type = type;
        mOpen.start = start;
        mOpen.position = position;
        mHaveOpen = true;
        mSumPosition = 0;
        mSumDrive = 0;
        mSumSamples = 0;
    }

    void StepperDecoder::Close( uint64_t end, double delta, std::vector<Segment>& out )
    {
        mHaveOpen = false;
        if( end <= mOpen.start || mSumSamples <= 0 )
            return;

        Segment segment = mOpen;
        segment.end = end;
        segment.delta = delta;
        segment.direction = delta > 0 ? 1 : ( delta < 0 ? -1 : 0 );
        segment.drive = mSumDrive / mSumSamples;

        if( segment.type == SegmentType::Off )
        {
            segment.position = 0;
        }
        else
        {
            segment.mean_position = mSumPosition / mSumSamples;
            const double electrical = segment.mean_position * ( mConfig.invert ? -1.0 : 1.0 ) - mOffset;
            segment.angle_degrees = Wrap180( electrical * 90.0 );
        }

        out.push_back( segment );
    }
}
