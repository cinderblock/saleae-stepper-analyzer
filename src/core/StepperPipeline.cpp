#include "StepperPipeline.h"

#include <algorithm>
#include <cmath>

namespace stepper
{
    namespace
    {
        PwmEstimator::Config EstimatorConfig( const PipelineConfig& config )
        {
            PwmEstimator::Config estimator;
            estimator.max_period = config.max_pwm_period * config.sample_rate;
            estimator.horizon = ( config.estimator_history + config.lookahead ) * config.sample_rate;
            return estimator;
        }

        DecoderConfig DecoderConfigFor( const PipelineConfig& config )
        {
            DecoderConfig decoder = config.decoder;
            decoder.min_energized_samples = uint64_t( std::llround( config.minimum_energized * config.sample_rate ) );
            return decoder;
        }
    }

    StepperPipeline::StepperPipeline( const PipelineConfig& config )
        : mConfig( config ), mEstimator( EstimatorConfig( config ), TERMINAL_COUNT ), mDecoder( DecoderConfigFor( config ) )
    {
        mLookahead = uint64_t( std::ceil( config.lookahead * config.sample_rate ) );
        if( mConfig.pwm_mode == PwmMode::Manual && mConfig.pwm_frequency > 0 )
            mStickyPeriod = mConfig.sample_rate / mConfig.pwm_frequency;
    }

    void StepperPipeline::Start( uint64_t start_sample, const bool initial_states[ TERMINAL_COUNT ] )
    {
        for( int i = 0; i < TERMINAL_COUNT; ++i )
        {
            mQueues[ i ].Reset( start_sample, initial_states[ i ] );
            mStates[ i ] = initial_states[ i ];
        }
        mCursor = start_sample;
        mFilled = start_sample;
        mFraction = 0;
    }

    void StepperPipeline::PushEdge( int channel, uint64_t sample )
    {
        mQueues[ channel ].Push( sample );
        mStates[ channel ] = !mStates[ channel ];
        if( mStates[ channel ] && mConfig.pwm_mode == PwmMode::Auto )
            mEstimator.AddRisingEdge( channel, sample );
    }

    void StepperPipeline::SetFilled( uint64_t sample )
    {
        mFilled = std::max( mFilled, sample );
    }

    double StepperPipeline::CurrentWindow()
    {
        if( mConfig.pwm_mode == PwmMode::Auto )
        {
            const double period = mEstimator.PeriodAt( mCursor );
            if( period > 0 )
                mStickyPeriod = period;
        }

        const double minimum = std::max( 1.0, mConfig.minimum_window * mConfig.sample_rate );
        if( mConfig.pwm_mode == PwmMode::None || mStickyPeriod <= 0 )
            return minimum;
        return std::max( minimum, mStickyPeriod * std::max( 1, mConfig.periods_per_window ) );
    }

    uint64_t StepperPipeline::NeededFill()
    {
        return mCursor + uint64_t( std::ceil( CurrentWindow() ) ) + mLookahead;
    }

    void StepperPipeline::Process( std::vector<Segment>& out )
    {
        while( DecodeOneWindow( false, out ) )
        {
        }
    }

    void StepperPipeline::Finish( std::vector<Segment>& out )
    {
        while( DecodeOneWindow( true, out ) )
        {
        }
        mDecoder.Flush( out );
    }

    bool StepperPipeline::DecodeOneWindow( bool finishing, std::vector<Segment>& out )
    {
        const double window = CurrentWindow();

        // Whole samples for this window; the fractional part carries into the next one so the
        // average window length stays an exact multiple of the PWM period.
        const double exact = window + mFraction;
        const uint64_t length = std::max<uint64_t>( 1, uint64_t( std::floor( exact ) ) );
        uint64_t end = mCursor + length;

        const uint64_t available = finishing ? mFilled : ( mFilled > mLookahead ? mFilled - mLookahead : 0 );
        if( end > available )
        {
            if( !finishing || mCursor >= mFilled )
                return false;
            end = mFilled;
        }
        mFraction = std::max( 0.0, exact - double( length ) );

        // Stretch over idle time: if no channel changes for several windows, they would all decode
        // identically, so cover them with one window.
        uint64_t next_edge = mFilled;
        for( const EdgeQueue& queue : mQueues )
            next_edge = std::min( next_edge, queue.NextEdge() );
        if( next_edge > end && next_edge - mCursor >= 2 * length )
        {
            end = mCursor + ( ( next_edge - mCursor ) / length ) * length;
            mFraction = 0;
        }

        double duty[ TERMINAL_COUNT ];
        const double width = double( end - mCursor );
        for( int i = 0; i < TERMINAL_COUNT; ++i )
            duty[ i ] = double( mQueues[ i ].ConsumeHighSamples( end ) ) / width;

        mDecoder.AddWindow( mCursor, end, duty, out );
        mCursor = end;
        return true;
    }
}
