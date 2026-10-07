#include "StepperMotorCoilsAnalyzer.h"

#include <AnalyzerChannelData.h>

#include "StepperPipeline.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace
{
    const char* const kAnalyzerName = "Stepper Motor Coils";

    const char* FrameTypeName( stepper::SegmentType type )
    {
        switch( type )
        {
        case stepper::SegmentType::Position:
            return "position";
        case stepper::SegmentType::Off:
            return "off";
        case stepper::SegmentType::Ambiguous:
            return "ambiguous";
        }
        return "position";
    }
}

StepperMotorCoilsAnalyzer::StepperMotorCoilsAnalyzer()
    : Analyzer2(), mSettings(), mSimulationInitialized( false ), mSampleRateHz( 0 ), mHavePreviousPosition( false ), mPreviousPosition( 0 )
{
    SetAnalyzerSettings( &mSettings );
    UseFrameV2();
}

StepperMotorCoilsAnalyzer::~StepperMotorCoilsAnalyzer()
{
    KillThread();
}

void StepperMotorCoilsAnalyzer::SetupResults()
{
    // SetupResults is called each time the analyzer is run. Because the same instance can be used for multiple runs, we need to clear the
    // results each time.
    mResults.reset( new StepperMotorCoilsAnalyzerResults( this, &mSettings ) );
    SetAnalyzerResults( mResults.get() );
    mResults->AddChannelBubblesWillAppearOn( mSettings.mChannels[ stepper::A_POS ] );
}

void StepperMotorCoilsAnalyzer::WorkerThread()
{
    mSampleRateHz = GetSampleRate();
    mHavePreviousPosition = false;

    AnalyzerChannelData* channels[ stepper::TERMINAL_COUNT ];
    bool initial[ stepper::TERMINAL_COUNT ];
    U64 start = 0;
    for( int i = 0; i < stepper::TERMINAL_COUNT; ++i )
    {
        channels[ i ] = GetAnalyzerChannelData( mSettings.mChannels[ i ] );
        start = std::max( start, channels[ i ]->GetSampleNumber() );
    }
    for( int i = 0; i < stepper::TERMINAL_COUNT; ++i )
    {
        channels[ i ]->AdvanceToAbsPosition( start );
        initial[ i ] = channels[ i ]->GetBitState() == BIT_HIGH;
    }

    stepper::StepperPipeline pipeline( mSettings.MakePipelineConfig( double( mSampleRateHz ) ) );
    pipeline.Start( start, initial );

    std::vector<stepper::Segment> segments;
    U64 filled = start;

    for( ;; )
    {
        // Read every channel up to the point the pipeline needs. Advancing blocks until the
        // capture has data there, which is how a live capture is followed.
        const U64 needed = pipeline.NeededFill();
        if( needed > filled )
        {
            for( int i = 0; i < stepper::TERMINAL_COUNT; ++i )
            {
                AnalyzerChannelData* data = channels[ i ];
                while( data->WouldAdvancingToAbsPositionCauseTransition( needed ) )
                {
                    data->AdvanceToNextEdge();
                    pipeline.PushEdge( i, data->GetSampleNumber() );
                }
                data->AdvanceToAbsPosition( needed );
            }
            filled = needed;
            pipeline.SetFilled( filled );
        }

        segments.clear();
        pipeline.Process( segments );

        for( const stepper::Segment& segment : segments )
            EmitSegment( segment );

        if( !segments.empty() )
            mResults->CommitResults();
        ReportProgress( pipeline.Cursor() );
    }
}

void StepperMotorCoilsAnalyzer::EmitSegment( const stepper::Segment& segment )
{
    if( segment.end <= segment.start )
        return;

    mResults->AddFrame( StepperMotorCoilsAnalyzerResults::EncodeFrame( segment ) );

    const double duration = double( segment.end - segment.start ) / double( mSampleRateHz );

    FrameV2 frame_v2;
    if( segment.type != stepper::SegmentType::Off )
    {
        char position[ 80 ];
        mResults->FormatPosition( segment.position, true, position, sizeof( position ) );
        frame_v2.AddString( "position", position );
        frame_v2.AddDouble( "steps", segment.position );
        frame_v2.AddDouble( "electrical_angle", segment.angle_degrees );
    }
    frame_v2.AddDouble( "drive", std::round( segment.drive * 1000.0 ) / 10.0 );
    if( segment.type == stepper::SegmentType::Position )
    {
        frame_v2.AddString( "direction", segment.direction > 0 ? "+" : ( segment.direction < 0 ? "-" : "" ) );
        // Speed implied by the change that ended this span: the position moved by `delta` over its duration.
        frame_v2.AddDouble( "rate", duration > 0 ? segment.delta / duration : 0 );
    }
    mResults->AddFrameV2( frame_v2, FrameTypeName( segment.type ), segment.start, segment.end - 1 );

    // Full-step markers: an arrow on A+ wherever the position crosses into a new whole step.
    if( mSettings.mStepMarkers && segment.type == stepper::SegmentType::Position )
    {
        if( mHavePreviousPosition && std::floor( segment.position ) != std::floor( mPreviousPosition ) )
        {
            const auto marker = segment.position > mPreviousPosition ? AnalyzerResults::UpArrow : AnalyzerResults::DownArrow;
            mResults->AddMarker( segment.start, marker, mSettings.mChannels[ stepper::A_POS ] );
        }
        mPreviousPosition = segment.position;
        mHavePreviousPosition = true;
    }
}

bool StepperMotorCoilsAnalyzer::NeedsRerun()
{
    return false;
}

U32 StepperMotorCoilsAnalyzer::GenerateSimulationData( U64 minimum_sample_index, U32 device_sample_rate,
                                                       SimulationChannelDescriptor** simulation_channels )
{
    if( mSimulationInitialized == false )
    {
        mSimulationDataGenerator.Initialize( GetSimulationSampleRate(), &mSettings );
        mSimulationInitialized = true;
    }

    return mSimulationDataGenerator.GenerateSimulationData( minimum_sample_index, device_sample_rate, simulation_channels );
}

U32 StepperMotorCoilsAnalyzer::GetMinimumSampleRateHz()
{
    // The simulated 20 kHz PWM needs a few hundred samples per period for accurate duty cycles.
    return 4000000;
}

const char* StepperMotorCoilsAnalyzer::GetAnalyzerName() const
{
    return kAnalyzerName;
}

const char* GetAnalyzerName()
{
    return kAnalyzerName;
}

Analyzer* CreateAnalyzer()
{
    return new StepperMotorCoilsAnalyzer();
}

void DestroyAnalyzer( Analyzer* analyzer )
{
    delete analyzer;
}
