#include "StepperMotorCoilsAnalyzer.h"

#include <AnalyzerChannelData.h>

#include "MoveGrouper.h"
#include "StepperPipeline.h"

#include <algorithm>
#include <cmath>
#include <utility>
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
        case stepper::SegmentType::Move:
            return "move";
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
    mFlusher.Stop();
}

void StepperMotorCoilsAnalyzer::SetupResults()
{
    // A previous run's flusher must not touch the results being replaced.
    mFlusher.Stop();

    // SetupResults is called each time the analyzer is run. Because the same instance can be used for multiple runs, we need to clear the
    // results each time.
    mResults.reset( new StepperMotorCoilsAnalyzerResults( this, &mSettings ) );
    SetAnalyzerResults( mResults.get() );
    mResults->AddChannelBubblesWillAppearOn( mSettings.mChannels[ stepper::A_POS ] );
}

void StepperMotorCoilsAnalyzer::WorkerThread()
{
    mSampleRateHz = GetSampleRate();

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

    {
        std::lock_guard<std::mutex> lock( mFlusher.Lock() );
        mHavePreviousPosition = false;
        mPipeline.reset( new stepper::StepperPipeline( mSettings.MakePipelineConfig( double( mSampleRateHz ) ) ) );
        mPipeline->Start( start, initial );

        // With moves and holds, quick successive positions are grouped before they become results.
        mGrouper.reset();
        if( ResultMode( mSettings.mResultMode ) == ResultMode::MovesAndHolds )
            mGrouper.reset( new stepper::MoveGrouper( U64( double( mSettings.mHoldTime ) * 1e-3 * double( mSampleRateHz ) ) ) );
    }
    mFlusher.Start( [ this ]() { FlushOpen(); }, std::chrono::milliseconds( 100 ), std::chrono::milliseconds( 2000 ) );

    std::vector<std::pair<int, U64>> edges;
    U64 filled = start;

    for( ;; )
    {
        U64 needed;
        {
            std::lock_guard<std::mutex> lock( mFlusher.Lock() );
            needed = mPipeline->NeededFill();
        }

        // Read every channel up to the point the pipeline needs. Advancing blocks until the
        // capture has data there, which is how a live capture is followed, and also how the end of
        // a capture looks: the flusher shows the open segment then.
        edges.clear();
        if( needed > filled )
        {
            mFlusher.Waiting();
            for( int i = 0; i < stepper::TERMINAL_COUNT; ++i )
            {
                AnalyzerChannelData* data = channels[ i ];
                while( data->WouldAdvancingToAbsPositionCauseTransition( needed ) )
                {
                    data->AdvanceToNextEdge();
                    edges.emplace_back( i, data->GetSampleNumber() );
                }
                data->AdvanceToAbsPosition( needed );
            }
            mFlusher.Progressed();
        }

        std::lock_guard<std::mutex> lock( mFlusher.Lock() );
        if( needed > filled )
        {
            for( const auto& edge : edges )
                mPipeline->PushEdge( edge.first, edge.second );
            filled = needed;
            mPipeline->SetFilled( filled );
        }

        mSegments.clear();
        mPipeline->Process( mSegments );
        for( const stepper::Segment& segment : mSegments )
        {
            if( !segment.continuation )
            {
                mFlusher.Activity();
                break;
            }
        }
        EmitResults( false );
        ReportProgress( mPipeline->Cursor() );
    }
}

void StepperMotorCoilsAnalyzer::FlushOpen()
{
    // Runs on the flusher thread, holding its lock, while the worker waits for data.
    if( !mPipeline )
        return;
    mSegments.clear();
    mPipeline->Checkpoint( mSegments );
    EmitResults( true );
}

void StepperMotorCoilsAnalyzer::EmitResults( bool flush )
{
    const std::vector<stepper::Segment>* results = &mSegments;
    if( mGrouper )
    {
        mGrouped.clear();
        for( const stepper::Segment& segment : mSegments )
            mGrouper->Add( segment, mGrouped );
        // Results that wait in the grouper are emitted as soon as they are known: Logic only takes
        // late results into its data table while the analyzer is still running.
        if( flush )
            mGrouper->Flush( mGrouped );
        else
            mGrouper->Tick( mPipeline->Cursor(), mPipeline->Decoder().HasOpen(), mPipeline->Decoder().OpenSegment(), mGrouped );
        results = &mGrouped;
    }

    for( const stepper::Segment& segment : *results )
        EmitSegment( segment );

    if( !results->empty() )
        mResults->CommitResults();
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
        // A move spans many angles, so it has none of its own.
        if( segment.type != stepper::SegmentType::Move )
            frame_v2.AddDouble( "electrical_angle", segment.angle_degrees );
    }
    frame_v2.AddDouble( "drive", std::round( segment.drive * 1000.0 ) / 10.0 );
    if( segment.type == stepper::SegmentType::Move )
    {
        frame_v2.AddDouble( "from", segment.from_position );
        frame_v2.AddDouble( "delta", segment.delta );
    }
    if( segment.type == stepper::SegmentType::Position || segment.type == stepper::SegmentType::Move )
    {
        frame_v2.AddString( "direction", segment.direction > 0 ? "+" : ( segment.direction < 0 ? "-" : "" ) );
        // For a position: the speed implied by the change that ended it. For a move: its mean speed.
        frame_v2.AddDouble( "rate", duration > 0 ? segment.delta / duration : 0 );
    }
    mResults->AddFrameV2( frame_v2, FrameTypeName( segment.type ), segment.start, segment.end - 1 );

    // Full-step markers: an arrow on A+ wherever the position crosses into a new whole step.
    // (A move is a single result, so the steps inside it are not marked.)
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
