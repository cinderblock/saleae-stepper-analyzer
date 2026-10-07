// Exercises the analyzer's simulation data generator against the real Analyzer SDK library, the
// way Logic's demo device calls it, and checks it reproduces the StepperWaveform program.
//
// Logic's demo device only plays an analyzer's simulation when the analyzer is added in the UI
// before capturing, which the automation API cannot do, so this is how it gets tested.

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include "StepperMotorCoilsAnalyzerSettings.h"
#include "StepperMotorCoilsSimulationDataGenerator.h"
#include "StepperWaveform.h"

#include <vector>

TEST_CASE( "Simulation generator plays the demo program on the selected channels" )
{
    const U32 sample_rate = 4000000;

    StepperMotorCoilsAnalyzerSettings settings;
    for( U32 i = 0; i < stepper::TERMINAL_COUNT; ++i )
        settings.mChannels[ i ] = Channel( 0x1234, 3 + i, DIGITAL_CHANNEL );

    StepperMotorCoilsSimulationDataGenerator generator;
    generator.Initialize( sample_rate, &settings );

    // Logic asks for data in increasing chunks.
    SimulationChannelDescriptor* channels = nullptr;
    U32 count = 0;
    const U64 target = U64( 2.5 * stepper::StepperWaveform::ProgramDuration() * sample_rate );
    for( U64 requested = 1000; requested < target; requested += 77777 )
        count = generator.GenerateSimulationData( requested, sample_rate, &channels );
    count = generator.GenerateSimulationData( target, sample_rate, &channels );

    REQUIRE( count == stepper::TERMINAL_COUNT );

    // Every channel is generated to the same point, at least as far as requested.
    const U64 reached = channels[ 0 ].GetCurrentSampleNumber();
    CHECK( reached >= target );
    for( U32 i = 0; i < count; ++i )
    {
        CAPTURE( i );
        CHECK( channels[ i ].GetChannel() == settings.mChannels[ i ] );
        CHECK( channels[ i ].GetCurrentSampleNumber() == reached );
    }

    // Replay the same program independently: each channel must end at the same level.
    stepper::StepperWaveform reference( sample_rate );
    std::vector<stepper::Transition> transitions;
    bool level[ stepper::TERMINAL_COUNT ] = {};
    for( U32 i = 0; i < stepper::TERMINAL_COUNT; ++i )
        level[ i ] = reference.InitialState( int( i ) );
    U64 generated = 0;
    U64 toggles = 0;
    while( generated < reached )
    {
        transitions.clear();
        generated = reference.GenerateNext( transitions );
        for( const stepper::Transition& transition : transitions )
            level[ transition.channel ] = !level[ transition.channel ];
        toggles += transitions.size();
    }
    REQUIRE( generated == reached );
    CHECK( toggles > 1000 );

    for( U32 i = 0; i < count; ++i )
    {
        CAPTURE( i );
        CHECK( ( channels[ i ].GetCurrentBitState() == BIT_HIGH ) == level[ i ] );
    }
}
