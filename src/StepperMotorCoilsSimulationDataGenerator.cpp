#include "StepperMotorCoilsSimulationDataGenerator.h"

#include "StepperMotorCoilsAnalyzerSettings.h"

#include <AnalyzerHelpers.h>

StepperMotorCoilsSimulationDataGenerator::StepperMotorCoilsSimulationDataGenerator()
    : mSettings( nullptr ), mSimulationSampleRateHz( 0 ), mGenerated( 0 ), mChannels()
{
}

StepperMotorCoilsSimulationDataGenerator::~StepperMotorCoilsSimulationDataGenerator()
{
}

void StepperMotorCoilsSimulationDataGenerator::Initialize( U32 simulation_sample_rate, StepperMotorCoilsAnalyzerSettings* settings )
{
    mSimulationSampleRateHz = simulation_sample_rate;
    mSettings = settings;

    const double pwm_frequency =
        stepper::PwmMode( settings->mPwmMode ) == stepper::PwmMode::Manual ? double( settings->mPwmFrequency ) : 20000.0;
    mWaveform.reset( new stepper::StepperWaveform( simulation_sample_rate, pwm_frequency ) );
    mGenerated = 0;

    for( int i = 0; i < stepper::TERMINAL_COUNT; ++i )
    {
        const BitState initial = mWaveform->InitialState( i ) ? BIT_HIGH : BIT_LOW;
        mChannels[ i ] = mGroup.Add( settings->mChannels[ i ], simulation_sample_rate, initial );
    }
}

U32 StepperMotorCoilsSimulationDataGenerator::GenerateSimulationData( U64 largest_sample_requested, U32 sample_rate,
                                                                      SimulationChannelDescriptor** simulation_channels )
{
    const U64 adjusted_largest_sample_requested =
        AnalyzerHelpers::AdjustSimulationTargetSample( largest_sample_requested, sample_rate, mSimulationSampleRateHz );

    while( mGenerated < adjusted_largest_sample_requested )
    {
        mTransitions.clear();
        mGenerated = mWaveform->GenerateNext( mTransitions );

        for( const stepper::Transition& transition : mTransitions )
        {
            SimulationChannelDescriptor* channel = mChannels[ transition.channel ];
            const U64 current = channel->GetCurrentSampleNumber();
            if( transition.sample > current )
                channel->Advance( U32( transition.sample - current ) );
            channel->Transition();
        }

        // Every channel must reach the end of the generated span, including ones that did not change.
        for( SimulationChannelDescriptor* channel : mChannels )
        {
            const U64 current = channel->GetCurrentSampleNumber();
            if( mGenerated > current )
                channel->Advance( U32( mGenerated - current ) );
        }
    }

    *simulation_channels = mGroup.GetArray();
    return mGroup.GetCount();
}
