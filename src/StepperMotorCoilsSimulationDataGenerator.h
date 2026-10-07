#ifndef STEPPER_MOTOR_COILS_SIMULATION_DATA_GENERATOR
#define STEPPER_MOTOR_COILS_SIMULATION_DATA_GENERATOR

#include <SimulationChannelDescriptor.h>

#include "StepperWaveform.h"

#include <memory>
#include <vector>

class StepperMotorCoilsAnalyzerSettings;

// Plays StepperWaveform's demo program on the four coil channels of Logic's simulation device.
class StepperMotorCoilsSimulationDataGenerator
{
  public:
    StepperMotorCoilsSimulationDataGenerator();
    ~StepperMotorCoilsSimulationDataGenerator();

    void Initialize( U32 simulation_sample_rate, StepperMotorCoilsAnalyzerSettings* settings );
    U32 GenerateSimulationData( U64 newest_sample_requested, U32 sample_rate, SimulationChannelDescriptor** simulation_channels );

  protected:
    StepperMotorCoilsAnalyzerSettings* mSettings;
    U32 mSimulationSampleRateHz;

    std::unique_ptr<stepper::StepperWaveform> mWaveform;
    std::vector<stepper::Transition> mTransitions;
    U64 mGenerated;

    SimulationChannelDescriptorGroup mGroup;
    SimulationChannelDescriptor* mChannels[ stepper::TERMINAL_COUNT ];
};

#endif // STEPPER_MOTOR_COILS_SIMULATION_DATA_GENERATOR
