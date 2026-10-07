#ifndef STEPPER_MOTOR_COILS_ANALYZER_H
#define STEPPER_MOTOR_COILS_ANALYZER_H

#include <Analyzer.h>

#include "StepperMotorCoilsAnalyzerSettings.h"
#include "StepperMotorCoilsAnalyzerResults.h"
#include "StepperMotorCoilsSimulationDataGenerator.h"
#include "StepperDecoder.h"

#include <memory>

class ANALYZER_EXPORT StepperMotorCoilsAnalyzer : public Analyzer2
{
  public:
    StepperMotorCoilsAnalyzer();
    virtual ~StepperMotorCoilsAnalyzer();

    virtual void SetupResults();
    virtual void WorkerThread();

    virtual U32 GenerateSimulationData( U64 newest_sample_requested, U32 sample_rate, SimulationChannelDescriptor** simulation_channels );
    virtual U32 GetMinimumSampleRateHz();

    virtual const char* GetAnalyzerName() const;
    virtual bool NeedsRerun();

  protected:
    void EmitSegment( const stepper::Segment& segment );

    StepperMotorCoilsAnalyzerSettings mSettings;
    std::unique_ptr<StepperMotorCoilsAnalyzerResults> mResults;

    StepperMotorCoilsSimulationDataGenerator mSimulationDataGenerator;
    bool mSimulationInitialized;

    U32 mSampleRateHz;
    bool mHavePreviousPosition;
    double mPreviousPosition;
};

extern "C" ANALYZER_EXPORT const char* __cdecl GetAnalyzerName();
extern "C" ANALYZER_EXPORT Analyzer* __cdecl CreateAnalyzer();
extern "C" ANALYZER_EXPORT void __cdecl DestroyAnalyzer( Analyzer* analyzer );

#endif // STEPPER_MOTOR_COILS_ANALYZER_H
