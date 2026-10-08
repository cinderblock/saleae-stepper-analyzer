#ifndef STEPPER_MOTOR_COILS_ANALYZER_H
#define STEPPER_MOTOR_COILS_ANALYZER_H

#include <Analyzer.h>

#include "StepperMotorCoilsAnalyzerSettings.h"
#include "StepperMotorCoilsAnalyzerResults.h"
#include "StepperMotorCoilsSimulationDataGenerator.h"
#include "IdleFlusher.h"
#include "MoveGrouper.h"
#include "StepperDecoder.h"
#include "StepperPipeline.h"

#include <memory>
#include <vector>

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
    void EmitResults( bool flush );
    void FlushOpen();

    StepperMotorCoilsAnalyzerSettings mSettings;
    std::unique_ptr<StepperMotorCoilsAnalyzerResults> mResults;

    StepperMotorCoilsSimulationDataGenerator mSimulationDataGenerator;
    bool mSimulationInitialized;

    U32 mSampleRateHz;
    bool mHavePreviousPosition;
    double mPreviousPosition;

    // Decoding state. Members rather than worker locals because the flusher thread uses them too,
    // always under mFlusher.Lock().
    IdleFlusher mFlusher;
    std::unique_ptr<stepper::StepperPipeline> mPipeline;
    std::unique_ptr<stepper::MoveGrouper> mGrouper;
    std::vector<stepper::Segment> mSegments;
    std::vector<stepper::Segment> mGrouped;
};

extern "C" ANALYZER_EXPORT const char* __cdecl GetAnalyzerName();
extern "C" ANALYZER_EXPORT Analyzer* __cdecl CreateAnalyzer();
extern "C" ANALYZER_EXPORT void __cdecl DestroyAnalyzer( Analyzer* analyzer );

#endif // STEPPER_MOTOR_COILS_ANALYZER_H
