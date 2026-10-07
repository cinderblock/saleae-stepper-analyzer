#ifndef STEPPER_MOTOR_COILS_ANALYZER_SETTINGS
#define STEPPER_MOTOR_COILS_ANALYZER_SETTINGS

#include <AnalyzerSettings.h>
#include <AnalyzerTypes.h>

#include "StepperPipeline.h"

#include <memory>

enum class PositionUnits : U32
{
    FullSteps = 0,
    Degrees = 1,
    Revolutions = 2,
};

class StepperMotorCoilsAnalyzerSettings : public AnalyzerSettings
{
  public:
    StepperMotorCoilsAnalyzerSettings();
    virtual ~StepperMotorCoilsAnalyzerSettings();

    virtual bool SetSettingsFromInterfaces();
    void UpdateInterfacesFromSettings();
    virtual void LoadSettings( const char* settings );
    virtual const char* SaveSettings();

    stepper::PipelineConfig MakePipelineConfig( double sample_rate ) const;

    // Converts full steps to the configured display unit.
    double ToUnits( double full_steps ) const;
    const char* UnitSuffix() const;

    Channel mChannels[ stepper::TERMINAL_COUNT ];
    U32 mPwmMode;
    U32 mPwmFrequency;
    U32 mPeriodsPerWindow;
    U32 mResolution;         // denominator: positions are reported in 1/mResolution full steps
    U32 mEnergizedThreshold; // percent of full drive on one coil
    U32 mUnits;
    U32 mStepsPerRevolution;
    bool mInvert;
    bool mStartAtZero;
    bool mStepMarkers;

  protected:
    void UpdateChannels( bool is_used );

    std::unique_ptr<AnalyzerSettingInterfaceChannel> mChannelInterfaces[ stepper::TERMINAL_COUNT ];
    std::unique_ptr<AnalyzerSettingInterfaceNumberList> mPwmModeInterface;
    std::unique_ptr<AnalyzerSettingInterfaceInteger> mPwmFrequencyInterface;
    std::unique_ptr<AnalyzerSettingInterfaceInteger> mPeriodsPerWindowInterface;
    std::unique_ptr<AnalyzerSettingInterfaceNumberList> mResolutionInterface;
    std::unique_ptr<AnalyzerSettingInterfaceInteger> mEnergizedThresholdInterface;
    std::unique_ptr<AnalyzerSettingInterfaceNumberList> mUnitsInterface;
    std::unique_ptr<AnalyzerSettingInterfaceInteger> mStepsPerRevolutionInterface;
    std::unique_ptr<AnalyzerSettingInterfaceBool> mInvertInterface;
    std::unique_ptr<AnalyzerSettingInterfaceBool> mStartAtZeroInterface;
    std::unique_ptr<AnalyzerSettingInterfaceBool> mStepMarkersInterface;
};

#endif // STEPPER_MOTOR_COILS_ANALYZER_SETTINGS
