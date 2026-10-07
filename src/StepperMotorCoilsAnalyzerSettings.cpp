#include "StepperMotorCoilsAnalyzerSettings.h"

#include <AnalyzerHelpers.h>

#include <cstdio>

namespace
{
    const char* const kChannelNames[ stepper::TERMINAL_COUNT ] = { "A+", "A-", "B+", "B-" };
    const char* const kChannelTooltips[ stepper::TERMINAL_COUNT ] = {
        "Coil A, first terminal (or the H-bridge input that drives it high)",
        "Coil A, second terminal (or the H-bridge input that drives it high)",
        "Coil B, first terminal (or the H-bridge input that drives it high)",
        "Coil B, second terminal (or the H-bridge input that drives it high)",
    };
    const U32 kResolutions[] = { 1, 2, 4, 8, 16, 32, 64, 128, 256 };
}

StepperMotorCoilsAnalyzerSettings::StepperMotorCoilsAnalyzerSettings()
    : mPwmMode( U32( stepper::PwmMode::Auto ) ),
      mPwmFrequency( 20000 ),
      mPeriodsPerWindow( 2 ),
      mResultMode( U32( ResultMode::EveryChange ) ),
      mHoldTime( 20 ),
      mResolution( 16 ),
      mEnergizedThreshold( 10 ),
      mUnits( U32( PositionUnits::FullSteps ) ),
      mStepsPerRevolution( 200 ),
      mInvert( false ),
      mStartAtZero( true ),
      mStepMarkers( false )
{
    for( int i = 0; i < stepper::TERMINAL_COUNT; ++i )
    {
        mChannels[ i ] = UNDEFINED_CHANNEL;
        mChannelInterfaces[ i ].reset( new AnalyzerSettingInterfaceChannel() );
        mChannelInterfaces[ i ]->SetTitleAndTooltip( kChannelNames[ i ], kChannelTooltips[ i ] );
        mChannelInterfaces[ i ]->SetChannel( mChannels[ i ] );
        AddInterface( mChannelInterfaces[ i ].get() );
    }

    mPwmModeInterface.reset( new AnalyzerSettingInterfaceNumberList() );
    mPwmModeInterface->SetTitleAndTooltip( "PWM filtering", "How the coil PWM (current chopping) is averaged away. Auto-detect "
                                                            "measures the PWM period from the signals." );
    mPwmModeInterface->AddNumber( double( stepper::PwmMode::Auto ), "Auto-detect", "Measure the PWM period from the edges" );
    mPwmModeInterface->AddNumber( double( stepper::PwmMode::Manual ), "Fixed frequency", "Average over the PWM frequency entered below" );
    mPwmModeInterface->AddNumber( double( stepper::PwmMode::None ), "None (logic-level stepping)",
                                  "The terminals switch only when the motor steps; no averaging" );
    AddInterface( mPwmModeInterface.get() );

    mPwmFrequencyInterface.reset( new AnalyzerSettingInterfaceInteger() );
    mPwmFrequencyInterface->SetTitleAndTooltip( "PWM frequency (Hz)", "Used when PWM filtering is set to a fixed frequency" );
    mPwmFrequencyInterface->SetMin( 100 );
    mPwmFrequencyInterface->SetMax( 10000000 );
    AddInterface( mPwmFrequencyInterface.get() );

    mPeriodsPerWindowInterface.reset( new AnalyzerSettingInterfaceInteger() );
    mPeriodsPerWindowInterface->SetTitleAndTooltip(
        "PWM periods per sample", "Number of PWM periods averaged into each position sample. More is smoother but responds slower." );
    mPeriodsPerWindowInterface->SetMin( 1 );
    mPeriodsPerWindowInterface->SetMax( 1000 );
    AddInterface( mPeriodsPerWindowInterface.get() );

    mResultModeInterface.reset( new AnalyzerSettingInterfaceNumberList() );
    mResultModeInterface->SetTitleAndTooltip( "Results",
                                              "One result per position change, or whole moves between holds (reads better zoomed out)" );
    mResultModeInterface->AddNumber( double( ResultMode::EveryChange ), "Every position change",
                                     "A result each time the position changes" );
    mResultModeInterface->AddNumber( double( ResultMode::MovesAndHolds ), "Moves and holds",
                                     "Quick successive changes become one move result, from start to end position" );
    AddInterface( mResultModeInterface.get() );

    mHoldTimeInterface.reset( new AnalyzerSettingInterfaceInteger() );
    mHoldTimeInterface->SetTitleAndTooltip( "Hold time (ms)",
                                            "With moves and holds: a position held at least this long ends a move and is shown as a hold" );
    mHoldTimeInterface->SetMin( 1 );
    mHoldTimeInterface->SetMax( 1000000 );
    AddInterface( mHoldTimeInterface.get() );

    mResolutionInterface.reset( new AnalyzerSettingInterfaceNumberList() );
    mResolutionInterface->SetTitleAndTooltip( "Position resolution", "Smallest position change that starts a new result" );
    for( U32 resolution : kResolutions )
    {
        char label[ 32 ];
        if( resolution == 1 )
            std::snprintf( label, sizeof( label ), "Full step" );
        else
            std::snprintf( label, sizeof( label ), "1/%u step", resolution );
        mResolutionInterface->AddNumber( resolution, label, "" );
    }
    AddInterface( mResolutionInterface.get() );

    mEnergizedThresholdInterface.reset( new AnalyzerSettingInterfaceInteger() );
    mEnergizedThresholdInterface->SetTitleAndTooltip( "Off below drive (%)",
                                                      "Coil drive below this percentage counts as de-energized (no position)" );
    mEnergizedThresholdInterface->SetMin( 1 );
    mEnergizedThresholdInterface->SetMax( 100 );
    AddInterface( mEnergizedThresholdInterface.get() );

    mUnitsInterface.reset( new AnalyzerSettingInterfaceNumberList() );
    mUnitsInterface->SetTitleAndTooltip( "Display units", "Units for the position in results" );
    mUnitsInterface->AddNumber( double( PositionUnits::FullSteps ), "Full steps", "" );
    mUnitsInterface->AddNumber( double( PositionUnits::Degrees ), "Degrees (shaft)", "Uses full steps per revolution" );
    mUnitsInterface->AddNumber( double( PositionUnits::Revolutions ), "Revolutions", "Uses full steps per revolution" );
    AddInterface( mUnitsInterface.get() );

    mStepsPerRevolutionInterface.reset( new AnalyzerSettingInterfaceInteger() );
    mStepsPerRevolutionInterface->SetTitleAndTooltip( "Full steps per revolution", "200 for a 1.8 degree motor, 400 for 0.9 degree" );
    mStepsPerRevolutionInterface->SetMin( 1 );
    mStepsPerRevolutionInterface->SetMax( 1000000 );
    AddInterface( mStepsPerRevolutionInterface.get() );

    mInvertInterface.reset( new AnalyzerSettingInterfaceBool() );
    mInvertInterface->SetTitleAndTooltip( "Invert direction", "Count the other rotation direction as positive" );
    mInvertInterface->SetCheckBoxText( "Invert direction" );
    AddInterface( mInvertInterface.get() );

    mStartAtZeroInterface.reset( new AnalyzerSettingInterfaceBool() );
    mStartAtZeroInterface->SetTitleAndTooltip(
        "Start at zero", "Position 0 is where the motor is first energized. Otherwise positions follow the electrical angle." );
    mStartAtZeroInterface->SetCheckBoxText( "Start at zero" );
    AddInterface( mStartAtZeroInterface.get() );

    mStepMarkersInterface.reset( new AnalyzerSettingInterfaceBool() );
    mStepMarkersInterface->SetTitleAndTooltip( "Full-step markers", "Mark each full-step crossing with an arrow on A+" );
    mStepMarkersInterface->SetCheckBoxText( "Full-step markers" );
    AddInterface( mStepMarkersInterface.get() );

    AddExportOption( 0, "Export as text/csv file" );
    AddExportExtension( 0, "text", "txt" );
    AddExportExtension( 0, "csv", "csv" );

    UpdateInterfacesFromSettings();
    UpdateChannels( false );
}

StepperMotorCoilsAnalyzerSettings::~StepperMotorCoilsAnalyzerSettings()
{
}

void StepperMotorCoilsAnalyzerSettings::UpdateChannels( bool is_used )
{
    ClearChannels();
    for( int i = 0; i < stepper::TERMINAL_COUNT; ++i )
        AddChannel( mChannels[ i ], kChannelNames[ i ], is_used );
}

bool StepperMotorCoilsAnalyzerSettings::SetSettingsFromInterfaces()
{
    Channel channels[ stepper::TERMINAL_COUNT ];
    for( int i = 0; i < stepper::TERMINAL_COUNT; ++i )
    {
        channels[ i ] = mChannelInterfaces[ i ]->GetChannel();
        if( channels[ i ] == UNDEFINED_CHANNEL )
        {
            SetErrorText( "Please select all four coil channels." );
            return false;
        }
    }
    if( AnalyzerHelpers::DoChannelsOverlap( channels, stepper::TERMINAL_COUNT ) )
    {
        SetErrorText( "Each coil terminal needs its own channel." );
        return false;
    }

    for( int i = 0; i < stepper::TERMINAL_COUNT; ++i )
        mChannels[ i ] = channels[ i ];
    mPwmMode = U32( mPwmModeInterface->GetNumber() );
    mPwmFrequency = U32( mPwmFrequencyInterface->GetInteger() );
    mPeriodsPerWindow = U32( mPeriodsPerWindowInterface->GetInteger() );
    mResultMode = U32( mResultModeInterface->GetNumber() );
    mHoldTime = U32( mHoldTimeInterface->GetInteger() );
    mResolution = U32( mResolutionInterface->GetNumber() );
    mEnergizedThreshold = U32( mEnergizedThresholdInterface->GetInteger() );
    mUnits = U32( mUnitsInterface->GetNumber() );
    mStepsPerRevolution = U32( mStepsPerRevolutionInterface->GetInteger() );
    mInvert = mInvertInterface->GetValue();
    mStartAtZero = mStartAtZeroInterface->GetValue();
    mStepMarkers = mStepMarkersInterface->GetValue();

    UpdateChannels( true );
    return true;
}

void StepperMotorCoilsAnalyzerSettings::UpdateInterfacesFromSettings()
{
    for( int i = 0; i < stepper::TERMINAL_COUNT; ++i )
        mChannelInterfaces[ i ]->SetChannel( mChannels[ i ] );
    mPwmModeInterface->SetNumber( mPwmMode );
    mPwmFrequencyInterface->SetInteger( mPwmFrequency );
    mPeriodsPerWindowInterface->SetInteger( mPeriodsPerWindow );
    mResultModeInterface->SetNumber( mResultMode );
    mHoldTimeInterface->SetInteger( mHoldTime );
    mResolutionInterface->SetNumber( mResolution );
    mEnergizedThresholdInterface->SetInteger( mEnergizedThreshold );
    mUnitsInterface->SetNumber( mUnits );
    mStepsPerRevolutionInterface->SetInteger( mStepsPerRevolution );
    mInvertInterface->SetValue( mInvert );
    mStartAtZeroInterface->SetValue( mStartAtZero );
    mStepMarkersInterface->SetValue( mStepMarkers );
}

void StepperMotorCoilsAnalyzerSettings::LoadSettings( const char* settings )
{
    SimpleArchive text_archive;
    text_archive.SetString( settings );

    for( int i = 0; i < stepper::TERMINAL_COUNT; ++i )
        text_archive >> mChannels[ i ];
    text_archive >> mPwmMode;
    text_archive >> mPwmFrequency;
    text_archive >> mPeriodsPerWindow;
    text_archive >> mResolution;
    text_archive >> mEnergizedThreshold;
    text_archive >> mUnits;
    text_archive >> mStepsPerRevolution;
    text_archive >> mInvert;
    text_archive >> mStartAtZero;
    text_archive >> mStepMarkers;
    text_archive >> mResultMode;
    text_archive >> mHoldTime;

    UpdateChannels( true );
    UpdateInterfacesFromSettings();
}

const char* StepperMotorCoilsAnalyzerSettings::SaveSettings()
{
    SimpleArchive text_archive;

    for( int i = 0; i < stepper::TERMINAL_COUNT; ++i )
        text_archive << mChannels[ i ];
    text_archive << mPwmMode;
    text_archive << mPwmFrequency;
    text_archive << mPeriodsPerWindow;
    text_archive << mResolution;
    text_archive << mEnergizedThreshold;
    text_archive << mUnits;
    text_archive << mStepsPerRevolution;
    text_archive << mInvert;
    text_archive << mStartAtZero;
    text_archive << mStepMarkers;
    text_archive << mResultMode;
    text_archive << mHoldTime;

    return SetReturnString( text_archive.GetString() );
}

stepper::PipelineConfig StepperMotorCoilsAnalyzerSettings::MakePipelineConfig( double sample_rate ) const
{
    stepper::PipelineConfig config;
    config.sample_rate = sample_rate;
    config.pwm_mode = stepper::PwmMode( mPwmMode );
    config.pwm_frequency = mPwmFrequency;
    config.periods_per_window = int( mPeriodsPerWindow );
    config.decoder.resolution = 1.0 / double( mResolution );
    config.decoder.energized_threshold = double( mEnergizedThreshold ) / 100.0;
    config.decoder.invert = mInvert;
    config.decoder.start_at_zero = mStartAtZero;
    return config;
}

double StepperMotorCoilsAnalyzerSettings::ToUnits( double full_steps ) const
{
    switch( PositionUnits( mUnits ) )
    {
    case PositionUnits::Degrees:
        return full_steps * 360.0 / double( mStepsPerRevolution );
    case PositionUnits::Revolutions:
        return full_steps / double( mStepsPerRevolution );
    case PositionUnits::FullSteps:
    default:
        return full_steps;
    }
}

const char* StepperMotorCoilsAnalyzerSettings::UnitSuffix() const
{
    switch( PositionUnits( mUnits ) )
    {
    case PositionUnits::Degrees:
        return "\xC2\xB0"; // degree sign, UTF-8
    case PositionUnits::Revolutions:
        return " rev";
    case PositionUnits::FullSteps:
    default:
        return " steps";
    }
}
