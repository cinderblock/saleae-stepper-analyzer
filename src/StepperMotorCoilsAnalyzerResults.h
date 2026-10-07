#ifndef STEPPER_MOTOR_COILS_ANALYZER_RESULTS
#define STEPPER_MOTOR_COILS_ANALYZER_RESULTS

#include <AnalyzerResults.h>

#include "StepperDecoder.h"

class StepperMotorCoilsAnalyzer;
class StepperMotorCoilsAnalyzerSettings;

// Frame encoding (FrameV2 carries the same data as named fields for the data table and HLAs):
//   mType   stepper::SegmentType
//   mData1  position in full steps, the bits of a double
//   mData2  high 32 bits: electrical angle in degrees (for a move: its start position in full steps),
//           low 32 bits: drive magnitude; each the bits of a float
//   mFlags  FRAME_FLAG_POSITIVE / FRAME_FLAG_NEGATIVE for the change that ended the frame
#define FRAME_FLAG_POSITIVE ( 1 << 0 )
#define FRAME_FLAG_NEGATIVE ( 1 << 1 )

class StepperMotorCoilsAnalyzerResults : public AnalyzerResults
{
  public:
    StepperMotorCoilsAnalyzerResults( StepperMotorCoilsAnalyzer* analyzer, StepperMotorCoilsAnalyzerSettings* settings );
    virtual ~StepperMotorCoilsAnalyzerResults();

    virtual void GenerateBubbleText( U64 frame_index, Channel& channel, DisplayBase display_base );
    virtual void GenerateExportFile( const char* file, DisplayBase display_base, U32 export_type_user_id );

    virtual void GenerateFrameTabularText( U64 frame_index, DisplayBase display_base );
    virtual void GeneratePacketTabularText( U64 packet_id, DisplayBase display_base );
    virtual void GenerateTransactionTabularText( U64 transaction_id, DisplayBase display_base );

    static Frame EncodeFrame( const stepper::Segment& segment );

    // Position in the configured units, formatted without trailing zeros, with the unit suffix
    // when `with_unit` is set.
    void FormatPosition( double full_steps, bool with_unit, char* out, size_t size ) const;

  protected:
    struct Decoded
    {
        stepper::SegmentType type;
        double position;
        double angle; // for a move: where it started, in full steps
        double drive;
        int direction;
    };
    static Decoded DecodeFrame( const Frame& frame );

    StepperMotorCoilsAnalyzerSettings* mSettings;
    StepperMotorCoilsAnalyzer* mAnalyzer;
};

#endif // STEPPER_MOTOR_COILS_ANALYZER_RESULTS
