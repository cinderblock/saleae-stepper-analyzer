#ifndef STEPPER_DECODER_H
#define STEPPER_DECODER_H

#include <cstdint>
#include <vector>

namespace stepper
{
    // Indexes of the four drive channels.
    enum Terminal
    {
        A_POS = 0,
        A_NEG = 1,
        B_POS = 2,
        B_NEG = 3,
        TERMINAL_COUNT = 4
    };

    struct DecoderConfig
    {
        double resolution = 1.0 / 16;       // full steps per reported position increment
        double hysteresis = 0.25;           // extra dead band, as a fraction of `resolution`
        double energized_threshold = 0.1;   // drive magnitude below which the motor counts as de-energized
        double ambiguous_degrees = 135;     // electrical change between windows that is too large to trust the direction of
        bool invert = false;                // report positive positions for the opposite rotation
        bool start_at_zero = true;          // position 0 is where the motor was first energized
        uint64_t min_energized_samples = 0; // shorter energized spans after an off span are ignored as glitches
    };

    enum class SegmentType
    {
        Position,  // energized, holding a quantized position
        Off,       // de-energized: the drive gives no position information
        Ambiguous, // the electrical angle jumped too far between windows to know which way it went
    };

    // A span of samples [start, end) with one reported state.
    struct Segment
    {
        SegmentType type = SegmentType::Off;
        uint64_t start = 0;
        uint64_t end = 0;
        double position = 0;      // quantized, in full steps
        double mean_position = 0; // unquantized mean over the span, in full steps
        double angle_degrees = 0; // electrical angle of the mean position, (-180, 180]
        double drive = 0;         // mean drive magnitude: 1 = one coil fully driven, sqrt(2) = both
        double delta = 0;         // quantized position change that ended the span, in full steps
        int direction = 0;        // sign of `delta`
    };

    // Turns per-window terminal duty cycles into motor position segments.
    //
    // Each coil's drive is the duty difference between its two terminals, so the current vector
    // (A, B) works the same for logic-level wave/full/half stepping and for PWM microstepping.
    // Its angle is the electrical angle, and one full step is 90 electrical degrees.
    class StepperDecoder
    {
      public:
        explicit StepperDecoder( const DecoderConfig& config );

        // Feed the mean duty (0..1) of each terminal over [start, end). Windows must be contiguous.
        // Segments that become complete are appended to `out`.
        void AddWindow( uint64_t start, uint64_t end, const double duty[ TERMINAL_COUNT ], std::vector<Segment>& out );

        // Close the open segment at the end of the last window, appending it to `out`.
        void Flush( std::vector<Segment>& out );

        // Electrical angle in degrees of a coil drive vector.
        static double AngleDegrees( double a, double b );

      private:
        struct Window
        {
            uint64_t start;
            uint64_t end;
            double a;
            double b;
            double drive;
        };

        void AddOff( const Window& window, std::vector<Segment>& out );
        void AddEnergized( const Window& window, std::vector<Segment>& out );
        void FlushPending( std::vector<Segment>& out );
        void Close( uint64_t end, double delta, std::vector<Segment>& out );
        void Open( SegmentType type, uint64_t start, double position );
        double Quantize( double position ) const;

        DecoderConfig mConfig;

        bool mHaveOpen = false;
        Segment mOpen;
        double mSumPosition = 0;
        double mSumDrive = 0;
        double mSumSamples = 0;

        bool mHaveAngle = false; // whether the previous window was energized
        bool mEverEnergized = false;
        double mPreviousAngle = 0; // degrees
        double mUnwrapped = 0;     // electrical position in full steps, continuous
        double mOffset = 0;        // added to mUnwrapped to get the reported position
        uint64_t mLastEnd = 0;

        std::vector<Window> mPending; // energized windows waiting to last min_energized_samples
        uint64_t mPendingSamples = 0;
    };
}

#endif // STEPPER_DECODER_H
