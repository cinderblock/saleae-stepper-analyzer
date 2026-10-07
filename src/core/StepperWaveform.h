#ifndef STEPPER_WAVEFORM_H
#define STEPPER_WAVEFORM_H

#include "StepperDecoder.h"

#include <cstdint>
#include <vector>

namespace stepper
{
    struct Transition
    {
        uint64_t sample;
        int channel; // Terminal index; the channel toggles at `sample`
    };

    // Synthesizes bipolar stepper drive signals for a looping demo program, with a known commanded
    // position at every instant. It covers the cases the decoder must handle:
    //
    //   - de-energized gaps,
    //   - center-aligned PWM microstepping holds and moves (both directions, varying current),
    //   - logic-level half-stepping with no PWM,
    //   - re-energizing after a gap.
    //
    // Used for Logic's simulation device and for the unit tests.
    class StepperWaveform
    {
      public:
        StepperWaveform( double sample_rate, double pwm_frequency = 20000.0 );

        // All terminals start low (de-energized).
        bool InitialState( int ) const
        {
            return false;
        }

        // Generate the next PWM period of the program. Toggles are appended in sample order.
        // Returns the sample at which the generated span ends.
        uint64_t GenerateNext( std::vector<Transition>& out );

        // Commanded position in full steps at a time within the program. Logic-level phases report
        // the half step being driven. The program loops, returning to position 0 each loop.
        static double PositionAt( double seconds );

        // Whether the motor is driven at a time within the program.
        static bool EnergizedAt( double seconds );

        static double ProgramDuration();

      private:
        double mSampleRate;
        double mPeriod; // seconds
        uint64_t mPeriodIndex = 0;
        bool mLevel[ TERMINAL_COUNT ] = {};
    };
}

#endif // STEPPER_WAVEFORM_H
