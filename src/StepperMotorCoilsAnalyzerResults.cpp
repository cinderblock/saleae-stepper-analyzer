#include "StepperMotorCoilsAnalyzerResults.h"

#include <AnalyzerHelpers.h>

#include "StepperMotorCoilsAnalyzer.h"
#include "StepperMotorCoilsAnalyzerSettings.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>

namespace
{
    // Trailing zeros (and a trailing decimal point) after printf("%.Nf").
    void TrimZeros( char* text )
    {
        if( std::strchr( text, '.' ) == nullptr )
            return;
        size_t length = std::strlen( text );
        while( length > 0 && text[ length - 1 ] == '0' )
            text[ --length ] = '\0';
        if( length > 0 && text[ length - 1 ] == '.' )
            text[ --length ] = '\0';
    }

    U32 FloatBits( float value )
    {
        U32 bits;
        std::memcpy( &bits, &value, sizeof( bits ) );
        return bits;
    }

    float BitsFloat( U32 bits )
    {
        float value;
        std::memcpy( &value, &bits, sizeof( value ) );
        return value;
    }
}

StepperMotorCoilsAnalyzerResults::StepperMotorCoilsAnalyzerResults( StepperMotorCoilsAnalyzer* analyzer,
                                                                    StepperMotorCoilsAnalyzerSettings* settings )
    : AnalyzerResults(), mSettings( settings ), mAnalyzer( analyzer )
{
}

StepperMotorCoilsAnalyzerResults::~StepperMotorCoilsAnalyzerResults()
{
}

Frame StepperMotorCoilsAnalyzerResults::EncodeFrame( const stepper::Segment& segment )
{
    Frame frame;
    frame.mStartingSampleInclusive = S64( segment.start );
    frame.mEndingSampleInclusive = S64( segment.end ) - 1;
    frame.mType = U8( segment.type );
    std::memcpy( &frame.mData1, &segment.position, sizeof( frame.mData1 ) );
    frame.mData2 = ( U64( FloatBits( float( segment.angle_degrees ) ) ) << 32 ) | FloatBits( float( segment.drive ) );
    frame.mFlags = 0;
    if( segment.direction > 0 )
        frame.mFlags |= FRAME_FLAG_POSITIVE;
    if( segment.direction < 0 )
        frame.mFlags |= FRAME_FLAG_NEGATIVE;
    if( segment.type == stepper::SegmentType::Ambiguous )
        frame.mFlags |= DISPLAY_AS_ERROR_FLAG;
    return frame;
}

StepperMotorCoilsAnalyzerResults::Decoded StepperMotorCoilsAnalyzerResults::DecodeFrame( const Frame& frame )
{
    Decoded decoded;
    decoded.type = stepper::SegmentType( frame.mType );
    std::memcpy( &decoded.position, &frame.mData1, sizeof( decoded.position ) );
    decoded.angle = BitsFloat( U32( frame.mData2 >> 32 ) );
    decoded.drive = BitsFloat( U32( frame.mData2 & 0xFFFFFFFFull ) );
    decoded.direction = ( frame.mFlags & FRAME_FLAG_POSITIVE ) ? 1 : ( ( frame.mFlags & FRAME_FLAG_NEGATIVE ) ? -1 : 0 );
    return decoded;
}

void StepperMotorCoilsAnalyzerResults::FormatPosition( double full_steps, bool with_unit, char* out, size_t size ) const
{
    // Enough decimals to show the configured resolution in the chosen unit.
    const double step = mSettings->ToUnits( 1.0 / double( mSettings->mResolution ) );
    int decimals = 0;
    while( decimals < 9 && std::fabs( step * std::pow( 10.0, decimals ) - std::round( step * std::pow( 10.0, decimals ) ) ) > 1e-6 )
        ++decimals;

    char number[ 64 ];
    std::snprintf( number, sizeof( number ), "%.*f", decimals, mSettings->ToUnits( full_steps ) );
    TrimZeros( number );
    if( std::strcmp( number, "-0" ) == 0 )
        std::snprintf( number, sizeof( number ), "0" );

    std::snprintf( out, size, "%s%s", number, with_unit ? mSettings->UnitSuffix() : "" );
}

void StepperMotorCoilsAnalyzerResults::GenerateBubbleText( U64 frame_index, Channel& channel, DisplayBase display_base )
{
    ClearResultStrings();
    const Decoded frame = DecodeFrame( GetFrame( frame_index ) );

    switch( frame.type )
    {
    case stepper::SegmentType::Position:
    {
        char position[ 64 ];
        char with_unit[ 80 ];
        char detail[ 160 ];
        FormatPosition( frame.position, false, position, sizeof( position ) );
        FormatPosition( frame.position, true, with_unit, sizeof( with_unit ) );
        std::snprintf( detail, sizeof( detail ), "Position %s (%.1f\xC2\xB0 electrical, drive %.0f%%)", with_unit, frame.angle,
                       frame.drive * 100.0 );
        AddResultString( position );
        AddResultString( with_unit );
        AddResultString( detail );
        break;
    }
    case stepper::SegmentType::Off:
        AddResultString( "Off" );
        AddResultString( "De-energized" );
        break;
    case stepper::SegmentType::Ambiguous:
        AddResultString( "?" );
        AddResultString( "Ambiguous" );
        AddResultString( "Ambiguous jump: direction unknown" );
        break;
    }
}

void StepperMotorCoilsAnalyzerResults::GenerateExportFile( const char* file, DisplayBase display_base, U32 export_type_user_id )
{
    std::ofstream file_stream( file, std::ios::out );

    const U64 trigger_sample = mAnalyzer->GetTriggerSample();
    const U32 sample_rate = mAnalyzer->GetSampleRate();

    file_stream << "Time [s],Duration [s],Type,Position,Full steps,Electrical angle [deg],Drive [%],Direction" << std::endl;

    const U64 num_frames = GetNumFrames();
    for( U64 i = 0; i < num_frames; i++ )
    {
        const Frame raw = GetFrame( i );
        const Decoded frame = DecodeFrame( raw );

        char time_str[ 128 ];
        AnalyzerHelpers::GetTimeString( raw.mStartingSampleInclusive, trigger_sample, sample_rate, time_str, 128 );
        const double duration = double( raw.mEndingSampleInclusive - raw.mStartingSampleInclusive + 1 ) / double( sample_rate );

        file_stream << time_str << "," << duration << ",";
        switch( frame.type )
        {
        case stepper::SegmentType::Position:
        {
            char position[ 64 ];
            FormatPosition( frame.position, false, position, sizeof( position ) );
            file_stream << "position," << position << "," << frame.position << "," << frame.angle << "," << frame.drive * 100.0 << ","
                        << ( frame.direction > 0 ? "+" : ( frame.direction < 0 ? "-" : "" ) );
            break;
        }
        case stepper::SegmentType::Off:
            file_stream << "off,,,," << frame.drive * 100.0 << ",";
            break;
        case stepper::SegmentType::Ambiguous:
            file_stream << "ambiguous,,," << frame.angle << "," << frame.drive * 100.0 << ",";
            break;
        }
        file_stream << std::endl;

        if( UpdateExportProgressAndCheckForCancel( i, num_frames ) == true )
        {
            file_stream.close();
            return;
        }
    }

    file_stream.close();
}

void StepperMotorCoilsAnalyzerResults::GenerateFrameTabularText( U64 frame_index, DisplayBase display_base )
{
#ifdef SUPPORTS_PROTOCOL_SEARCH
    ClearTabularText();
    const Decoded frame = DecodeFrame( GetFrame( frame_index ) );

    switch( frame.type )
    {
    case stepper::SegmentType::Position:
    {
        char with_unit[ 80 ];
        FormatPosition( frame.position, true, with_unit, sizeof( with_unit ) );
        AddTabularText( with_unit );
        break;
    }
    case stepper::SegmentType::Off:
        AddTabularText( "De-energized" );
        break;
    case stepper::SegmentType::Ambiguous:
        AddTabularText( "Ambiguous jump" );
        break;
    }
#endif
}

void StepperMotorCoilsAnalyzerResults::GeneratePacketTabularText( U64 packet_id, DisplayBase display_base )
{
    // not supported
}

void StepperMotorCoilsAnalyzerResults::GenerateTransactionTabularText( U64 transaction_id, DisplayBase display_base )
{
    // not supported
}
