#ifndef UNAVINTERFACE_H
#define UNAVINTERFACE_H

#include <serial_parser_packet/ParserPacket.h>
#include <string>

using namespace std;

class UNavInterface
{
public:
    UNavInterface();
    ~UNavInterface();

    bool connect(  const string& devname, unsigned int baud_rate );
    void disconnect();

    bool sendMotorParams(uint8_t motIdx, uint16_t cpr, float ratio,
                         int8_t versus, uint8_t enable_mode, uint8_t enc_pos, int16_t bridge_volt );

    bool sendPIDGains( uint8_t motorIdx, double kp, double ki, double kd );

    bool sendMotorSpeed( uint8_t motorIdx, int16_t speed );
    bool sendMotorSpeeds( int16_t speed_0, int16_t speed_1 );

    bool enableSpeedControl(uint8_t motIdx, bool enable );

    bool getMotorSpeed( uint8_t motIdx, double& outSpeed );

protected:

private:
    bool connected;

    ParserPacket* _uNav; ///< uNav communication object

};

#endif // UNAVINTERFACE_H
