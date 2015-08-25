#include "unavinterface.h"

#include <stdio.h>

UNavInterface::UNavInterface()
    : _uNav(NULL)
{

}

UNavInterface::~UNavInterface()
{
    if(_uNav)
    {
        delete _uNav;
        _uNav = NULL;
    }
}

bool UNavInterface::connect( const std::string& devname, unsigned int baud_rate )
{
    if(_uNav)
    {
        delete _uNav;
        _uNav = NULL;
    }

    try
    {
        _uNav = new ParserPacket( devname, baud_rate );

    }
    catch( parser_exception& e)
    {
        cout << "Connection error: " << e.what() << endl;

        return false;
    }
    catch( boost::system::system_error& e)
    {
        cout << "Connection error: " << e.what() << endl;

        return false;
    }
    catch(...)
    {
        cout << "Connection error: Unknown error";

        return false;
    }

    return true;
}

void UNavInterface::disconnect()
{
    if( _uNav )
        delete _uNav;

    _uNav = NULL;
}

bool UNavInterface::sendMotorParams(uint8_t motIdx, uint16_t cpr, float ratio,
                                    int8_t versus, uint8_t enable_mode, uint8_t enc_pos,
                                    int16_t bridge_volt )
{
    if( !_uNav )
        return false;

    try
    {
        motor_parameter_t param;
        param.encoder.cpr = cpr;
        param.bridge.enable = enable_mode;
        param.encoder.position = enc_pos;
        param.ratio = ratio;
        param.rotation = versus;
        param.bridge.volt = bridge_volt;

        motor_command_map_t command;
        command.bitset.motor = motIdx;
        command.bitset.command = MOTOR_PARAMETER;

        packet_t packet_send = _uNav->encoder(_uNav->createDataPacket(command.command_message, HASHMAP_MOTOR, (message_abstract_u*) &param));

        _uNav->sendSyncPacket(packet_send, 3, boost::posix_time::millisec(200));
    }
    catch( parser_exception& e)
    {
        cout << "Serial error: " << e.what() << endl;

        throw e;
        return false;
    }
    catch( boost::system::system_error& e)
    {
        cout << "Serial error: " << e.what() << endl;

        throw e;
        return false;
    }
    catch(...)
    {
        cout << "Serial error: Unknown error";

        throw;
        return false;
    }

    sleep(1);

    return true;
}

bool UNavInterface::enableSpeedControl(uint8_t motIdx, bool enable )
{
    try
    {
        motor_control_t enable_val = enable ? STATE_CONTROL_VELOCITY : STATE_CONTROL_DISABLE;

        motor_command_map_t command;
        command.bitset.motor = motIdx;
        command.bitset.command = MOTOR_STATE;

        packet_t packet_send = _uNav->encoder(_uNav->createDataPacket(command.command_message, HASHMAP_MOTOR, (message_abstract_u*) &enable_val));

        _uNav->sendSyncPacket(packet_send, 3, boost::posix_time::millisec(200));
    }
    catch( parser_exception& e)
    {
        cout << "Serial error: " << e.what() << endl;

        return false;
    }
    catch( boost::system::system_error& e)
    {
        cout << "Serial error: " << e.what() << endl;

        return false;
    }
    catch(...)
    {
        cout << "Serial error: Unknown error";

        return false;
    }

    return true;
}

bool UNavInterface::getMotorSpeed( uint8_t motIdx, double& outSpeed )
{
    try
    {
        motor_command_map_t command;

        command.bitset.command = MOTOR_MEASURE;
        command.bitset.motor = motIdx;

        packet_information_t send = _uNav->createPacket( command.command_message, PACKET_REQUEST, HASHMAP_MOTOR);
        packet_t received = _uNav->sendSyncPacket( _uNav->encoder(send), 3, boost::posix_time::millisec(200) );

        // parse packet
        vector<packet_information_t> list = _uNav->parsing(received);

        //get first packet
        packet_information_t first = list.at(0);

        if(first.option == PACKET_DATA)
        {
            if(first.type == HASHMAP_MOTOR)
            {
                motor_t motor0, motor1;
                motor_command_map_t command;
                command.command_message = first.command;

                if(command.bitset.command == MOTOR_MEASURE )
                {
                    switch(command.bitset.motor)
                    {
                    case 0:

                        motor0 = first.message.motor.motor;
                        outSpeed = ((double)motor0.velocity)/1000.0;
                        break;

                    case 1:
                        motor1 = first.message.motor.motor;
                        outSpeed = ((double)motor1.velocity)/1000.0;
                        break;
                    }
                }
            }
        }
    }

    catch( parser_exception& e)
    {
        cout << "Serial error: " << e.what() << endl;

        throw e;
        return false;
    }
    catch( boost::system::system_error& e)
    {
        cout << "Serial error: " << e.what() << endl;

        throw e;
        return false;
    }
    catch(...)
    {
        cout << "Serial error: Unknown error";

        throw;
        return false;
    }

    return true;
}

bool UNavInterface::sendMotorSpeed( uint8_t motorIdx, int16_t speed )
{

    try
    {
        motor_command_map_t command;
        command.bitset.motor = motorIdx;
        command.bitset.command = MOTOR_VEL_REF;

        packet_t packet_send = _uNav->encoder(_uNav->createDataPacket(command.command_message, HASHMAP_MOTOR, (message_abstract_u*) &speed));

        _uNav->sendSyncPacket(packet_send, 3, boost::posix_time::millisec(200));
    }
    catch( parser_exception& e)
    {
        cout << "Serial error: " << e.what() << endl;

        throw e;
        return false;
    }
    catch( boost::system::system_error& e)
    {
        cout << "Serial error: " << e.what() << endl;

        throw e;
        return false;
    }
    catch(...)
    {
        cout << "Serial error: Unknown error";

        throw;
        return false;
    }

    return true;
}

bool UNavInterface::sendPIDGains( uint8_t motorIdx, double kp, double ki, double kd )
{
    try
    {
        motor_pid_t pid;
        pid.kp = kp;
        pid.ki = ki;
        pid.kd = kd;

        motor_command_map_t command;
        command.bitset.motor = motorIdx;
        command.bitset.command = MOTOR_VEL_PID;

        packet_t packet_send = _uNav->encoder(_uNav->createDataPacket(command.command_message, HASHMAP_MOTOR, (message_abstract_u*) &pid));

        _uNav->sendSyncPacket(packet_send, 3, boost::posix_time::millisec(200));
    }
    catch( parser_exception& e)
    {
        cout << "Serial error: " << e.what() << endl;

        throw e;
        return false;
    }
    catch( boost::system::system_error& e)
    {
        cout << "Serial error: " << e.what() << endl;

        throw e;
        return false;
    }
    catch(...)
    {
        cout << "Serial error: Unknown error";

        throw;
        return false;
    }

    return true;
}
