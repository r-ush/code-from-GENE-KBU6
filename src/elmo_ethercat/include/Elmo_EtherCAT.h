#pragma once

// EtherCAT Mater
#include <ethercattype.h>
#include <nicdrv.h>
#include <ethercatbase.h>
#include <ethercatmain.h>
#include <ethercatdc.h>
#include <ethercatcoe.h>
#include <ethercatfoe.h>
#include <ethercatconfig.h>
#include <ethercatprint.h>
#include <ethercat.h>



#define EC_TIMEOUTMON 500
#define INITIAL_POS 0

#define TORQUE_MODE 10

#define ELMO_VENDOR_ID 0x0000009a
#define ELMO_GOLD_PRODUCT_CODE 0x00030924

#define HMS_VENDOR_ID 0x0000001B 
#define HMS_ANYBUS_PRODUCT_CODE 0x0000001d 

// Number of X
#define NUM_ELMO_DRIVER 2
#define NUM_HMS_ANYBUS 1

#define STATUSWORD_READY_TO_SWITCH_ON_BIT 0
#define STATUSWORD_SWITCHED_ON_BIT 1
#define STATUSWORD_OPERATION_ENABLE_BIT 2
#define STATUSWORD_FAULT_BIT 3

// WRITE PDO
#define TARGET_POSITION 0x607a0
#define TARGET_VELOCITY 0x60ff0
#define TARGET_TORQUE 0x60710
#define MAX_TORQUE 0x60720
#define CONTROL_WORD 0x60400
#define MODE_OF_OPERATION 0x60600
#define DIGITAL_OUTPUTS 0x60fe1

// READ PDO
#define POSITION_ACTUAL_VALUE 0x60640
#define TORQUE_ACTUAL_VALUE 0x60770
#define STATUS_WORD 0x60410
#define MODE_OF_OPERATION_DISPLAY 0x60610
#define VELOCITY_ACTUAL_VALUE 0x606c0
#define DC_LINK_CIRCUIT_VOLTAGE 0x60790
#define DIGITAL_INPUTS 0x60fd0
#define ANALOG_INPUT_1 0x22051
#define AUXILIARY_POSITION_ACTUAL_VALUE 0x20a00
#define CURRENT_ACTUAL_VALUE 0x60780

// WRITE CAN PDO
#define OUTPUT_BYTE_0000 0x21001
#define OUTPUT_BYTE_0001 0x21002
#define OUTPUT_BYTE_0002 0x21003
#define OUTPUT_BYTE_0003 0x21004
#define OUTPUT_BYTE_0004 0x21005
#define OUTPUT_BYTE_0005 0x21006
#define OUTPUT_BYTE_0006 0x21007
#define OUTPUT_BYTE_0007 0x21008

// READ CAN PDO

#define INPUT_BYTE_0000 0x20001
#define INPUT_BYTE_0001 0x20002
#define INPUT_BYTE_0002 0x20003
#define INPUT_BYTE_0003 0x20004
#define INPUT_BYTE_0004 0x20005
#define INPUT_BYTE_0005 0x20006
#define INPUT_BYTE_0006 0x20007
#define INPUT_BYTE_0007 0x20008
#define INPUT_BYTE_0008 0x20009
#define INPUT_BYTE_0009 0x200010
#define INPUT_BYTE_0010 0x200011
#define INPUT_BYTE_0011 0x200012
#define INPUT_BYTE_0012 0x200013
#define INPUT_BYTE_0013 0x200014
#define INPUT_BYTE_0014 0x200015
#define INPUT_BYTE_0015 0x200016
#define INPUT_BYTE_0016 0x200017
#define INPUT_BYTE_0017 0x200018
#define INPUT_BYTE_0018 0x200019
#define INPUT_BYTE_0019 0x200020
#define INPUT_BYTE_0020 0x200021
#define INPUT_BYTE_0021 0x200022
#define INPUT_BYTE_0022 0x200023
#define INPUT_BYTE_0023 0x200024
#define INPUT_BYTE_0024 0x200025
#define INPUT_BYTE_0025 0x200026
#define INPUT_BYTE_0026 0x200027
#define INPUT_BYTE_0027 0x200028



/**
 * helper macros
 */

#define CHECKERROR(slaveId)   \
{   \
    ec_readstate();\
    printf("EC> \"%s\" %x - %x [%s] \n", (char*)ec_elist2string(), ec_slave[slaveId].state, ec_slave[slaveId].ALstatuscode, (char*)ec_ALstatuscode2string(ec_slave[slaveId].ALstatuscode));    \
}

struct ELMO_GOLD_IN
{
    int32_t		TargetPosition; 	// 0x607A
    int32_t		TargetVelocity; 	// 0x60FF
    int16_t		TargetTorque; 	// 0xro6071
    uint16_t	MaxTorque; 	// 0x6072
    uint16_t	ControlWord; 	// 0x6040
    int8_t		ModeOfOperation; 	// 0x6060	
	int32_t		DigitalOutputs; 	// 0x60FE	
	// int8_t		temp1;			
};
struct ELMO_GOLD_OUT
{
    int32_t		PositionActualValue; 	// 0x6064
    int16_t		TorqueActualValue; 	// 0x6077
    uint16_t	StatusWord; 	// 0x6041
    int32_t		VelocityActualValue; 	// 0x606C
    uint32_t	DCLinkCircuitVoltage; 	// 0x6079
    uint32_t	DigitalInputs; 	// 0x60FD
    		
    int32_t		AuxiliaryPositionActualValue; 	// 0x20A0
    int16_t     AnalogInput1;   //0x2205
	int16_t		CurrentActualValue; 	// 0x6078
};

struct ELMO_GOLD
{
    struct ELMO_GOLD_IN 	*InParam;
    struct ELMO_GOLD_OUT 	*OutParam;
};

struct HMS_ANYBUS_IN
{
    uint8_t     OutputByte0000;
    uint8_t     OutputByte0001;
    uint8_t     OutputByte0002;
    // uint8_t     OutputByte0003;
    // uint8_t     OutputByte0004;
    // uint8_t     OutputByte0005;
    // uint8_t     OutputByte0006;
    // uint8_t     OutputByte0007;
};
struct HMS_ANYBUS_OUT
{
    uint8_t     InputByte0000;
    uint8_t     InputByte0001;
    uint8_t     InputByte0002;
    uint8_t     InputByte0003;
    uint8_t     InputByte0004;
    uint8_t     InputByte0005;
    // uint8_t     InputByte0006;
    // uint8_t     InputByte0007;
    // uint8_t     InputByte0008;
    // uint8_t     InputByte0009;
    // uint8_t     InputByte0010;
    // uint8_t     InputByte0011;
    // uint8_t     InputByte0012;
    // uint8_t     InputByte0013;
    // uint8_t     InputByte0014;
    // uint8_t     InputByte0015;
    // uint8_t     InputByte0016;
    // uint8_t     InputByte0017;
    // uint8_t     InputByte0018;
    // uint8_t     InputByte0019;
    // uint8_t     InputByte0020;
    // uint8_t     InputByte0021;
    // uint8_t     InputByte0022;
    // uint8_t     InputByte0023;
    // uint8_t     InputByte0024;
    // uint8_t     InputByte0025;
    // uint8_t     InputByte0026;
    // uint8_t     InputByte0027;
};

struct HMS_ANYBUS
{

   //struct HMS_ANYBUS_IN 	*InParam;
   struct HMS_ANYBUS_OUT 	*OutParam;

};
/****************************************************************************/
// Class

class EthercatMaster
{
    private:
        
        

        char IOmap[4096];

    public:
        ELMO_GOLD _elmo_gold[NUM_ELMO_DRIVER];
        HMS_ANYBUS _hms_anybus[NUM_HMS_ANYBUS];
        EthercatMaster();
        ~EthercatMaster();

        int init(const int8_t mode_op, char *ifname);
        int initSlave();
        int showSlaveInfo();
        int registerParam();
        int testDriveState(const int8_t mode_op);
 
        void processRxDomain();
        int processTxDomain();
       

        void readBuffer(const int EntryID, void* data);
        void writeBuffer(const int EntryID, void* data);
        
        void servoOn(const int position);


};
