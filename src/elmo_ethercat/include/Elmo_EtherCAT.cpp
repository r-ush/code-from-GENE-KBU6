#include "Elmo_EtherCAT.h"
#include <iostream>

using namespace std;

EthercatMaster::EthercatMaster(){}
EthercatMaster::~EthercatMaster()
{
    ec_slave[0].state = EC_STATE_INIT;
	/* request INIT state for all slaves */
	ec_writestate(0);


	printf("End simple test, close socket\n");
	/* stop SOEM, close socket */
	ec_close();
	
}

int EthercatMaster::init(const int8_t mode_op, char *ifname)
{      
   //Ethercat SOEM
	// char *ifname;
	// ifname = "enp2s0";
	int j, oloop, iloop, wkc_count, chk;

    //inOP = FALSE;

	 if (ec_init(ifname))
    {
        printf("ec_init on %s succeeded.\n",ifname);
        /* find and auto-config slaves */

        /** network discovery */
        if ( ec_config_init(FALSE) > 0 )
        {
            printf("%d slaves found and configured.\n", ec_slavecount);

			//Elmo Config
			this->initSlave();
            
            /** if CA disable => automapping works */
            ec_config_map(&IOmap);
			
            // show slave info
			this->showSlaveInfo();
			
            printf("Slaves mapped, state to SAFE_OP.\n");

            int timestep = 700;

            /* wait for all slaves to reach SAFE_OP state */
            printf("test00\n");
			ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE * 4);
			
			/** old SOEM code, inactive */
            oloop = ec_slave[0].Obytes;
            if ((oloop == 0) && (ec_slave[0].Obits > 0)) oloop = 1;
            if (oloop > 20) oloop = 8;
            iloop = ec_slave[0].Ibytes;
            if ((iloop == 0) && (ec_slave[0].Ibits > 0)) iloop = 1;
            if (iloop > 20) iloop = 8;

			//printf("test11\n");
            int expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
            printf("Calculated workcounter %d\n", expectedWKC);

			//Elmo output initialize
			
			

            /** going operational */
            ec_slave[0].state = EC_STATE_OPERATIONAL;
            /* send one valid process data to make outputs in slaves happy*/
            ec_send_processdata();
            ec_receive_processdata(EC_TIMEOUTRET);
            
            /* request OP state for all slaves */
            ec_writestate(0);
            chk = 40;
            /* wait for all slaves to reach OP state */
            do
            {
                ec_send_processdata();
                ec_receive_processdata(EC_TIMEOUTRET);
                ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
            }

            while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));

			this->registerParam();
			
            if (ec_slave[0].state == EC_STATE_OPERATIONAL )
            {

				printf("Operational state reached for all slaves.\n");
                wkc_count = 0;
                //inOP = TRUE;

                /**
                 * Drive state machine transistions
                 *   0 -> 6 -> 7 -> 15
                 */
                this->testDriveState(mode_op);
				
				return 1;
            }
            else
            {
                printf("Not all slaves reached operational state.\n");
                ec_readstate();
                for(int i = 1; i<=ec_slavecount ; i++)
                {
                    if(ec_slave[i].state != EC_STATE_OPERATIONAL)
                    {
                        printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
                            i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
                    }
                }
            }
        }
        else
        {
            printf("No slaves found!\n");
        }
    }
    else
    {
        printf("No socket connection on %s\nExcecute as root\n",ifname);
    }

	return 0;
}

int Elmosetup(uint16 slave)
{
    int retval;
    uint16 u16val;
    
    retval = 0;
	
	uint16 map_1c12[] = {0x0002, 0x1605, 0x161D};
	uint16 map_1c13[] = {0x0009,  0x1A0E, 0x1A13, 0x1A0A, 0x1A11, 0x1A18, 0x1A1C, 0x1A1E,  0x1A1D, 0x1A1F};
	//uint16 map_1c13[] = {0x0007,  0x1A02,  0x1A11, 0x1A18, 0x1A1C,  0x1A1E, 0x1A1D, 0x1A1F};

	retval +=ec_SDOwrite(slave,0x1c12,0x00,TRUE,sizeof(map_1c12),&map_1c12,EC_TIMEOUTSAFE);
	retval +=ec_SDOwrite(slave,0x1c13,0x00,TRUE,sizeof(map_1c13),&map_1c13,EC_TIMEOUTSAFE);
    printf("EL7031 slave %d set, retval = %d\n", slave, retval);
    return 1;
}

int AnybusSetup(uint16 slave)
{
    int retval;
    uint16 u16val;
    
    retval = 0;

	uint16 map_1c13[] = {(uint16)0x20001,  (uint16)0x20002, (uint16)0x20003, (uint16)0x20004, 
	                     (uint16)0x20005, (uint16)0x20006};

	retval +=ec_SDOwrite(slave,0x1c13,0x00,TRUE,sizeof(map_1c13),&map_1c13,EC_TIMEOUTSAFE);
    printf("EL7031 slave %d set, retval = %d\n", slave, retval);
    return 1;
}

int EthercatMaster::initSlave()
{
	for(int i = 1 ; i <= NUM_ELMO_DRIVER ; i++)
	{
		if((ec_slave[i].eep_man == ELMO_VENDOR_ID) && (ec_slave[i].eep_id == ELMO_GOLD_PRODUCT_CODE))
		{
			printf("Found %s at position %d\n", ec_slave[i].name, i);
			/* link slave specific setup to preop->safeop hook */
			ec_slave[i].PO2SOconfig = Elmosetup;

		}
	}

	for (int i = NUM_ELMO_DRIVER + 1; i<= NUM_ELMO_DRIVER + NUM_HMS_ANYBUS; i++)
	{
		if((ec_slave[i].eep_man == HMS_VENDOR_ID) && (ec_slave[i].eep_id == HMS_ANYBUS_PRODUCT_CODE))
		{
			printf("Found %s at position %d\n", ec_slave[i].name, i);
			/* link slave specific setup to preop->safeop hook */
			ec_slave[i].PO2SOconfig = AnybusSetup;
		}
	}


	return 0;
}

int EthercatMaster::showSlaveInfo()
{
	for (int i=1; i<=ec_slavecount; i++) 
	{
		printf("\nSlave:%d\n Name:%s\n Output size: %dbits\n Input size: %dbits\n State: %d\n Delay: %d[ns]\n Has DC: %d\n",
		i, ec_slave[i].name, ec_slave[i].Obits, ec_slave[i].Ibits,
		ec_slave[i].state, ec_slave[i].pdelay, ec_slave[i].hasdc);
	}  
}
int EthercatMaster::registerParam()
{
	
	for(int i = 0 ; i < NUM_ELMO_DRIVER ; i++)
	{
		_elmo_gold[i].InParam = (struct ELMO_GOLD_IN *)(ec_slave[i + 1].outputs);
		_elmo_gold[i].OutParam = (struct ELMO_GOLD_OUT *)(ec_slave[i + 1].inputs);
	}

	for(int i = NUM_ELMO_DRIVER ; i < NUM_ELMO_DRIVER + NUM_HMS_ANYBUS ; i++)
	{
		//_hms_anybus[i - NUM_ELMO_DRIVER].InParam = (struct HMS_ANYBUS_IN *)(ec_slave[i + 1].outputs);
		_hms_anybus[i - NUM_ELMO_DRIVER].OutParam = (struct HMS_ANYBUS_OUT *)(ec_slave[i + 1].inputs);
	}

}
int EthercatMaster::testDriveState(const int8_t mode_op)
{
	for (int i=0; i<NUM_ELMO_DRIVER; i++) 
	{

		_elmo_gold[i].InParam->ModeOfOperation = mode_op;
		_elmo_gold[i].InParam->MaxTorque = 1000;
		ec_send_processdata();
		
		ec_receive_processdata(EC_TIMEOUTRET); 
		_elmo_gold[i].InParam->ControlWord = 0x6;
		ec_send_processdata();
		ec_receive_processdata(EC_TIMEOUTRET);  
		_elmo_gold[i].InParam->ControlWord = 0x7;
		ec_send_processdata();
		ec_receive_processdata(EC_TIMEOUTRET); 

		_elmo_gold[i].InParam->ControlWord = 0x0F;
		ec_send_processdata();
		ec_receive_processdata(EC_TIMEOUTRET); 

	}
}

void EthercatMaster::processRxDomain()
{

	// TODO: Write control data from axes to servos via master

	for (int i=0; i<NUM_ELMO_DRIVER; ++i)
	{
		servoOn(i);
		
	}
	ec_send_processdata();
	// for (int i=0; i<NUM_HMS_ANYBUS; ++i)
	// {
	// 	//servoOn(i);
		
	// }
				
}

int EthercatMaster::processTxDomain()
{
	return ec_receive_processdata(EC_TIMEOUTRET);		
}


void EthercatMaster::writeBuffer(const int EntryID, void* data)
{
    switch (EntryID)
	{
		case TARGET_POSITION:
		{
			int32_t* _targetPosition = static_cast<int32_t *>(data);
            for (int i=0; i<NUM_ELMO_DRIVER; i++)
            {
                _elmo_gold[i].InParam->TargetPosition = _targetPosition[i];
            }
		}
			break;						

		case TARGET_VELOCITY:
		{
			int32_t * const _targetVelocity = static_cast<int32_t * const>(data);
            for (int i=0; i<NUM_ELMO_DRIVER; i++)
            {
                _elmo_gold[i].InParam->TargetVelocity = _targetVelocity[i];
            }
		}
			break;						

		case TARGET_TORQUE:
		{
			int16_t * const _targetTorque = static_cast<int16_t * const>(data);		
            for (int i=0; i<NUM_ELMO_DRIVER; i++)
            {
                _elmo_gold[i].InParam->TargetTorque = _targetTorque[i];
            }
		}
			break;						

		case MAX_TORQUE:
		{
			uint16_t * const _maxTorque = static_cast<uint16_t * const>(data);
            for (int i=0; i<NUM_ELMO_DRIVER; i++)
            {
                _elmo_gold[i].InParam->MaxTorque = _maxTorque[i];
            }
		}
			break;						

		case CONTROL_WORD:
		{
			uint16_t * const _controlWord = static_cast<uint16_t * const>(data);
            for (int i=0; i<NUM_ELMO_DRIVER; i++)
            {
                _elmo_gold[i].InParam->ControlWord = _controlWord[i];
            }
		}
			break;						

		case MODE_OF_OPERATION:
		{
			int8_t * const _modeOfOperation = static_cast<int8_t * const>(data);
            for (int i=0; i<NUM_ELMO_DRIVER; i++)
            {
                _elmo_gold[i].InParam->ModeOfOperation = _modeOfOperation[i];
            }
		}
			break;

		case DIGITAL_OUTPUTS:
		{
			int32_t * const _digitalOuputs = static_cast<int32_t * const>(data);
            for (int i=0; i<NUM_ELMO_DRIVER; i++)
            {
                _elmo_gold[i].InParam->DigitalOutputs = _digitalOuputs[i];
            }
		}
			break;

		// case OUTPUT_BYTE_0000:
		// {
		// 	uint8_t* _OutputByte0000= static_cast<uint8_t *>(data);
        //     for (int i=0; i<NUM_HMS_ANYBUS; i++)
        //     {
        //         _hms_anybus[i].InParam->OutputByte0000= _OutputByte0000[i];
		// 	}
		// }
		// 	break;

		// case OUTPUT_BYTE_0001:
		// {
		// 	uint8_t* _OutputByte0001= static_cast<uint8_t *>(data);
        //     for (int i=0; i<NUM_HMS_ANYBUS; i++)
        //     {
        //         _hms_anybus[i].InParam->OutputByte0001= _OutputByte0001[i];
        //     }
		// }
		// 	break;

		// case OUTPUT_BYTE_0002:
		// {
		// 	uint8_t* _OutputByte0002= static_cast<uint8_t *>(data);
        //     for (int i=0; i<NUM_HMS_ANYBUS; i++)
        //     {
        //         _hms_anybus[i].InParam->OutputByte0002= _OutputByte0002[i];
        //     }
		// }
		// 	break;

		// case OUTPUT_BYTE_0003:
		// {
		// 	uint8_t* _OutputByte0003= static_cast<uint8_t *>(data);
        //     for (int i=0; i<NUM_HMS_ANYBUS; i++)
        //     {
        //         _hms_anybus[i].InParam->OutputByte0003= _OutputByte0003[i];
        //     }
		// }
		// 	break;

		// case OUTPUT_BYTE_0004:
		// {
		// 	uint8_t* _OutputByte0004=static_cast<uint8_t *>(data);
        //     for (int i=0; i<NUM_HMS_ANYBUS; i++)
        //     {
        //         _hms_anybus[i].InParam->OutputByte0004= _OutputByte0004[i];
        //     }
		// }
		// 	break;

		// case OUTPUT_BYTE_0005:
		// {
		// 	uint8_t* _OutputByte0005= static_cast<uint8_t *>(data);
        //     for (int i=0; i<NUM_HMS_ANYBUS; i++)
        //     {
        //         _hms_anybus[i].InParam->OutputByte0005= _OutputByte0005[i];
        //     }
		// }
		// 	break;

		// case OUTPUT_BYTE_0006:
		// {
		// 	uint8_t* _OutputByte0006= static_cast<uint8_t *>(data);
        //     for (int i=0; i<NUM_HMS_ANYBUS; i++)
        //     {
        //         _hms_anybus[i].InParam->OutputByte0006= _OutputByte0006[i];
        //     }
		// }
		// 	break;

		// case OUTPUT_BYTE_0007:
		// {
			
		// 	uint8_t* _OutputByte0007= static_cast<uint8_t *>(data);
        //     for (int i=0; i<NUM_HMS_ANYBUS; i++)
        //     {
        //         _hms_anybus[i].InParam->OutputByte0007= _OutputByte0007[i];

        //     }
		// }
		// 	break;						
							
		default:	// Undefined Entry ID	
			
			break;
	}
}

void EthercatMaster::readBuffer(const int EntryID, void* data)
{
    switch (EntryID)
	{		
		case POSITION_ACTUAL_VALUE:
		{
            int32_t * _positionActualValue = static_cast<int32_t * const>(data);
            for (int i=0; i<NUM_ELMO_DRIVER; i++)
            {
                _positionActualValue[i] = _elmo_gold[i].OutParam->PositionActualValue;
            }
			
		}
			break;			

		case TORQUE_ACTUAL_VALUE:
		{
			int16_t * _torqueActualValue = static_cast<int16_t * const>(data);
            for (int i=0; i<NUM_ELMO_DRIVER; i++)
            {
                _torqueActualValue[i] = _elmo_gold[i].OutParam->TorqueActualValue;
            }
		}
			break;			

		case STATUS_WORD:
		{
			uint16_t * _statusWord = static_cast<uint16_t * const>(data);
            for (int i=0; i<NUM_ELMO_DRIVER; i++)
            {
                _statusWord[i] = _elmo_gold[i].OutParam->StatusWord;
            }
		}
			break;					

		case VELOCITY_ACTUAL_VALUE:
		{
			int32_t * _velocityActualValue = static_cast<int32_t * const>(data);
            for (int i=0; i<NUM_ELMO_DRIVER; i++)
            {
                _velocityActualValue[i] = _elmo_gold[i].OutParam->VelocityActualValue;
            }
		}
			break;			

		case DC_LINK_CIRCUIT_VOLTAGE:
		{
			unsigned int * _dCLinkCircuitVoltage = static_cast<unsigned int * const>(data);
            for (int i=0; i<NUM_ELMO_DRIVER; i++)
            {
                _dCLinkCircuitVoltage[i] = _elmo_gold[i].OutParam->DCLinkCircuitVoltage;
            }
		}
			break;			

		case DIGITAL_INPUTS:
		{
			int32_t * _digitalInputs = static_cast<int32_t * const>(data);
            for (int i=0; i<NUM_ELMO_DRIVER; i++)
            {
                _digitalInputs[i] = _elmo_gold[i].OutParam->DigitalInputs;
            }
		}
			break;		
				
		case ANALOG_INPUT_1:
		{
			int16_t * _analogInput1 = static_cast<int16_t * const>(data);
            for (int i=0; i<NUM_ELMO_DRIVER; i++)
            {
                _analogInput1[i] = _elmo_gold[i].OutParam->AnalogInput1;
	
            }
		}
			break;		

		case AUXILIARY_POSITION_ACTUAL_VALUE:
		{
			int32_t * _auxiliaryPositionActualValue = static_cast<int32_t * const>(data);
            for (int i=0; i<NUM_ELMO_DRIVER; i++)
            {
                _auxiliaryPositionActualValue[i] = _elmo_gold[i].OutParam->AuxiliaryPositionActualValue;
            }
		}
			break;			

		case CURRENT_ACTUAL_VALUE:
		{
			int16_t * _currentActualValue = static_cast<int16_t * const>(data);
            for (int i=0; i<NUM_ELMO_DRIVER; i++)
            {
                _currentActualValue[i] = _elmo_gold[i].OutParam->CurrentActualValue;
            }
		}
		break;	

		case INPUT_BYTE_0000:
		{
            uint8_t * _InputByte0000 = static_cast<uint8_t * const>(data);
            for (int i=0; i<NUM_HMS_ANYBUS; i++)
            {
                _InputByte0000[i] = _hms_anybus[i].OutParam->InputByte0000;
            }
			
		}
			break;		

		case INPUT_BYTE_0001:
		{
            uint8_t * _InputByte0001 = static_cast<uint8_t * const>(data);
            for (int i=0; i<NUM_HMS_ANYBUS; i++)
            {
                _InputByte0001[i] = _hms_anybus[i].OutParam->InputByte0001;
            }
			
		}
			break;		

		case INPUT_BYTE_0002:
		{
            uint8_t * _InputByte0002 = static_cast<uint8_t * const>(data);
            for (int i=0; i<NUM_HMS_ANYBUS; i++)
            {
                _InputByte0002[i] = _hms_anybus[i].OutParam->InputByte0002;
            }
		}
			break;			
		case INPUT_BYTE_0003:
		{
            uint8_t * _InputByte0003 = static_cast<uint8_t * const>(data);
            for (int i=0; i<NUM_HMS_ANYBUS; i++)
            {
                _InputByte0003[i] = _hms_anybus[i].OutParam->InputByte0003;
            }
		}
			break;	
		case INPUT_BYTE_0004:
		{
            uint8_t * _InputByte0004 = static_cast<uint8_t * const>(data);
            for (int i=0; i<NUM_HMS_ANYBUS; i++)
            {
                _InputByte0004[i] = _hms_anybus[i].OutParam->InputByte0004;
            }
		}
			break;	
		case INPUT_BYTE_0005:
		{
            uint8_t * _InputByte0005 = static_cast<uint8_t * const>(data);
            for (int i=0; i<NUM_HMS_ANYBUS; i++)
            {
                _InputByte0005[i] = _hms_anybus[i].OutParam->InputByte0005;
            }
		}
			break;	
		// case INPUT_BYTE_0006:
		// {
        //     uint8_t * _InputByte0006 = static_cast<uint8_t * const>(data);
        //     for (int i=0; i<NUM_HMS_ANYBUS; i++)
        //     {
        //         _InputByte0006[i] = _hms_anybus[i].OutParam->InputByte0006;
        //     }
		// }
		// 	break;	
		// case INPUT_BYTE_0007:
		// {
        //     uint8_t * _InputByte0007 = static_cast<uint8_t * const>(data);
        //     for (int i=0; i<NUM_HMS_ANYBUS; i++)
        //     {
        //         _InputByte0007[i] = _hms_anybus[i].OutParam->InputByte0007;
        //     }
		// }
		// 	break;	
		// case INPUT_BYTE_0008:
		// {
        //     uint8_t * _InputByte0008 = static_cast<uint8_t * const>(data);
        //     for (int i=0; i<NUM_HMS_ANYBUS; i++)
        //     {
        //         _InputByte0008[i] = _hms_anybus[i].OutParam->InputByte0008;
        //     }
		// }
		// 	break;	
		// case INPUT_BYTE_0009:
		// {
        //     uint8_t * _InputByte0009 = static_cast<uint8_t * const>(data);
        //     for (int i=0; i<NUM_HMS_ANYBUS; i++)
        //     {
        //         _InputByte0009[i] = _hms_anybus[i].OutParam->InputByte0009;
        //     }
		// }
		// 	break;	
		// case INPUT_BYTE_0010:
		// {
        //     uint8_t * _InputByte0010 = static_cast<uint8_t * const>(data);
        //     for (int i=0; i<NUM_HMS_ANYBUS; i++)
        //     {
        //         _InputByte0010[i] = _hms_anybus[i].OutParam->InputByte0010;
        //     }
		// }
		// 	break;
		// case INPUT_BYTE_0011:
		// {
        //     uint8_t * _InputByte0011 = static_cast<uint8_t * const>(data);
        //     for (int i=0; i<NUM_HMS_ANYBUS; i++)
        //     {
        //         _InputByte0011[i] = _hms_anybus[i].OutParam->InputByte0011;
        //     }
		// }
		// 	break;	
		// case INPUT_BYTE_0012:
		// {
        //     uint8_t * _InputByte0012 = static_cast<uint8_t * const>(data);
        //     for (int i=0; i<NUM_HMS_ANYBUS; i++)
        //     {
        //         _InputByte0012[i] = _hms_anybus[i].OutParam->InputByte0012;
        //     }
		// }
		// 	break;		
		// case INPUT_BYTE_0013:
		// {
        //     uint8_t * _InputByte0013 = static_cast<uint8_t * const>(data);
        //     for (int i=0; i<NUM_HMS_ANYBUS; i++)
        //     {
        //         _InputByte0013[i] = _hms_anybus[i].OutParam->InputByte0013;
        //     }
		// }
		// 	break;	
		// case INPUT_BYTE_0014:
		// {
        //     uint8_t * _InputByte0014 = static_cast<uint8_t * const>(data);
        //     for (int i=0; i<NUM_HMS_ANYBUS; i++)
        //     {
        //         _InputByte0014[i] = _hms_anybus[i].OutParam->InputByte0014;
        //     }
		// }
		// 	break;
		// case INPUT_BYTE_0015:
		// {
        //     uint8_t * _InputByte0015 = static_cast<uint8_t * const>(data);
        //     for (int i=0; i<NUM_HMS_ANYBUS; i++)
        //     {
        //         _InputByte0015[i] = _hms_anybus[i].OutParam->InputByte0015;
        //     }
		// }
		// 	break;		
		// case INPUT_BYTE_0016:
		// {
        //     uint8_t * _InputByte0016 = static_cast<uint8_t * const>(data);
        //     for (int i=0; i<NUM_HMS_ANYBUS; i++)
        //     {
        //         _InputByte0016[i] = _hms_anybus[i].OutParam->InputByte0016;
        //     }
		// }
		// 	break;	
		// case INPUT_BYTE_0017:
		// {
        //     uint8_t * _InputByte0017 = static_cast<uint8_t * const>(data);
        //     for (int i=0; i<NUM_HMS_ANYBUS; i++)
        //     {
        //         _InputByte0017[i] = _hms_anybus[i].OutParam->InputByte0017;
        //     }
		// }
		// 	break;	
		// case INPUT_BYTE_0018:
		// {
        //     uint8_t * _InputByte0018 = static_cast<uint8_t * const>(data);
        //     for (int i=0; i<NUM_HMS_ANYBUS; i++)
        //     {
        //         _InputByte0018[i] = _hms_anybus[i].OutParam->InputByte0018;
        //     }
		// }
		// 	break;	
		// case INPUT_BYTE_0019:
		// {
        //     uint8_t * _InputByte0019 = static_cast<uint8_t * const>(data);
        //     for (int i=0; i<NUM_HMS_ANYBUS; i++)
        //     {
        //         _InputByte0019[i] = _hms_anybus[i].OutParam->InputByte0019;
        //     }
		// }
		// 	break;	
		// case INPUT_BYTE_0020:
		// {
        //     uint8_t * _InputByte0020 = static_cast<uint8_t * const>(data);
        //     for (int i=0; i<NUM_HMS_ANYBUS; i++)
        //     {
        //         _InputByte0020[i] = _hms_anybus[i].OutParam->InputByte0020;
        //     }
		// }
		// 	break;	
		// case INPUT_BYTE_0021:
		// {
        //     uint8_t * _InputByte0021 = static_cast<uint8_t * const>(data);
        //     for (int i=0; i<NUM_HMS_ANYBUS; i++)
        //     {
        //         _InputByte0021[i] = _hms_anybus[i].OutParam->InputByte0021;
        //     }
		// }
		// 	break;
		// case INPUT_BYTE_0022:
		// {
        //     uint8_t * _InputByte0022 = static_cast<uint8_t * const>(data);
        //     for (int i=0; i<NUM_HMS_ANYBUS; i++)
        //     {
        //         _InputByte0022[i] = _hms_anybus[i].OutParam->InputByte0022;
        //     }
		// }
		// 	break;
		// case INPUT_BYTE_0023:
		// {
        //     uint8_t * _InputByte0023 = static_cast<uint8_t * const>(data);
        //     for (int i=0; i<NUM_HMS_ANYBUS; i++)
        //     {
        //         _InputByte0023[i] = _hms_anybus[i].OutParam->InputByte0023;
        //     }
		// }
		// 	break;	
		// case INPUT_BYTE_0024:
		// {
        //     uint8_t * _InputByte0024 = static_cast<uint8_t * const>(data);
        //     for (int i=0; i<NUM_HMS_ANYBUS; i++)
        //     {
        //         _InputByte0024[i] = _hms_anybus[i].OutParam->InputByte0024;
        //     }
		// }
		// 	break;
		// case INPUT_BYTE_0025:
		// {
        //     uint8_t * _InputByte0025 = static_cast<uint8_t * const>(data);
        //     for (int i=0; i<NUM_HMS_ANYBUS; i++)
        //     {
        //         _InputByte0025[i] = _hms_anybus[i].OutParam->InputByte0025;
        //     }
		// }
		// 	break;	
		// case INPUT_BYTE_0026:
		// {
        //     uint8_t * _InputByte0026 = static_cast<uint8_t * const>(data);
        //     for (int i=0; i<NUM_HMS_ANYBUS; i++)
        //     {
        //         _InputByte0026[i] = _hms_anybus[i].OutParam->InputByte0026;
        //     }
		// }
		// 	break;		
		// case INPUT_BYTE_0027:
		// {
        //     uint8_t * _InputByte0027 = static_cast<uint8_t * const>(data);
        //     for (int i=0; i<NUM_HMS_ANYBUS; i++)
        //     {
        //         _InputByte0027[i] = _hms_anybus[i].OutParam->InputByte0027;
        //     }
		// }			
			break;	
		default:	// Undefined Entry ID
		//std::cout<< "error!!!~~"<< std::hex << EntryID << INPUT_BYTE_0027 << "\t";
			break;
	}
}


void EthercatMaster::servoOn(const int i)
{		
    if (!(this->_elmo_gold[i].OutParam->StatusWord & (1<<STATUSWORD_OPERATION_ENABLE_BIT)))
	{
		if (!(this->_elmo_gold[i].OutParam->StatusWord & (1<<STATUSWORD_SWITCHED_ON_BIT))) {
			if (!(this->_elmo_gold[i].OutParam->StatusWord & (1<<STATUSWORD_READY_TO_SWITCH_ON_BIT))) {
				if ((this->_elmo_gold[i].OutParam->StatusWord & (1<<STATUSWORD_FAULT_BIT))) {
					this->_elmo_gold[i].InParam->ControlWord = 0x80; //fault reset
				}
				else
				{
					this->_elmo_gold[i].InParam->ControlWord = 0x06; //shutdown
				}
			}
			else
			{
				this->_elmo_gold[i].InParam->ControlWord = 0x07; //switch on
			}
		}
		else
		{
			this->_elmo_gold[i].InParam->ControlWord = 0x0F; //switch on
		}
	}
	else
	{
		this->_elmo_gold[i].InParam->ControlWord = 0x0F; //switch on
	}
}

