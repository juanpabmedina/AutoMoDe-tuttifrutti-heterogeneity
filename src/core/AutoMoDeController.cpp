/*
 * @file <src/core/AutoMoDeController.cpp>
 *
 * @author Antoine Ligot - <aligot@ulb.ac.be>
 *
 * @package ARGoS3-AutoMoDe
 *
 * @license MIT License
 */

#include "AutoMoDeController.h"

namespace argos {

	/****************************************/
	/****************************************/

	AutoMoDeController::AutoMoDeController() {
        m_pcRobotState = new ReferenceModel3Dot0();
		m_unTimeStep = 0;
		m_strFsmConfiguration = "";
		m_bMaintainHistory = false;
		m_bPrintReadableFsm = false;
		m_strHistoryFolder = "./";
		m_bFiniteStateMachineGiven = false;
	}

	/****************************************/
	/****************************************/

	AutoMoDeController::~AutoMoDeController() {
		delete m_pcRobotState;
		if (m_strFsmConfiguration.compare("") != 0) {
			delete m_pcFsmBuilder;
		}
	}

	/****************************************/
	/****************************************/

	void AutoMoDeController::Init(TConfigurationNode& t_node) {
		// Parsing parameters
		try {
			GetNodeAttributeOrDefault(t_node, "fsm-config", m_strFsmConfiguration, m_strFsmConfiguration);
			GetNodeAttributeOrDefault(t_node, "history", m_bMaintainHistory, m_bMaintainHistory);
			GetNodeAttributeOrDefault(t_node, "hist-folder", m_strHistoryFolder, m_strHistoryFolder);
			GetNodeAttributeOrDefault(t_node, "readable", m_bPrintReadableFsm, m_bPrintReadableFsm);
		} catch (CARGoSException& ex) {
			THROW_ARGOSEXCEPTION_NESTED("Error parsing <params>", ex);
		}

		m_unRobotID = GetRobotNumericId();

		/*
		 * If a FSM configuration is given as parameter of the experiment file, create a FSM from it
		 */
		if (m_strFsmConfiguration.compare("") != 0 && !m_bFiniteStateMachineGiven) {
			m_pcFsmBuilder = new AutoMoDeFsmBuilder();
			std::string strGroupFsm = ExtractGroupFsmConfig(m_strFsmConfiguration, m_unRobotID);
			// std::cout << "Robot id: "<< m_unRobotID << " FSM: " << strGroupFsm << std::endl;
			SetFiniteStateMachine(m_pcFsmBuilder->BuildFiniteStateMachine(strGroupFsm));
			if (m_bMaintainHistory) {
				m_pcFiniteStateMachine->SetHistoryFolder(m_strHistoryFolder);
				m_pcFiniteStateMachine->MaintainHistory();
			}
			if (m_bPrintReadableFsm) {
				std::cout << "Finite State Machine description: " << std::endl;
				std::cout << m_pcFiniteStateMachine->GetReadableFormat() << std::endl;
			}
		} else {
			LOGERR << "Warning: No finite state machine configuration found in .argos" << std::endl;
		}



		/*
		 *  Initializing sensors and actuators
		 */
		try{
			m_pcProximitySensor = GetSensor<CCI_EPuckProximitySensor>("epuck_proximity");
			m_pcLightSensor = GetSensor<CCI_EPuckLightSensor>("epuck_light");
			m_pcGroundSensor = GetSensor<CCI_EPuckGroundSensor>("epuck_ground");
			 m_pcRabSensor = GetSensor<CCI_EPuckRangeAndBearingSensor>("epuck_range_and_bearing");
			 m_pcCameraSensor = GetSensor<CCI_EPuckOmnidirectionalCameraSensor>("epuck_omnidirectional_camera");
		} catch (CARGoSException ex) {
			LOGERR<<"Error while initializing a Sensor!\n";
		}

        if(m_pcCameraSensor != NULL){
            m_pcCameraSensor->Enable();
        }

		try{
			m_pcWheelsActuator = GetActuator<CCI_EPuckWheelsActuator>("epuck_wheels");
			m_pcRabActuator = GetActuator<CCI_EPuckRangeAndBearingActuator>("epuck_range_and_bearing");
			m_pcLEDsActuator = GetActuator<CCI_EPuckRGBLEDsActuator>("epuck_rgb_leds");
		} catch (CARGoSException ex) {
			LOGERR<<"Error while initializing an Actuator!\n";
		}

		/*
		 * Starts actuation.
		 */
		 InitializeActuation();
	}
	/****************************************/
	/****************************************/

	std::string AutoMoDeController::ExtractGroupFsmConfig(const std::string& strFullConfig, UInt32 unRobotId) {
		std::istringstream iss(strFullConfig);
		std::vector<std::string> tokens;
		std::string token;

		// Tokenize input
		while (iss >> token) {
			tokens.push_back(token);
		}

		UInt32 unNumGroups = 0;
		std::vector<UInt32> vecGroupSizes;

		// First pass: extract group sizes
		for (size_t i = 0; i < tokens.size(); ++i) {
			if (tokens[i] == "--ngroups" && i + 1 < tokens.size()) {
				unNumGroups = std::stoi(tokens[i + 1]);
			} else if (tokens[i].substr(0, 3) == "--g") {
				vecGroupSizes.push_back(std::stoi(tokens[i + 1]));
			}
		}

		if (unNumGroups != vecGroupSizes.size()) {
			throw std::runtime_error("Mismatch between --ngroups and number of --g<i> entries");
		}

		// Determine group index of the robot
		UInt32 unGroupIndex = 0;
		UInt32 unAccumulatedRobots = 0;
		for (UInt32 i = 0; i < vecGroupSizes.size(); ++i) {
			unAccumulatedRobots += vecGroupSizes[i];
			if (unRobotId < unAccumulatedRobots) {
				unGroupIndex = i;
				break;
			}
		}

		// Second pass: extract only group-specific parameters (ending in _<groupIndex>)
		std::ostringstream oss;
		std::string groupSuffix = "_" + std::to_string(unGroupIndex);
		for (size_t i = 0; i + 1 < tokens.size(); ++i) {
			const std::string& key = tokens[i];
			const std::string& value = tokens[i + 1];

			// Skip general flags like --ngroups, --g0, etc.
			if (key.find("--g") == 0 || key == "--ngroups") continue;

			// Include if it ends with _<groupIndex>
			if (key.size() > groupSuffix.size() &&
				key.compare(key.size() - groupSuffix.size(), groupSuffix.size(), groupSuffix) == 0) {
				oss << key << " " << value << " ";
				++i; // Skip value
			}
		}

		return oss.str();
	}

	/****************************************/
	/****************************************/

	void AutoMoDeController::ControlStep() {
		/*
		 * 1. Update RobotDAO
		 */
		if(m_pcRabSensor != NULL){
			const CCI_EPuckRangeAndBearingSensor::TPackets& packets = m_pcRabSensor->GetPackets();
			//m_pcRobotState->SetNumberNeighbors(packets.size());
			m_pcRobotState->SetRangeAndBearingMessages(packets);
		}
		if (m_pcGroundSensor != NULL) {
			const CCI_EPuckGroundSensor::SReadings& readings = m_pcGroundSensor->GetReadings();
			m_pcRobotState->SetGroundInput(readings);
		}
		if (m_pcLightSensor != NULL) {
			const CCI_EPuckLightSensor::TReadings& readings = m_pcLightSensor->GetReadings();
			m_pcRobotState->SetLightInput(readings);
		}
		if (m_pcProximitySensor != NULL) {
			const CCI_EPuckProximitySensor::TReadings& readings = m_pcProximitySensor->GetReadings();
			m_pcRobotState->SetProximityInput(readings);
		}
        if(m_pcCameraSensor != NULL){
            const CCI_EPuckOmnidirectionalCameraSensor::SReadings& readings = m_pcCameraSensor->GetReadings();
            m_pcRobotState->SetCameraInput(readings);
        }

		/*
		 * 2. Execute step of FSM
		 */
		m_pcFiniteStateMachine->ControlStep();

		/*
		 * 3. Update Actuators
		 */
		if (m_pcWheelsActuator != NULL) {
			m_pcWheelsActuator->SetLinearVelocity(m_pcRobotState->GetLeftWheelVelocity(),m_pcRobotState->GetRightWheelVelocity());
		}
        if (m_pcLEDsActuator != NULL) {
            //m_pcLEDsActuator->SetColors(m_pcRobotState->GetLEDsColor());
            m_pcLEDsActuator->SetColor(2,m_pcRobotState->GetLEDsColor());
        }

		/*
		 * 4. Update variables and sensors
		 */
		if (m_pcRabSensor != NULL) {
			m_pcRabSensor->ClearPackets();
		}
		m_unTimeStep++;

	}
	UInt32 AutoMoDeController::GetRobotNumericId() {
		return atoi(GetId().substr(5, 6).c_str());
	}

	/****************************************/
	/****************************************/

	void AutoMoDeController::Destroy() {}

	/****************************************/
	/****************************************/

	void AutoMoDeController::Reset() {
		m_pcFiniteStateMachine->Reset();
		m_pcRobotState->Reset();
		// Restart actuation.
		InitializeActuation();
	}

	/****************************************/
	/****************************************/

	void AutoMoDeController::SetFiniteStateMachine(AutoMoDeFiniteStateMachine* pc_finite_state_machine) {
		m_pcFiniteStateMachine = pc_finite_state_machine;
		m_pcFiniteStateMachine->SetRobotDAO(m_pcRobotState);
		m_pcFiniteStateMachine->Init();
		m_bFiniteStateMachineGiven = true;
	}

	/****************************************/
	/****************************************/

	void AutoMoDeController::SetHistoryFlag(bool b_history_flag) {
		if (b_history_flag) {
			m_pcFiniteStateMachine->MaintainHistory();
		}
	}

	/****************************************/
	/****************************************/

	void AutoMoDeController::InitializeActuation() {
		/*
		 * Constantly send range-and-bearing messages containing the robot integer identifier.
		 */
		if (m_pcRabActuator != NULL) {
			UInt8 data[4];
			data[0] = m_unRobotID;
			data[1] = 0;
			data[2] = 0;
			data[3] = 0;
			m_pcRabActuator->SetData(data);
		}
	}

	REGISTER_CONTROLLER(AutoMoDeController, "automode_controller");
}
