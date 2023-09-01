#include "qcr_kinova/KinovaGen3.hpp"

#include <mutex>
#include <thread>
#include <sstream>

// TODO: Check these.
#define IP_ADDRESS "192.168.1.10"
#define PORT 10000
#define PORT_REAL_TIME 10001

#define USERNAME "admin"
#define PASSWORD "admin"

#define DIRECT_VELOCITY false


KinovaGen3::KinovaGen3(ros::NodeHandle nh) { 
    kinova_api_init();
    register_interfaces();
    std::cout << "Kinova driver initialisation finished" << std::endl;
}

double deg2rad(double deg) {
    return deg * 3.14159265358979 / 180.0;
}

double rad2deg(double rad) {
    return rad * 180.0 / 3.14159265358979;
}

// Waiting time during actions
const auto ACTION_WAITING_TIME = std::chrono::seconds(1);

// Create closure to set finished to true after an END or an ABORT
std::function<void(k_api::Base::ActionNotification)> 
check_for_end_or_abort(bool& finished)
{
    return [&finished](k_api::Base::ActionNotification notification)
    {
        std::cout << "EVENT : " << k_api::Base::ActionEvent_Name(notification.action_event()) << std::endl;

        // The action is finished when we receive a END or ABORT event
        switch(notification.action_event())
        {
        case k_api::Base::ActionEvent::ACTION_ABORT:
        case k_api::Base::ActionEvent::ACTION_END:
            finished = true;
            break;
        default:
            break;
        }
    };
}


void example_move_to_home_position(k_api::Base::BaseClient* base)
{
    // Make sure the arm is in Single Level Servoing before executing an Action
    auto servoingMode = k_api::Base::ServoingModeInformation();
    servoingMode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    base->SetServoingMode(servoingMode);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Move arm to ready position
    std::cout << "Moving the arm to a safe position" << std::endl;
    auto action_type = k_api::Base::RequestedActionType();
    action_type.set_action_type(k_api::Base::REACH_JOINT_ANGLES);
    auto action_list = base->ReadAllActions(action_type);
    auto action_handle = k_api::Base::ActionHandle();
    action_handle.set_identifier(0);
    for (auto action : action_list.action_list()) 
    {
        if (action.name() == "Home") 
        {
            action_handle = action.handle();
        }
    }

    if (action_handle.identifier() == 0) 
    {
        std::cout << "Can't reach safe position, exiting" << std::endl;
    } 
    else 
    {
        bool action_finished = false; 
        // Notify of any action topic event
        auto options = k_api::Common::NotificationOptions();
        auto notification_handle = base->OnNotificationActionTopic(
            check_for_end_or_abort(action_finished),
            options
        );

        base->ExecuteActionFromReference(action_handle);

        while(!action_finished)
        { 
            std::this_thread::sleep_for(ACTION_WAITING_TIME);
        }

        base->Unsubscribe(notification_handle);
    }
}

void KinovaGen3::kinova_api_init() {

    // Create API objects
    auto error_callback = [](k_api::KError err){ cout << "_________ callback error _________" << err.toString(); };
    
    transport = new k_api::TransportClientTcp();
    router = new k_api::RouterClient(transport, error_callback);
    transport->connect(IP_ADDRESS, PORT);

    transport_real_time = new k_api::TransportClientUdp();
    router_real_time = new k_api::RouterClient(transport_real_time, error_callback);
    transport_real_time->connect(IP_ADDRESS, PORT_REAL_TIME);

    // Set session data connection information
    auto create_session_info = k_api::Session::CreateSessionInfo();
    create_session_info.set_username(USERNAME);
    create_session_info.set_password(PASSWORD);
    create_session_info.set_session_inactivity_timeout(60000);   // (milliseconds)
    create_session_info.set_connection_inactivity_timeout(2000); // (milliseconds)

    // Session manager service wrapper
    std::cout << "Creating sessions for communication" << std::endl;
    session_manager = new k_api::SessionManager(router);
    session_manager->CreateSession(create_session_info);
    session_manager_real_time = new k_api::SessionManager(router_real_time);
    session_manager_real_time->CreateSession(create_session_info);
    std::cout << "Sessions created" << std::endl;

    // Create services
    _api_base = new k_api::Base::BaseClient(router);
    _api_base_cyclic = new k_api::BaseCyclic::BaseCyclicClient(router_real_time);
    
    // example_move_to_home_position(_api_base);

    auto servoingMode = k_api::Base::ServoingModeInformation();
    servoingMode.set_servoing_mode(k_api::Base::ServoingMode::LOW_LEVEL_SERVOING);
    _api_base->SetServoingMode(servoingMode);
    _api_base_feedback = _api_base_cyclic->RefreshFeedback();

    // Gripper Stuff
    gripper_position = _api_base_feedback.interconnect().gripper_feedback().motor()[0].position();
    target_gripper_position = gripper_position;

    _api_base_command.mutable_interconnect()->mutable_command_id()->set_identifier(0);
    gripper_command = _api_base_command.mutable_interconnect()->mutable_gripper_command()->add_motor_cmd();
    gripper_command->set_position(gripper_position);
    gripper_command->set_velocity(0.0);
    gripper_command->set_force(100.0);


    // Actuator init
    int actuator_count = _api_base->GetActuatorCount().count();
    
    std::cout << "Actuator count: " << actuator_count << std::endl;

    for (int i = 0; i < actuator_count; i++) {
        _api_base_command.add_actuators()->set_position(_api_base_feedback.actuators(i).position());
    }

    _api_base_feedback = _api_base_cyclic->Refresh(_api_base_command);

    if (DIRECT_VELOCITY) {
        actuator_config = new k_api::ActuatorConfig::ActuatorConfigClient(router);
        auto control_mode_message = k_api::ActuatorConfig::ControlModeInformation();
        control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::VELOCITY);

        for (int i = 1; i <= actuator_count; i++) {
            actuator_config->SetControlMode(control_mode_message, i);
        }
    }
        
}   

void KinovaGen3::set_gripper_position_ros(const std_msgs::Float32::ConstPtr& msg) {
    set_gripper_position(msg->data);
}

void KinovaGen3::set_gripper_position(float pos) {
    target_gripper_position = pos;
    std::cout << "New gripper target: " << pos << ". Current: " << gripper_position << std::endl;
    gripper_command->set_position(pos);

	/*
    k_api::Base::GripperCommand gripper_command;
    gripper_command.set_mode(k_api::Base::GRIPPER_POSITION);
    auto finger = gripper_command.mutable_gripper()->add_finger();
    finger->set_finger_identifier(1);

    if (pos > 1) {
        pos = 1;
    } else if (pos < 0) {
        pos = 0;
    }
    finger->set_value(pos);
    _api_base->SendGripperCommand(gripper_command);
    */
}

void KinovaGen3::register_interfaces() {
    std::cout << "Starting registering interfaces" << std::endl;
    // connect and register the joint state interface
    hardware_interface::JointStateHandle jointHandles[N_JOINTS];

    for (int i = 0; i < N_JOINTS; i++) {
        std::stringstream joint_name;
        joint_name << "joint_" << i;

        _joints[i] = KinovaJoint();

        jointHandles[i] = hardware_interface::JointStateHandle(
            joint_name.str(),
            &_joints[i].position,
            &_joints[i].velocity,
            &_joints[i].effort
        );
        _joint_state_interface.registerHandle(jointHandles[i]);
        // std::cout << "joint_" + i << std::endl;
        hardware_interface::JointHandle vel_handle(
            _joint_state_interface.getHandle(joint_name.str()), 
            &_joints[i].velocity_command
        );
        _velocity_joint_interface.registerHandle(vel_handle);
    }

    double pos;
    for (int i = 0; i < N_JOINTS; i++) {
        pos = fmod(_api_base_feedback.actuators(i).position() + 180.0, 360.0) - 180.0;
        _joints[i].position_command = deg2rad(pos);
    }
    
    registerInterface(&_joint_state_interface);
    registerInterface(&_velocity_joint_interface);
    std::cout << "Finished registering interfaces" << std::endl;
}

void KinovaGen3::read() {
    std::scoped_lock(_lock);  // This lock unused
    // _api_base_feedback = _api_base_cyclic->RefreshFeedback();

    double pos;
    for (int i = 0; i < N_JOINTS; i++) {
        pos = fmod(_api_base_feedback.actuators(i).position() + 180.0, 360.0) - 180.0;
        _joints[i].position = deg2rad(pos);
        _joints[i].velocity = deg2rad(_api_base_feedback.actuators(i).velocity());
        _joints[i].effort = _api_base_feedback.actuators(i).torque();
    }

    gripper_position = _api_base_feedback.interconnect().gripper_feedback().motor()[0].position();
}

void fct_callback(const Kinova::Api::Error &err, const k_api::BaseCyclic::Feedback data) {
    // We are printing the data of the moving actuator just for the example purpose,
    // avoid this in a real-time loop
    std::string serialized_data;
    // google::protobuf::util::MessageToJsonString(data.actuators(data.actuators_size() - 1), &serialized_data);
    // std::cout << serialized_data << std::endl << std::endl;
};

void KinovaGen3::write() {
    // std::cout << "Writing velocity command: " << std::endl;
    _api_base_command.set_frame_id(_api_base_command.frame_id() + 1);
    if (_api_base_command.frame_id() > 65535)
        _api_base_command.set_frame_id(0);

    // std::cout << std::endl;
    for (int i = 0; i < N_JOINTS; i++) {
        // std::cout << "Joint " << i << ":" << _joints[i].velocity_command << std::endl;
        _api_base_command.mutable_actuators(i)->set_command_id(_api_base_command.frame_id());
        if (DIRECT_VELOCITY) {
            _api_base_command.mutable_actuators(i)->set_velocity(
                rad2deg(_joints[i].velocity_command)
            );

            double pos_deg = fmod(rad2deg(_joints[i].position), 360.0f);
            if (pos_deg < 0) {
                pos_deg += 360;
            }
            // Must send position to prevent following error. 
            _api_base_command.mutable_actuators(i)->set_position(pos_deg);
        } else {
            
            _joints[i].position_command += _joints[i].velocity_command * 0.001f;

            double pos_command_deg = fmod(rad2deg(_joints[i].position_command), 360.0f);
            if (pos_command_deg < 0) {
                pos_command_deg += 360;
            }
            // std::cout << i <<  " : " << _joints[i].position_command  << " , " << fmod(rad2deg(_joints[i].position_command), 360.0f) << " , " << pos_command_deg << " , " << _api_base_feedback.actuators(i).position() << std::endl;
            _api_base_command.mutable_actuators(i)->set_position(pos_command_deg);
        }
        	   
    }

    

    _api_base_feedback = _api_base_cyclic->Refresh(_api_base_command);
    // _api_base_cyclic->Refresh_callback(_api_base_command, fct_callback, 0);
}

void KinovaGen3::cleanup() {
    std::cout << "Running Kinova cleanup" << std::endl;
    if (DIRECT_VELOCITY) {
        std::cout << "Returning actuators to position control" << std::endl;
        // actuator_config = new k_api::ActuatorConfig::ActuatorConfigClient(router);
        auto control_mode_message = k_api::ActuatorConfig::ControlModeInformation();
        control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::POSITION);

        for (int i = 1; i <= N_JOINTS; i++) {
            actuator_config->SetControlMode(control_mode_message, i);
        }
        std::cout << "Actuators in position mode" << std::endl;
    }
    // std::cout << "Waiting" << std::endl;
    // std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    
    std::cout << "Returning arm to single level servoing" << std::endl;
    auto servoingMode = k_api::Base::ServoingModeInformation();
    servoingMode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    _api_base->SetServoingMode(servoingMode);

    // std::this_thread::sleep_for(std::chrono::milliseconds(2000));

     // Close API session
    session_manager->CloseSession();
    session_manager_real_time->CloseSession();

    // Deactivate the router and cleanly disconnect from the transport object
    router->SetActivationStatus(false);
    transport->disconnect();
    router_real_time->SetActivationStatus(false);
    transport_real_time->disconnect();

}
