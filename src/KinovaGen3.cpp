#include "qcr_kinova/KinovaGen3.hpp"

#include <mutex>
#include <thread>
#include <sstream>

#include <ActuatorConfigClientRpc.h>

// TODO: Check these.
#define IP_ADDRESS "192.168.1.10"
#define PORT 10000
#define PORT_REAL_TIME 10001

#define USERNAME "admin"
#define PASSWORD "admin"

KinovaGen3::KinovaGen3(ros::NodeHandle nh) { 
    kinova_api_init();
    register_interfaces();
    std::cout << "Kinova driver initialisation finished" << std::endl;
}


// // Waiting time during actions
// const auto ACTION_WAITING_TIME = std::chrono::seconds(1);

// // Create closure to set finished to true after an END or an ABORT
// std::function<void(k_api::Base::ActionNotification)> 
// check_for_end_or_abort(bool& finished)
// {
//     return [&finished](k_api::Base::ActionNotification notification)
//     {
//         std::cout << "EVENT : " << k_api::Base::ActionEvent_Name(notification.action_event()) << std::endl;

//         // The action is finished when we receive a END or ABORT event
//         switch(notification.action_event())
//         {
//         case k_api::Base::ActionEvent::ACTION_ABORT:
//         case k_api::Base::ActionEvent::ACTION_END:
//             finished = true;
//             break;
//         default:
//             break;
//         }
//     };
// }


// void example_move_to_home_position(k_api::Base::BaseClient* base)
// {
//     // Make sure the arm is in Single Level Servoing before executing an Action
//     auto servoingMode = k_api::Base::ServoingModeInformation();
//     servoingMode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
//     base->SetServoingMode(servoingMode);
//     std::this_thread::sleep_for(std::chrono::milliseconds(500));

//     // Move arm to ready position
//     std::cout << "Moving the arm to a safe position" << std::endl;
//     auto action_type = k_api::Base::RequestedActionType();
//     action_type.set_action_type(k_api::Base::REACH_JOINT_ANGLES);
//     auto action_list = base->ReadAllActions(action_type);
//     auto action_handle = k_api::Base::ActionHandle();
//     action_handle.set_identifier(0);
//     for (auto action : action_list.action_list()) 
//     {
//         if (action.name() == "Home") 
//         {
//             action_handle = action.handle();
//         }
//     }

//     if (action_handle.identifier() == 0) 
//     {
//         std::cout << "Can't reach safe position, exiting" << std::endl;
//     } 
//     else 
//     {
//         bool action_finished = false; 
//         // Notify of any action topic event
//         auto options = k_api::Common::NotificationOptions();
//         auto notification_handle = base->OnNotificationActionTopic(
//             check_for_end_or_abort(action_finished),
//             options
//         );

//         base->ExecuteActionFromReference(action_handle);

//         while(!action_finished)
//         { 
//             std::this_thread::sleep_for(ACTION_WAITING_TIME);
//         }

//         base->Unsubscribe(notification_handle);
//     }
// }

void KinovaGen3::kinova_api_init() {

    // Create API objects
    auto error_callback = [](k_api::KError err){ cout << "_________ callback error _________" << err.toString(); };
    
    auto transport = new k_api::TransportClientTcp();
    auto router = new k_api::RouterClient(transport, error_callback);
    transport->connect(IP_ADDRESS, PORT);

    auto transport_real_time = new k_api::TransportClientUdp();
    auto router_real_time = new k_api::RouterClient(transport_real_time, error_callback);
    transport_real_time->connect(IP_ADDRESS, PORT_REAL_TIME);

    // Set session data connection information
    auto create_session_info = k_api::Session::CreateSessionInfo();
    create_session_info.set_username(USERNAME);
    create_session_info.set_password(PASSWORD);
    create_session_info.set_session_inactivity_timeout(60000);   // (milliseconds)
    create_session_info.set_connection_inactivity_timeout(2000); // (milliseconds)

    // Session manager service wrapper
    std::cout << "Creating sessions for communication" << std::endl;
    auto session_manager = new k_api::SessionManager(router);
    session_manager->CreateSession(create_session_info);
    auto session_manager_real_time = new k_api::SessionManager(router_real_time);
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

    int actuator_count = _api_base->GetActuatorCount().count();
    
    std::cout << "Actuator count: " << actuator_count << std::endl;

   

    for (int i = 0; i < actuator_count; i++) {

        _api_base_command.add_actuators()->set_position(_api_base_feedback.actuators(i).position());
    }

     auto actuator_config = new k_api::ActuatorConfig::ActuatorConfigClient(router);
    auto control_mode_message = k_api::ActuatorConfig::ControlModeInformation();
    control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::VELOCITY);



     for (int i = 1; i <= actuator_count; i++) {
        actuator_config->SetControlMode(control_mode_message,i);
     }

    
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

    
    registerInterface(&_joint_state_interface);
    registerInterface(&_velocity_joint_interface);
    std::cout << "Finished registering interfaces" << std::endl;
}

void KinovaGen3::read() {
    std::scoped_lock(_lock);  // This lock unused
    // _api_base_feedback = _api_base_cyclic->RefreshFeedback();
    
    for (int i = 0; i < N_JOINTS; i++) {
        _joints[i].position = _api_base_feedback.actuators(i).position();
        _joints[i].velocity = _api_base_feedback.actuators(i).velocity();
        _joints[i].effort = _api_base_feedback.actuators(i).torque();
    }
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
    
    for (int i = 0; i < N_JOINTS; i++) {
        // std::cout << "Joint " << i << ":" << _joints[i].velocity_command << std::endl;
        _api_base_command.mutable_actuators(i)->set_velocity(
            _joints[i].velocity_command
        );	   

        _api_base_command.mutable_actuators(i)->set_position(
            _joints[i].position
        );

    }
    _api_base_feedback = _api_base_cyclic->Refresh(_api_base_command);
    // _api_base_cyclic->Refresh_callback(_api_base_command, fct_callback, 0);
}