#include "qcr_kinova/KinovaGen3.hpp"

#include <mutex>
#include <thread>




// TODO: Check these.
#define IP_ADDRESS "192.168.0.10"
#define PORT 10000
#define PORT_REAL_TIME 10001

#define USERNAME "username"
#define PASSWORD "password"


KinovaGen3::KinovaGen3(ros::NodeHandle nh) { 
    kinova_api_init();
    register_interfaces();
}

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
    auto base = new k_api::Base::BaseClient(router);
    _api_base_cyclic = new k_api::BaseCyclic::BaseCyclicClient(router_real_time);

    auto servoingMode = k_api::Base::ServoingModeInformation();
    servoingMode.set_servoing_mode(k_api::Base::ServoingMode::LOW_LEVEL_SERVOING);
    base->SetServoingMode(servoingMode);
    _api_base_feedback = _api_base_cyclic->RefreshFeedback();

    int actuator_count = base->GetActuatorCount().count();

    // Initialize each actuator to its current position
    // for(int i = 0; i < actuator_count; i++) {
        // commands.push_back(_api_base_feedback.actuators(i).position());
        // _api_base_command.add_actuators()->set_position(_api_base_feedback.actuators(i).position());
    // }

}   

void KinovaGen3::register_interfaces() {
    // connect and register the joint state interface
    hardware_interface::JointStateHandle jointHandles[N_JOINTS];

    for (int i = 0; i < N_JOINTS; i++) {
        _joints[i] = KinovaJoint();

        jointHandles[i] = hardware_interface::JointStateHandle(
            "joint_" + i,
            &_joints[i].position,
            &_joints[i].velocity,
            &_joints[i].effort
        );
        _joint_state_interface.registerHandle(jointHandles[i]);

        hardware_interface::JointHandle vel_handle(
            _joint_state_interface.getHandle("joint_" + i), 
            &_joints[i].velocity_command
        );
        _velocity_joint_interface.registerHandle(vel_handle);
    }

    
    registerInterface(&_joint_state_interface);
    registerInterface(&_velocity_joint_interface);
}

void KinovaGen3::read() {
    std::scoped_lock(_lock);  // This lock unused

    for (int i = 0; i < N_JOINTS; i++) {
        _joints[i].position = _api_base_feedback.actuators(i).position();
        _joints[i].velocity = _api_base_feedback.actuators(i).velocity();
        _joints[i].effort = _api_base_feedback.actuators(i).torque();
    }
}

void KinovaGen3::write() {
    for (int i = 0; i < N_JOINTS; i++) {
        _api_base_command.mutable_actuators(i)->set_velocity(
            _joints[i].velocity_command
        );	   
    }
}