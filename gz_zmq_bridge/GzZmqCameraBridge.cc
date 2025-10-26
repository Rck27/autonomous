#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/image.pb.h>

// ADDED: The necessary header for all gz::sim components (System, Entity, etc.)
#include <gz/sim/System.hh> 

#include <zmq.hpp>
#include <iostream>
#include <string>

// CORRECTED: Inherit from gz::sim::System, not gz::plugin::System
class GzZmqCameraBridge : public gz::sim::System 
{
public:
    GzZmqCameraBridge() : context(1), socket(context, ZMQ_PUB) {}

    void Configure(const gz::sim::Entity &,
                   const std::shared_ptr<const sdf::Element> &_sdf,
                   gz::sim::EntityComponentManager &,
                   gz::sim::EventManager &) override
    {
        // Get camera topic from SDF, default to /camera
        std::string camera_topic = _sdf->Get<std::string>("camera_topic", "/camera").first;
        
        // Get ZMQ address from SDF, default to tcp://*:5555
        std::string zmq_address = _sdf->Get<std::string>("zmq_address", "tcp://*:5555").first;

        socket.bind(zmq_address);
        std::cout << "[ZMQ Bridge] Publisher bound to " << zmq_address << std::endl;
        
        node.Subscribe(camera_topic, &GzZmqCameraBridge::OnImage, this);
        std::cout << "[ZMQ Bridge] Subscribed to Gazebo topic: " << camera_topic << std::endl;
    }

private:
    void OnImage(const gz::msgs::Image &msg)
    {
        // Part 1: Send shape (height, width, channels) as int32 array
        int32_t shape[] = {static_cast<int32_t>(msg.height()), static_cast<int32_t>(msg.width()), 3};
        zmq::message_t shape_msg(sizeof(shape));
        memcpy(shape_msg.data(), shape, sizeof(shape));
        socket.send(shape_msg, zmq::send_flags::sndmore);

        // Part 2: Send the raw image data
        zmq::message_t data_msg(msg.data().size());
        memcpy(data_msg.data(), msg.data().c_str(), msg.data().size());
        socket.send(data_msg, zmq::send_flags::none);
    }

    gz::transport::Node node;
    zmq::context_t context;
    zmq::socket_t socket;
};

GZ_PLUGIN_REGISTER_CLASS(GzZmqCameraBridge)