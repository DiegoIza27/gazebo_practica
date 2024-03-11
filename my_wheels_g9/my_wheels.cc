#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

namespace gazebo
{

    class MyWheels : public ModelPlugin
    {
    public:
        void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
        {
            // Store the pointer to the model
            model = _model;

            if (!_sdf->HasElement("left_joint"))
                gzerr << "MyWheels plugin missing <left_joint> element\n";

            if (!_sdf->HasElement("right_joint"))
                gzerr << "MyWheels plugin missing <right_joint> element\n";

            leftJoint = _model->GetJoint(_sdf->GetElement("left_joint")->Get<std::string>());
            rightJoint = _model->GetJoint(_sdf->GetElement("right_joint")->Get<std::string>());

            if (!leftJoint)
                gzerr << "Unable to find left joint[" << _sdf->GetElement("left_joint")->Get<std::string>() << "]\n";
            if (!rightJoint)
                gzerr << "Unable to find right joint[" << _sdf->GetElement("right_joint")->Get<std::string>() << "]\n";

            // Listen to the update event. This event is broadcast every simulation iteration.
            updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&MyWheels::OnUpdate, this));
        }

        // Called by the world update start event
        void OnUpdate()
        {
            float left_vel = 1.0;
            float right_vel = 1.0;

            ignition::math::Pose3d pose = model->WorldPose();
            printf("At: %f %f %f\n", pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z());

            double targetX1 = 2.5;
            double targetY1 = 4.0;
            double distance1 = std::sqrt(std::pow(pose.Pos().X() - targetX1, 2) + std::pow(pose.Pos().Y() - targetY1, 2));

            double threshold1 = 1;

            if (distance1 >= threshold1 && pose.Pos().X() < targetX1 && pose.Pos().Y() < targetY1)
            {
                if ( x % 5 == 0 ) {
                    left_vel = -left_vel;
                }                
            }            
            else {
                if ( x % 10 == 0 ) {
                    right_vel = -right_vel;
                }                   
            }


            leftJoint->SetVelocity(0, left_vel);
            rightJoint->SetVelocity(0, right_vel);

            x++;
        }
    
    private:
        physics::ModelPtr model; // Pointer to the model
        physics::JointPtr leftJoint, rightJoint;
        event::ConnectionPtr updateConnection; // Pointer to the update event connection

        static int x;
    };

    int MyWheels::x = 0.0;

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(MyWheels)
}